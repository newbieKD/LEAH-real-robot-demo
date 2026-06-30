import rclpy
from rclpy.node import Node
import torch
import numpy as np
import clip
import cv2
import time
from collections import deque
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from realsense2_camera_msgs.msg import RGBD
from std_msgs.msg import Float32MultiArray, Float32, Header
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import interp1d

import sys, os
sys.path.append('/home/user/RoboBounce')
from training.utils import Resize, get_pcd, get_training_extrinsics, filter_pcd_background
from training.preprocessing import crop_rgbd


# Uniform upward shift added to every action z = (raised z_min 0.274 - original 0.21).
# Z_OFFSET = 0.064
Z_OFFSET = 0.0
# Uniform shift added to every action x, in the world frame (+ = world +x).
X_OFFSET = 0.00

# Return-to-home routine: every HOME_EVERY_N_INFERENCES inferences, command the robot to this
# recorded home EE pose (world frame: [x,y,z, qx,qy,qz,qw]), wait HOME_WAIT_SEC for it to
# arrive, then resume inference.
HOME_POSE = [0.539, 0.075, 0.432, 0.022, 0.719, -0.011, 0.694]
HOME_EVERY_N_INFERENCES = 6
HOME_WAIT_SEC = 5.0
# At node startup, command the robot once to HOME_POSE and wait this long to reach it.
RESET_SETTLE_SEC = 5.0


class DiffuserInferenceNode(Node):
    def __init__(self):
        super().__init__('diffuser_inference_node')

        # Parameters
        self.declare_parameter("checkpoint", "")
        self.declare_parameter("instruction", "stack the cups")
        self.declare_parameter("input_rgbd_topic", "/input/rgbd_image")
        self.declare_parameter("output_traj_topic", "/diffuser_actor/trajectory")
        self.declare_parameter("model_type", "ours")  # "ours" or "baseline"

        ckpt_path = self.get_parameter("checkpoint").value
        self.model_type = self.get_parameter("model_type").value
        self.instruction_text = self.get_parameter("instruction").value

        if not ckpt_path:
            self.get_logger().error("No checkpoint provided!")
            return

        self.device = "cuda:0" if torch.cuda.is_available() else "cpu"

        # Load Model
        self.get_logger().info(f"Loading checkpoint from {ckpt_path}")
        ckpt = torch.load(ckpt_path, map_location=self.device, weights_only=False)

        # DEBUG: Print checkpoint contents
        self.get_logger().info(f"[DEBUG] Checkpoint keys: {list(ckpt.keys())}")
        if 'iteration' in ckpt:
            self.get_logger().info(f"[DEBUG] Checkpoint iteration: {ckpt['iteration']}")
        if 'model_state_dict' in ckpt:
            state_dict_keys = list(ckpt['model_state_dict'].keys())
            self.get_logger().info(f"[DEBUG] State dict has {len(state_dict_keys)} keys")
            self.get_logger().info(f"[DEBUG] First 5 keys: {state_dict_keys[:5]}")

        self.model_args = ckpt['args']
        self.get_logger().info(f"[DEBUG] Model args: relative={self.model_args.relative}, nhist={self.model_args.nhist}, interpolation_length={self.model_args.interpolation_length}")

        # Override diffusion timesteps for faster inference (0 = use training value)
        self.declare_parameter("inference_diffusion_timesteps", 0)
        inference_timesteps = self.get_parameter("inference_diffusion_timesteps").value
        if inference_timesteps > 0:
            original_timesteps = self.model_args.diffusion_timesteps
            self.model_args.diffusion_timesteps = inference_timesteps
            self.get_logger().info(f"Overriding diffusion_timesteps: {original_timesteps} -> {inference_timesteps} for faster inference")
        else:
            self.get_logger().info(f"Using training diffusion_timesteps: {self.model_args.diffusion_timesteps}")

        self.get_logger().info("sys path: " + str(sys.path))

        # Import model based on model_type
        if self.model_type == "baseline":
            from diffuser_actor.trajectory_optimization.diffuser_actor_baseline import DiffuserActor
            self.get_logger().info("Using BASELINE model (original DiffuserActor)")
        elif self.model_type == "lie":
            from diffuser_actor.trajectory_optimization.lie_diffuser_actor import DiffuserActor
            self.get_logger().info("Using LIE model (Lie group diffusion without GAT)")
        else:
            from diffuser_actor.trajectory_optimization.diffuser_actor import DiffuserActor
            self.get_logger().info("Using OUR model (with GAT)")

        self.model = DiffuserActor(
            backbone=self.model_args.backbone,
            image_size=tuple(map(int, self.model_args.image_size.split(","))),
            embedding_dim=self.model_args.embedding_dim,
            num_vis_ins_attn_layers=self.model_args.num_vis_ins_attn_layers,
            use_instruction=bool(self.model_args.use_instruction),
            fps_subsampling_factor=self.model_args.fps_subsampling_factor,
            gripper_loc_bounds=np.array([[-1.0, -1.0, -0.5], [1.0, 1.0, 1.5]]),
            rotation_parametrization=self.model_args.rotation_parametrization,
            quaternion_format=self.model_args.quaternion_format,
            diffusion_timesteps=self.model_args.diffusion_timesteps,
            nhist=self.model_args.nhist,
            relative=bool(self.model_args.relative),
            lang_enhanced=bool(self.model_args.lang_enhanced),
            args=self.model_args
        ).to(self.device)

        if 'model_state_dict' in ckpt:
            # DEBUG: Check a weight before loading
            sample_key = list(self.model.state_dict().keys())[0]
            before_weight = self.model.state_dict()[sample_key].clone()

            self.model.load_state_dict(ckpt['model_state_dict'])

            # DEBUG: Check the same weight after loading
            after_weight = self.model.state_dict()[sample_key]
            weight_changed = not torch.equal(before_weight, after_weight)
            self.get_logger().info(f"[DEBUG] Weight '{sample_key}' changed after load: {weight_changed}")
            self.get_logger().info(f"[DEBUG] Before: {before_weight.flatten()[:5]}")
            self.get_logger().info(f"[DEBUG] After: {after_weight.flatten()[:5]}")
        else:
            self.model.load_state_dict(ckpt)
        self.model.eval()
        self.get_logger().info("[DEBUG] Model loaded and set to eval mode")

        # Load CLIP
        self.clip_model, _ = clip.load("ViT-B/32", device=self.device)
        self.clip_model.eval()

        # Embed Instruction once
        with torch.no_grad():
            tokenized = clip.tokenize([self.instruction_text]).to(self.device)
            self.instr_embed = self.clip_model.encode_text(tokenized).float().unsqueeze(1)  # (1, 1, 512)

        # Buffers
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_camera_info = None
        self.latest_info_timestamp = None

        # Movement-based gripper history buffer (matching training preprocessing)
        # Only captures when robot moves significantly, like preprocessing.py
        self.declare_parameter("history_capture_rate", 10.0)  # Hz, check rate for movement
        self.declare_parameter("history_pos_thresh", 0.05)  # 5cm position threshold
        self.declare_parameter("history_rot_thresh_deg", 3.0)  # 3 degrees rotation threshold
        self.declare_parameter("history_gripper_thresh", 0.1)  # 10% gripper change threshold
        self.history_capture_rate = self.get_parameter("history_capture_rate").value
        self.history_pos_thresh = self.get_parameter("history_pos_thresh").value
        self.history_rot_thresh = np.radians(self.get_parameter("history_rot_thresh_deg").value)
        self.history_gripper_thresh = self.get_parameter("history_gripper_thresh").value

        # History buffer stores actions only when significant movement occurs
        # We just need nhist entries (like training takes consecutive frames)
        self.history_buffer = deque(maxlen=self.model_args.nhist * 2)  # Small buffer
        self.last_captured_pose = None  # Track last captured pose for movement comparison
        self.last_captured_gripper = None
        self.get_logger().info(
            f"History: movement-based, pos_thresh={self.history_pos_thresh}m, "
            f"rot_thresh={np.degrees(self.history_rot_thresh):.1f}deg, "
            f"gripper_thresh={self.history_gripper_thresh}, nhist={self.model_args.nhist}"
        )

        # Trajectory execution parameters
        self.declare_parameter("trajectory_steps", 0)  # 0 = use all, N = use first N steps
        self.trajectory_steps = self.get_parameter("trajectory_steps").value

        # Safety limits
        self.declare_parameter("z_min", 0.23)  # Minimum z height to avoid table collision
        self.z_min = self.get_parameter("z_min").value
        self.get_logger().info(f"Safety limit: z_min = {self.z_min}")

        self.cv_bridge = CvBridge()

        self.declare_parameter("joint_base_names", ["shoulder_link", "forearm_link", "wrist_1_link", "wrist_2_link", "rg2_base_link"])
        self.joint_base_names = self.get_parameter("joint_base_names").value
        self.declare_parameter("base_frame_id", "world")
        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.declare_parameter("gripper_width_topic", "/rg2/gripper_width")

        self.current_gripper_width = None  # Wait for real data from gripper_callback

        # Gripper close wait (pause when grasping: open → close)
        self.declare_parameter("gripper_close_wait_duration", 1.5)
        self.gripper_close_wait_duration = self.get_parameter("gripper_close_wait_duration").value
        self.last_gripper_command = None  # Track last commanded gripper state
        self.waiting_for_gripper = False
        self.gripper_wait_start_time = None

        # Subscriptions
        self.create_subscription(RGBD, self.get_parameter("input_rgbd_topic").value, self.rgbd_callback, 10)
        self.gripper_sub = self.create_subscription(Float32, self.get_parameter("gripper_width_topic").value, self.gripper_callback, 10)

        # Publishers
        self.target_pose_pub = self.create_publisher(PoseStamped, "/target_pose", 10)
        self.gripper_command_pub = self.create_publisher(Float32, "/rg2/command", 10)

        # Pointcloud publisher for RViz visualization
        self.pointcloud_pub = self.create_publisher(PointCloud2, "/diffuser_inference/pointcloud", 10)

        self.declare_parameter("inference_rate", 5.0)
        self.declare_parameter("control_rate", 20.0)

        # Buffers for RHC (Receding Horizon Control)
        self.latest_plan = None
        self.plan_index = 0

        # Return-to-home routine state
        self.returning_home = False
        self.home_start_time = None

        # Frequency tracking for evaluation
        self.inference_count = 0
        self.action_count = 0
        self.freq_tracking_start_time = None
        self.freq_log_interval = 5.0  # Log frequencies every 5 seconds
        self.inference_times = []  # Store inference times in milliseconds
        
        # Metrics file logging
        self.declare_parameter("save_metrics", True)
        self.declare_parameter("metrics_dir", "eval_metrics")
        self.save_metrics = self.get_parameter("save_metrics").value
        self.metrics_dir = self.get_parameter("metrics_dir").value
        self.metrics_file = None
        
        if self.save_metrics:
            # Create metrics directory if it doesn't exist
            os.makedirs(self.metrics_dir, exist_ok=True)
            
            # Create metrics file with timestamp
            from datetime import datetime
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"metrics_{self.model_type}_{timestamp}.txt"
            self.metrics_file = os.path.join(self.metrics_dir, filename)
            
            # Write header to metrics file
            with open(self.metrics_file, 'w') as f:
                f.write(f"Model: {self.model_type}\n")
                f.write(f"Task: {self.instruction_text}\n")
                f.write(f"Checkpoint: {ckpt_path}\n")
                f.write(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"\n")
            
            self.get_logger().info(f"Metrics will be saved to: {self.metrics_file}")

        self.create_timer(1.0 / self.get_parameter("inference_rate").value, self.inference_loop)
        self.create_timer(1.0 / self.get_parameter("control_rate").value, self.control_loop)
        self.create_timer(1.0 / self.history_capture_rate, self.history_capture_callback)
        self.create_timer(self.freq_log_interval, self.log_frequencies)
        self.get_logger().info(f"inference rate: {self.get_parameter('inference_rate').value} Hz, control rate: {self.get_parameter('control_rate').value} Hz, history capture rate: {self.history_capture_rate} Hz")

        # TF
        from tf2_ros.buffer import Buffer
        from tf2_ros.transform_listener import TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_tf = None
        self.extrinsics_matrix = None

        # Augmentation (same as training with scale=1.0)
        self._resize = Resize(scales=(1.0, 1.0))

        # Move the robot to the reset/home pose once at startup, then begin inference.
        self._publish_reset_pose_on_start()

    def _publish_reset_pose_on_start(self):
        """Publish the reset/home pose once at startup and wait for the robot to reach it."""
        # Wait briefly for a /target_pose subscriber (ur_pose_tracking) so the single
        # publish isn't lost to discovery.
        for _ in range(50):  # up to ~5s
            if self.target_pose_pub.get_subscription_count() > 0:
                break
            time.sleep(0.1)
        self.get_logger().info("[reset] publishing reset pose once at startup")
        self._publish_home_pose()  # HOME_POSE + gripper open
        self.get_logger().info(f"[reset] waiting {RESET_SETTLE_SEC}s for robot to reach reset pose...")
        time.sleep(RESET_SETTLE_SEC)
        self.get_logger().info("[reset] done; starting inference")

    def rgbd_callback(self, msg):
        try:
            self.latest_rgb = self.cv_bridge.imgmsg_to_cv2(msg.rgb, "bgr8")
            self.latest_depth = self.cv_bridge.imgmsg_to_cv2(msg.depth, "16UC1")
            self.latest_camera_info = msg.rgb_camera_info
            self.latest_info_timestamp = self.get_clock().now()
        except Exception as e:
            self.get_logger().error(f"Image decode failed: {e}")

    def gripper_callback(self, msg: Float32):
        self.current_gripper_width = msg.data

    def _quaternion_angular_diff(self, q1, q2):
        """Returns absolute angular difference in radians between two quaternions [x,y,z,w]."""
        # q_diff = q2 * q1^-1
        # For unit quaternions, q^-1 = conjugate = [-x, -y, -z, w]
        q1_conj = np.array([-q1[0], -q1[1], -q1[2], q1[3]])
        # Quaternion multiply: q2 * q1_conj
        x1, y1, z1, w1 = q1_conj
        x2, y2, z2, w2 = q2
        q_diff = np.array([
            w2*x1 + x2*w1 + y2*z1 - z2*y1,
            w2*y1 - x2*z1 + y2*w1 + z2*x1,
            w2*z1 + x2*y1 - y2*x1 + z2*w1,
            w2*w1 - x2*x1 - y2*y1 - z2*z1
        ])
        # Angular difference = 2 * arccos(|w|)
        w = np.clip(q_diff[3], -1.0, 1.0)
        return 2.0 * np.arccos(np.abs(w))

    def history_capture_callback(self):
        """Capture gripper state only when significant movement occurs (like training preprocessing)."""
        # Skip until we have real gripper data
        if self.current_gripper_width is None:
            return

        try:
            time_now = rclpy.time.Time()
            # Get EE pose (last joint = rg2_base_link)
            trans = self.tf_buffer.lookup_transform(self.base_frame_id, self.joint_base_names[-1], time_now)
            curr_pose = np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z,
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w,
            ])
            curr_gripper = self.current_gripper_width

            # Check if we should capture (movement-based filtering)
            should_capture = False

            if self.last_captured_pose is None:
                # First capture, always store
                should_capture = True
            else:
                # Check position difference
                pos_diff = np.linalg.norm(curr_pose[:3] - self.last_captured_pose[:3])

                # Check rotation difference
                rot_diff = self._quaternion_angular_diff(self.last_captured_pose[3:7], curr_pose[3:7])

                # Check gripper difference
                gripper_diff = abs(curr_gripper - self.last_captured_gripper)

                # Capture if any threshold exceeded (like preprocessing.py)
                if pos_diff >= self.history_pos_thresh or \
                   rot_diff >= self.history_rot_thresh or \
                   gripper_diff >= self.history_gripper_thresh:
                    should_capture = True

            if should_capture:
                action = np.concatenate([curr_pose, [curr_gripper]])  # (8,)
                self.history_buffer.append(action)
                self.last_captured_pose = curr_pose.copy()
                self.last_captured_gripper = curr_gripper

        except Exception as e:
            # TF not ready yet, skip
            pass

    def _get_history(self, nhist):
        """Get last nhist entries from history buffer (like training takes consecutive frames)."""
        if len(self.history_buffer) == 0:
            return None

        buffer_list = list(self.history_buffer)

        if len(buffer_list) >= nhist:
            # Take last nhist entries
            return np.stack(buffer_list[-nhist:])  # (nhist, 8)
        else:
            # Pad with first entry if not enough history
            padded = []
            for i in range(nhist):
                idx = max(0, len(buffer_list) - (nhist - i))
                if idx < len(buffer_list):
                    padded.append(buffer_list[idx])
                else:
                    padded.append(buffer_list[0])
            return np.stack(padded)  # (nhist, 8)

    def publish_pointcloud(self, pcd_np, rgb_image, frame_id="camera_depth_optical_frame"):
        """
        Publish pointcloud to RViz
        Args:
            pcd_np: (3, H, W) numpy array with x, y, z coordinates in meters
            rgb_image: (H, W, 3) numpy array with RGB values 0-255
            frame_id: TF frame for the pointcloud
        """
        h, w = pcd_np.shape[1], pcd_np.shape[2]
        points_xyz = pcd_np.reshape(3, -1).T  # (N, 3)

        # Get RGB colors
        if rgb_image is not None and rgb_image.shape[0] == h and rgb_image.shape[1] == w:
            if len(rgb_image.shape) == 3 and rgb_image.shape[2] == 3:
                # Expect RGB input, no channel swap needed
                rgb_flat = rgb_image.reshape(-1, 3).astype(np.uint8)  # (N, 3)
            else:
                rgb_flat = np.zeros((points_xyz.shape[0], 3), dtype=np.uint8)
        else:
            # No color, use white
            self.get_logger().warn(f"RGB image shape {rgb_image.shape if rgb_image is not None else None} doesn't match pcd shape ({h}, {w})")
            rgb_flat = np.ones((points_xyz.shape[0], 3), dtype=np.uint8) * 255

        # Filter out invalid points (z <= 0)
        # valid_mask = points_xyz[:, 2] > 0
        valid_mask = np.ones(points_xyz.shape[0], dtype=bool)  # Keep all points

        # Debug: Check where invalid points are located
        invalid_count = np.sum(~valid_mask)
        total_count = len(valid_mask)
        if invalid_count > 0:
            invalid_2d = (~valid_mask).reshape(h, w)
            invalid_per_row = invalid_2d.sum(axis=1)
            top_invalid = invalid_per_row[:h//4].sum()
            bottom_invalid = invalid_per_row[-h//4:].sum()
            self.get_logger().info(
                f"Invalid points: {invalid_count}/{total_count} "
                f"(top quarter: {top_invalid}, bottom quarter: {bottom_invalid})",
                throttle_duration_sec=2.0
            )

        points_xyz = points_xyz[valid_mask]
        rgb_flat = rgb_flat[valid_mask]

        # Create PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # Pack RGB into float for RViz
        points_with_color = []
        for i in range(len(points_xyz)):
            x, y, z = points_xyz[i]
            r, g, b = rgb_flat[i]
            rgb_int = (int(r) << 16) | (int(g) << 8) | int(b)
            rgb_as_float = np.array(rgb_int, dtype=np.uint32).view(np.float32)
            points_with_color.append([x, y, z, rgb_as_float])

        pc2_msg = point_cloud2.create_cloud(header, fields, points_with_color)
        self.pointcloud_pub.publish(pc2_msg)
        self.get_logger().info(f"Published pointcloud with {len(points_xyz)} valid points", throttle_duration_sec=2.0)

    def inference_loop(self):
        # Pause inference while returning to the home pose.
        if self.returning_home:
            return

        if self.latest_rgb is None or self.latest_camera_info is None:
            return

        # Wait for gripper data
        if self.current_gripper_width is None:
            self.get_logger().info("Waiting for gripper data...", throttle_duration_sec=1.0)
            return

        # If plan is still valid, skip inference
        if self.latest_plan is not None and self.plan_index < len(self.latest_plan):
            return

        # Get All Joint Poses
        joint_states = []

        try:
            time_now = rclpy.time.Time()
            for joint_name in self.joint_base_names:
                trans = self.tf_buffer.lookup_transform(self.base_frame_id, joint_name, time_now)
                pose = [
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z,
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w,
                ]
                joint_states.append(pose)

            if self.extrinsics_matrix is None:
                self.extrinsics_matrix = get_training_extrinsics()
                # get_training_extrinsics maps optical -> base_link (the calibration's
                # robot_base_frame). Compose world<-base_link so the pcd input is in
                # world, matching the proprio/action frame.
                try:
                    _tf = self.tf_buffer.lookup_transform("world", "base_link", rclpy.time.Time())
                    _t = _tf.transform.translation
                    _q = _tf.transform.rotation
                    _T = np.eye(4)
                    _T[:3, :3] = R.from_quat([_q.x, _q.y, _q.z, _q.w]).as_matrix()
                    _T[:3, 3] = [_t.x, _t.y, _t.z]
                    self.extrinsics_matrix = _T @ self.extrinsics_matrix
                    self.get_logger().info("[pcd-frame] pcd converted base_link -> world")
                except Exception as e:
                    self.get_logger().warn(f"[pcd-frame] world<-base_link TF not ready: {e}")
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        # Prepare inputs - ALIGNED WITH train.py preprocessing
        # 1. Convert BGR to RGB (same as dataset.py line 144)
        rgb_list = [self.latest_rgb[..., ::-1].copy()]
        depth_list = [self.latest_depth]
        camera_info_dict = {
            'rgb': {'K': list(self.latest_camera_info.k), 'width': self.latest_camera_info.width, 'height': self.latest_camera_info.height},
            'depth': {'K': list(self.latest_camera_info.k), 'width': self.latest_camera_info.width, 'height': self.latest_camera_info.height}
        }

        # 2. Crop and resize RGBD to 256x256 (same as preprocessing.py)
        cropped_rgb_list, cropped_depth_list, camera_info = crop_rgbd(rgb_list, depth_list, camera_info_dict, target_sz=256)

        # 3. Convert to tensor and normalize to [0, 1] (same as dataset.py line 145)
        rgb_np = cropped_rgb_list[0]  # (H, W, 3) RGB
        rgb_tensor = torch.from_numpy(rgb_np).permute(2, 0, 1).float() / 255.0  # (3, H, W), [0, 1]

        depth = cropped_depth_list[0]

        # 4. Get PCD with updated intrinsics
        depth_info = camera_info['rgb']
        latest_intrinsics = {
            'K': np.array(depth_info['K']).reshape(3, 3),
            'height': depth_info['height'],
            'width': depth_info['width']
        }
        pcd_np = get_pcd(depth, latest_intrinsics, extrinsics=self.extrinsics_matrix)

        # Filter background from pointcloud (green background + black cloth)
        pcd_np = filter_pcd_background(pcd_np, rgb_np)

        pcd_tensor = torch.from_numpy(pcd_np).float()

        # 5. Apply augmentation (same as dataset.py lines 219-227, with scale=1.0)
        # Reshape to (T, N, C, H, W) for Resize
        rgb_for_resize = rgb_tensor.unsqueeze(0).unsqueeze(0)  # (1, 1, 3, H, W)
        pcd_for_resize = pcd_tensor.unsqueeze(0).unsqueeze(0)  # (1, 1, 3, H, W)
        modals = self._resize(rgbs=rgb_for_resize, pcds=pcd_for_resize)
        rgb_tensor = modals["rgbs"].squeeze(0).squeeze(0)  # (3, H, W)
        pcd_tensor = modals["pcds"].squeeze(0).squeeze(0)  # (3, H, W)

        # 6. Normalize RGB to [-1, 1] (same as dataset.py line 216)
        rgb_tensor = rgb_tensor * 2.0 - 1.0

        # Publish pointcloud for visualization (use cropped RGB which is still [0,255] numpy)
        self.publish_pointcloud(pcd_np, cropped_rgb_list[0], frame_id="world")

        # 7. Batch dims (B=1, ncam=1)
        rgb_input = rgb_tensor.unsqueeze(0).unsqueeze(0).to(self.device).float()  # (1, 1, 3, H, W)
        pcd_input = pcd_tensor.unsqueeze(0).unsqueeze(0).to(self.device).float()  # (1, 1, 3, H, W)

        # Joints (B=1, N_joints, 7)
        joints_input = torch.tensor(joint_states).float().to(self.device).unsqueeze(0)

        # Current Gripper (EE Pose + Width)
        ee_pose = joint_states[-1]  # [x,y,z,qx,qy,qz,qw]
        gripper_width = self.current_gripper_width
        current_action = np.array(ee_pose + [gripper_width])  # (8,)

        # Get history (movement-based, like training preprocessing)
        # Takes last nhist entries from buffer where each entry represents significant movement
        curr_gripper_np = self._get_history(self.model_args.nhist)
        if curr_gripper_np is None:
            # Not enough history yet, pad with current action
            curr_gripper_np = np.tile(current_action, (self.model_args.nhist, 1))  # (nhist, 8)
        curr_gripper_tensor = torch.from_numpy(curr_gripper_np).float().to(self.device).unsqueeze(0)  # (1, nhist, 8)

        # Construct sample for DiffuserActor
        sample = {
            'rgbs': rgb_input,
            'pcds': pcd_input,
            'instr': self.instr_embed,
            'joints_coords': joints_input
        }
        # Trajectory mask - shape should match training: (B, interpolation_length)
        # In training, mask is True for valid points, but model mainly uses it for shape
        trajectory_mask = torch.ones(
            [1, self.model_args.interpolation_length], dtype=torch.bool
        ).to(self.device)

        # DEBUG: Log inputs
        self.get_logger().info(f"[DEBUG] Input shapes - RGB: {rgb_input.shape}, PCD: {pcd_input.shape}, Gripper: {curr_gripper_tensor.shape}")
        self.get_logger().info(f"[DEBUG] RGB range: [{rgb_input.min():.2f}, {rgb_input.max():.2f}]")
        self.get_logger().info(f"[DEBUG] PCD range: [{pcd_input.min():.2f}, {pcd_input.max():.2f}]")
        self.get_logger().info(f"[DEBUG] Current gripper history:\n{curr_gripper_np}")

        # Start frequency tracking timer right before first inference
        if self.freq_tracking_start_time is None:
            self.freq_tracking_start_time = time.time()
            self.get_logger().info("[FREQ] Starting frequency tracking timer")

        with torch.no_grad():
            # Track inference time
            inference_start = time.time()
            trajectory = self.model(
                gt_trajectory=None,
                trajectory_mask=trajectory_mask,
                rgb_obs=rgb_input,
                pcd_obs=pcd_input,
                instruction=self.instr_embed,
                curr_gripper=curr_gripper_tensor,
                run_inference=True,
                sample=sample
            )
            inference_time_ms = (time.time() - inference_start) * 1000
            self.inference_times.append(inference_time_ms)
            
            # Trajectory: (1, T, 8)
            traj_np = trajectory[0].cpu().numpy()

            # DEBUG: Log raw output
            self.get_logger().info(f"[DEBUG] Raw model output (first 3):\n{traj_np[:3]}")
            self.get_logger().info(f"[DEBUG] Raw model output (last 3):\n{traj_np[-3:]}")

            if self.model_args.relative:
                # Convert relative to absolute
                self.get_logger().info(f"[DEBUG] RELATIVE MODE: adding offset {curr_gripper_np[-1, :3]}")
                traj_np[:, :3] += curr_gripper_np[-1, :3]
                self.get_logger().info(f"[DEBUG] After relative conversion (first 3):\n{traj_np[:3]}")

            # Truncate trajectory if trajectory_steps is set
            if self.trajectory_steps > 0 and len(traj_np) > self.trajectory_steps:
                traj_np = traj_np[:self.trajectory_steps]
                self.get_logger().info(f"Truncated trajectory to first {self.trajectory_steps} steps")
            else:
                self.get_logger().info(f"Generated trajectory with {traj_np.shape[0]} steps")

            # Raise the whole trajectory uniformly in z (z should be higher).
            traj_np[:, 2] += Z_OFFSET
            # Shift the whole trajectory in world x.
            traj_np[:, 0] += X_OFFSET

            # Safety: clamp z to avoid table collision
            z_violations = traj_np[:, 2] < self.z_min
            if z_violations.any():
                num_clamped = z_violations.sum()
                traj_np[z_violations, 2] = self.z_min
                self.get_logger().warn(f"Clamped {num_clamped} waypoints to z_min={self.z_min}")

        # Update Plan (Receding Horizon)
        if len(traj_np) > 0:
            self.latest_plan = traj_np
            self.plan_index = 0
            self.inference_count += 1

            # Every N inferences, return to the recorded home pose before continuing.
            if HOME_EVERY_N_INFERENCES > 0 and self.inference_count % HOME_EVERY_N_INFERENCES == 0:
                self.returning_home = True
                self.home_start_time = time.time()
                self.latest_plan = None   # stop executing the current plan
                self.plan_index = 0
                self.get_logger().info(
                    f"[home] returning to initial pose for {HOME_WAIT_SEC}s "
                    f"(after {self.inference_count} inferences)")

    def _publish_home_pose(self):
        """Publish the recorded home EE pose (world frame) as the target."""
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.base_frame_id
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = float(HOME_POSE[0])
        pose_msg.pose.position.y = float(HOME_POSE[1])
        pose_msg.pose.position.z = float(HOME_POSE[2])
        pose_msg.pose.orientation.x = float(HOME_POSE[3])
        pose_msg.pose.orientation.y = float(HOME_POSE[4])
        pose_msg.pose.orientation.z = float(HOME_POSE[5])
        pose_msg.pose.orientation.w = float(HOME_POSE[6])
        self.target_pose_pub.publish(pose_msg)

        # Keep the gripper OPEN (1.0) at the home pose.
        gripper_msg = Float32()
        gripper_msg.data = 1.0
        self.gripper_command_pub.publish(gripper_msg)
        self.last_gripper_command = 1.0

    def control_loop(self):
        # Return-to-home: command the recorded home pose and wait for the robot to reach it.
        if self.returning_home:
            if self.home_start_time is not None and \
                    (time.time() - self.home_start_time) >= HOME_WAIT_SEC:
                self.returning_home = False
                self.get_logger().info("[home] reached initial pose, resuming inference")
            else:
                self._publish_home_pose()
                return

        # Check if waiting for gripper close action to complete
        if self.waiting_for_gripper:
            elapsed = time.time() - self.gripper_wait_start_time
            if elapsed >= self.gripper_close_wait_duration:
                self.get_logger().info(f"Gripper close wait complete after {elapsed:.2f}s")
                self.waiting_for_gripper = False
            else:
                # Still waiting, don't advance trajectory
                return

        if self.latest_plan is None:
            return

        if self.plan_index >= len(self.latest_plan):
            self.get_logger().info("Completed current plan.")
            self.latest_plan = None
            self.plan_index = 0
            return

        self.get_logger().info(f"Executing plan step {self.plan_index+1}/{len(self.latest_plan)}")
        # Get next action
        next_action = self.latest_plan[self.plan_index]  # [x, y, z, qx, qy, qz, qw, open]
        self.plan_index += 1

        # Compute gripper command
        raw_gripper = float(next_action[7])
        gripper_command = 1.0 if raw_gripper > 0.5 else 0.0

        # Check for open → close transition (grasping) - need to wait
        if self.last_gripper_command == 1.0 and gripper_command == 0.0:
            self.waiting_for_gripper = True
            self.gripper_wait_start_time = time.time()
            self.get_logger().info(
                f"Gripper closing (grasping), waiting {self.gripper_close_wait_duration}s..."
            )

        self.last_gripper_command = gripper_command

        # Publish Target Pose
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.base_frame_id
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = float(next_action[0])
        pose_msg.pose.position.y = float(next_action[1])
        pose_msg.pose.position.z = float(next_action[2])
        pose_msg.pose.orientation.x = float(next_action[3])
        pose_msg.pose.orientation.y = float(next_action[4])
        pose_msg.pose.orientation.z = float(next_action[5])
        pose_msg.pose.orientation.w = float(next_action[6])

        self.target_pose_pub.publish(pose_msg)

        # Publish Gripper
        gripper_msg = Float32()
        gripper_msg.data = gripper_command

        self.gripper_command_pub.publish(gripper_msg)

        # Track action output frequency
        self.action_count += 1

    def log_frequencies(self):
        """Log inference and action output frequencies periodically."""
        if self.freq_tracking_start_time is None:
            return

        elapsed = time.time() - self.freq_tracking_start_time
        if elapsed < 1.0:
            return

        inference_freq = self.inference_count / elapsed
        action_freq = self.action_count / elapsed

        log_msg = (
            f"[FREQ] Inference: {inference_freq:.2f} Hz ({self.inference_count} calls in {elapsed:.1f}s) | "
            f"Action output: {action_freq:.2f} Hz ({self.action_count} actions)"
        )
        self.get_logger().info(log_msg)
        
        # Write to metrics file if enabled
        if self.save_metrics and self.metrics_file:
            # Format elapsed time as [MM:SS]
            minutes = int(elapsed // 60)
            seconds = int(elapsed % 60)
            time_str = f"[{minutes:02d}:{seconds:02d}]"
            
            with open(self.metrics_file, 'a') as f:
                f.write(
                f"{time_str} Inference: {inference_freq:.2f} Hz ({self.inference_count} calls) | "
                f"Actions: {action_freq:.2f} Hz ({self.action_count} actions)\n"
            )
    
    def write_final_summary(self):
        """Write final metrics summary to file on shutdown."""
        if not self.save_metrics or not self.metrics_file:
            return
        
        if self.freq_tracking_start_time is None:
            return
        
        elapsed = time.time() - self.freq_tracking_start_time
        
        # Calculate averages
        avg_inference_freq = self.inference_count / elapsed if elapsed > 0 else 0
        avg_action_freq = self.action_count / elapsed if elapsed > 0 else 0
        avg_inference_time_ms = np.mean(self.inference_times) if self.inference_times else 0
        std_inference_time_ms = np.std(self.inference_times) if self.inference_times else 0
        
        # Write final summary
        with open(self.metrics_file, 'a') as f:
            f.write("\n")
            f.write("=" * 50 + "\n")
            f.write("=== FINAL SUMMARY ===\n")
            f.write("=" * 50 + "\n")
            f.write(f"Total Runtime: {elapsed:.1f} seconds\n")
            f.write(f"Total Inferences: {self.inference_count}\n")
            f.write(f"Total Actions: {self.action_count}\n")
            f.write(f"Average Inference Frequency: {avg_inference_freq:.2f} Hz\n")
            f.write(f"Average Action Frequency: {avg_action_freq:.2f} Hz\n")
            f.write(f"Average Inference Time: {avg_inference_time_ms:.0f} ± {std_inference_time_ms:.0f} ms\n")
            f.write("=" * 50 + "\n")
        
        self.get_logger().info(f"Final metrics summary written to: {self.metrics_file}")


def main(args=None):
    rclpy.init(args=args)
    node = DiffuserInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Write final summary before shutdown
        node.write_final_summary()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
