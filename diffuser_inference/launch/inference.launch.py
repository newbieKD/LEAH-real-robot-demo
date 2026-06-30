import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('diffuser_inference')
    config_file = os.path.join(pkg_dir, 'config', 'inference_config.yaml')

    # Get launch argument values
    config_file_val = LaunchConfiguration('config_file').perform(context)
    instruction_val = LaunchConfiguration('instruction').perform(context)
    checkpoint_val = LaunchConfiguration('checkpoint').perform(context)
    model_type_val = LaunchConfiguration('model_type').perform(context)

    # Build parameters list - start with config file
    parameters = [config_file_val]

    # Only add overrides if they're non-empty
    overrides = {}
    if instruction_val:
        overrides['instruction'] = instruction_val
    if checkpoint_val:
        overrides['checkpoint'] = checkpoint_val
    if model_type_val:
        overrides['model_type'] = model_type_val

    if overrides:
        parameters.append(overrides)

    node = Node(
        package='diffuser_inference',
        executable='diffuser_inference_node',
        name='diffuser_inference',
        output='screen',
        parameters=parameters
    )

    return [node]

def generate_launch_description():
    pkg_dir = get_package_share_directory('diffuser_inference')
    config_file = os.path.join(pkg_dir, 'config', 'inference_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to YAML configuration file'
        ),
        DeclareLaunchArgument(
            'instruction',
            default_value='',
            description='Task instruction (overrides config file if provided)'
        ),
        DeclareLaunchArgument(
            'checkpoint',
            default_value='',
            description='Model checkpoint path (overrides config file if provided)'
        ),
        DeclareLaunchArgument(
            'model_type',
            default_value='',
            description='Model type: ours or baseline (overrides config file if provided)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
