#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

python3 scripts/check_ur5e_ws.py "$@"

python3 - <<'PYTHON_CHECK'
import socket
host = '127.0.0.1'
port = 8000
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(2.0)
try:
    sock.connect((host, port))
except OSError as exc:
    raise SystemExit(f'Policy endpoint check failed: {host}:{port} is not reachable ({exc})')
finally:
    sock.close()
print(f'Policy endpoint reachable: {host}:{port}')
PYTHON_CHECK
