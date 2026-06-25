#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 user@server [remote_port] [local_port]" >&2
  exit 2
fi

REMOTE="$1"
REMOTE_PORT="${2:-8000}"
LOCAL_PORT="${3:-8000}"
REMOTE_HOST="${REMOTE_HOST:-127.0.0.1}"

echo "Forwarding laptop 127.0.0.1:${LOCAL_PORT} -> ${REMOTE}:${REMOTE_HOST}:${REMOTE_PORT}"
echo "Keep this process running while the demo is active."
exec ssh -N -L "${LOCAL_PORT}:${REMOTE_HOST}:${REMOTE_PORT}" "${REMOTE}"
