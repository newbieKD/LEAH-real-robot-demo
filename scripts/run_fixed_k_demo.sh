#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

MODE="--dry-run"
PROMPT=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)
      MODE="--dry-run"
      shift
      ;;
    --execute)
      MODE="--execute"
      shift
      ;;
    --prompt)
      PROMPT="${2:-}"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

if [[ -z "${PROMPT}" ]]; then
  echo "Usage: $0 [--dry-run|--execute] --prompt 'task instruction'" >&2
  exit 2
fi

python3 scripts/openpi_real_bridge.py   --config configs/ur5e_demo.yaml   --prompt "${PROMPT}"   "${MODE}"
