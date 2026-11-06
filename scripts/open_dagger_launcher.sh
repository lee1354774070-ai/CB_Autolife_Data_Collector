#!/usr/bin/env bash
# Display the existing desktop UI; the operator starts each session explicitly.
set -e
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# User-owned per-robot defaults; opening the UI itself never starts hardware.
config="${DAGGER_LAUNCHER_CONFIG:-$HOME/.config/autolife_collector/dagger.env}"
if [ -f "$config" ]; then
  set -a
  source "$config"
  set +a
elif [ -n "${DAGGER_LAUNCHER_CONFIG:-}" ]; then
  echo "Launcher configuration missing: $config" >&2
  exit 1
fi
source "$ROOT/scripts/source_ros_env.sh"
set +u
export DAGGER_PUBLISH="${DAGGER_PUBLISH:-0}"
export DAGGER_SERVER_URL="${DAGGER_SERVER_URL:-http://192.168.8.224:8777}"
export DAGGER_TOOLS_ROOT="${DAGGER_TOOLS_ROOT:-/home/ubuntu/Autolife_VLA_Tools}"
export OUTPUT_BASE_DIR="${OUTPUT_BASE_DIR:-/home/ubuntu/nas/dagger}"
export ACTION_MODE=joint
exec /usr/bin/python3 "$ROOT/gui/dagger_launcher.py"
