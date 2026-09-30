#!/usr/bin/env bash
# Robot-300 MZJ environment. No hardware publishing unless explicitly requested.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export COLLECTOR_MODE=dagger
export DAGGER_PUBLISH="${DAGGER_PUBLISH:-0}"
export DAGGER_SERVER_URL="${DAGGER_SERVER_URL:-http://192.168.8.224:8777}"
export DAGGER_DEPENDENCY_ROOT="${DAGGER_DEPENDENCY_ROOT:-/home/ubuntu/ros2_ws/src}"
export DAGGER_TOOLS_ROOT="${DAGGER_TOOLS_ROOT:-/home/ubuntu/collector_validation/20260929_jpeg_only/tools}"
export OUTPUT_BASE_DIR="${OUTPUT_BASE_DIR:-/home/ubuntu/nas/dagger}"
export TASK_TEXT="${TASK_TEXT:-Pick the laundry bag.}"
export WITH_HEAD=1 WITH_UPPER_WAIST=1 WITH_WAIST=0
export WITH_DEPTH="${WITH_DEPTH:-1}"
export DAGGER_WEB_PORT="${DAGGER_WEB_PORT:-8447}"
exec bash "$ROOT/start_lerobot_official_collect.sh" "$@"
