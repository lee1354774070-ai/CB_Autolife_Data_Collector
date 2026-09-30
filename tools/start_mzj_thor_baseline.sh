#!/usr/bin/env bash
# Run on Thor. Requires audited sources, existing model/token, and Docker access.
set -euo pipefail
export GROOT_RUNTIME_ROOT="${GROOT_RUNTIME_ROOT:-/home/wayne/groot_n1_7_runtime}"
export GROOT_CONTAINER_NAME="${GROOT_CONTAINER_NAME:-groot-n1.7}"
export GROOT_SERVER_PORT="${GROOT_SERVER_PORT:-8777}"
phase=()
case "${MZJ_GRIPPER_PHASE_AWARE:-1}" in
  1) phase=(--gripper-phase-aware) ;;
  0) ;;
  *) echo 'MZJ_GRIPPER_PHASE_AWARE must be 0 or 1' >&2; exit 2 ;;
esac
exec bash "$GROOT_RUNTIME_ROOT/repo/deploy/groot_n1_7/thor/manage.sh" start baseline "${phase[@]}" "$@"
