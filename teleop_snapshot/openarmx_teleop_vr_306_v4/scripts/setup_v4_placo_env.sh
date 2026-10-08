#!/usr/bin/env bash
set -euo pipefail

BASE_PYTHON="/home/ubuntu/ros2_ws/venvs/openarmx_v4_placo/bin/python"
VENV_DIR="/home/ubuntu/ros2_ws/venvs/openarmx_v4_placo"

if [ ! -x "$BASE_PYTHON" ]; then
  echo "robot_env Python not found: $BASE_PYTHON" >&2
  exit 2
fi

if [ ! -x "$VENV_DIR/bin/python" ]; then
  "$BASE_PYTHON" -m venv --system-site-packages "$VENV_DIR"
fi

"$VENV_DIR/bin/python" -m pip install --disable-pip-version-check \
  'placo==0.9.17'

echo "V4 Placo environment ready: $VENV_DIR"
