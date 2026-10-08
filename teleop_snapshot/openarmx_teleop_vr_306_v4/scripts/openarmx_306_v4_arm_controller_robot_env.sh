#!/usr/bin/env bash
set -e

V4_PYTHON="/home/ubuntu/ros2_ws/venvs/openarmx_v4_placo/bin/python"
if [ ! -x "$V4_PYTHON" ]; then
  echo "V4 Placo Python environment not found: $V4_PYTHON" >&2
  echo "Run scripts/setup_v4_placo_env.sh once, then rebuild this package." >&2
  exit 2
fi

exec "$V4_PYTHON" -m openarmx_teleop_vr_306_v4.controller_node "$@"
