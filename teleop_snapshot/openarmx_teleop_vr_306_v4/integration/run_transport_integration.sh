#!/usr/bin/env bash
set +u

workspace=/home/ubuntu/ros2_ws
log_file=/tmp/openarmx_306_v4_transport_integration.log
status_file=/tmp/openarmx_306_v4_transport_status.json
launch_pid=""
launch_pgid=""

group_non_zombie_count() {
  ps -eo pgid=,stat= | awk -v group_id="$launch_pgid" \
    '$1 == group_id && $2 !~ /^Z/ { count += 1 } END { print count + 0 }'
}

stop_dry_run() {
  set +e
  trap - EXIT INT TERM
  if [[ -z "$launch_pid" || -z "$launch_pgid" ]]; then
    return
  fi
  kill -INT -- "-$launch_pgid" 2>/dev/null
  for _ in $(seq 1 30); do
    if [[ "$(group_non_zombie_count)" == "0" ]]; then
      break
    fi
    sleep 0.2
  done
  if [[ "$(group_non_zombie_count)" != "0" ]]; then
    kill -TERM -- "-$launch_pgid" 2>/dev/null
    sleep 1
  fi
  if [[ "$(group_non_zombie_count)" != "0" ]]; then
    kill -KILL -- "-$launch_pgid" 2>/dev/null
  fi
  wait "$launch_pid" 2>/dev/null
  launch_pid=""
  launch_pgid=""
}

trap stop_dry_run EXIT INT TERM

cd "$workspace" || exit 90
source /opt/ros/jazzy/setup.bash || exit 91
source install/setup.bash || exit 92
: > "$log_file"

setsid ros2 launch openarmx_teleop_vr_306_v4 full_vr_teleop.launch.py \
  dry_run:=true > "$log_file" 2>&1 &
launch_pid=$!
launch_pgid="$(ps -o pgid= -p "$launch_pid" | tr -d ' ')"
if [[ -z "$launch_pgid" || "$launch_pgid" == "1" ]]; then
  echo "INVALID_LAUNCH_PROCESS_GROUP: $launch_pgid"
  exit 94
fi

ready=0
for _ in $(seq 1 100); do
  if curl -ksS --max-time 1 \
      https://127.0.0.1:8446/api/status > "$status_file" 2>/dev/null; then
    ready=1
    break
  fi
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    break
  fi
  sleep 0.2
done

if [[ "$ready" != "1" ]]; then
  echo BRIDGE_NOT_READY
  tail -120 "$log_file"
  exit 93
fi

echo BRIDGE_READY
set +e
/home/ubuntu/ros2_ws/venvs/openarmx_v4_placo/bin/python \
  "$workspace/src/openarmx_teleop_vr_306_v4/integration/transport_integration_client.py" \
  --base-url https://127.0.0.1:8446
test_rc=$?
set -e
echo "INTEGRATION_RC=$test_rc"

stop_dry_run
trap - EXIT INT TERM

echo REMAINING_OPENARMX
pgrep -af \
  'openarmx_teleop_vr_306_v4|independent_arm_controller|independent_vr_mapper_306_v4|vr_web_bridge' \
  || true
echo "VENDOR_SERVICE=$(systemctl --user is-active arm-control-service.service)"
echo LOG_TAIL
tail -100 "$log_file"
exit "$test_rc"
