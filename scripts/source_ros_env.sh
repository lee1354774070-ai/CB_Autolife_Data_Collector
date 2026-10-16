#!/usr/bin/env bash
# Source this file in every shell used to inspect/control AutoLife collection nodes.

hg_dagger_restore_nounset=0
case "$-" in
  *u*) hg_dagger_restore_nounset=1 ;;
esac

set +u
source /opt/ros/jazzy/setup.bash || return $?
# S3 SDK installations publish native topics without a separate ROS workspace.
if [ -f /home/ubuntu/ros2_ws/install/setup.bash ]; then
  source /home/ubuntu/ros2_ws/install/setup.bash || return $?
fi

if [ "${hg_dagger_restore_nounset}" -eq 1 ]; then
  set -u
else
  set +u
fi
unset hg_dagger_restore_nounset

export ROS_DOMAIN_ID=0
hg_dagger_host_robot_id="$(hostname | sed -n 's/.*-\([0-9][0-9]*\)$/\1/p')"
export ROBOT_ID="${HG_DAGGER_ROBOT_ID:-${ROBOT_ID:-${hg_dagger_host_robot_id:-300}}}"
unset hg_dagger_host_robot_id
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'

echo "AutoLife ROS environment loaded (robot ${ROBOT_ID}, CycloneDDS loopback)."
