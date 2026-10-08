#!/usr/bin/env bash
# Source this file in every shell used to inspect/control AutoLife collection nodes.

hg_dagger_restore_nounset=0
case "$-" in
  *u*) hg_dagger_restore_nounset=1 ;;
esac

set +u
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash

if [ "${hg_dagger_restore_nounset}" -eq 1 ]; then
  set -u
else
  set +u
fi
unset hg_dagger_restore_nounset

export ROS_DOMAIN_ID=0
export ROBOT_ID="${HG_DAGGER_ROBOT_ID:-${ROBOT_ID:-300}}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'

echo "AutoLife ROS environment loaded (robot ${ROBOT_ID}, CycloneDDS loopback)."
