# openarmx_teleop_vr_306_v4

V4 keeps the proven robot-306 transport and hardware boundary from V3, while
replacing the arm-control middle layer with the design used by the local
Quest3-Teleoperation project.

```text
unchanged V3 boundary
WebXR keys + HTTPS/WebRTC/WSS + newest-frame mailbox
                         |
V4 Quest middle          v
adjacent-frame Cartesian increments
  -> persistent end-effector targets
  -> measured-joint refresh on every new target
  -> Placo one-step QP IK
  -> manipulability task + kinetic-energy regularization
                         |
unchanged V3 boundary    v
hard limits/watchdogs/arbitration -> vendor SYNC position topic
  -> vendor CAN worker -> DM motor internal position loop
```

Only active arm joints are unmasked during each Placo solve. The inactive arm,
neck, ankle and knee remain at measured feedback; waist pitch/yaw are opened
only when the inherited UI explicitly enables the IK waist-follow profile.
Old IK results cannot overwrite
a newer WebXR frame, Grip release invalidates the old anchor, and stale input
holds output through V3's fail-closed watchdogs. Quest3 itself does not provide
complete stale-frame, dropout or collision handling; V4 therefore retains the
stronger V3 outer guards instead of removing them. SRDF self-collision remains
an optional controller parameter and is disabled by the inherited supervised
teleoperation profile.

## Dependency setup

Placo 0.9.17 is installed in an isolated environment so V3's robot environment
is not modified:

```bash
cd /home/ubuntu/ros2_ws/src/openarmx_teleop_vr_306_v4
bash scripts/setup_v4_placo_env.sh
```

## Dry-run validation

```bash
cd /home/ubuntu/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch openarmx_teleop_vr_306_v4 full_vr_teleop.launch.py dry_run:=true
```

Open `https://<robot-ip>:8446`. Dry-run runs mapping and IK but does not enable
or command the real robot.

## Real hardware

Real-hardware operation is intentionally unchanged from V3:

```bash
ros2 launch openarmx_teleop_vr_306_v4 full_vr_teleop.launch.py dry_run:=false
```

The page button logic, X+A reset, head/waist options, gripper behavior, vendor
SYNC session, position command entry and shutdown behavior are inherited from
V3. Do not run V3, V4 or the navigation teleop package together in real mode.
