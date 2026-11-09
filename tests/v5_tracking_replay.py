"""Recorded actions + a deliberately slow simulated servo; no hardware I/O.

This checks pacing and the real controller tick, not robot task performance.
"""
import json
import os
from pathlib import Path
import sys
import threading
import time
from unittest.mock import Mock
import numpy as np


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '211' and os.environ.get('DAGGER_PUBLISH') == '0'
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root))
    from dagger.runtime_copy import prepare_copy
    from dagger.v5_runtime import controller_parameters
    from dagger.robot_profile import resolve_robot
    from test_thor_bridge import bridge_type
    runtime = prepare_copy(root / 'teleop_snapshot', root / '.runtime')
    sys.path.insert(0, str(runtime))
    import rclpy
    from std_msgs.msg import String
    from openarmx_teleop_vr_306_v4.controller_node import IndependentArmController
    from openarmx_teleop_vr_306_v4.schema import JointFeedback
    robot = resolve_robot(os.environ)
    params = {**robot.controller_params(runtime, os.environ), **controller_parameters(runtime),
              'dry_run': True, 'topic_suffix': '211_340',
              'follow_authority_topic': '/hg_dagger/control_state',
              'authority_heartbeat_topic': '/hg_dagger/authority_heartbeat'}
    args = ['--ros-args', '-r', '__node:=independent_arm_controller_306_v4',
            '--params-file', str(runtime / 'config/controller.yaml')]
    for key, value in params.items():
        args += ['-p', f'{key}:={json.dumps(value)}']
    rclpy.init(args=args)
    node = IndependentArmController()
    try:
        assert node._joint_pub is None and node._gripper_pub is None and node._dry_run
        node._hardware_output_guard_error = Mock(return_value='')
        node._publish_owned_command = Mock()
        node._set_hardware_fault = Mock()
        bridge = bridge_type()()
        bridge._condition = threading.Condition()
        bridge._generation_valid = Mock(return_value=True)
        bridge._set_phase = Mock()
        for epoch, name in enumerate(sys.argv[1:], 1):
            rows = [json.loads(line)['payload'] for line in Path(name).open()]
            actual = np.asarray(rows[0]['measured_policy_state']).copy()
            goal = actual.copy()
            lower = rows[0]['measured_leg_waist'][:2]
            counts = {'wait_ticks': 0, 'max_submitted_error_deg': 0.}
            def groups(action):
                return dict(leg_waist=np.asarray([*lower, *action[19:21]]),
                    left_arm=action[:7].copy(), right_arm=action[7:14].copy(),
                    neck=action[16:19].copy(), left_gripper=action[14:15].copy(),
                    right_gripper=action[15:16].copy())
            def feedback():
                return [*lower, *actual[19:21], *actual[:14], *actual[14:16], *actual[16:19]]
            bridge._fresh_q23 = feedback
            def tick():
                now = time.monotonic()
                node._feedback = JointFeedback(**groups(actual))
                node._feedback_time = node._follow_authority_time = node._authority_heartbeat_time = now
                node._last_tick = now - .01
                node._control_tick()
                node._set_hardware_fault.assert_not_called()
            def catch_up(*_):
                counts['wait_ticks'] += 1
                assert counts['wait_ticks'] < 10000, 'No simulated tracking progress'
                actual[:] += np.clip(goal - actual, -.35, .35)  # 35 degrees / simulated second.
                tick()
            bridge._condition.wait = catch_up
            node._feedback = JointFeedback(**groups(actual))
            node._feedback_time = time.monotonic()
            node._begin_fresh_teleop_session_locked(groups(actual))
            node._on_follow_authority(String(data=json.dumps(dict(
                mode='POLICY_ACTIVE', authority_epoch=epoch, session_id='tracking-replay'))))
            node._hardware_enabled = True
            held_steps = 0
            original_excess = 0
            for row in rows:
                action = row['action']
                indices = (*range(14), 16, 17, 18)
                original_excess += max(abs(action[i] - row['measured_policy_state'][i]) for i in indices) > 22
                before = counts['wait_ticks']
                bridge._wait_policy_tracking(action, 1)
                held_steps += counts['wait_ticks'] != before
                error = max(abs(action[i] - actual[i]) for i in indices)
                assert error <= 18.
                counts['max_submitted_error_deg'] = max(counts['max_submitted_error_deg'], error)
                goal = np.asarray(action).copy()
                desired = groups(goal)
                packet = {key + '_target_joints_position': desired[key].tolist()
                          for key in ('leg_waist', 'left_arm', 'right_arm', 'neck')}
                node._on_joint_target(String(data=json.dumps(dict(packet,
                    authority_epoch=epoch, collector_session_id='tracking-replay'))))
                assert node._target_source == 'joint_space', node._reason
                tick()
                np.testing.assert_allclose(node._last_position_command, node._group_vector(desired), atol=1e-6, rtol=0)
                # Three servo ticks elapse during the normal 30 Hz action period.
                for _ in range(3):
                    actual[:] += np.clip(goal - actual, -.35, .35)
                    tick()
            print(json.dumps(dict(trace=str(Path(name).parent.name), frames=len(rows),
                original_over_22=original_excess, paced_steps=held_steps, **counts,
                hardware_publishers=0, hardware_calls=0)))
        node._publish_owned_command.assert_not_called()
        print('V5_TRACKING_REPLAY_PASS simulated_servo=true exact_body_targets=true hardware_io=0')
    finally:
        node._hardware_enabled = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
