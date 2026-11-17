"""Real V5 controller, recorded model actions, simulated joints; ROS 211 only."""
import json
import os
from pathlib import Path
import sys
import time
from unittest.mock import Mock

import numpy as np


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '211'
    assert os.environ.get('DAGGER_PUBLISH') == '0'
    assert os.environ.get('DAGGER_LOCAL_POLICY_EXECUTION') == '1'
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root))
    from dagger.runtime_copy import prepare_copy
    from dagger.v5_runtime import controller_parameters
    from dagger.robot_profile import resolve_robot
    from dagger.local_policy_execution import binding, matches
    runtime = prepare_copy(root / 'teleop_snapshot', root / '.runtime')
    sys.path.insert(0, str(runtime))
    import rclpy
    from std_msgs.msg import String
    from openarmx_teleop_vr_306_v4.controller_node import IndependentArmController
    from openarmx_teleop_vr_306_v4.schema import JointFeedback
    robot = resolve_robot(os.environ)
    params = {**robot.controller_params(runtime, os.environ), **controller_parameters(runtime),
        'dry_run': True, 'topic_suffix': '211_340', 'allow_waist_in_ik': True,
        'output_control_mode': 'position',
        'follow_authority_topic': '/hg_dagger/control_state',
        'authority_heartbeat_topic': '/hg_dagger/authority_heartbeat'}
    args = ['--ros-args', '-r', '__node:=independent_arm_controller_306_v4',
            '--params-file', str(runtime / 'config/controller.yaml')]
    for key, value in params.items():
        args += ['-p', f'{key}:={json.dumps(value)}']
    rclpy.init(args=args)
    node = IndependentArmController()
    reports = []
    try:
        assert node._dry_run and node._joint_pub is None and node._gripper_pub is None
        assert node._local_policy_enabled
        assert node._output_control_mode == 'position'
        node._hardware_output_guard_error = Mock(return_value='')
        node._publish_owned_command = Mock()
        node._set_hardware_fault = Mock()
        receipts = []
        node._publish_local_policy_receipt = lambda p: receipts.append(json.loads(json.dumps(p)))
        for epoch, name in enumerate(sys.argv[1:], 100):
            rows = [json.loads(x)['payload'] for x in Path(name).read_text().splitlines() if x.strip()]
            actual = np.asarray(rows[0]['measured_policy_state'], dtype=float).copy()
            legs = rows[0]['measured_leg_waist'][:2]
            def groups():
                return dict(leg_waist=np.asarray([*legs, *actual[19:21]]),
                    left_arm=actual[:7].copy(), right_arm=actual[7:14].copy(),
                    neck=actual[16:19].copy(), left_gripper=actual[14:15].copy(),
                    right_gripper=actual[15:16].copy())
            node._feedback = JointFeedback(**groups())
            node._feedback_time = time.monotonic()
            node._begin_fresh_teleop_session_locked(groups())
            authority = dict(mode='POLICY_ACTIVE', authority_epoch=epoch, session_id='local-replay')
            node._on_follow_authority(String(data=json.dumps(authority)))
            node._hardware_enabled = True
            projections = 0
            max_error = 0.
            def tick():
                nonlocal max_error
                now = time.monotonic()
                node._feedback = JointFeedback(**groups())
                node._feedback_time = node._follow_authority_time = node._authority_heartbeat_time = now
                node._last_tick = now - .01
                node._control_tick()
                node._set_hardware_fault.assert_not_called()
                command = node._last_position_command
                policy = np.r_[command[4:18], actual[14:16], command[18:21], command[2:4]]
                max_error = max(max_error, float(np.max(np.abs(policy[:14] - actual[:14]))))
                assert max_error <= 12.1
                actual[:] += np.clip(policy - actual, -.35, .35)
            for row in rows:
                packet = dict(row, collector_session_id='local-replay', authority_epoch=epoch,
                              timestamp_ns=time.time_ns())
                original = list(packet['action'])
                node._feedback_time = time.monotonic()
                node._on_local_policy_submission(String(data=json.dumps(packet)))
                result = receipts[-1]
                assert matches(result, binding(packet)), result
                assert result['accepted'], result
                assert result['requested_action'] == original
                assert packet['action'] == original
                projections += bool(result['limited_joints'])
                for _ in range(3):
                    tick()
            # Duplicate/query must not refresh the target or mutate motion.
            stamp = node._target_time
            node._on_local_policy_submission(String(data=json.dumps(dict(packet, query_only=True))))
            assert receipts[-1]['accepted'] and node._target_time == stamp
            node._on_local_policy_submission(String(data=json.dumps(packet)))
            assert receipts[-1]['accepted'] and node._target_time == stamp
            before = {k: v.copy() for k, v in node._target_groups.items()}
            grips = dict(node._gripper_targets)
            invalid_units = dict(packet, chunk_step=packet['chunk_step'] + 1000,
                                 units='radians', timestamp_ns=time.time_ns())
            node._on_local_policy_submission(String(data=json.dumps(invalid_units)))
            assert receipts[-1]['accepted'] is False and 'degrees' in receipts[-1]['reason']
            bad_action = list(packet['action'])
            bad_action[8] = 90.
            node._on_local_policy_submission(String(data=json.dumps(dict(packet,
                chunk_step=packet['chunk_step'] + 1001, action=bad_action, timestamp_ns=time.time_ns()))))
            assert receipts[-1]['accepted'] is False and 'Shoulder_Outer' in receipts[-1]['reason']
            node._on_local_policy_submission(String(data=json.dumps(dict(packet,
                chunk_step=packet['chunk_step'] + 1002, timestamp_ns=time.time_ns() - 1_000_000_000))))
            assert receipts[-1]['accepted'] is False and 'stale' in receipts[-1]['reason']
            for k in before:
                np.testing.assert_array_equal(node._target_groups[k], before[k])
            assert node._gripper_targets == grips
            # A revoked authority cannot apply any late policy, including cached steps.
            node._on_follow_authority(String(data=json.dumps(dict(authority,
                mode='EXPERT_READY', authority_epoch=epoch + 1))))
            old = {k: v.copy() for k, v in node._target_groups.items()}
            node._on_local_policy_submission(String(data=json.dumps(packet)))
            assert receipts[-1]['accepted'] is False
            for k in old:
                np.testing.assert_array_equal(node._target_groups[k], old[k])
            node._on_local_policy_submission(String(data=json.dumps(dict(packet, query_only=True))))
            assert receipts[-1]['accepted'] is True  # Recover the prior accepted fact, without motion.
            reports.append(dict(trace=name, model_steps=len(rows), constrained_targets=projections,
                                max_command_tracking_deg=max_error, hardware_publishers=0))
        node._publish_owned_command.assert_not_called()
        print('LOCAL_POLICY_REPLAY_PASS=' + json.dumps(reports), flush=True)
        (root / 'local_policy_replay_results.json').write_text(json.dumps(reports, indent=2))
    finally:
        node._hardware_enabled = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
