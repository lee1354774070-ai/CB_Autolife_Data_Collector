"""Recorded failed trials through local receipts and native V5 servo, no hardware.

Servo motion is deliberately simulated at 35 deg/s; this is not real grasp validation.
"""
import ast
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
    from dagger.control.groot_bridge_core import array_digest_float32
    from test_thor_bridge import ThorBridgeTest
    runtime = prepare_copy(root/'teleop_snapshot', root/'.runtime')
    # Same direct servo AST as the user-approved removal of smoothing.
    def tick_ast(path):
        return next(n for n in ast.walk(ast.parse(path.read_text()))
                    if isinstance(n, ast.FunctionDef) and n.name == '_control_tick')
    before = Path('/home/ubuntu/collector_validation/dagger_native_execution_20261110/collector/.runtime/v5-8a13b729082001635f7e/openarmx_teleop_vr_306_v4/controller_node.py')
    current = runtime/'openarmx_teleop_vr_306_v4/controller_node.py'
    assert ast.dump(tick_ast(before)) == ast.dump(tick_ast(current))
    assert '_bounded_local_policy_command' not in ast.unparse(tick_ast(current))
    sys.path.insert(0, str(runtime))
    import rclpy
    from std_msgs.msg import String
    from openarmx_teleop_vr_306_v4.controller_node import IndependentArmController
    from openarmx_teleop_vr_306_v4.schema import JointFeedback
    robot = resolve_robot(os.environ)
    params = {**robot.controller_params(runtime, os.environ), **controller_parameters(runtime),
              'dry_run': True, 'topic_suffix': '211_340', 'output_control_mode': 'position',
              'follow_authority_topic': '/hg_dagger/control_state',
              'authority_heartbeat_topic': '/hg_dagger/authority_heartbeat'}
    args = ['--ros-args', '-r', '__node:=independent_arm_controller_306_v4', '--params-file', str(runtime/'config/controller.yaml')]
    for k,v in params.items(): args += ['-p', f'{k}:={json.dumps(v)}']
    rclpy.init(args=args)
    node = IndependentArmController()
    reports = []
    try:
        assert node._dry_run and node._joint_pub is None and node._gripper_pub is None
        node._hardware_output_guard_error = Mock(return_value='')
        node._publish_owned_command = Mock()
        node._set_hardware_fault = Mock()
        node._bounded_local_policy_command = Mock(side_effect=AssertionError('smoothing restored'))
        for epoch,name in enumerate(sys.argv[1:],1):
            rows = [json.loads(l)['payload'] for l in Path(name).read_text().splitlines()]
            fixture = ThorBridgeTest(); fixture.setUp()
            bridge = fixture.node
            bridge._v5_policy_pacing = bridge._local_execution = True
            bridge._local_pending = None
            bridge._local_answers = {}
            bridge._generation_valid = Mock(return_value=True)
            # Use the real receipt waiter, not the baseline test fixture's ACK stub.
            del bridge._wait_forward_ack
            bridge._collector_session_id = 'replay'
            bridge._authority_epoch = epoch
            bridge._proposal_authority = (1, 'replay', epoch)
            actual = np.asarray(rows[0]['measured_policy_state']).copy()
            goal = actual.copy()
            lower = rows[0]['measured_leg_waist'][:2]
            indices = [*range(14),16,17,18]
            counts = dict(wait_ticks=0, paced_steps=0, max_submission_error_deg=0., controller_ticks=0)
            def groups(a):
                return dict(leg_waist=np.asarray([*lower,*a[19:21]]),left_arm=a[:7].copy(),
                    right_arm=a[7:14].copy(),neck=a[16:19].copy(),left_gripper=a[14:15].copy(),right_gripper=a[15:16].copy())
            bridge._fresh_q23 = lambda: [*lower,*actual[19:21],*actual[:14],*actual[14:16],*actual[16:19]]
            def tick():
                now=time.monotonic()
                node._feedback=JointFeedback(**groups(actual))
                node._feedback_time=node._follow_authority_time=node._authority_heartbeat_time=now
                node._last_tick=now-.01
                node._control_tick()
                counts['controller_ticks']+=1
                node._set_hardware_fault.assert_not_called()
            def follow(*_):
                counts['wait_ticks']+=1
                assert counts['wait_ticks']<20000,'no reachable tracking progress'
                actual[:] += np.clip(goal-actual,-.35,.35)
                tick()
            bridge._condition.wait=follow
            node._feedback=JointFeedback(**groups(actual));node._feedback_time=time.monotonic()
            node._begin_fresh_teleop_session_locked(groups(actual))
            node._on_follow_authority(String(data=json.dumps(dict(mode='POLICY_ACTIVE',authority_epoch=epoch,session_id='replay'))))
            node._hardware_enabled=True
            receipts=[]
            def receipt(p):
                receipts.append(p)
                bridge._on_forward_ack(String(data=json.dumps(p)))
            node._publish_local_policy_receipt=receipt
            actions=[r['action'] for r in rows]
            proposal=dict(proposal_id=f'replay:{epoch}',context_digest='test',
                          executable_chunk_digest=array_digest_float32(actions))
            for step,action in enumerate(actions):
                bridge._local_capability=dict(version=1,timestamp_ns=time.time_ns())
                old_waits=counts['wait_ticks']
                payload=bridge._policy_payload(action,proposal,step,1,0,rows[step]['measured_policy_state'],shadow_only=False)
                counts['paced_steps']+=counts['wait_ticks']>old_waits
                assert payload['action']==action
                err=max(abs(action[i]-actual[i]) for i in indices)
                assert err<=18.
                counts['max_submission_error_deg']=max(counts['max_submission_error_deg'],err)
                node._feedback=JointFeedback(**groups(actual));node._feedback_time=time.monotonic()
                node._on_local_policy_submission(String(data=json.dumps(payload)))
                assert bridge._wait_forward_ack(proposal['proposal_id'],step,1)
                assert receipts[-1]['requested_action']==action
                goal[:]=receipts[-1]['bounded_target']
                tick()
                np.testing.assert_allclose(node._last_position_command,node._group_vector(groups(goal)),atol=1e-6,rtol=0)
                for _ in range(3):
                    actual[:] += np.clip(goal-actual,-.35,.35)
                    tick()
            assert array_digest_float32([r['requested_action'] for r in receipts])==proposal['executable_chunk_digest']
            reports.append(dict(trial=Path(name).parent.name,frames=len(rows),**counts))
            print(json.dumps(reports[-1]),flush=True)
        node._publish_owned_command.assert_not_called()
        node._bounded_local_policy_command.assert_not_called()
        report=dict(status='passed',hardware_publishers=0,hardware_calls=0,
            simulated_servo_deg_sec=35,servo_ast_unchanged=True,smoothing_calls=0,
            original_action_digests_preserved=True,trials=reports)
        (root.parent/'tracking_replay_results.json').write_text(json.dumps(report,indent=2))
        print('V5_LOCAL_TRACKING_REPLAY_PASS hardware_io=0')
    finally:
        node._hardware_enabled=False;node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
