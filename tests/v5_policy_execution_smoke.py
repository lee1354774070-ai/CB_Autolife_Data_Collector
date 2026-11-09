"""Real control tick, with no motor publishers and all hardware operations mocked."""
import ast
import __future__
from collections import deque
import json
import os
from pathlib import Path
import sys
import time
from unittest.mock import Mock, patch
from types import SimpleNamespace
import numpy as np


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '211' and os.environ.get('DAGGER_PUBLISH') == '0'
    root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root))
    from dagger.runtime_copy import prepare_copy
    from dagger.v5_runtime import controller_parameters
    from dagger.robot_profile import resolve_robot
    runtime=prepare_copy(root/'teleop_snapshot',root/'.runtime');sys.path.insert(0,str(runtime))
    import rclpy
    from std_msgs.msg import String
    from openarmx_teleop_vr_306_v4.controller_node import IndependentArmController
    from openarmx_teleop_vr_306_v4.schema import JointFeedback
    robot=resolve_robot(os.environ)
    params={**robot.controller_params(runtime,os.environ),**controller_parameters(runtime),
            'dry_run':True,'topic_suffix':'211_340','follow_authority_topic':'/hg_dagger/control_state',
            'authority_heartbeat_topic':'/hg_dagger/authority_heartbeat'}
    args=['--ros-args','-r','__node:=independent_arm_controller_306_v4','--params-file',str(runtime/'config/controller.yaml')]
    for k,v in params.items():args+=['-p',f'{k}:={json.dumps(v)}']
    rclpy.init(args=args);node=IndependentArmController()
    try:
        assert node._joint_pub is None and node._gripper_pub is None and node._dry_run
        assert node._output_control_mode == 'position'
        node._hardware_output_guard_error=Mock(return_value='')
        node._publish_owned_command=Mock()
        node._set_hardware_fault=Mock()
        def authority(mode='POLICY_ACTIVE',epoch=1):
            node._on_follow_authority(String(data=json.dumps(dict(mode=mode,authority_epoch=epoch,session_id='native-test'))))
        groups=dict(leg_waist=np.zeros(4),left_arm=np.array([20.,0.,0.,-100.,0.,0.,0.]),
                    right_arm=np.array([20.,0.,0.,-100.,0.,0.,0.]),neck=np.array([0.,25.,0.]),
                    left_gripper=np.array([10.]),right_gripper=np.array([10.]))
        node._feedback=JointFeedback(**groups);node._feedback_time=time.monotonic()
        node._begin_fresh_teleop_session_locked(groups);authority()
        command={**groups,'right_arm':groups['right_arm']+np.array([0.,0.,0.,-18.,0.,0.,0.]),
                 'neck':np.array([0.,25.,15.])}
        def submit(values=command):
            packet={group+'_target_joints_position':values[group].tolist() for group in ('leg_waist','left_arm','right_arm','neck')}
            node._on_joint_target(String(data=json.dumps(dict(packet,authority_epoch=node._collector_authority_epoch,collector_session_id='native-test'))))
        submit();expected=node._group_vector(command)
        node._hardware_enabled=True
        def tick(age=0.):
            now=time.monotonic();node._last_tick=now-.01;node._feedback_time=now
            node._follow_authority_time=now;node._authority_heartbeat_time=now;node._target_time=now-age
            node._control_tick()
        tick();np.testing.assert_allclose(node._last_position_command,expected,atol=1e-6)
        assert node._collector_policy_position_active()
        # HTTP inference takes >300ms; the previous goal must remain unchanged.
        tick(.7);np.testing.assert_allclose(node._last_position_command,expected,atol=1e-6)
        node._set_hardware_fault.assert_not_called()
        # Actual refreshes must remain causal/fresh to the real recorder even
        # when inference/pacing holds a constant grip for two seconds.
        from time_sync import latest_at_or_before
        source=root/'record_lerobot_official.py'
        method=next(n for n in ast.walk(ast.parse(source.read_text()))
                    if isinstance(n,ast.FunctionDef) and n.name=='_held_signal')
        ns={'latest_at_or_before':latest_at_or_before}
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec',
                     flags=__future__.annotations.compiler_flag),ns)
        recorder=SimpleNamespace(args=SimpleNamespace(max_action_hold_sec=.5,max_sync_delta_sec=.03))
        grip_pub=Mock();body_pub=Mock();node._gripper_pub=grip_pub;node._joint_pub=body_pub
        node._gripper_targets={'left':330.,'right':10.}
        node._gripper_dirty={'left':False,'right':False}
        samples=deque();clock=time.monotonic();node._collector_gripper_last_publish_time=clock-1
        def publish(pub,topic,body):
            if pub is body_pub:
                return
            assert pub is grip_pub
            packet=json.loads(body)
            assert packet=={'left_gripper_target_joints_position':[330.],
                            'right_gripper_target_joints_position':[10.]},packet
            samples.append(SimpleNamespace(stamp_sec=time.monotonic(),received_sec=time.monotonic()))
        node._publish_owned_command.side_effect=publish
        with patch('time.monotonic',side_effect=lambda:clock):
            for i in range(201):
                tick(.7)
                if i>=10:
                    assert ns['_held_signal'](recorder,samples,clock-.08,clock) is not None
                clock+=.01
        assert 9<=len(samples)<=11,len(samples)
        print('V5_GRIPPER_RECORD_HOLD_PASS duration=2s actual_refreshes='+str(len(samples))+' recorder_stale_frames=0')
        node._gripper_pub=None;node._joint_pub=None;node._publish_owned_command.side_effect=None
        node._publish_owned_command.reset_mock()
        node._last_tick=time.monotonic()-.01  # Return from the simulated clock.
        node._authority_heartbeat_time=time.monotonic()
        # A lost authority cannot keep this exemption alive.
        node._follow_authority_time=time.monotonic()-1
        assert not node._collector_policy_position_active()
        node._target_time=time.monotonic()-1;node._feedback_time=time.monotonic()
        node._control_tick()
        np.testing.assert_allclose(node._last_position_command,node._group_vector(groups),atol=1e-6)
        # Expert handoff and reset retain their own paths.
        authority('EXPERT_ACTIVE',2);assert not node._collector_policy_position_active()
        np.testing.assert_allclose(node._group_vector(node._target_groups),node._group_vector(groups),atol=1e-6)
        authority('POLICY_ACTIVE',3);submit();node._reset_active=True;assert not node._collector_policy_position_active();node._reset_active=False
        # Existing excessive following-error and feedback watchdogs still reject.
        large={**command,'right_arm':groups['right_arm']+np.array([0.,0.,0.,-30.,0.,0.,0.])}
        submit(large);tick();assert 'tracking error' in node._set_hardware_fault.call_args.args[0]
        node._set_hardware_fault.reset_mock();node._feedback_time=time.monotonic()-10
        node._control_tick();assert 'feedback watchdog' in node._set_hardware_fault.call_args.args[0]
        node._publish_owned_command.assert_not_called()
        print('V5_NATIVE_POLICY_PASS exact_body_targets=true inference_gap_holds_target=true handoff_and_watchdogs=true hardware_publishers=0 hardware_calls=0')
    finally:
        node._hardware_enabled=False;node._reset_active=False;node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
