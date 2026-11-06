"""Real Mink/controller construction and handoff, isolated ROS; no motor I/O."""
import json
import os
from pathlib import Path
import sys
import tempfile
import time
from unittest.mock import Mock

import numpy as np


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '211':
        raise SystemExit('Requires isolated ROS_DOMAIN_ID=211')
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root))
    from dagger.runtime_copy import prepare_copy
    from dagger.robot_profile import resolve_robot
    runtime = prepare_copy(root / 'teleop_snapshot', root / '.runtime')
    sys.path.insert(0, str(runtime))
    robot = resolve_robot(os.environ)
    from dagger.v5_runtime import controller_parameters
    params = {**robot.controller_params(runtime, os.environ), **controller_parameters(runtime)}
    import rclpy
    from std_msgs.msg import String
    from openarmx_teleop_vr_306_v4.controller_node import IndependentArmController
    from openarmx_teleop_vr_306_v4.schema import JointFeedback
    from dagger.mapper import CollectorVrMapper
    args = ['--ros-args', '-r', '__node:=independent_arm_controller_306_v4', '--params-file', str(runtime / 'config/controller.yaml')]
    for key, value in dict(params, dry_run=True, topic_suffix='211_340',
            follow_authority_topic='/hg_dagger/control_state',
            authority_heartbeat_topic='/hg_dagger/authority_heartbeat',
            allow_waist_in_ik=True, reset_before_hardware_enable=False).items():
        args += ['-p', f'{key}:={json.dumps(value)}']
    rclpy.init(args=args)
    node = mapper = None
    try:
        node = IndependentArmController()
        assert node._dry_run
        assert node._joint_pub is None and node._gripper_pub is None
        assert type(node._ik_kinematics).__name__ == 'MinkKinematics'
        groups = dict(leg_waist=np.array([0.,0.,0.,0.]),
                      left_arm=np.array([20.,0.,0.,-100.,0.,0.,0.]),
                      right_arm=np.array([20.,0.,0.,-100.,0.,0.,0.]),
                      neck=np.array([0.,0.,0.]), left_gripper=np.array([10.]),right_gripper=np.array([10.]))
        node._feedback = JointFeedback(**groups)
        node._feedback_time = time.monotonic()
        node._begin_fresh_teleop_session_locked(groups)
        def authority(mode, epoch):
            node._feedback_time = time.monotonic()
            node._on_follow_authority(String(data=json.dumps(dict(
                mode=mode, authority_epoch=epoch, session_id='test-v5'))))
        authority('POLICY_ACTIVE', 1)
        assert not node._waist_follow_active()
        target = dict(left_arm_target_joints_position=groups['left_arm'].tolist(),
                      neck_target_joints_position=[0.,5.,0.], authority_epoch=1,
                      collector_session_id='test-v5')
        node._on_joint_target(String(data=json.dumps(target)))
        assert node._target_source == 'joint_space', node._reason
        np.testing.assert_array_equal(node._target_groups['neck'], [0.,5.,0.])
        authority('EXPERT_READY', 2)
        np.testing.assert_array_equal(node._target_groups['neck'], groups['neck'])
        assert node._target_source == 'fresh_session_measured_hold'
        node._on_joint_target(String(data=json.dumps(target)))
        assert node._target_source == 'fresh_session_measured_hold'
        assert not node._waist_follow_active()
        authority('EXPERT_ACTIVE', 2)
        assert node._waist_follow_active()
        node._on_gripper(String(data=json.dumps(dict(authority_epoch=2,
            collector_session_id='test-v5', left_gripper_target_joints_position=[330.],
            right_gripper_target_joints_position=[330.]))))
        assert node._gripper_targets == dict(left=330.,right=330.),node._reason
        # A delayed old policy must not erase a newer expert gripper target.
        node._on_gripper(String(data=json.dumps(dict(authority_epoch=1,
            left_gripper_target_joints_position=[10.]))))
        assert node._gripper_targets['left'] == 330.
        # Real asynchronous Mink path with a sequenced right-hand clutch.
        pose=node._ik_kinematics.eef_poses(node._ik_kinematics.q_from_feedback(groups))['right']
        intent=dict(authority_epoch=2,collector_session_id='test-v5',clutch_session='hand-v5',
            clutch_sequence=1,clutch_state={'left':False,'right':True},reanchor_sides=['right'],
            pos_right_in_robot=list(pose['position']),quat_right_in_robot=list(pose['orientation']))
        node._feedback_time=time.monotonic()
        node._on_target(String(data=json.dumps(intent)))
        deadline=time.monotonic()+1
        while node._target_source!='cartesian_ik' and time.monotonic()<deadline:time.sleep(.005)
        assert node._target_source=='cartesian_ik',node._reason
        assert node._collector_body_origin_epoch==2
        # Supervisor release barrier is not a V5 mapper clutch packet.
        hold = Mock(wraps=node._hold_sides_locked)
        node._hold_sides_locked = hold
        node._on_release_hold(String(data=json.dumps(dict(source='hg_dagger_authority_barrier',
            authority_epoch=2,sides=['left','right']))))
        hold.assert_called_once()
        node._on_release_hold(String(data=json.dumps(dict(source='hg_dagger_authority_barrier',
            authority_epoch=1,sides=['left','right']))))
        hold.assert_called_once()
        authority('POLICY_ACTIVE', 3)
        assert not node._waist_follow_active()
        node._follow_authority_time -= 2
        assert not node._collector_target_allowed(dict(authority_epoch=3),policy_only=True)
        # Run the real constrained solver with no displacement, both sides.
        q=node._ik_kinematics.q_from_feedback(groups)
        poses=node._ik_kinematics.eef_poses(q)
        for side in ('left','right'):
            p=poses[side]
            result=node._ik_kinematics.solve({side:(p['position'],p['orientation'])},q,
                feedback_q=q,posture_reference=q,reset_reference=q,session_epoch=1,integration_dt=.01)
            assert result.success,result.reason
            np.testing.assert_allclose(result.q,q,atol=1e-6)
        # Actual saved-pose reset planning, collision checks retained, dry run.
        from dagger.task_reset import task_pose
        from openarmx_teleop_vr_306_v4.controller_node import RESET_REQUIRED_JOINT_NAMES
        assert len(RESET_REQUIRED_JOINT_NAMES)==len(node._reset_required_vector(groups))==21
        if os.environ.get('DAGGER_RESET_PLAN'):
            snapshot=task_pose(os.environ)
            with tempfile.TemporaryDirectory() as tmp:
                path=Path(tmp)/'reset.json';path.write_text(json.dumps(snapshot))
                os.environ['DAGGER_RESET_STATE_PATH']=str(path)
                error=node._start_quick_reset_locked(time.monotonic(),node._feedback)
                assert not error,error
                from openarmx_teleop_vr_306_v4.reset_pose_loader import load_quick_reset_targets
                expected=load_quick_reset_targets('340')
                for group in ('leg_waist','left_arm','right_arm','neck'):
                    np.testing.assert_allclose(node._target_groups[group],expected[group],atol=1e-7)
                assert node._reset_active
                print('V5_TASK_RESET_PLAN_PASS all_body_joints=21 grippers=10,10 hardware_calls=0')
                node._reset_active=False
        mapper=CollectorVrMapper()
        from rclpy.parameter import Parameter
        mapper.set_parameters([Parameter('gripper_closed_position',value=360.),
            Parameter('gripper_filter_alpha',value=1.),Parameter('gripper_max_step_per_cycle',value=350.)])
        mapper._gripper_pub.publisher=Mock()
        mapper._on_authority(String(data=json.dumps(dict(session_id='test-v5',authority_epoch=2,mode='EXPERT_ACTIVE'))))
        mapper._publish_grippers({'left':0.,'right':0.},time.monotonic(),force=True)
        mapper._publish_grippers({'left':1.,'right':1.},time.monotonic(),force=True)
        packet=json.loads(mapper._gripper_pub.publisher.publish.call_args.args[0].data)
        assert packet['authority_epoch']==2
        assert packet['collector_session_id']=='test-v5'
        assert packet['left_gripper_target_joints_position'][0]>300.,packet
        print('V5_MINK_HANDOFF_PASS policy_fenced=true waist_expert_only=true gripper=true hardware_publishers=0 hardware_calls=0')
    finally:
        if mapper is not None:mapper.destroy_node()
        if node is not None:node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':main()
