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
        assert not node._collision_required
        assert node._kinematics.collision_available  # Reset guard is still available.
        pitch = node._kinematics.joint_q['Joint_Neck_Pitch']
        assert abs(np.rad2deg(node._kinematics.upper[pitch]) - 34.5) < 1e-7
        assert abs(np.rad2deg(node._ik_kinematics.upper[pitch]) - 34.5) < 1e-7

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
        original_neck = node._target_groups['neck'].copy()
        node._on_joint_target(String(data=json.dumps({**target, 'neck_target_joints_position':[0.,35.,0.]})))
        assert 'soft limits' in node._reason, node._reason
        np.testing.assert_array_equal(node._target_groups['neck'], original_neck)
        node._on_joint_target(String(data=json.dumps({**target, 'neck_target_joints_position':[0.,34.,0.]})))
        assert node._target_source == 'joint_space', node._reason
        np.testing.assert_array_equal(node._target_groups['neck'], [0.,34.,0.])
        # The same pose remains forbidden as a quick-reset path.
        blocked = {**groups, 'neck': np.array([0.,34.,0.])}
        assert node._kinematics.collision_pairs(node._kinematics.q_from_feedback(blocked))
        node._collector_published_grippers = {'left':330.,'right':10.}
        node._feedback.left_gripper = np.array([267.])
        authority('EXPERT_READY', 2)
        assert node._gripper_targets == {'left':330.,'right':10.}
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
                replay = os.environ.get('DAGGER_RESET_REPLAY_FILE')
                if replay:
                    observed = json.loads(Path(replay).read_text())
                    actual = {group: np.asarray(observed[group + '_joint_state'])
                              for group in ('leg_waist','left_arm','right_arm','neck')}
                    actual.update(left_gripper=np.asarray(observed['left_gripper_state']),
                                  right_gripper=np.asarray(observed['right_gripper_state']))
                    node._reset_active = False
                    error = node._start_quick_reset_locked(time.monotonic(), JointFeedback(**actual))
                    assert not error, error
                    durations = []
                    q = node._kinematics.q_from_feedback(actual, clip=False)
                    for _ in range(500):
                        begin = time.perf_counter()
                        assert not node._kinematics.collision_pairs(q)
                        durations.append((time.perf_counter()-begin)*1000)
                    print('V5_OBSERVED_RESET_PASS neck_pitch=' + str(actual['neck'][1])
                          + ' max_angle=34.5 collision_p95_ms=' + str(round(float(np.percentile(durations,95)),3)))
                    # Replay the logged head pose that blocked X. No motor publishers.
                    from dagger.v5_reset_guard import head_first_sample, head_first_progress
                    blocked = {key:value.copy() for key,value in actual.items()}
                    blocked['neck'] = np.array([-4.25121927,31.55050278,2.48072481])
                    blocked['left_arm'][3] += 5.  # Exercise the second, full-body stage too.
                    node._reset_active = False
                    node._feedback = JointFeedback(**blocked); node._feedback_time=time.monotonic()
                    error=node._start_quick_reset_locked(time.monotonic(),node._feedback)
                    assert not error,error
                    waypoint=node._collector_reset_waypoint
                    assert waypoint is not None
                    start=node._reset_path_start; goal=node._reset_path_goal; split=node._collector_reset_split
                    assert node._reset_active and node._kinematics.collision_available
                    for phase in np.linspace(0,split,100):
                        sample=head_first_sample(start,waypoint,goal,phase,split)
                        np.testing.assert_array_equal(np.delete(sample,19),np.delete(start,19))
                    assert head_first_progress(start,waypoint,goal,start,split,split+.01,split,2)==split
                    assert head_first_progress(start,waypoint,goal,waypoint,split,split+.01,split,2)>split
                    assert not node._kinematics.collision_pairs(node._kinematics.q_from_feedback(
                        node._kinematics_groups_from_vector(waypoint),clip=False))
                    # Physical feedback is frozen first: arm and waist references must stay put.
                    for _ in range(150):
                        node._last_tick=time.monotonic()-.02;node._feedback_time=time.monotonic()
                        node._control_tick()
                        np.testing.assert_allclose(np.delete(node._last_position_command,19),np.delete(start,19),atol=1e-6)
                    # Then simulate tracking. This invokes the actual staged reset control tick.
                    completed=False
                    for _ in range(2500):
                        vector=node._last_position_command.copy()
                        node._feedback=JointFeedback(**{**blocked,**node._kinematics_groups_from_vector(vector)})
                        node._last_tick=time.monotonic()-.02;node._feedback_time=time.monotonic()
                        node._control_tick()
                        if not node._reset_active:completed=True;break
                    assert completed,node._reason
                    assert 'quick reset completed' in node._reason,node._reason
                    np.testing.assert_allclose(node._last_position_command,goal,
                        atol=float(node.get_parameter('quick_reset_tolerance_deg').value))
                    print('V5_HEAD_FIRST_RESET_PASS head_only_then_full_body=true completion=true hardware_calls=0')

                node._reset_active=False
        mapper=CollectorVrMapper()
        from rclpy.parameter import Parameter
        mapper.set_parameters([Parameter('gripper_closed_position',value=360.),
            Parameter('gripper_closed_percent',value=100.*(330.-10.)/350.),
            Parameter('gripper_filter_alpha',value=1.),Parameter('gripper_max_step_per_cycle',value=350.)])
        mapper._gripper_pub.publisher=Mock()
        mapper._on_authority(String(data=json.dumps(dict(session_id='test-v5',authority_epoch=2,mode='EXPERT_ACTIVE'))))
        mapper._publish_grippers({'left':0.,'right':0.},time.monotonic(),force=True)
        mapper._publish_grippers({'left':1.,'right':1.},time.monotonic(),force=True)
        packet=json.loads(mapper._gripper_pub.publisher.publish.call_args.args[0].data)
        assert packet['authority_epoch']==2
        assert packet['collector_session_id']=='test-v5'
        assert abs(packet['left_gripper_target_joints_position'][0]-330.)<1e-6,packet
        print('V5_MINK_HANDOFF_PASS policy_fenced=true waist_expert_only=true gripper=true hardware_publishers=0 hardware_calls=0')
    finally:
        if mapper is not None:mapper.destroy_node()
        if node is not None:node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':main()
