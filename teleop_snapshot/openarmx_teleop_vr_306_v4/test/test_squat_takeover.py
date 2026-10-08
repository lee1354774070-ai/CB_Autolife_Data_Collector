"""Real controller methods with fake feedback; never creates a ROS node."""
from types import SimpleNamespace as NS
from unittest.mock import Mock
import numpy as np
import pytest
from openarmx_teleop_vr_306_v4.controller_node import IndependentArmController as Controller
from openarmx_teleop_vr_306_v4.teleop_core import forward_reach_to_waist_pitch
from openarmx_teleop_vr_306_v4.reset_path import (
    continuous_direct_reset, conservative_reset_duration, feedback_bounded_reset_progress)


def groups(height):
    return dict(leg_waist=np.array([76.,155.,81.,0.])*height/.56,
                left_arm=np.zeros(7),right_arm=np.zeros(7),neck=np.zeros(3),
                left_gripper=np.array([240.]),right_gripper=np.array([180.]))


@pytest.mark.parametrize('height',[-.53,-.4,-.3,-.1,0,.1,.3,.5])
def test_no_hand_motion_preserves_signed_crouch(height):
    neutral=groups(height)['leg_waist'][2]
    for reach in [0.,.05,.19,.20]:
        assert forward_reach_to_waist_pitch(reach,neutral,.20,.26,10.)==pytest.approx(neutral)
    assert forward_reach_to_waist_pitch(.26,neutral,.20,.26,10.)==pytest.approx(neutral+10.)


@pytest.mark.parametrize('height',[-.53,-.3,.3,.5])
def test_session_rebases_neutral_not_zero(height):
    g=groups(height)
    c=NS(_build_motion_controller_state=Mock(return_value={}),
         _latest_target_mailbox=NS(reset=Mock()),_commit_motion_controller_state=Mock(),
         _velocity_active_sides=set(),_waist_follow_enabled=True,
         _waist_follow_profile='forward_pitch_only')
    Controller._begin_fresh_teleop_session_locked(c,g)
    assert c._waist_follow_neutral_pitch_deg==g['leg_waist'][2]
    assert c._waist_follow_pitch_target_deg==g['leg_waist'][2]
    np.testing.assert_allclose(c._target_groups['leg_waist'],g['leg_waist'])


@pytest.mark.parametrize('height',[-.53,-.3,.3,.5])
@pytest.mark.parametrize('hold',[False,True])
def test_both_reset_chords_preserve_measured_crouch(height,hold):
    g=groups(height);g['leg_waist'][3]=12.
    params=dict(quick_reset_left_arm_joints=[40,0,10,130,0,0,0],
        quick_reset_right_arm_joints=[-40,0,-10,-130,0,0,0],
        quick_reset_neck_joints=[0,-30,0],quick_reset_open_grippers=True,
        quick_reset_gripper_open_position=10,gripper_min_position=10,gripper_max_position=360)
    def path(start,goal):
        return '',dict(start=Controller._group_vector(start),goal=Controller._group_vector(goal),duration_sec=12.)
    c=NS(get_parameter=lambda k:NS(value=params[k]),
        _configuration_guard_error=Mock(return_value=('',None)),
        _kinematics=NS(groups_from_q_deg=lambda q,g:g),
        _reset_required_vector=Controller._reset_required_vector,
        _build_motion_controller_state=Mock(return_value={}),
        _prepare_quick_reset_path=path,_latest_target_mailbox=NS(reset=Mock()),
        _commit_motion_controller_state=Mock(),_lock_head_follow_for_control_boundary_locked=Mock(),
        _deactivate_body_height_locked=Mock(),_velocity_active_sides=set(),
        _waist_follow_enabled=True,_waist_follow_profile='forward_pitch_only',
        _gripper_targets={'left':240.,'right':180.})
    assert Controller._start_quick_reset_locked(c,10.,NS(as_dict=lambda:g),preserve_grippers=hold)==''
    np.testing.assert_allclose(c._target_groups['leg_waist'],g['leg_waist'])
    assert c._body_height_reverse_mode == (height < 0)
    assert c._target_groups['leg_waist'][3]==12.
    assert c._waist_follow_pitch_target_deg==c._waist_follow_neutral_pitch_deg
    assert c._gripper_targets==({'left':240.,'right':180.} if hold else {'left':10.,'right':10.})
    start = c._reset_path_start
    goal = c._reset_path_goal
    for phase in np.linspace(0., 1., 21):
        np.testing.assert_allclose(continuous_direct_reset(start, goal, phase)[:4],
                                   g['leg_waist'])


@pytest.mark.parametrize('height',[-.53,-.3,.3,.5])
def test_slow_knee_reset_stays_coordinated_and_finishes(height):
    start=Controller._group_vector(groups(height));goal=start.copy();goal[:3]=0
    actual=start.copy();phase=0.;duration=conservative_reset_duration(start,goal,23.4,30.)
    assert duration>=1.875*np.max(np.abs(start[:3]))/23.4
    # Deliberately much slower knee than ankle and waist, like hardware.
    for _ in range(6000):
        previous=phase
        phase=feedback_bounded_reset_progress(start,goal,actual,phase,min(1,phase+.01/duration),5.)
        command=continuous_direct_reset(start,goal,phase)
        assert phase>=previous
        assert np.max(np.abs(command[:3]-actual[:3]))<=5.+1.e-5
        ratios=(command[:3]-start[:3])/(goal[:3]-start[:3])
        assert np.ptp(ratios)<1.e-8
        actual[:3]+=np.clip(command[:3]-actual[:3],-np.array([20,5,20])*.01,np.array([20,5,20])*.01)
        if phase==1. and np.max(np.abs(actual[:3]))<.05:break
    assert np.max(np.abs(actual[:3]))<.05


def test_stalled_joint_does_not_run_reset_clock_to_end():
    start=Controller._group_vector(groups(-.53));goal=start.copy();goal[:3]=0;phase=0.
    for _ in range(1000):
        phase=feedback_bounded_reset_progress(start,goal,start,phase,min(1,phase+.01),5.)
    assert phase<.2
    assert np.max(np.abs(continuous_direct_reset(start,goal,phase)[:3]-start[:3]))<=5.+1.e-6


def test_arm_only_reset_keeps_original_clock():
    start=Controller._group_vector(groups(-.3));goal=start.copy();goal[4]=30
    assert feedback_bounded_reset_progress(start,goal,start,.3,.4,5.)==.4


def test_invalid_feedback_fails_closed():
    with pytest.raises(ValueError):
        feedback_bounded_reset_progress(np.zeros(21),np.zeros(21),np.full(21,np.nan),0,.1,5.)


@pytest.mark.parametrize('height',[-.30,.30])
def test_manual_branch_ignores_waist_angle_and_stays_latched_at_standing(height):
    from openarmx_teleop_vr_306_v4.body_height import measured_reverse_squat
    legs=groups(height)['leg_waist']
    for waist in [-77.,0.,77.]:
        legs[2]=waist
        assert measured_reverse_squat(legs)==(height<0)
    assert measured_reverse_squat([0,0,70],previous=height<0)==(height<0)


def height_subject(height):
    from test_height_watchdog_regression import Harness
    c=Harness();c.measured=groups(height)['leg_waist']
    c._body_height_joint_targets=c.measured[:3].copy()
    c._body_height_lowering_m=height;c._body_height_command_progress=height/.56
    c._body_height_last_direction=0.
    c._waist_follow_neutral_pitch_deg=c._waist_follow_pitch_target_deg=c.measured[2]
    c._body_height_update_time=9.95
    return c


def height_command(c,now,**payload):
    import json,time
    from unittest.mock import patch
    c._feedback_time=now
    with patch.object(time,'monotonic',return_value=now):
        c._on_body_height_command(NS(data=json.dumps({'enabled':True,**payload})))


@pytest.mark.parametrize('start,goal',[(-.3,.2),(.3,-.2),(-.53,.53),(.53,-.53)])
def test_task_direction_changes_without_previous_mode_leak(start,goal):
    from openarmx_teleop_vr_306_v4.body_height import estimate_body_lowering
    c=height_subject(start)
    for i in range(1500):
        now=10+i*.05
        height_command(c,now,target_lowering_m=abs(goal),reverse_squat=goal<0)
        target=c._body_height_joint_targets
        c.measured[:3]+=(target-c.measured[:3])*.3
        if abs(estimate_body_lowering(c.measured,signed=True)-goal)<.003:break
    assert abs(estimate_body_lowering(c.measured,signed=True)-goal)<.003
    assert c._body_height_reverse_mode==(goal<0)
    # Manual lowering after the task must use the achieved branch.
    height_command(c,now+.05,direction=1)
    assert c._body_height_reverse_mode==(goal<0)
    before=c._body_height_lowering_m
    height_command(c,now+.1,direction=1)
    assert (c._body_height_lowering_m-before)*goal>=0


def test_manual_raise_from_reverse_does_not_flip_at_minus_two_mm():
    c=height_subject(-.01)
    for i in range(30):
        height_command(c,10+i*.05,direction=-1)
        c.measured[:3]=c._body_height_joint_targets
    assert c._body_height_lowering_m==pytest.approx(0.)
    assert c._body_height_reverse_mode is True
    height_command(c,11.55,direction=0)
    height_command(c,11.6,direction=1)
    assert c._body_height_lowering_m<0


def test_absolute_to_manual_discards_advanced_reference():
    from openarmx_teleop_vr_306_v4.body_height import estimate_body_lowering
    c=height_subject(-.20)
    c._body_height_input_absolute=True;c._body_height_lowering_m=.15
    c._body_height_command_progress=.15/.56
    height_command(c,10.,direction=1)
    assert c._body_height_lowering_m==pytest.approx(estimate_body_lowering(c.measured,signed=True))
    assert c._body_height_reverse_mode is True


def test_forward_assist_near_positive_height_limit_cannot_exceed_waist_limit():
    c=height_subject(.53)
    c._kinematics.upper[2]=np.deg2rad(78.)
    c._waist_follow_pitch_target_deg=c.measured[2]+10.
    c._body_height_command_time=10.
    out=c._body_height_output_target_locked(10.01,c._feedback)
    assert out[2]<=78.
    height_command(c,10.02,target_lowering_m=.53)
    assert c._target_groups['leg_waist'][2]<=78.
