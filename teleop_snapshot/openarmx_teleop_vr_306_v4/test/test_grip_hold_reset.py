import json
import threading
from types import SimpleNamespace as NS
import numpy as np
import pytest
from openarmx_teleop_vr_306_v4.reset_state import ClutchedTrigger, QuickResetState
from openarmx_teleop_vr_306_v4.vr_mapper_node import IndependentVrMapper
from openarmx_teleop_vr_306_v4.controller_node import IndependentArmController


def test_clutch_release_and_retake_do_not_open_held_object():
    gate=ClutchedTrigger()
    assert gate.observe(0) is None
    assert gate.observe(1)==1
    gate.release()
    assert gate.observe(0) is None  # re-grip with trigger released: retain
    assert gate.observe(0.01) is None
    assert gate.observe(1)==1       # explicit new squeeze arms the trigger
    assert gate.observe(0)==0       # release while clutched may now open


@pytest.mark.parametrize('grip_active',[False,True])
def test_reliable_trigger_release_only_updates_input(grip_active):
    sent=[]
    sample=NS(trigger=1,grip_active=grip_active)
    subject=NS(_samples={'left':sample},_lock=threading.RLock(),_now=lambda:3,
               _publish_grippers=lambda *a,**k:sent.append(a))
    IndependentVrMapper._on_vr_input(subject,NS(data=json.dumps({'hand':'left','triggerReleased':True})))
    assert sample.trigger==0
    assert sent==[]


def test_yb_requires_one_second_fresh_frames_and_once_per_press():
    state=QuickResetState(button_keys=('left_y','right_b'))
    state.update_full_snapshot(True,True,1)
    assert state.gesture_state(1,.2,1)=='holding'
    state.update_full_snapshot(True,True,1.9)
    assert state.gesture_state(1.9,.2,1)=='holding'
    state.update_full_snapshot(True,True,2.01)
    assert state.gesture_state(2.01,.2,1)=='ready'
    generation=state.begin_request(2.01)
    assert state.complete_request(generation)
    assert state.gesture_state(2.02,.2,1)=='consumed'
    state.begin_boundary()
    assert not state.both_pressed
    assert state.gesture_state(3,.2,1)=='idle'


@pytest.mark.parametrize('preserve',[True,False])
def test_reset_preserves_closed_targets_only_for_new_mode(preserve):
    params={'quick_reset_left_arm_joints':[40,0,0,130,0,0,0],
            'quick_reset_right_arm_joints':[-40,0,0,-130,0,0,0],
            'quick_reset_neck_joints':[0,-30,0], 'quick_reset_open_grippers':True,
            'quick_reset_gripper_open_position':10,'gripper_min_position':10,'gripper_max_position':360}
    groups={'left_arm':np.zeros(7),'right_arm':np.zeros(7),'neck':np.zeros(3),
            'leg_waist':np.zeros(4),'left_gripper':np.array([80.]),'right_gripper':np.array([90.])}
    subject=NS(get_parameter=lambda n:NS(value=params[n]),
        _configuration_guard_error=lambda *a,**k:('',None),
        _kinematics=NS(groups_from_q_deg=lambda q,g:g),
        _reset_required_vector=lambda g:np.r_[g['left_arm'],g['right_arm'],g['neck']],
        _build_motion_controller_state=lambda g:{},
        _prepare_quick_reset_path=lambda *a:('',{'start':np.zeros(17),'goal':np.ones(17),'duration_sec':2}),
        _latest_target_mailbox=NS(reset=lambda:None),_commit_motion_controller_state=lambda s:None,
        _lock_head_follow_for_control_boundary_locked=lambda:None,
        _deactivate_body_height_locked=lambda:None,_velocity_active_sides=set(),_waist_follow_enabled=False,
        _gripper_targets={'left':270.,'right':250.})
    result=IndependentArmController._start_quick_reset_locked(subject,1.,NS(as_dict=lambda:groups),preserve_grippers=preserve)
    assert result==''
    assert subject._gripper_targets==({'left':270.,'right':250.} if preserve else {'left':10.,'right':10.})
    assert subject._reset_active


@pytest.mark.parametrize('target,direction',[(.20,1),(0.,-1),(.56,1)])
def test_continuous_height_targets_use_existing_guarded_controller(target,direction,monkeypatch):
    from test_height_watchdog_regression import Harness
    from openarmx_teleop_vr_306_v4 import controller_node
    subject=Harness()
    subject.params['body_height_lowering_rate_m_sec']=.18
    subject.params['body_height_motion_command_lead_deg']=5.
    subject._body_height_update_time=9.99
    before=subject._body_height_lowering_m
    monkeypatch.setattr(controller_node.time,'monotonic',lambda:10.)
    IndependentArmController._on_body_height_command(subject,NS(data=json.dumps({
        'enabled':True,'target_lowering_m':target})))
    assert (subject._body_height_lowering_m-before)*direction>0
    assert abs(subject._body_height_lowering_m-before)<=.18*.05+1e-9
    assert subject._body_height_active


@pytest.mark.parametrize('target',[-.01,.57,float('nan'),float('inf')])
def test_invalid_height_target_does_not_change_motion(target):
    from test_height_watchdog_regression import Harness
    subject=Harness();before=subject._body_height_joint_targets.copy()
    IndependentArmController._on_body_height_command(subject,NS(data=json.dumps({
        'enabled':True,'target_lowering_m':target})))
    np.testing.assert_array_equal(subject._body_height_joint_targets,before)
    assert 'rejected' in subject._reason
