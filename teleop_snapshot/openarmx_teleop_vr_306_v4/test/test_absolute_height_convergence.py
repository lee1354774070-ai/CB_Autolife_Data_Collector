"""Execute actual controller methods offline; no ROS nodes or robot output."""
import json,time
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
import pytest
from test_height_watchdog_regression import Harness
from openarmx_teleop_vr_306_v4.body_height import body_height_joint_targets,estimate_body_lowering


def height_subject(actual,reference):
    c=Harness()
    c.measured=np.array([*body_height_joint_targets(actual),0.])
    c._body_height_lowering_m=reference
    c._body_height_command_progress=actual/.56
    c._body_height_joint_targets=c.measured[:3].copy()
    c._target_groups={'leg_waist':c.measured.copy()}
    c._body_height_last_direction=1 if reference>actual else -1
    c._waist_follow_neutral_pitch_deg=c.measured[2]
    c._waist_follow_pitch_target_deg=c.measured[2]
    c._body_height_update_time=9.95
    c.params.update(body_height_lowering_rate_m_sec=.18,
                    body_height_motion_command_lead_deg=5.)
    return c


@pytest.mark.parametrize('actual,reference,target',[(.162,.1982,.2),(.24,.2018,.2),(.48,.5582,.56),(.08,.0018,0)])
def test_reference_arrival_does_not_freeze_actual_body_short(actual,reference,target):
    c=height_subject(actual,reference)
    with patch.object(time,'monotonic',return_value=10.):
        c._on_body_height_command(NS(data=json.dumps({'enabled':True,'target_lowering_m':target})))
    assert c._body_height_lowering_m==pytest.approx(reference)
    next_height=estimate_body_lowering((*c._body_height_joint_targets,0))
    assert (next_height-actual)*(target-actual)>0


def test_manual_zero_direction_still_stops_immediately_at_measured_pose():
    c=height_subject(.162,.1982)
    with patch.object(time,'monotonic',return_value=10.):
        c._on_body_height_command(NS(data='{"enabled":true,"direction":0}'))
    np.testing.assert_allclose(c._body_height_joint_targets,c.measured[:3])
    assert c._body_height_lowering_m==pytest.approx(.162)


@pytest.mark.parametrize('start,target',[(0,.2),(0,.3),(.3,0),(.3,.2),(.2,.3)])
def test_absolute_goal_converges_with_lagging_encoder_feedback(start,target):
    c=height_subject(start,start)
    error=abs(target-start)
    deadline=error/.05+3
    history=[]
    for step in range(int(deadline/.05)):
        now=10+step*.05;c._feedback_time=now
        with patch.object(time,'monotonic',return_value=now):
            c._on_body_height_command(NS(data=json.dumps({'enabled':True,'target_lowering_m':target})))
        desired=estimate_body_lowering((*c._body_height_joint_targets,0))
        # Coupled physical profile with .25-second lag, bounded knee speed.
        measured=estimate_body_lowering(c.measured)
        measured+=np.clip((desired-measured)*.2,-.56/155*50*.05,.56/155*50*.05)
        c.measured=np.array([*body_height_joint_targets(measured),0.])
        history.append(measured)
        if abs(measured-target)<=.008:break
    assert abs(history[-1]-target)<=.008,(target,history[-1])
    assert all((b-a)*(target-start)>=-1e-10 for a,b in zip(history,history[1:]))
