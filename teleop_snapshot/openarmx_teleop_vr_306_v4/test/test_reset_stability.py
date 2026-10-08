import numpy as np
from openarmx_teleop_vr_306_v4.reset_path import update_reset_stability


def update(window,t,value=0,now=None,duration=1):
    return update_reset_stability(window,[value,0],t,t if now is None else now,duration,0.3,0.2)


def test_one_second_of_fresh_stable_feedback():
    window=None
    for i in range(11):
        window,done=update(window,10+i/10,0.02*(i%2))
        assert done == (i==10)


def test_repeated_and_stale_feedback_cannot_complete():
    window,_=update(None,10)
    for now in (10.1,10.19,11.1):
        window,done=update(window,10,now=now)
        assert not done


def test_motion_and_slow_drift_restart_stability_window():
    window=None
    for i in range(30):
        window,done=update(window,10+i/10,i*0.1)
        assert not done


def test_feedback_gap_requires_new_full_window():
    window=None
    for i in range(9):window,_=update(window,10+i/10)
    window,done=update(window,11.2)
    assert not done and window[0]==11.2


def test_invalid_sample_rejected_and_legacy_mode_retained():
    assert update(None,10,np.nan)==(None,False)
    assert update(None,10,now=9)==(None,False)
    assert update(None,10,duration=0)==(None,True)
