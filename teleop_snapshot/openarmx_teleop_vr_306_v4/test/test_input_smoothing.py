from openarmx_teleop_vr_306_v4.teleop_core import grip_release_state


def test_one_false_frame_holds_latched_arm():
    state, since = grip_release_state(False, True, None, 10.0, 0.06)
    assert state == 'pending'
    assert since == 10.0
    state, since = grip_release_state(True, True, since, 10.01, 0.06)
    assert state == 'active'
    assert since is None


def test_continuous_release_is_confirmed_after_bound():
    state, since = grip_release_state(False, True, None, 10.0, 0.06)
    assert state == 'pending'
    state, since = grip_release_state(False, True, since, 10.059, 0.06)
    assert state == 'pending'
    state, since = grip_release_state(False, True, since, 10.061, 0.06)
    assert state == 'released'


def test_unlatched_arm_does_not_engage_on_false_sample():
    state, since = grip_release_state(False, False, None, 10.0, 0.06)
    assert state == 'released'
    assert since is None
