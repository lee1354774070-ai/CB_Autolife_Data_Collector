import ast
import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'openarmx_teleop_vr_306_v4'
RESET_STATE = SOURCE / 'reset_state.py'
MAPPER = SOURCE / 'vr_mapper_node.py'


def _load_reset_state():
    spec = importlib.util.spec_from_file_location(
        'openarmx_306_v4_reset_state_subject', RESET_STATE
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


state = _load_reset_state()


def _function_source(path, name):
    source = path.read_text(encoding='utf-8')
    tree = ast.parse(source)
    node = next(
        candidate for candidate in ast.walk(tree)
        if isinstance(candidate, ast.FunctionDef) and candidate.name == name
    )
    return ast.get_source_segment(source, node)


def test_pending_quick_reset_expires_and_old_callback_cannot_clear_new_request():
    quick = state.QuickResetState(request_timeout_sec=2.0)
    quick.update_full_snapshot(True, True, 10.0)
    assert quick.gesture_state(10.0, 0.30, 1.0) == 'holding'
    assert quick.gesture_state(11.01, 0.30, 1.0) == 'idle'

    # A current two-hand frame is required throughout the hold.
    quick.update_full_snapshot(True, True, 11.02)
    assert quick.gesture_state(11.02, 0.30, 1.0) == 'holding'
    quick.update_full_snapshot(True, True, 12.03)
    assert quick.gesture_state(12.03, 0.30, 1.0) == 'ready'
    first = quick.begin_request(12.03)
    assert quick.pending
    assert quick.expire_request(14.02) is None
    assert quick.expire_request(14.04) == first
    assert not quick.pending

    # Release either half of X+A, then form a new chord.
    quick.update_full_snapshot(False, True, 14.05)
    assert quick.gesture_state(14.05, 0.30, 1.0) == 'idle'
    quick.update_full_snapshot(True, True, 14.06)
    assert quick.gesture_state(14.06, 0.30, 1.0) == 'holding'
    quick.update_full_snapshot(True, True, 15.07)
    assert quick.gesture_state(15.07, 0.30, 1.0) == 'ready'
    second = quick.begin_request(15.07)

    assert second > first
    assert not quick.complete_request(first)
    assert quick.pending_generation == second
    assert quick.complete_request(second)
    assert not quick.pending


def test_consumed_chord_rearms_when_either_button_is_released():
    quick = state.QuickResetState()
    quick.update_full_snapshot(True, True, 1.0)
    assert quick.gesture_state(1.0, 0.30, 1.0) == 'holding'
    quick.update_full_snapshot(True, True, 2.1)
    assert quick.gesture_state(2.1, 0.30, 1.0) == 'ready'
    quick.consume_chord()
    assert quick.gesture_state(2.1, 0.30, 1.0) == 'consumed'

    quick.update_full_snapshot(True, False, 2.2)
    assert not quick.chord_consumed
    quick.update_full_snapshot(True, True, 2.3)
    assert quick.gesture_state(2.3, 0.30, 1.0) == 'holding'


def test_session_boundary_invalidates_pending_generation_and_button_snapshot():
    quick = state.QuickResetState()
    quick.update_full_snapshot(True, True, 5.0)
    token = quick.begin_request(5.0)

    assert quick.begin_boundary() == token
    assert not quick.pending
    assert quick.full_input_time == 0.0
    assert quick.chord_consumed
    assert not quick.complete_request(token)


def test_grip_rearm_is_completed_by_fresh_release_samples_outside_control_tick():
    grip = state.GripRearmState()
    grip.begin_boundary()

    assert not grip.observe('left', True, 20.0, 0.06)
    assert grip.waiting['left']
    assert not grip.observe('left', False, 20.01, 0.06)
    assert not grip.observe('left', False, 20.069, 0.06)
    assert grip.observe('left', False, 20.071, 0.06)
    assert not grip.waiting['left']

    # Once re-armed, a later press is free to latch in the mapper.
    assert not grip.observe('left', True, 20.08, 0.06)
    assert not grip.waiting['left']


def test_new_boundary_discards_partial_release_from_previous_session():
    grip = state.GripRearmState()
    assert not grip.observe('right', False, 30.0, 0.06)
    grip.begin_boundary()
    assert not grip.observe('right', False, 30.04, 0.06)
    assert grip.waiting['right']
    assert grip.observe('right', False, 30.101, 0.06)


def test_one_hundred_enable_reset_cycles_never_accumulate_a_stuck_latch():
    """Repeated enable/reset boundaries must leave both controls reusable.

    This models the failure sequence seen on the headset: enable, release both
    Grip buttons, hold X+A for quick reset, cross the reset boundary, then
    release Grip again.  One hundred repetitions make leaked pending requests,
    consumed X+A chords and release-debounce state deterministic regressions.
    """
    quick = state.QuickResetState(request_timeout_sec=2.0)
    grip = state.GripRearmState()
    previous_token = 0

    for cycle in range(100):
        now = 100.0 + cycle * 10.0

        # Enabling starts a fresh session.  A held Grip must not punch through
        # the boundary; each side is re-armed only by a fresh stable release.
        quick.begin_boundary()
        enable_generation = grip.begin_boundary()
        assert enable_generation == cycle * 2 + 1
        for side in state.ARM_SIDES:
            assert not grip.observe(side, True, now, 0.06)
            assert not grip.observe(side, False, now + 0.01, 0.06)
            assert grip.observe(side, False, now + 0.071, 0.06)
            assert not grip.waiting[side]

        # The post-boundary chord is intentionally consumed.  Releasing either
        # X or A in a fresh full snapshot must re-arm it before the next press.
        release_left = cycle % 2 == 0
        quick.update_full_snapshot(not release_left, release_left, now + 0.10)
        assert quick.gesture_state(now + 0.10, 0.30, 1.0) == 'idle'
        assert not quick.chord_consumed
        quick.update_full_snapshot(True, True, now + 0.11)
        assert quick.gesture_state(now + 0.11, 0.30, 1.0) == 'holding'
        quick.update_full_snapshot(True, True, now + 1.12)
        assert quick.gesture_state(now + 1.12, 0.30, 1.0) == 'ready'

        token = quick.begin_request(now + 1.12)
        assert token > previous_token
        previous_token = token
        assert quick.pending
        assert quick.complete_request(token)
        assert not quick.pending

        # A successful reset creates another boundary.  It must clear every
        # latch and partial release, and must not make the next cycle harder.
        assert quick.begin_boundary() is None
        reset_generation = grip.begin_boundary()
        assert reset_generation == cycle * 2 + 2
        for side in state.ARM_SIDES:
            assert not grip.observe(side, False, now + 1.20, 0.06)
            assert grip.observe(side, False, now + 1.261, 0.06)
            assert not grip.waiting[side]

        assert not quick.pending
        assert quick.chord_consumed
        assert quick.full_input_time == 0.0

    assert grip.generation == 200
    assert all(not grip.waiting[side] for side in state.ARM_SIDES)


def test_mapper_integrates_timeout_generation_and_input_callback_rearm():
    mapper = MAPPER.read_text(encoding='utf-8')
    check_reset = _function_source(MAPPER, '_check_reset_gesture')
    input_callback = _function_source(MAPPER, '_on_vr_input')
    boundary = _function_source(MAPPER, '_begin_session_boundary_locked')

    assert "'quick_reset_request_timeout_sec': 2.0" in mapper
    assert 'expire_request(now)' in check_reset
    assert 'complete_request(generation)' in check_reset
    assert 'self._quick_future.cancel()' not in check_reset
    assert 'future.cancel()' in check_reset
    assert 'self._grip_rearm.observe(' in input_callback
    assert 'self._samples.clear()' in boundary
    assert 'self._sample_times.clear()' in boundary
    assert 'self._quick_state.begin_boundary()' in mapper
