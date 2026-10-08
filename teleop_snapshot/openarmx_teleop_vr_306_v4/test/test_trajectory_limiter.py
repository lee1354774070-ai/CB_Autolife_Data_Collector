import numpy as np
import pytest

from ._subject import load_subject


limiter_subject = load_subject('trajectory_limiter')
TrajectoryLimiter = limiter_subject.TrajectoryLimiter
bounded_target_by_feedback = limiter_subject.bounded_target_by_feedback


def test_feedback_relative_target_lead_matches_old_teleop_strategy():
    bounded = bounded_target_by_feedback(
        current=[10.0, -20.0, 5.0],
        target=[40.0, -21.0, -50.0],
        maximum_lead=[12.0, 12.0, 5.0],
    )

    assert np.allclose(bounded, [22.0, -21.0, 0.0])


def test_feedback_relative_target_lead_rejects_invalid_contract():
    with pytest.raises(ValueError, match='positive'):
        bounded_target_by_feedback([0.0], [1.0], [0.0])


@pytest.mark.parametrize(
    'lower,upper,velocity,acceleration,match',
    [
        ([0.0], [0.0], [1.0], [1.0], 'invalid joint bounds'),
        ([0.0], [1.0, 2.0], [1.0], [1.0], 'same shape'),
        ([0.0], [1.0], [0.0], [1.0], 'must be positive'),
        ([0.0], [1.0], [1.0], [-1.0], 'must be positive'),
    ],
)
def test_constructor_rejects_invalid_limits(lower, upper, velocity, acceleration, match):
    with pytest.raises(ValueError, match=match):
        TrajectoryLimiter(lower, upper, velocity, acceleration)


def test_reset_clips_to_hard_bounds_and_clears_velocity():
    limiter = TrajectoryLimiter([-1.0, -2.0], [1.0, 2.0], 10.0, 20.0)
    limiter.velocity[:] = 5.0

    position = limiter.reset([-5.0, 5.0])

    assert np.allclose(position, [-1.0, 2.0])
    assert np.allclose(limiter.velocity, [0.0, 0.0])


def test_reset_and_step_reject_wrong_shape():
    limiter = TrajectoryLimiter([-1.0], [1.0], 1.0, 2.0)
    with pytest.raises(ValueError, match='shape mismatch'):
        limiter.reset([0.0, 0.0])
    limiter.reset([0.0])
    with pytest.raises(ValueError, match='shape mismatch'):
        limiter.step([0.0, 0.0], 0.1)


def test_velocity_and_acceleration_ramp_are_limited_away_from_target():
    limiter = TrajectoryLimiter([-10.0], [10.0], [1.0], [2.0])
    limiter.reset([0.0])
    previous_velocity = limiter.velocity.copy()

    for _ in range(5):
        position = limiter.step([8.0], 0.1)
        velocity_change = limiter.velocity - previous_velocity
        assert np.max(np.abs(limiter.velocity)) <= 1.0 + 1e-12
        assert np.max(np.abs(velocity_change)) <= 0.2 + 1e-12
        assert -10.0 <= position[0] <= 10.0
        previous_velocity = limiter.velocity.copy()


def test_hard_joint_limits_are_never_exceeded_for_extreme_targets():
    limiter = TrajectoryLimiter([-0.5, -1.0], [0.5, 1.0], [10.0, 10.0], [100.0, 100.0])
    limiter.reset([0.0, 0.0])

    for index in range(200):
        target = [1000.0, -1000.0] if index < 100 else [-1000.0, 1000.0]
        position = limiter.step(target, 0.01)
        assert np.all(position >= limiter.lower - 1e-12)
        assert np.all(position <= limiter.upper + 1e-12)
        assert np.all(np.abs(limiter.velocity) <= limiter.max_velocity + 1e-12)


def test_dt_is_clamped_to_safe_minimum_and_maximum():
    def one_step(dt):
        limiter = TrajectoryLimiter([-10.0], [10.0], [1.0], [2.0])
        limiter.reset([0.0])
        return limiter.step([8.0], dt), limiter.velocity.copy()

    tiny_position, tiny_velocity = one_step(0.0)
    minimum_position, minimum_velocity = one_step(1e-4)
    huge_position, huge_velocity = one_step(100.0)
    maximum_position, maximum_velocity = one_step(0.1)

    assert np.allclose(tiny_position, minimum_position)
    assert np.allclose(tiny_velocity, minimum_velocity)
    assert np.allclose(huge_position, maximum_position)
    assert np.allclose(huge_velocity, maximum_velocity)


def test_randomized_commands_preserve_bounds_and_velocity_invariants():
    rng = np.random.default_rng(283)
    lower = np.asarray([-2.0, -1.0, -0.5, -3.0])
    upper = np.asarray([2.0, 1.0, 0.5, 3.0])
    maximum_velocity = np.asarray([1.0, 0.8, 0.4, 1.5])
    maximum_acceleration = np.asarray([3.0, 2.0, 1.0, 4.0])
    limiter = TrajectoryLimiter(lower, upper, maximum_velocity, maximum_acceleration)
    limiter.reset(np.zeros(4))
    previous_velocity = limiter.velocity.copy()

    for _ in range(10000):
        target = rng.uniform(lower - 5.0, upper + 5.0)
        dt = float(rng.uniform(-0.05, 0.25))
        position = limiter.step(target, dt)
        effective_dt = float(np.clip(dt, 1e-4, 0.1))
        assert np.all(np.isfinite(position))
        assert np.all(position >= lower - 1e-12)
        assert np.all(position <= upper + 1e-12)
        assert np.all(np.abs(limiter.velocity) <= maximum_velocity + 1e-12)
        assert np.all(
            np.abs(limiter.velocity - previous_velocity)
            <= maximum_acceleration * effective_dt + 1e-12
        )
        previous_velocity = limiter.velocity.copy()


def test_terminal_step_does_not_drop_velocity_faster_than_acceleration_limit():
    limiter = TrajectoryLimiter([-10.0], [10.0], [10.0], [1.0])
    limiter.reset([0.95])
    limiter.velocity[:] = 1.0
    previous_velocity = limiter.velocity.copy()

    limiter.step([1.0], 0.1)

    assert np.max(np.abs(limiter.velocity - previous_velocity)) <= 0.1 + 1e-12


@pytest.mark.parametrize('bad_target', [np.nan, np.inf, -np.inf])
def test_nonfinite_target_is_rejected(bad_target):
    limiter = TrajectoryLimiter([-1.0], [1.0], [1.0], [1.0])
    limiter.reset([0.0])

    with pytest.raises(ValueError, match='finite'):
        limiter.step([bad_target], 0.1)


@pytest.mark.parametrize('bad_dt', [np.nan, np.inf, -np.inf])
def test_nonfinite_dt_is_rejected(bad_dt):
    limiter = TrajectoryLimiter([-1.0], [1.0], [1.0], [1.0])
    limiter.reset([0.0])

    with pytest.raises(ValueError, match='finite scalar'):
        limiter.step([0.5], bad_dt)


def test_stationary_point_to_point_move_converges_without_overshoot():
    limiter = TrajectoryLimiter([-2.0], [2.0], [1.0], [2.0])
    limiter.reset([0.0])
    positions = []
    velocities = []

    for _ in range(200):
        positions.append(float(limiter.step([1.0], 0.1)[0]))
        velocities.append(float(limiter.velocity[0]))

    assert np.all(np.diff(positions) >= -1e-12)
    assert np.max(positions) <= 1.0 + 1e-12
    assert positions[-1] == pytest.approx(1.0, abs=1e-10)
    assert velocities[-1] == pytest.approx(0.0, abs=1e-10)
    assert np.max(np.abs(np.diff(velocities))) <= 0.2 + 1e-12


def test_nonfinite_limits_and_reset_position_are_rejected():
    with pytest.raises(ValueError, match='finite values'):
        TrajectoryLimiter([np.nan], [1.0], [1.0], [1.0])

    limiter = TrajectoryLimiter([-1.0], [1.0], [1.0], [1.0])
    with pytest.raises(ValueError, match='finite values'):
        limiter.reset([np.inf])


def test_reset_indices_immediately_holds_only_selected_joints():
    limiter = TrajectoryLimiter(
        [-2.0, -2.0, -2.0, -2.0],
        [2.0, 2.0, 2.0, 2.0],
        [1.0, 1.0, 1.0, 1.0],
        [2.0, 2.0, 2.0, 2.0],
    )
    limiter.reset([0.1, 0.2, 0.3, 0.4])
    limiter.velocity[:] = [0.5, 0.6, 0.7, 0.8]

    held = limiter.reset_indices([1, 3], [-0.25, 0.75])

    assert np.allclose(held, [-0.25, 0.75])
    assert np.allclose(limiter.position, [0.1, -0.25, 0.3, 0.75])
    assert np.allclose(limiter.velocity, [0.5, 0.0, 0.7, 0.0])


def test_per_move_limits_can_safely_slow_quick_reset():
    limiter = TrajectoryLimiter([-180.0], [180.0], [100.0], [450.0])
    limiter.reset([0.0])
    previous_velocity = 0.0
    for _ in range(100):
        limiter.step(
            [110.0], 0.01, max_velocity=18.0, max_acceleration=50.0
        )
        assert abs(limiter.velocity[0]) <= 18.0 + 1e-12
        assert abs(limiter.velocity[0] - previous_velocity) <= 0.5 + 1e-12
        previous_velocity = float(limiter.velocity[0])
