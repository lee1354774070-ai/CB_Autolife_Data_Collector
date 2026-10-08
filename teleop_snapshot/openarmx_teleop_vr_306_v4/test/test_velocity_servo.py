import numpy as np

from _subject import load_subject


velocity_servo_module = load_subject('velocity_servo')
VelocityServo = velocity_servo_module.VelocityServo
lookahead_reference = velocity_servo_module.lookahead_reference


def servo():
    return VelocityServo(
        lower=[-100.0, -50.0],
        upper=[100.0, 50.0],
        maximum_velocity=[80.0, 60.0],
        maximum_acceleration=[400.0, 300.0],
        position_gain=8.0,
        feedforward_gain=0.5,
        limit_gain=6.0,
    )


def test_only_active_joints_can_move_and_release_is_immediate():
    subject = servo()
    velocity = subject.step([0.0, 0.0], [20.0, 20.0], 0.01, [0])
    assert velocity[0] > 0.0
    assert velocity[1] == 0.0
    assert subject.stop_indices([0])[0] == 0.0


def test_acceleration_and_velocity_are_bounded():
    subject = servo()
    previous = np.zeros(2)
    for _ in range(100):
        command = subject.step([0.0, 0.0], [100.0, 50.0], 0.01, [0, 1])
        assert np.all(
            np.abs(command - previous) <= np.asarray([4.0, 3.0]) + 1e-12
        )
        assert np.all(np.abs(command) <= [80.0, 60.0])
        previous = command


def test_soft_limit_prevents_outward_motion():
    subject = servo()
    command = subject.step([100.0, -50.0], [120.0, -80.0], 0.01, [0, 1])
    assert command[0] == 0.0
    assert command[1] == 0.0


def test_watchdog_stop_is_not_acceleration_ramped():
    subject = servo()
    subject.step([0.0, 0.0], [20.0, 20.0], 0.05, [0, 1])
    assert np.any(subject.velocity != 0.0)
    assert np.all(subject.stop() == 0.0)


def test_reclutch_rebases_goal_history_without_feedforward_spike():
    subject = servo()
    subject.step([0.0, 0.0], [20.0, 20.0], 0.01, [0])
    subject.rebase_indices([0], [5.0, 20.0])
    command = subject.step([5.0, 0.0], [5.0, 20.0], 0.01, [0])
    assert command[0] == 0.0


def test_jerk_is_bounded_during_sustained_motion():
    subject = servo()
    previous_acceleration = subject.acceleration.copy()
    for _ in range(20):
        subject.step([0.0, 0.0], [80.0, 40.0], 0.01, [0, 1])
        assert np.all(
            np.abs(subject.acceleration - previous_acceleration)
            <= subject.maximum_jerk * 0.01 + 1e-12
        )
        previous_acceleration = subject.acceleration.copy()


def test_reset_profile_overrides_are_stricter_than_teleop_limits():
    subject = servo()
    for _ in range(100):
        command = subject.step(
            [0.0, 0.0], [80.0, 40.0], 0.01, [0, 1],
            maximum_velocity=18.0,
            maximum_acceleration=50.0,
            maximum_jerk=250.0,
        )
    assert np.all(np.abs(command) <= 18.0 + 1e-12)
    assert np.all(np.abs(subject.acceleration) <= 50.0 + 1e-12)


def test_hybrid_reference_is_smooth_and_never_overshoots_raw_target():
    reference = lookahead_reference(
        position=[1.0, -1.0, 2.0],
        velocity=[10.0, -10.0, 10.0],
        target=[1.3, -1.3, 1.8],
        lookahead_sec=0.04,
    )
    assert np.allclose(reference, [1.3, -1.3, 2.0])


def test_zero_lookahead_returns_position_reference_for_safe_reset():
    reference = lookahead_reference([1.0], [20.0], [5.0], 0.0)
    assert np.allclose(reference, [1.0])
