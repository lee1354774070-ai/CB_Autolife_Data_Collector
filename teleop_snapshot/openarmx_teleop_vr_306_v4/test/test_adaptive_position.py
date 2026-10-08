import numpy as np

from openarmx_teleop_vr_306_v4.adaptive_position import (
    AdaptivePositionFeedforward,
    JointTargetVelocityEstimator,
    predict_latest_target,
    project_position,
)


def _update(controller, measured, command, velocity, goal, dt=0.01):
    return controller.update(
        measured, command, velocity, goal, dt,
        minimum_lookahead=0.01,
        maximum_lookahead=0.09,
        filter_tau=0.0,
        velocity_floor=5.0,
        minimum_velocity=90.0,
        maximum_velocity=120.0,
        nominal_acceleration=1200.0,
        maximum_acceleration=2400.0,
        error_low=2.0,
        error_high=12.0,
    )


def test_delay_estimate_and_acceleration_are_per_joint():
    controller = AdaptivePositionFeedforward(4, [1, 2])
    lookahead, velocity_limit, acceleration, error = _update(
        controller,
        measured=[0.0, 0.0, 0.0, 0.0],
        command=[0.0, 8.0, 1.0, 9.0],
        velocity=[0.0, 80.0, 80.0, 80.0],
        goal=[0.0, 20.0, 20.0, 20.0],
    )
    assert np.allclose(error, [0.0, 8.0, 1.0, 9.0])
    assert np.isclose(lookahead[1], 0.09)  # 8/80=0.10, capped
    assert np.isclose(lookahead[2], 0.0125)  # measured lag / planned speed
    assert lookahead[0] == 0.0 and lookahead[3] == 0.0
    assert acceleration[1] > acceleration[2]
    assert acceleration[0] == 1200.0 and acceleration[3] == 1200.0
    assert velocity_limit[1] == 120.0
    assert velocity_limit[2] > 90.0
    assert velocity_limit[0] == 120.0 and velocity_limit[3] == 120.0


def test_prediction_drops_immediately_when_joint_stops():
    controller = AdaptivePositionFeedforward(1, [0])
    first, _, _, _ = _update(controller, [0.0], [6.0], [60.0], [20.0])
    stopped, _, _, _ = _update(controller, [0.0], [6.0], [0.0], [20.0])
    assert first[0] == 0.09
    assert stopped[0] == 0.0


def test_projection_cannot_overshoot_latest_goal():
    result = project_position(
        [0.0, 10.0], [100.0, -100.0], [3.0, 8.0], [0.09, 0.09]
    )
    assert np.allclose(result, [3.0, 8.0])


def test_latest_ik_velocity_estimator_is_filtered_and_clipped():
    estimator = JointTargetVelocityEstimator(3, [0, 1])
    assert np.allclose(
        estimator.update([0.0, 0.0, 0.0], 1.0, filter_tau=0.0,
                         maximum_velocity=180.0),
        0.0,
    )
    velocity = estimator.update(
        [20.0, -5.0, 30.0], 1.1, filter_tau=0.0,
        maximum_velocity=180.0,
    )
    assert np.allclose(velocity, [180.0, -50.0, 0.0])


def test_target_prediction_goes_beyond_latest_sample_but_obeys_bounds():
    predicted = predict_latest_target(
        [10.0, -10.0], [100.0, -100.0], [0.05, 0.05],
        [-30.0, -12.0], [12.0, 30.0],
    )
    assert np.allclose(predicted, [12.0, -12.0])
