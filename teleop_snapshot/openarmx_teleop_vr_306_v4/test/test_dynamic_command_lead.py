import numpy as np
import pytest

from ._subject import load_subject


DynamicCommandLead = load_subject('adaptive_position').DynamicCommandLead


def update(guard, measured, desired, velocity, previous, **overrides):
    parameters = {
        'minimum': 8.0,
        'nominal': 12.0,
        'maximum': 20.0,
        'speed_low': 5.0,
        'speed_high': 70.0,
        'target_error_low': 2.0,
        'target_error_high': 24.0,
        'following_error_soft': 12.0,
        'following_error_hard': 20.0,
        'expansion_filter_tau': 0.08,
    }
    parameters.update(overrides)
    return guard.update(
        measured, desired, velocity, previous, 0.01, **parameters
    )


def test_expands_smoothly_for_large_well_tracked_motion():
    guard = DynamicCommandLead(4, [1, 2])
    guard.reset(12.0)
    first, following = update(
        guard,
        np.zeros(4),
        np.array([0.0, 40.0, -40.0, 0.0]),
        np.array([0.0, 60.0, -60.0, 0.0]),
        np.array([0.0, 3.0, -3.0, 0.0]),
    )
    assert np.all(first[[1, 2]] > 12.0)
    assert np.all(first[[1, 2]] < 20.0)
    assert np.allclose(following[[1, 2]], 3.0)
    for _ in range(100):
        lead, _ = update(
            guard,
            np.zeros(4),
            np.array([0.0, 40.0, -40.0, 0.0]),
            np.array([0.0, 60.0, -60.0, 0.0]),
            np.array([0.0, 3.0, -3.0, 0.0]),
        )
    assert np.all(lead[[1, 2]] <= 20.0)
    assert np.all(lead[[1, 2]] > 19.5)
    assert np.allclose(lead[[0, 3]], 0.0)


def test_large_following_error_contracts_immediately():
    guard = DynamicCommandLead(3, [0, 1, 2])
    guard.reset(20.0)
    lead, following = update(
        guard,
        np.zeros(3),
        np.full(3, 60.0),
        np.full(3, 70.0),
        np.full(3, 21.0),
    )
    assert np.allclose(following, 21.0)
    assert np.allclose(lead, 8.0)


def test_still_target_returns_to_nominal_not_zero():
    guard = DynamicCommandLead(2, [0, 1])
    guard.reset(18.0)
    lead, _ = update(
        guard,
        np.zeros(2),
        np.zeros(2),
        np.zeros(2),
        np.zeros(2),
    )
    assert np.allclose(lead, 12.0)


def test_rejects_invalid_bounds():
    guard = DynamicCommandLead(1, [0])
    with pytest.raises(ValueError, match='bounds'):
        update(
            guard,
            np.zeros(1),
            np.zeros(1),
            np.zeros(1),
            np.zeros(1),
            minimum=12.0,
            nominal=10.0,
        )
