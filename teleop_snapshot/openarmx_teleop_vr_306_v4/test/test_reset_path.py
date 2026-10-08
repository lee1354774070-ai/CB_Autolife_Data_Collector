import importlib.util
from pathlib import Path

import numpy as np


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / 'openarmx_teleop_vr_306_v4'
    / 'reset_path.py'
)
SPEC = importlib.util.spec_from_file_location('reset_path_subject', MODULE_PATH)
reset_path = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(reset_path)


def test_continuous_reset_preserves_exact_endpoints_and_system_goal():
    start = np.linspace(-20.0, 20.0, 21)
    goal = np.linspace(30.0, -30.0, 21)
    assert np.allclose(
        reset_path.continuous_direct_reset(start, goal, 0.0), start
    )
    assert np.allclose(
        reset_path.continuous_direct_reset(start, goal, 1.0), goal
    )


def test_middle_of_reset_stays_on_the_direct_joint_segment():
    start = np.linspace(-20.0, 20.0, 21)
    goal = np.linspace(30.0, -30.0, 21)
    middle = reset_path.continuous_direct_reset(start, goal, 0.5)
    assert np.allclose(middle, 0.5 * (start + goal))


def test_duration_accounts_for_reset_distance():
    start = np.zeros(21)
    goal = np.zeros(21)
    goal[7] = 110.0
    direct = reset_path.conservative_reset_duration(start, goal, 35.0, 45.0)
    assert direct >= 1.875 * 110.0 / 35.0


def test_invalid_shapes_fail_closed():
    with np.testing.assert_raises(ValueError):
        reset_path.continuous_direct_reset(np.zeros(20), np.zeros(20), 0.5)
