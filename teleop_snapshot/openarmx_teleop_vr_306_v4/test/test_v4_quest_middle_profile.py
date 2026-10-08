from pathlib import Path

import yaml


def _parameters():
    config = Path(__file__).parents[1] / "config" / "controller.yaml"
    document = yaml.safe_load(config.read_text(encoding="utf-8"))
    assert len(document) == 1
    return next(iter(document.values()))["ros__parameters"]


def test_v3_has_bounded_watchdog_grace_period():
    parameters = _parameters()
    assert parameters["target_timeout_sec"] == 0.30
    assert 0.15 < parameters["target_timeout_sec"] <= 0.50


def test_v3_bounds_second_pass_ik_budget():
    parameters = _parameters()
    assert parameters["ik_fallback_max_iterations"] == 40
    assert parameters["ik_fallback_max_iterations"] < parameters["ik_max_iterations"]


def test_v3_keeps_first_stage_damping_unchanged():
    parameters = _parameters()
    assert parameters["ik_damping"] == 0.025
