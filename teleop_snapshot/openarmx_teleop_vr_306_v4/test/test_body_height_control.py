from pathlib import Path

from openarmx_teleop_vr_306_v4.body_height import (
    MAX_BODY_LOWERING_M,
    body_height_joint_targets,
    coordinated_body_height_targets,
    estimate_body_lowering,
)


def test_body_height_curve_matches_control_center_profile():
    assert body_height_joint_targets(0.0) == (0.0, 0.0, 0.0)
    assert body_height_joint_targets(0.28) == (38.0, 77.5, 40.5)
    assert body_height_joint_targets(MAX_BODY_LOWERING_M) == (76.0, 155.0, 81.0)
    assert abs(estimate_body_lowering([38.0, 77.5, 40.5, 12.0]) - 0.28) < 1e-9


def test_controller_has_watchdog_guarded_navigation_only_height_path():
    source = (
        Path(__file__).parents[1]
        / 'openarmx_teleop_vr_306_v4'
        / 'controller_node.py'
    ).read_text(encoding='utf-8')
    assert "'body_height_control_enabled': False" in source
    assert "'body_height_command_timeout_sec': 0.25" in source
    assert "'body_height_hold_command_lead_deg': 2.0" in source
    assert "'body_height_motion_command_lead_deg': 6.0" in source
    assert "'body_height_command_lead_slew_deg_sec': 60.0" in source
    assert 'maximum_lead[0:3] = body_height_output_lead' in source
    assert "'waist_forward_assist_command_lead_deg': 8.0" in source
    assert 'if waist_assist_output_active and not self._reset_active:' in source
    assert 'maximum_lead[2], waist_assist_output_lead' in source
    assert 'def _on_body_height_command' in source
    assert 'not self._hardware_enabled' in source
    assert 'self._body_height_joint_targets' in source
    assert 'waist_follow_enabled = False' not in source
    assert "waist_neutral_pitch = float(body_height_targets[2])" in source
    assert "accepted_groups['leg_waist'][0:2] = body_height_targets[0:2]" in source
    assert 'if waist_pitch_target is None:' in source
    assert 'abs(self._body_height_last_direction) > 1.0e-6' in source
    assert "feedback_height[2] -= waist_assist_delta" in source
    assert "groups['leg_waist'][2] = float(targets[2]) + waist_assist_delta" in source
    assert "{'left', 'right'}.issubset(self._grip_release_held)" in source
    assert "groups['leg_waist'][2] = neutral_pitch" in source
    assert 'desired[2] = neutral_pitch' in source
    assert 'body_height_watchdog_target = desired[0:3].copy()' in source
    assert 'desired[0:3] = body_height_watchdog_target' in source
    assert 'not body_height_output_active' in source
    assert 'self._deactivate_body_height_locked()' in source


def test_full_launch_exposes_forward_waist_follow_command_lead():
    source = (
        Path(__file__).parents[1]
        / 'launch'
        / 'full_vr_teleop.launch.py'
    ).read_text(encoding='utf-8')
    assert (
        "waist_forward_assist_command_lead_deg = LaunchConfiguration(" in source
    )
    assert "'waist_forward_assist_command_lead_deg', default_value='8.0'" in source
    assert "'waist_forward_assist_command_lead_deg': ParameterValue(" in source


def test_body_height_commands_share_one_progress_while_lowering():
    targets, progress = coordinated_body_height_targets(
        lowering_m=MAX_BODY_LOWERING_M,
        measured_deg=(10.0, 5.0, 10.5),
        previous_progress=20.0 / 155.0,
        command_lead_deg=2.0,
    )
    assert progress == 20.0 / 155.0
    assert targets == (
        76.0 * progress,
        155.0 * progress,
        81.0 * progress,
    )


def test_body_height_commands_share_one_progress_while_rising():
    targets, progress = coordinated_body_height_targets(
        lowering_m=0.0,
        measured_deg=(60.0, 130.0, 70.0),
        previous_progress=0.90,
        command_lead_deg=2.0,
    )
    expected = max(
        60.0 / 76.0 - 2.0 / 76.0,
        130.0 / 155.0 - 2.0 / 155.0,
        70.0 / 81.0 - 2.0 / 81.0,
    )
    assert progress == expected
    assert targets == (
        76.0 * progress,
        155.0 * progress,
        81.0 * progress,
    )


def test_body_height_command_does_not_reverse_on_noisy_feedback():
    previous = 0.25
    history = []
    for measured in (
        (18.0, 37.0, 20.0),
        (17.8, 36.7, 19.7),
        (20.0, 41.0, 21.5),
    ):
        _targets, previous = coordinated_body_height_targets(
            lowering_m=0.30,
            measured_deg=measured,
            previous_progress=previous,
            command_lead_deg=2.0,
        )
        history.append(previous)
    assert history == sorted(history)
