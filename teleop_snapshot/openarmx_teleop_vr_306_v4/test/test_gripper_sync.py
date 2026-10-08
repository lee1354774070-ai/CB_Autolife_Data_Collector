from types import SimpleNamespace
import json

from _subject import load_subject


IndependentVrMapper = load_subject('vr_mapper_node').IndependentVrMapper


PARAMETERS = {
    'gripper_left_trigger_scale': 1.0,
    'gripper_right_trigger_scale': 1.0,
    'gripper_full_close_threshold': 0.88,
    'gripper_dual_sync_enabled': True,
    'gripper_dual_sync_activation': 0.15,
    'gripper_dual_sync_window_sec': 0.20,
    'gripper_dual_sync_max_difference': 0.40,
    'enable_gripper': True,
    'gripper_open_position': 10.0,
    'gripper_closed_position': 330.0,
    'gripper_closed_width_cm': 0.0,
    'gripper_trigger_deadzone': 0.05,
    'gripper_filter_alpha': 0.78,
    'gripper_max_step_per_cycle': 32.0,
    'gripper_command_epsilon': 0.75,
    'gripper_resend_interval': 0.15,
}


def mapper_subject():
    subject = IndependentVrMapper.__new__(IndependentVrMapper)
    subject._gripper_input_raw = {'left': 0.0, 'right': 0.0}
    subject._gripper_input_effective = {'left': 0.0, 'right': 0.0}
    subject._gripper_press_time = {'left': None, 'right': None}
    subject._gripper_dual_sync_active = False
    subject._gripper_filtered = {'left': None, 'right': None}
    subject._gripper_last_sent = {'left': None, 'right': None}
    subject._gripper_last_time = {'left': 0.0, 'right': 0.0}
    subject.get_parameter = lambda name: SimpleNamespace(value=PARAMETERS[name])
    return subject


def test_simultaneous_unequal_squeeze_uses_one_full_close_target():
    subject = mapper_subject()
    result = subject._synchronise_gripper_triggers(
        {'left': 0.64, 'right': 0.89}, 10.0
    )
    assert result == {'left': 1.0, 'right': 1.0}
    assert subject._gripper_dual_sync_active is True


def test_single_hand_remains_linear_and_independent():
    subject = mapper_subject()
    result = subject._synchronise_gripper_triggers({'left': 0.40}, 10.0)
    assert result == {'left': 0.40}
    assert subject._gripper_dual_sync_active is False


def test_staggered_two_hand_press_does_not_couple():
    subject = mapper_subject()
    subject._synchronise_gripper_triggers({'left': 0.40, 'right': 0.0}, 10.0)
    result = subject._synchronise_gripper_triggers(
        {'left': 0.40, 'right': 0.60}, 10.30
    )
    assert result == {'left': 0.40, 'right': 0.60}
    assert subject._gripper_dual_sync_active is False


def test_releasing_either_side_ends_dual_sync():
    subject = mapper_subject()
    subject._synchronise_gripper_triggers(
        {'left': 0.50, 'right': 0.50}, 10.0
    )
    result = subject._synchronise_gripper_triggers(
        {'left': 0.0, 'right': 0.60}, 10.1
    )
    assert result == {'left': 0.0, 'right': 0.60}
    assert subject._gripper_dual_sync_active is False


def test_both_sides_are_emitted_in_one_message_and_one_control_step():
    subject = mapper_subject()
    subject._gripper_filtered = {'left': 10.0, 'right': 10.0}
    published = []
    subject._gripper_pub = SimpleNamespace(publish=published.append)

    subject._publish_grippers({'left': 0.64, 'right': 0.89}, 10.0)

    assert len(published) == 1
    payload = json.loads(published[0].data)
    assert payload == {
        'left_gripper_target_joints_position': [42.0],
        'right_gripper_target_joints_position': [42.0],
    }


def test_full_close_respects_configured_motor_endpoint_and_width():
    # DAgger rejects >330 degrees; width=0 must not silently bypass that limit.
    for width, endpoint in ((0.0, 330.0), (5.0, 360.0 - 5.0 / 9.5 * 350.0)):
        subject = mapper_subject()
        params = dict(PARAMETERS, gripper_closed_width_cm=width,
                      gripper_filter_alpha=1.0, gripper_max_step_per_cycle=320.0)
        subject.get_parameter = lambda name: SimpleNamespace(value=params[name])
        published = []
        subject._gripper_pub = SimpleNamespace(publish=published.append)
        for n in range(20):
            subject._publish_grippers({'left': 1.0, 'right': 1.0}, 10.0 + n * .02, force=True)
        for msg in published:
            assert all(10.0 <= v[0] <= 330.0 for v in json.loads(msg.data).values())
        assert json.loads(published[-1].data) == {
            'left_gripper_target_joints_position': [endpoint],
            'right_gripper_target_joints_position': [endpoint],
        }
