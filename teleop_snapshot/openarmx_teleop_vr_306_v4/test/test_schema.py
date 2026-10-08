import numpy as np
import pytest

from ._subject import load_subject


schema = load_subject('schema')


def valid_feedback(as_objects=True):
    values = {
        'leg_waist_joint_state': [0.0, 1.0, 2.0, 3.0],
        'left_arm_joint_state': list(range(7)),
        'right_arm_joint_state': list(range(10, 17)),
        'neck_joint_state': [20.0, 21.0, 22.0],
        'left_gripper_state': [30.0],
        'right_gripper_state': [31.0],
    }
    if as_objects:
        return {key: {'position': value} for key, value in values.items()}
    return values


def valid_body_motion_feedback():
    payload = valid_feedback(as_objects=True)
    for group, (state_key, target_key, _expected) in schema.BODY_GROUP_FIELDS.items():
        positions = payload[state_key]['position']
        payload[state_key]['speed'] = [0.25] * len(positions)
        payload[target_key] = [value + 1.0 for value in positions]
    return payload


@pytest.mark.parametrize('as_objects', [False, True])
def test_joint_feedback_accepts_array_and_position_object_forms(as_objects):
    feedback = schema.parse_joint_feedback(valid_feedback(as_objects))

    assert feedback.leg_waist.shape == (4,)
    assert feedback.left_arm.shape == (7,)
    assert feedback.right_arm.shape == (7,)
    assert feedback.neck.shape == (3,)
    assert feedback.left_gripper.tolist() == [30.0]
    assert feedback.right_gripper.tolist() == [31.0]
    assert set(feedback.as_dict()) == set(schema.GROUP_FIELDS)


def test_joint_feedback_rejects_missing_group():
    payload = valid_feedback()
    payload.pop('right_arm_joint_state')

    with pytest.raises(ValueError, match='missing right_arm_joint_state'):
        schema.parse_joint_feedback(payload)


@pytest.mark.parametrize(
    'field,bad_position',
    [
        ('leg_waist_joint_state', [0.0] * 3),
        ('left_arm_joint_state', [0.0] * 8),
        ('neck_joint_state', []),
        ('left_gripper_state', [0.0, 1.0]),
    ],
)
def test_joint_feedback_rejects_wrong_group_size(field, bad_position):
    payload = valid_feedback()
    payload[field] = {'position': bad_position}

    with pytest.raises(ValueError, match='expected'):
        schema.parse_joint_feedback(payload)


@pytest.mark.parametrize('bad_value', [np.nan, np.inf, -np.inf])
def test_joint_feedback_rejects_nonfinite_values(bad_value):
    payload = valid_feedback()
    payload['left_arm_joint_state']['position'][3] = bad_value

    with pytest.raises(ValueError, match='NaN or Inf'):
        schema.parse_joint_feedback(payload)


def test_body_motion_feedback_reports_maximum_error_and_speed():
    payload = valid_body_motion_feedback()
    motion = schema.parse_body_motion_feedback(payload)
    current = schema.parse_joint_feedback(payload).as_dict()

    maximum_error, maximum_speed = motion.maximum_error_and_speed(current)

    assert maximum_error == pytest.approx(1.0)
    assert maximum_speed == pytest.approx(0.25)
    assert set(motion.targets) == set(schema.BODY_GROUP_FIELDS)


@pytest.mark.parametrize(
    'mutation,expected_message',
    [
        (lambda data: data['neck_joint_state'].pop('speed'), 'neck_joint_state.speed'),
        (lambda data: data.pop('left_arm_target_joint_state'), 'left_arm_target_joint_state'),
        (
            lambda data: data['right_arm_joint_state'].__setitem__('speed', [0.0]),
            'right_arm_joint_state.speed',
        ),
    ],
)
def test_body_motion_feedback_rejects_incomplete_packets(mutation, expected_message):
    payload = valid_body_motion_feedback()
    mutation(payload)

    with pytest.raises(ValueError, match=expected_message):
        schema.parse_body_motion_feedback(payload)


def test_eef_target_accepts_one_arm_and_normalizes_quaternion():
    targets = schema.parse_eef_target({
        'pos_left_in_robot': [0.4, 0.25, 0.8],
        'quat_left_in_robot': [0.0, 0.0, 0.0, 2.0],
    })

    assert set(targets) == {'left'}
    assert np.allclose(targets['left'][0], [0.4, 0.25, 0.8])
    assert np.allclose(targets['left'][1], [0.0, 0.0, 0.0, 1.0])


def test_eef_target_accepts_independent_targets_for_both_arms():
    targets = schema.parse_eef_target({
        'pos_left_in_robot': [0.4, 0.3, 0.8],
        'quat_left_in_robot': [0.0, 0.0, 0.0, 1.0],
        'pos_right_in_robot': [0.4, -0.3, 0.8],
        'quat_right_in_robot': [0.0, 0.0, 1.0, 0.0],
    })

    assert set(targets) == {'left', 'right'}


@pytest.mark.parametrize(
    'payload,expected_message',
    [
        ({}, 'neither left nor right'),
        ({'pos_left_in_robot': [0.4, 0.2, 0.8]}, 'both position and quaternion'),
        ({
            'pos_left_in_robot': [0.4, 0.2],
            'quat_left_in_robot': [0.0, 0.0, 0.0, 1.0],
        }, 'requires 3 position and 4 quaternion'),
        ({
            'pos_left_in_robot': [0.4, 0.2, 0.8],
            'quat_left_in_robot': [0.0, 0.0, 0.0, 0.0],
        }, 'zero length'),
        ({
            'pos_left_in_robot': [3.01, 0.0, 0.0],
            'quat_left_in_robot': [0.0, 0.0, 0.0, 1.0],
        }, 'workspace envelope'),
    ],
)
def test_eef_target_rejects_invalid_contract(payload, expected_message):
    with pytest.raises(ValueError, match=expected_message):
        schema.parse_eef_target(payload)


@pytest.mark.parametrize('bad_value', [np.nan, np.inf, -np.inf])
def test_eef_target_rejects_nonfinite_pose(bad_value):
    with pytest.raises(ValueError, match='NaN or Inf'):
        schema.parse_eef_target({
            'pos_right_in_robot': [0.4, bad_value, 0.8],
            'quat_right_in_robot': [0.0, 0.0, 0.0, 1.0],
        })


def test_finite_degrees_returns_plain_floats_and_rejects_nonfinite():
    assert schema.finite_degrees(np.asarray([1, 2, 3])) == [1.0, 2.0, 3.0]

    with pytest.raises(ValueError, match='NaN or Inf'):
        schema.finite_degrees([0.0, np.nan])
