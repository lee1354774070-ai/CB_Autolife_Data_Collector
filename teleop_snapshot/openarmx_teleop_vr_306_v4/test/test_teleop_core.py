import math
from pathlib import Path

import numpy as np
import pytest

from ._subject import load_subject


core = load_subject('teleop_core')
desktop_mode = load_subject('desktop_mode')


@pytest.mark.parametrize(
    'age,maximum_age,expected',
    [
        (0.0, 0.8, True),
        (0.8, 0.8, True),
        (0.801, 0.8, False),
        (-0.01, 0.8, False),
        (None, 0.8, False),
        (np.nan, 0.8, False),
        (np.inf, 0.8, False),
        (0.1, 0.0, False),
    ],
)
def test_vr_input_is_fresh_requires_a_finite_recent_sample(
        age, maximum_age, expected):
    assert core.vr_input_is_fresh(age, maximum_age) is expected


def test_forward_reach_maps_only_to_bounded_positive_waist_pitch():
    map_pitch = core.forward_reach_to_waist_pitch
    assert map_pitch(0.25, 0.0, 0.28, 0.52, 35.0) == pytest.approx(0.0)
    assert map_pitch(0.40, 0.0, 0.28, 0.52, 35.0) == pytest.approx(17.5)
    assert map_pitch(0.60, 0.0, 0.28, 0.52, 35.0) == pytest.approx(35.0)
    # A reverse-crouch baseline must not be pulled toward motor zero.
    assert map_pitch(0.25, -5.0, 0.28, 0.52, 35.0, 0.0) == pytest.approx(-5.0)


def test_waist_assist_starts_near_the_end_of_arm_extension():
    map_pitch = core.forward_reach_to_waist_pitch
    assert map_pitch(0.00, 0.0, 0.20, 0.26, 35.0) == pytest.approx(0.0)
    assert map_pitch(0.20, 0.0, 0.20, 0.26, 35.0) == pytest.approx(0.0)
    assert map_pitch(0.215, 0.0, 0.20, 0.26, 35.0) == pytest.approx(5.46875)
    assert map_pitch(0.23, 0.0, 0.20, 0.26, 35.0) == pytest.approx(17.5)
    assert map_pitch(0.245, 0.0, 0.20, 0.26, 35.0) == pytest.approx(29.53125)
    assert map_pitch(0.26, 0.0, 0.20, 0.26, 35.0) == pytest.approx(35.0)


def test_forward_reach_mapping_rejects_an_invalid_envelope():
    with pytest.raises(ValueError):
        core.forward_reach_to_waist_pitch(0.5, 0.0, 0.7, 0.4, 18.0)
    with pytest.raises(ValueError):
        core.forward_reach_to_waist_pitch(0.5, 0.0, 0.4, 0.7, -1.0)


def test_wrist_reserve_keeps_maximum_reach_at_each_rotation_angle():
    limit = core.forward_limit_with_wrist_reserve
    arguments = (0.26, 0.015, 0.070, 0.0, math.radians(70))
    assert limit(arguments[0], 0.0, *arguments[1:]) == pytest.approx(0.245)
    assert limit(
        arguments[0], 0.0, *arguments[1:]
    ) == pytest.approx(0.245)
    assert limit(
        arguments[0], math.radians(70), *arguments[1:]
    ) == pytest.approx(0.190)
    midpoint = limit(
        arguments[0], math.radians(35), *arguments[1:]
    )
    assert midpoint == pytest.approx(0.2175)


def test_wrist_reserve_rejects_impossible_distance_or_angle_envelopes():
    limit = core.forward_limit_with_wrist_reserve
    with pytest.raises(ValueError):
        limit(0.26, 0.0, 0.02, 0.30, 0.1, 1.0)
    with pytest.raises(ValueError):
        limit(0.26, 0.0, 0.02, 0.06, 1.0, 0.5)


def test_normal_teleop_launch_uses_forward_only_waist_profile_by_default():
    launch = (
        Path(__file__).parents[1] / 'launch' / 'full_vr_teleop.launch.py'
    ).read_text(encoding='utf-8')
    assert "'waist_follow_profile', default_value='forward_pitch_only'" in launch
    assert "'waist_follow_profile': ParameterValue(" in launch
    assert "'forward_position_scale', default_value='0.90'" in launch
    assert "'forward_position_scale': ParameterValue(" in launch
    assert 'forward_position_scale, value_type=float' in launch


@pytest.mark.parametrize(
    'age,expected',
    [
        (0.0, 'active'),
        (0.8, 'active'),
        (0.801, 'paused'),
        (5.0, 'paused'),
        (5.001, 'disabled'),
        (np.nan, 'disabled'),
        (-0.1, 'disabled'),
    ],
)
def test_vr_tracking_state_has_a_protected_pause_window(age, expected):
    assert core.vr_tracking_state(age, 0.8, 5.0) == expected


def test_vr_tracking_state_fails_closed_for_invalid_thresholds():
    assert core.vr_tracking_state(0.1, 0.8, 0.7) == 'disabled'


def controller_sample(position, orientation=(0.0, 0.0, 0.0, 1.0), grip=True, trigger=0.0):
    return core.ControllerSample(
        position=np.asarray(position, dtype=float),
        orientation=np.asarray(orientation, dtype=float),
        grip_active=grip,
        trigger=trigger,
    )


def head_sample(x=0.0, y=0.0, z=0.0, quaternion=None):
    return core.HeadSample(
        rotation=np.asarray([x, y, z], dtype=float),
        quaternion=(
            None if quaternion is None
            else np.asarray(quaternion, dtype=float)
        ),
    )


def axis_angle_quaternion(axis, angle_deg):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    half = np.deg2rad(float(angle_deg)) * 0.5
    return np.concatenate((axis * np.sin(half), [np.cos(half)]))


def test_webxr_yaw_comes_from_horizontal_quaternion_forward_vector():
    assert core.webxr_yaw_deg_from_quaternion(
        axis_angle_quaternion([0.0, 1.0, 0.0], 90.0)
    ) == pytest.approx(90.0)
    assert core.webxr_yaw_deg_from_quaternion(
        axis_angle_quaternion([0.0, 1.0, 0.0], -75.0)
    ) == pytest.approx(-75.0)
    # Roll does not change the horizontal forward direction.
    assert core.webxr_yaw_deg_from_quaternion(
        axis_angle_quaternion([0.0, 0.0, 1.0], 30.0)
    ) == pytest.approx(0.0)


def test_operator_yaw_basis_keeps_physical_forward_as_robot_forward():
    # After the operator turns left 90 degrees, their physical forward is room
    # -X. The captured yaw basis must convert it back to WebXR local -Z, which
    # the existing VR-to-robot transform maps to robot +X (forward).
    world_forward_after_turn = np.asarray([-0.10, 0.0, 0.0])
    basis = desktop_mode.world_to_operator_yaw_rotation(90.0)
    operator_forward = basis @ world_forward_after_turn
    robot_delta = core.VR_TO_ROBOT_ROT @ operator_forward
    assert np.allclose(operator_forward, [0.0, 0.0, -0.10], atol=1.0e-9)
    assert np.allclose(robot_delta, [0.10, 0.0, 0.0], atol=1.0e-9)


def head_mapper(**overrides):
    arguments = {
        'filter_alpha': 1.0,
        'maximum_step_deg': [100.0, 100.0, 100.0],
        'axis_gain': [1.0, 1.0, 1.0],
        'maximum_offset_deg': [15.0, 25.0, 40.0],
        'joint_minimum_deg': [-18.0, -40.0, -55.0],
        'joint_maximum_deg': [18.0, 25.0, 55.0],
    }
    arguments.update(overrides)
    return core.HeadMapper(**arguments)


def test_head_sample_requires_finite_xyz_rotation():
    sample = core.HeadSample.from_packet({
        'position': {'x': 0.1, 'y': 1.6, 'z': -0.2},
        'rotation': {'x': 1.0, 'y': 2.0, 'z': 3.0},
        'quaternion': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 2.0},
    })
    assert np.allclose(sample.rotation, [1.0, 2.0, 3.0])
    assert np.allclose(sample.position, [0.1, 1.6, -0.2])
    assert np.allclose(sample.quaternion, [0.0, 0.0, 0.0, 1.0])
    with pytest.raises(ValueError):
        core.HeadSample.from_packet({'rotation': {'x': 1.0, 'y': None, 'z': 3.0}})
    with pytest.raises(ValueError):
        core.HeadSample.from_packet({
            'position': {'x': 0.1, 'y': np.nan, 'z': -0.2},
            'rotation': {'x': 1.0, 'y': 2.0, 'z': 3.0},
        })
    with pytest.raises(ValueError):
        core.HeadSample.from_packet({
            'rotation': {'x': 1.0, 'y': 2.0, 'z': 3.0},
            'quaternion': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0},
        })


def test_relative_quaternion_head_mapper_uses_rotation_vector_without_euler_swap():
    subject = head_mapper(
        tracking_mode='relative_quaternion',
        deadband_deg=[0.0, 0.0, 0.0],
    )
    identity = np.asarray([0.0, 0.0, 0.0, 1.0])
    axis = np.asarray([1.0, 2.0, 3.0], dtype=float)
    axis = axis / np.linalg.norm(axis)
    subject.latch(head_sample(quaternion=identity), [2.0, -3.0, 4.0])

    result = subject.map(head_sample(
        quaternion=axis_angle_quaternion(axis, 12.0)
    ))
    expected_webxr = axis * 12.0
    expected_neck = np.asarray([2.0, -3.0, 4.0]) + expected_webxr[[2, 0, 1]]
    assert np.allclose(result, expected_neck, atol=1.0e-6)


def test_zero_roll_gain_keeps_head_level_but_preserves_pitch_and_yaw():
    subject = head_mapper(
        tracking_mode='relative_neutral_quaternion',
        neutral_neck_deg=[0.0, 0.0, 0.0],
        axis_gain=[0.0, 1.25, 1.25],
        deadband_deg=[0.0, 0.0, 0.0],
    )
    identity = np.asarray([0.0, 0.0, 0.0, 1.0])
    subject.latch(head_sample(quaternion=identity), [0.0, 0.0, 0.0])

    roll = subject.map(head_sample(
        quaternion=axis_angle_quaternion([0.0, 0.0, 1.0], 12.0)
    ))
    assert np.allclose(roll, [0.0, 0.0, 0.0], atol=1.0e-6)

    subject.release()
    subject.latch(head_sample(quaternion=identity), [0.0, 0.0, 0.0])
    pitch = subject.map(head_sample(
        quaternion=axis_angle_quaternion([1.0, 0.0, 0.0], 8.0)
    ))
    assert np.allclose(pitch, [0.0, 10.0, 0.0], atol=1.0e-6)

    subject.release()
    subject.latch(head_sample(quaternion=identity), [0.0, 0.0, 0.0])
    yaw = subject.map(head_sample(
        quaternion=axis_angle_quaternion([0.0, 1.0, 0.0], -8.0)
    ))
    assert np.allclose(yaw, [0.0, 0.0, -10.0], atol=1.0e-6)


def test_relative_quaternion_head_mapper_rebases_reference_jump_without_motion():
    subject = head_mapper(
        tracking_mode='relative_quaternion',
        reference_jump_deg=35.0,
    )
    identity = np.asarray([0.0, 0.0, 0.0, 1.0])
    subject.latch(head_sample(quaternion=identity), [0.0, 0.0, 0.0])
    before_jump = subject.map(head_sample(
        quaternion=axis_angle_quaternion([0.0, 1.0, 0.0], 10.0)
    ))
    assert np.allclose(before_jump, [0.0, 0.0, 10.0], atol=1.0e-6)

    recentered = head_sample(
        quaternion=axis_angle_quaternion([0.0, 1.0, 0.0], 80.0)
    )
    assert np.allclose(subject.map(recentered), before_jump)
    assert subject.reference_rebases == 1
    assert np.allclose(subject.map(recentered), before_jump)


def test_neutral_relative_head_mapper_returns_forward_to_calibrated_neutral():
    subject = head_mapper(
        tracking_mode='relative_neutral_quaternion',
        neutral_neck_deg=[0.0, 0.0, 0.0],
        deadband_deg=[0.0, 0.0, 0.0],
    )
    identity = np.asarray([0.0, 0.0, 0.0, 1.0])
    subject.latch(head_sample(quaternion=identity), [5.0, -7.0, 9.0])

    # The measured crooked pose initializes the filter only; operator-forward
    # targets the calibrated robot neutral instead of inheriting that offset.
    assert np.allclose(subject.map(head_sample(quaternion=identity)), [0.0, 0.0, 0.0])
    result = subject.map(head_sample(
        quaternion=axis_angle_quaternion([0.0, 1.0, 0.0], 12.0)
    ))
    assert np.allclose(result, [0.0, 0.0, 12.0], atol=1.0e-6)


def test_neutral_relative_reference_jump_holds_until_explicit_recalibration():
    subject = head_mapper(
        tracking_mode='relative_neutral_quaternion',
        neutral_neck_deg=[0.0, 0.0, 0.0],
        reference_jump_deg=35.0,
    )
    identity = np.asarray([0.0, 0.0, 0.0, 1.0])
    subject.latch(head_sample(quaternion=identity), [0.0, 0.0, 0.0])
    before_jump = subject.map(head_sample(
        quaternion=axis_angle_quaternion([0.0, 1.0, 0.0], 10.0)
    ))
    jumped = head_sample(
        quaternion=axis_angle_quaternion([0.0, 1.0, 0.0], 80.0)
    )

    assert np.allclose(subject.map(jumped), before_jump)
    assert subject.recalibration_required is True
    assert np.allclose(subject.map(jumped), before_jump)
    assert subject.reference_rebases == 0

    # X+A/reset or an explicit off/on toggle calls release before the next
    # latch; only that deliberate boundary accepts a new forward reference.
    subject.release()
    subject.latch(jumped, before_jump)
    assert subject.recalibration_required is False
    assert np.allclose(subject.map(jumped), [0.0, 0.0, 0.0])


def test_head_mapper_latches_without_enable_jump_and_maps_xyz_to_roll_pitch_yaw():
    subject = head_mapper()
    subject.latch(head_sample(10.0, 20.0, 30.0), [2.0, -3.0, 4.0])

    assert np.allclose(subject.map(head_sample(10.0, 20.0, 30.0)), [2.0, -3.0, 4.0])
    # WebXR delta X/Y/Z = pitch/yaw/roll; neck order is roll/pitch/yaw.
    assert np.allclose(subject.map(head_sample(15.0, 27.0, 33.0)), [5.0, 2.0, 11.0])


def test_absolute_head_mapper_targets_fixed_neutral_not_measured_offset():
    subject = head_mapper(
        tracking_mode='absolute_reference',
        neutral_neck_deg=[0.0, 0.0, 0.0],
    )
    subject.latch(head_sample(0.0, 0.0, 0.0), [4.0, -6.0, 8.0])

    # Looking forward always means the robot neutral pose, even if the neck
    # was offset when head following was enabled.
    assert np.allclose(subject.map(head_sample()), [0.0, 0.0, 0.0])
    # WebXR X/Y/Z = pitch/yaw/roll; robot order is Roll/Pitch/Yaw.
    assert np.allclose(
        subject.map(head_sample(x=12.0, y=-25.0, z=7.0)),
        [7.0, 12.0, -25.0],
    )


def test_absolute_head_mapper_uses_one_to_one_angles_and_robot_limits():
    subject = head_mapper(
        tracking_mode='absolute_reference',
        neutral_neck_deg=[0.0, 0.0, 0.0],
        maximum_offset_deg=[18.0, 40.0, 55.0],
    )
    subject.latch(head_sample(), [0.0, 0.0, 0.0])

    assert np.allclose(
        subject.map(head_sample(x=-60.0, y=80.0, z=30.0)),
        [18.0, -40.0, 55.0],
    )


def test_head_mapper_rejects_unknown_absolute_mode_and_bad_neutral():
    with pytest.raises(ValueError):
        head_mapper(tracking_mode='world_magic')
    with pytest.raises(ValueError):
        head_mapper(
            tracking_mode='absolute_reference',
            neutral_neck_deg=[0.0, 0.0, 90.0],
        )


def test_head_mapper_wraps_angles_and_enforces_step_and_joint_limits():
    subject = head_mapper(
        maximum_step_deg=[1.0, 2.0, 3.0],
        joint_minimum_deg=[-5.0, -6.0, -7.0],
        joint_maximum_deg=[5.0, 6.0, 7.0],
    )
    subject.latch(head_sample(179.0, 179.0, 179.0), [0.0, 0.0, 0.0])
    first = subject.map(head_sample(-170.0, -160.0, -150.0))
    assert np.allclose(first, [1.0, 2.0, 3.0])
    for _ in range(20):
        final = subject.map(head_sample(-170.0, -160.0, -150.0))
    assert np.allclose(final, [5.0, 6.0, 7.0])


def test_head_mapper_preserves_out_of_nominal_latch_without_moving_farther_out():
    subject = head_mapper()
    # 306 can report a calibrated pitch below the generic URDF minimum.
    subject.latch(head_sample(), [0.0, -55.0, 0.0])

    assert np.allclose(subject.map(head_sample()), [0.0, -55.0, 0.0])
    # Looking farther down must not push farther outside the configured range.
    assert subject.map(head_sample(x=-20.0))[1] == pytest.approx(-55.0)
    # Looking back toward the nominal range remains available.
    assert subject.map(head_sample(x=20.0))[1] == pytest.approx(-35.0)


def mapper(side='left', **overrides):
    arguments = {
        'position_scale': 0.70,
        'filter_alpha': 1.0,
        'maximum_position_step': 10.0,
        'maximum_orientation_step': math.pi,
        'maximum_anchor_displacement': 10.0,
        'workspace_minimum': [-5.0, -5.0, -5.0],
        'workspace_maximum': [5.0, 5.0, 5.0],
        'left_y_minimum': -5.0,
        'right_y_maximum': 5.0,
        'forbidden_box_minimum': [10.0, 10.0, 10.0],
        'forbidden_box_maximum': [11.0, 11.0, 11.0],
    }
    arguments.update(overrides)
    return core.ArmMapper(side=side, **arguments)


def robot_pose(position=(0.4, 0.3, 0.8), orientation=(0.0, 0.0, 0.0, 1.0)):
    return {
        'position': np.asarray(position, dtype=float),
        'orientation': np.asarray(orientation, dtype=float),
    }


def test_forward_extension_uses_independent_calibrated_ratio_and_limit():
    subject = mapper(
        position_scale=0.70,
        forward_position_scale=0.90,
        maximum_forward_displacement=0.26,
    )
    origin = robot_pose()
    subject.latch(controller_sample([0.0, 0.0, 0.0]), origin)

    proportional, reason = subject.map(controller_sample([0.0, 0.0, -0.20]))
    assert reason == ''
    assert proportional['position'][0] == pytest.approx(
        origin['position'][0] + 0.18
    )

    extended, reason = subject.map(controller_sample([0.0, 0.0, -0.40]))
    assert extended['position'][0] == pytest.approx(
        origin['position'][0] + 0.26
    )
    assert 'proportionally limited' in reason

    # The dedicated forward calibration must not change lateral or vertical
    # sensitivity inherited from the established scalar mapping.
    lateral, _ = subject.map(controller_sample([0.20, 0.0, 0.0]))
    vertical, _ = subject.map(controller_sample([0.0, 0.20, 0.0]))
    assert lateral['position'][1] == pytest.approx(origin['position'][1] - 0.14)
    assert vertical['position'][2] == pytest.approx(origin['position'][2] + 0.14)


def test_wrist_rotation_smoothly_reduces_only_the_forward_reach_cap():
    subject = mapper(
        position_scale=0.70,
        forward_position_scale=0.90,
        maximum_forward_displacement=0.26,
        wrist_orientation_gain=1.0,
        wrist_reach_reserve_enabled=True,
        wrist_reach_reserve_minimum_m=0.015,
        wrist_reach_reserve_maximum_m=0.070,
        wrist_reach_reserve_start_angle_deg=0.0,
        wrist_reach_reserve_full_angle_deg=70.0,
    )
    origin = robot_pose()
    subject.latch(controller_sample([0.0, 0.0, 0.0]), origin)

    neutral, neutral_reason = subject.map(
        controller_sample([0.0, 0.0, -0.40])
    )
    assert neutral['position'][0] == pytest.approx(origin['position'][0] + 0.245)
    assert 'reserved for wrist dexterity' in neutral_reason

    rotated, rotated_reason = subject.map(controller_sample(
        [0.0, 0.0, -0.40],
        orientation=axis_angle_quaternion([1.0, 0.0, 0.0], 90.0),
    ))
    assert rotated['position'][0] == pytest.approx(origin['position'][0] + 0.19)
    assert rotated['position'][1] == pytest.approx(origin['position'][1])
    assert rotated['position'][2] == pytest.approx(origin['position'][2])
    assert 'reserved for wrist dexterity' in rotated_reason


@pytest.mark.parametrize(
    'value',
    [
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, np.nan, 1.0],
        [0.0, 0.0, np.inf, 1.0],
    ],
)
def test_normalized_quaternion_rejects_invalid_input(value):
    with pytest.raises(ValueError):
        core.normalized_quaternion(value)


@pytest.mark.parametrize(
    'quaternion',
    [
        [0.0, 0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.2, -0.3, 0.4, 0.5],
    ],
)
def test_quaternion_matrix_round_trip_is_rotation_equivalent(quaternion):
    rotation = core.quaternion_to_matrix(quaternion)
    round_trip = core.matrix_to_quaternion(rotation)

    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-10)
    assert np.linalg.det(rotation) == pytest.approx(1.0, abs=1e-10)
    assert np.allclose(core.quaternion_to_matrix(round_trip), rotation, atol=1e-10)


def test_wrist_rotation_gain_increases_amplitude_on_same_axis():
    angle = math.radians(40.0)
    relative = np.asarray([
        math.sin(angle / 2.0), 0.0, 0.0, math.cos(angle / 2.0)
    ])

    amplified = core.scale_quaternion_rotation(relative, 1.18)

    assert core.quaternion_angle([0.0, 0.0, 0.0, 1.0], amplified) == pytest.approx(
        math.radians(47.2)
    )
    assert np.allclose(amplified[1:3], [0.0, 0.0], atol=1e-12)


def test_controller_sample_parses_webxr_contract_and_normalizes_orientation():
    sample = core.ControllerSample.from_packet({
        'position': {'x': 1, 'y': 2, 'z': 3},
        'quaternion': {'x': 0, 'y': 0, 'z': 0, 'w': 2},
        'gripActive': True,
        'trigger': 0.75,
    })

    assert np.allclose(sample.position, [1.0, 2.0, 3.0])
    assert np.allclose(sample.orientation, [0.0, 0.0, 0.0, 1.0])
    assert sample.grip_active is True
    assert sample.trigger == pytest.approx(0.75)


@pytest.mark.parametrize(
    'packet',
    [
        None,
        {},
        {'position': {'x': 0, 'y': 0, 'z': np.nan},
         'quaternion': {'x': 0, 'y': 0, 'z': 0, 'w': 1}},
        {'position': {'x': 0, 'y': 0, 'z': 0},
         'quaternion': {'x': 0, 'y': 0, 'z': 0, 'w': 0}},
        {'position': {'x': 0, 'y': 0, 'z': 0},
         'quaternion': {'x': 0, 'y': 0, 'z': 0, 'w': 1}, 'trigger': np.inf},
    ],
)
def test_controller_sample_rejects_invalid_packets(packet):
    with pytest.raises((ValueError, TypeError)):
        core.ControllerSample.from_packet(packet)


@pytest.mark.parametrize(
    'trigger,expected',
    [
        (-1.0, 10.0),
        (0.0, 10.0),
        (0.05, 10.0),
        (0.525, 170.0),
        (1.0, 330.0),
        (2.0, 330.0),
    ],
)
def test_linear_gripper_position_clamps_and_applies_deadzone(trigger, expected):
    assert core.linear_gripper_position(trigger, 10.0, 330.0, 0.05) == pytest.approx(expected)


def test_parse_eef_feedback_accepts_flat_and_nested_forms():
    parsed = core.parse_eef_feedback({
        'pos_left_in_robot': [0.4, 0.3, 0.8],
        'quat_left_in_robot': [0, 0, 0, 2],
        'right_eef_pose': {
            'position': [0.4, -0.3, 0.8],
            'orientation': [0, 0, 0, 1],
        },
    })

    assert set(parsed) == {'left', 'right'}
    assert np.allclose(parsed['left']['orientation'], [0, 0, 0, 1])
    assert np.allclose(parsed['right']['position'], [0.4, -0.3, 0.8])


@pytest.mark.parametrize(
    'vr_delta,expected_robot_delta',
    [
        ([0.1, 0.0, 0.0], [0.0, -0.07, 0.0]),
        ([0.0, 0.1, 0.0], [0.0, 0.0, 0.07]),
        ([0.0, 0.0, 0.1], [-0.07, 0.0, 0.0]),
    ],
)
def test_arm_mapper_axis_convention(vr_delta, expected_robot_delta):
    subject = mapper()
    origin = controller_sample([0.0, 0.0, 0.0])
    pose = robot_pose()
    subject.latch(origin, pose)

    target, reason = subject.map(controller_sample(vr_delta))

    assert reason == ''
    assert np.allclose(
        target['position'] - pose['position'],
        expected_robot_delta,
        atol=1e-12,
    )


def test_latch_has_no_initial_pose_jump_and_release_requires_new_latch():
    subject = mapper()
    sample = controller_sample([0.3, 1.2, -0.4])
    pose = robot_pose()
    subject.latch(sample, pose)

    target, reason = subject.map(sample)
    assert reason == ''
    assert np.allclose(target['position'], pose['position'])
    assert np.allclose(target['orientation'], pose['orientation'])

    subject.release()
    assert subject.latched is False
    with pytest.raises(RuntimeError, match='not been latched'):
        subject.map(sample)


def test_operator_room_translation_does_not_move_robot_target():
    subject = mapper()
    origin = controller_sample([0.25, 1.20, -0.35])
    pose = robot_pose()
    head_origin = np.asarray([0.0, 1.65, 0.0])
    subject.latch(origin, pose, body_position=head_origin)

    room_translation = np.asarray([0.60, 0.0, -0.40])
    moved_with_operator = controller_sample(
        origin.position + room_translation
    )
    target, reason = subject.map(
        moved_with_operator,
        body_position=head_origin + room_translation,
    )

    assert reason == ''
    assert np.allclose(target['position'], pose['position'])


def test_hand_motion_relative_to_walking_operator_is_preserved():
    subject = mapper()
    origin = controller_sample([0.25, 1.20, -0.35])
    pose = robot_pose()
    head_origin = np.asarray([0.0, 1.65, 0.0])
    subject.latch(origin, pose, body_position=head_origin)

    room_translation = np.asarray([0.60, 0.0, -0.40])
    relative_hand_motion = np.asarray([0.0, 0.0, -0.10])
    moved = controller_sample(
        origin.position + room_translation + relative_hand_motion
    )
    target, reason = subject.map(
        moved,
        body_position=head_origin + room_translation,
    )

    assert reason == ''
    assert np.allclose(
        target['position'] - pose['position'],
        [0.07, 0.0, 0.0],
    )


def test_relative_vr_rotation_is_conjugated_into_robot_axes():
    subject = mapper()
    origin = controller_sample([0.0, 0.0, 0.0])
    subject.latch(origin, robot_pose())
    half_angle = math.pi / 4.0
    vr_x_rotation = [math.sin(half_angle), 0.0, 0.0, math.cos(half_angle)]

    target, reason = subject.map(controller_sample([0.0, 0.0, 0.0], vr_x_rotation))

    expected_robot_minus_y = [0.0, -math.sin(half_angle), 0.0, math.cos(half_angle)]
    assert reason == ''
    assert np.allclose(
        core.quaternion_to_matrix(target['orientation']),
        core.quaternion_to_matrix(expected_robot_minus_y),
        atol=1e-10,
    )


def test_mapper_limits_position_and_orientation_per_cycle():
    position_limit = 0.012
    orientation_limit = math.radians(4.0)
    subject = mapper(
        filter_alpha=1.0,
        maximum_position_step=position_limit,
        maximum_orientation_step=orientation_limit,
    )
    origin = controller_sample([0.0, 0.0, 0.0])
    pose = robot_pose()
    subject.latch(origin, pose)
    half_angle = math.pi / 4.0
    moved = controller_sample(
        [0.5, 0.5, 0.5],
        [math.sin(half_angle), 0.0, 0.0, math.cos(half_angle)],
    )

    target, reason = subject.map(moved)

    assert reason == ''
    assert np.linalg.norm(target['position'] - pose['position']) <= position_limit + 1e-12
    assert core.quaternion_angle(pose['orientation'], target['orientation']) <= (
        orientation_limit + 1e-12
    )


def configured_safety_mapper(side='left', **overrides):
    arguments = {
        'maximum_anchor_displacement': 10.0,
        'workspace_minimum': [-0.10, -0.75, 0.25],
        'workspace_maximum': [0.80, 0.75, 1.45],
        'left_y_minimum': -0.22,
        'right_y_maximum': 0.22,
        'forbidden_box_minimum': [-0.10, -0.16, 0.35],
        'forbidden_box_maximum': [0.30, 0.16, 1.25],
    }
    arguments.update(overrides)
    return mapper(side=side, **arguments)


@pytest.mark.parametrize(
    'subject,origin_pose,moved_sample,reason_fragment',
    [
        (
            configured_safety_mapper(maximum_anchor_displacement=0.10),
            robot_pose(position=(0.45, 0.30, 0.8)),
            controller_sample([0.0, 0.2, 0.0]),
            'maximum clutch displacement boundary',
        ),
    ],
)
def test_mapper_projects_anchor_violations_without_freezing(
        subject, origin_pose, moved_sample, reason_fragment):
    subject.latch(controller_sample([0.0, 0.0, 0.0]), origin_pose)

    target, reason = subject.map(moved_sample)

    assert target is not None
    assert reason_fragment in reason
    assert np.linalg.norm(
        target['position'] - origin_pose['position']
    ) == pytest.approx(subject.maximum_anchor_displacement)


def test_mapper_projects_limits_and_torso_box_without_freezing():
    subject = configured_safety_mapper()
    subject.latch(
        controller_sample([0.0, 0.0, 0.0]),
        robot_pose(position=(0.35, 0.30, 0.9)),
    )

    workspace, workspace_reason = subject._constrain_position([0.90, 0.30, 0.9])
    centre, centre_reason = subject._constrain_position([0.45, -0.30, 0.9])
    torso, torso_reason = subject._constrain_position([0.20, 0.10, 0.9])

    assert workspace[0] == pytest.approx(0.80)
    assert 'workspace' in workspace_reason
    assert centre[1] == pytest.approx(-0.22)
    assert 'centre boundary' in centre_reason
    assert 'torso exclusion boundary' in torso_reason
    assert not (
        np.all(torso > subject.forbidden_box_minimum)
        and np.all(torso < subject.forbidden_box_maximum)
    )


def test_mapper_allows_useful_cross_body_motion_outside_torso_box():
    subject = configured_safety_mapper()
    pose = robot_pose(position=(0.45, 0.30, 0.8))
    subject.latch(controller_sample([0.0, 0.0, 0.0]), pose)

    target, reason = subject.map(controller_sample([0.6, 0.0, 0.0]))

    assert reason == ''
    assert target is not None
    assert target['position'][1] == pytest.approx(-0.12)


def test_anchor_projection_updates_filtered_pose_continuously():
    subject = configured_safety_mapper(maximum_anchor_displacement=0.10)
    pose = robot_pose(position=(0.45, 0.30, 0.8))
    subject.latch(controller_sample([0.0, 0.0, 0.0]), pose)
    valid_target, _ = subject.map(controller_sample([0.01, 0.0, 0.0]))
    projected, reason = subject.map(controller_sample([0.0, 0.3, 0.0]))

    assert valid_target is not None
    assert projected is not None
    assert 'maximum clutch displacement boundary' in reason
    assert np.allclose(subject.filtered_position, projected['position'])


def test_ik_rejected_sides_only_reports_live_failed_arms():
    status = {
        'state': 'HOLDING',
        'reason': 'IK rejected: iteration limit reached',
        'ik': {
            'success': False,
            'position_error_m': {'right': 0.12},
            'orientation_error_rad': {'right': 0.3},
        },
    }

    assert core.ik_rejected_sides(status) == ['right']
    status['state'] = 'ARMED'
    assert core.ik_rejected_sides(status) == []


def test_vendor_preview_payload_holds_idle_arm_at_current_pose():
    current = {
        'left': robot_pose(position=(0.4, 0.3, 0.8)),
        'right': robot_pose(position=(0.4, -0.3, 0.8)),
    }
    active_left = robot_pose(position=(0.5, 0.35, 0.9))

    payload = core.vendor_pose_payload({'left': active_left}, current)

    assert payload['pos_left_in_robot'] == [0.5, 0.35, 0.9]
    assert payload['pos_right_in_robot'] == [0.4, -0.3, 0.8]
    assert payload['quat_right_in_robot'] == [0.0, 0.0, 0.0, 1.0]
