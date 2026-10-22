"""Read the reset pose selected by the control center."""

from __future__ import annotations

import json
import math
from pathlib import Path


DEFAULT_STATE_PATH = (
    Path.home() / '.config' / 'autolife_robot_control_center' / 'vr_reset_pose.json'
)

LEG_JOINTS = (
    'Joint_Ankle',
    'Joint_Knee',
    'Joint_Waist_Pitch',
    'Joint_Waist_Yaw',
)
LEFT_ARM_JOINTS = (
    'Joint_Left_Shoulder_Inner',
    'Joint_Left_Shoulder_Outer',
    'Joint_Left_UpperArm',
    'Joint_Left_Elbow',
    'Joint_Left_Forearm',
    'Joint_Left_Wrist_Upper',
    'Joint_Left_Wrist_Lower',
)
RIGHT_ARM_JOINTS = tuple(name.replace('Left', 'Right') for name in LEFT_ARM_JOINTS)
NECK_JOINTS = ('Joint_Neck_Roll', 'Joint_Neck_Pitch', 'Joint_Neck_Yaw')
CUSTOM_POSE_JOINTS = LEG_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + NECK_JOINTS


def _finite_sequence(values, expected, label):
    if not isinstance(values, list) or len(values) != expected:
        raise ValueError(f'{label} must contain exactly {expected} values')
    result = []
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f'{label} contains a non-numeric value')
        number = float(value)
        if not math.isfinite(number):
            raise ValueError(f'{label} contains NaN or Inf')
        result.append(number)
    return result


def _custom_targets(custom):
    if not isinstance(custom, dict):
        raise ValueError('custom reset pose is missing')
    joints = custom.get('joints_deg')
    if not isinstance(joints, dict):
        raise ValueError('custom.joints_deg must be an object')
    missing = [name for name in CUSTOM_POSE_JOINTS if name not in joints]
    if missing:
        raise ValueError('custom reset pose is missing joints: ' + ', '.join(missing))
    values = {}
    for name in CUSTOM_POSE_JOINTS:
        value = joints[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f'{name} must be numeric degrees')
        number = float(value)
        if not math.isfinite(number):
            raise ValueError(f'{name} contains NaN or Inf')
        values[name] = number
    return {
        'leg_waist': [values[name] for name in LEG_JOINTS],
        'left_arm': [values[name] for name in LEFT_ARM_JOINTS],
        'right_arm': [values[name] for name in RIGHT_ARM_JOINTS],
        'neck': [values[name] for name in NECK_JOINTS],
    }


def _original_targets(original):
    if not isinstance(original, dict):
        raise ValueError('original reset pose is missing')
    configs = original.get('configs')
    if not isinstance(configs, dict) or not configs:
        raise ValueError('original reset-pose config snapshots are missing')
    config = next(iter(configs.values()))
    if not isinstance(config, dict):
        raise ValueError('original reset-pose config snapshot is invalid')

    def home_sequence(key, mode, expected, label):
        home = config.get(key)
        if not isinstance(home, dict):
            raise ValueError(f'{label} snapshot is missing')
        return _finite_sequence(home.get(mode), expected, label)

    left = home_sequence(
        'LEFT_ARM_MOTOR_HOME_ANGLE', 'standing', 11,
        'original left-arm standing pose',
    )
    right = home_sequence(
        'RIGHT_ARM_MOTOR_HOME_ANGLE', 'standing', 11,
        'original right-arm standing pose',
    )
    neck = home_sequence(
        'NECK_MOTOR_HOME_ANGLE', 'zero', 7, 'original neck zero pose'
    )
    return {
        'leg_waist': left[:4],
        'left_arm': left[4:11],
        'right_arm': right[4:11],
        'neck': neck[4:7],
    }


def load_quick_reset_targets(
    expected_robot_id,
    state_path=None,
):
    """Return the selected 21-joint reset pose in controller order, in degrees."""
    if expected_robot_id is None or not str(expected_robot_id).strip():
        raise ValueError('robot id could not be derived from topic_suffix')
    path = Path(DEFAULT_STATE_PATH if state_path is None else state_path).expanduser()
    with path.open('r', encoding='utf-8') as stream:
        state = json.load(stream)
    if (
        not isinstance(state, dict)
        or isinstance(state.get('version'), bool)
        or state.get('version') != 1
    ):
        raise ValueError('unsupported control-center reset-pose state')
    robot_id = str(expected_robot_id).strip()
    if not robot_id.isdigit():
        raise ValueError('robot id derived from topic_suffix is invalid')
    if str(state.get('robot_id', '')).strip() != robot_id:
        raise ValueError('saved reset pose belongs to a different robot')
    mode = state.get('mode')
    if mode == 'custom':
        return _custom_targets(state.get('custom'))
    if mode == 'original':
        return _original_targets(state.get('original'))
    raise ValueError('saved reset-pose mode must be custom or original')
