import math
from dataclasses import dataclass

import numpy as np


GROUP_FIELDS = {
    'leg_waist': ('leg_waist_joint_state', 4),
    'left_arm': ('left_arm_joint_state', 7),
    'right_arm': ('right_arm_joint_state', 7),
    'neck': ('neck_joint_state', 3),
    'left_gripper': ('left_gripper_state', 1),
    'right_gripper': ('right_gripper_state', 1),
}

BODY_GROUP_FIELDS = {
    'leg_waist': ('leg_waist_joint_state', 'leg_waist_target_joint_state', 4),
    'left_arm': ('left_arm_joint_state', 'left_arm_target_joint_state', 7),
    'right_arm': ('right_arm_joint_state', 'right_arm_target_joint_state', 7),
    'neck': ('neck_joint_state', 'neck_target_joint_state', 3),
}


@dataclass
class JointFeedback:
    leg_waist: np.ndarray
    left_arm: np.ndarray
    right_arm: np.ndarray
    neck: np.ndarray
    left_gripper: np.ndarray
    right_gripper: np.ndarray

    def as_dict(self):
        return {name: getattr(self, name) for name in GROUP_FIELDS}


@dataclass
class BodyMotionFeedback:
    speeds: dict
    targets: dict

    def maximum_error_and_speed(self, current_groups):
        errors = []
        speeds = []
        for group in BODY_GROUP_FIELDS:
            current = np.asarray(current_groups[group], dtype=float).reshape(-1)
            target = np.asarray(self.targets[group], dtype=float).reshape(-1)
            speed = np.asarray(self.speeds[group], dtype=float).reshape(-1)
            errors.append(np.abs(current - target))
            speeds.append(np.abs(speed))
        return float(np.max(np.concatenate(errors))), float(
            np.max(np.concatenate(speeds))
        )


def _positions(value):
    if isinstance(value, dict):
        value = value.get('position', [])
    array = np.asarray(value, dtype=float).reshape(-1)
    if not np.all(np.isfinite(array)):
        raise ValueError('joint feedback contains NaN or Inf')
    return array


def parse_joint_feedback(payload):
    values = {}
    for group, (key, expected) in GROUP_FIELDS.items():
        if key not in payload:
            raise ValueError(f'missing {key}')
        array = _positions(payload[key])
        if array.size != expected:
            raise ValueError(f'{key}: expected {expected}, got {array.size}')
        values[group] = array
    return JointFeedback(**values)


def parse_body_motion_feedback(payload):
    """Parse the vendor fields used to prove a full-body reset is complete."""
    speeds = {}
    targets = {}
    for group, (state_key, target_key, expected) in BODY_GROUP_FIELDS.items():
        state = payload.get(state_key)
        if not isinstance(state, dict):
            raise ValueError(f'{state_key}: expected an object with speed feedback')
        speed = np.asarray(state.get('speed', []), dtype=float).reshape(-1)
        target = np.asarray(payload.get(target_key, []), dtype=float).reshape(-1)
        if speed.size != expected:
            raise ValueError(f'{state_key}.speed: expected {expected}, got {speed.size}')
        if target.size != expected:
            raise ValueError(f'{target_key}: expected {expected}, got {target.size}')
        if not np.all(np.isfinite(speed)) or not np.all(np.isfinite(target)):
            raise ValueError('body motion feedback contains NaN or Inf')
        speeds[group] = speed
        targets[group] = target
    return BodyMotionFeedback(speeds=speeds, targets=targets)


def _pose(payload, side):
    pos_key = f'pos_{side}_in_robot'
    quat_key = f'quat_{side}_in_robot'
    if pos_key not in payload and quat_key not in payload:
        return None
    if pos_key not in payload or quat_key not in payload:
        raise ValueError(f'{side} target must contain both position and quaternion')
    position = np.asarray(payload[pos_key], dtype=float).reshape(-1)
    quaternion = np.asarray(payload[quat_key], dtype=float).reshape(-1)
    if position.size != 3 or quaternion.size != 4:
        raise ValueError(f'{side} target requires 3 position and 4 quaternion values')
    if not np.all(np.isfinite(position)) or not np.all(np.isfinite(quaternion)):
        raise ValueError(f'{side} target contains NaN or Inf')
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-6:
        raise ValueError(f'{side} quaternion has zero length')
    if float(np.linalg.norm(position)) > 3.0:
        raise ValueError(f'{side} target is outside the robot workspace envelope')
    return position, quaternion / norm


def parse_eef_target(payload):
    targets = {side: _pose(payload, side) for side in ('left', 'right')}
    targets = {side: value for side, value in targets.items() if value is not None}
    if not targets:
        raise ValueError('target contains neither left nor right arm pose')
    return targets


def finite_degrees(values):
    result = [float(item) for item in values]
    if not all(math.isfinite(item) for item in result):
        raise ValueError('joint command contains NaN or Inf')
    return result
