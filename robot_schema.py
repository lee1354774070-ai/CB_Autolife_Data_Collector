#!/usr/bin/env python3
"""Canonical Autolife state/action schema used by the LeRobot collector.

LeRobot's relative-action processor subtracts state and action by array index,
not by matching feature names.  This module therefore owns both the ordering
and the names for every joint-controlled dimension.  Keeping that information
in one place prevents a parser change from silently corrupting relative actions.

The first 16 dimensions preserve the collector's historical arm/gripper layout.
Optional head and waist groups are appended, so enabling a new observation
never shifts an existing arm or gripper index. The upper-waist option selects
only pitch/yaw from the controller's four-value leg/waist packet.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class JointGroup:
    """Describe one contiguous group in the robot's ROS JSON payloads."""

    name: str
    names: tuple[str, ...]
    state_key: str
    status_target_key: str | None
    command_target_key: str | None
    source_indices: tuple[int, ...] | None = None

    @property
    def size(self) -> int:
        return len(self.names)


# These orders come from Autolife's running arm controller on robot 283.  In
# particular, neck is Roll/Pitch/Yaw and leg_waist is Ankle/Knee/Pitch/Yaw.
LEFT_ARM = JointGroup(
    name="left_arm",
    names=(
        "left_shoulder_inner",
        "left_shoulder_outer",
        "left_upper_arm",
        "left_elbow",
        "left_forearm",
        "left_wrist_upper",
        "left_wrist_lower",
    ),
    state_key="left_arm_joint_state",
    status_target_key="left_arm_target_joint_state",
    command_target_key="left_arm_target_joints_position",
)
RIGHT_ARM = JointGroup(
    name="right_arm",
    names=(
        "right_shoulder_inner",
        "right_shoulder_outer",
        "right_upper_arm",
        "right_elbow",
        "right_forearm",
        "right_wrist_upper",
        "right_wrist_lower",
    ),
    state_key="right_arm_joint_state",
    status_target_key="right_arm_target_joint_state",
    command_target_key="right_arm_target_joints_position",
)
LEFT_GRIPPER = JointGroup(
    name="left_gripper",
    names=("left_gripper",),
    state_key="left_gripper_state",
    status_target_key=None,
    command_target_key=None,
)
RIGHT_GRIPPER = JointGroup(
    name="right_gripper",
    names=("right_gripper",),
    state_key="right_gripper_state",
    status_target_key=None,
    command_target_key=None,
)
HEAD = JointGroup(
    name="head",
    names=("neck_roll", "neck_pitch", "neck_yaw"),
    state_key="neck_joint_state",
    status_target_key="neck_target_joint_state",
    command_target_key="neck_target_joints_position",
)
WAIST = JointGroup(
    name="waist",
    names=("leg_ankle", "leg_knee", "waist_pitch", "waist_yaw"),
    state_key="leg_waist_joint_state",
    status_target_key="leg_waist_target_joint_state",
    command_target_key="leg_waist_target_joints_position",
)
UPPER_WAIST = JointGroup(
    name="upper_waist",
    names=("waist_pitch", "waist_yaw"),
    state_key="leg_waist_joint_state",
    status_target_key="leg_waist_target_joint_state",
    command_target_key="leg_waist_target_joints_position",
    source_indices=(2, 3),
)

BASE_GROUPS = (LEFT_ARM, RIGHT_ARM, LEFT_GRIPPER, RIGHT_GRIPPER)
WHOLE_BODY_GROUPS = (WAIST, LEFT_ARM, RIGHT_ARM, LEFT_GRIPPER, RIGHT_GRIPPER, HEAD)
WHOLE_BODY_DIM = sum(group.size for group in WHOLE_BODY_GROUPS)

WAIST_SLICE = slice(0, 4)
UPPER_WAIST_SLICE = slice(2, 4)
LEFT_ARM_SLICE = slice(4, 11)
RIGHT_ARM_SLICE = slice(11, 18)
LEFT_GRIPPER_INDEX = 18
RIGHT_GRIPPER_INDEX = 19
HEAD_SLICE = slice(20, 23)


def _numeric_list(
    value: Any,
    size: int,
    source_indices: tuple[int, ...] | None = None,
) -> list[float] | None:
    """Select one complete numeric group, rejecting partial payloads."""

    required_size = max(source_indices) + 1 if source_indices else size
    if not isinstance(value, (list, tuple)) or len(value) < required_size:
        return None
    values = [value[index] for index in source_indices] if source_indices else value[:size]
    if not all(isinstance(item, (int, float, bool)) for item in values):
        return None
    return [float(item) for item in values]


def _state_positions(payload: Any, group: JointGroup) -> list[float] | None:
    if not isinstance(payload, dict):
        return None
    state = payload.get(group.state_key)
    if not isinstance(state, dict):
        return None
    return _numeric_list(state.get("position"), group.size, group.source_indices)


def _top_level_positions(
    payload: Any,
    key: str | None,
    size: int,
    source_indices: tuple[int, ...] | None = None,
) -> list[float] | None:
    if not isinstance(payload, dict) or key is None:
        return None
    return _numeric_list(payload.get(key), size, source_indices)


@dataclass(frozen=True)
class RobotSchema:
    """Resolved state/action layout for one recording session.

    For joint and status-target action modes, ``names`` is used for both
    ``observation.state`` and ``action``.  This is the contract required for
    index-wise relative action conversion during training.
    """

    groups: tuple[JointGroup, ...]

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(name for group in self.groups for name in group.names)

    @property
    def size(self) -> int:
        return len(self.names)

    def parse_state(self, payload: Any) -> np.ndarray | None:
        values: list[float] = []
        for group in self.groups:
            group_values = _state_positions(payload, group)
            if group_values is None:
                return None
            values.extend(group_values)
        return np.asarray(values, dtype=np.float32)

    def parse_status_target(self, payload: Any) -> np.ndarray | None:
        """Parse controller targets embedded in the whole-body status topic.

        The status packet does not expose separate gripper targets.  For those
        two dimensions we retain the measured gripper position, matching the
        collector's existing behavior.  Arm/head/waist dimensions use actual
        controller targets and remain index-aligned with ``parse_state``.
        """

        values: list[float] = []
        for group in self.groups:
            if group.status_target_key is None:
                group_values = _state_positions(payload, group)
            else:
                group_values = _top_level_positions(
                    payload,
                    group.status_target_key,
                    group.size,
                    group.source_indices,
                )
            if group_values is None:
                return None
            values.extend(group_values)
        return np.asarray(values, dtype=np.float32)

    def parse_body_command(self, payload: Any) -> dict[str, list[float]] | None:
        """Parse all non-gripper groups from the whole-body command topic."""

        result: dict[str, list[float]] = {}
        for group in self.groups:
            if group.command_target_key is None:
                continue
            values = _top_level_positions(
                payload,
                group.command_target_key,
                group.size,
                group.source_indices,
            )
            # Some older publishers reuse the nested status representation.
            if values is None:
                values = _state_positions(payload, group)
            if values is None:
                return None
            result[group.name] = values
        return result

    def compose_command_action(
        self,
        body_groups: dict[str, list[float]],
        grippers: tuple[float | None, float | None],
    ) -> np.ndarray | None:
        """Merge whole-body and gripper topics in canonical schema order."""

        gripper_values = {
            "left_gripper": grippers[0],
            "right_gripper": grippers[1],
        }
        values: list[float] = []
        for group in self.groups:
            if group.name in gripper_values:
                value = gripper_values[group.name]
                if value is None:
                    return None
                values.append(float(value))
                continue
            group_values = body_groups.get(group.name)
            if group_values is None or len(group_values) != group.size:
                return None
            values.extend(group_values)
        return np.asarray(values, dtype=np.float32)


def build_robot_schema(
    with_head: bool = False,
    with_waist: bool = False,
    with_upper_waist: bool = False,
) -> RobotSchema:
    """Build a stable base-16 schema with optional groups appended."""

    if with_waist and with_upper_waist:
        raise ValueError("with_waist and with_upper_waist are mutually exclusive")
    groups = list(BASE_GROUPS)
    if with_head:
        groups.append(HEAD)
    if with_waist:
        groups.append(WAIST)
    elif with_upper_waist:
        groups.append(UPPER_WAIST)
    return RobotSchema(tuple(groups))


def parse_gripper_command(payload: Any) -> tuple[float | None, float | None]:
    """Parse the dedicated gripper command topic without inventing defaults."""

    left = _top_level_positions(payload, "left_gripper_target_joints_position", 1)
    right = _top_level_positions(payload, "right_gripper_target_joints_position", 1)
    return (left[0] if left else None, right[0] if right else None)


def parse_whole_body_state(payload: Any) -> np.ndarray | None:
    """Parse the controller's physical q23 order from one status packet.

    This order is used only at the ROS boundary. Policies always use the
    schema order, which keeps optional head and waist joints appended after the
    historical 16-D arm/gripper prefix.
    """

    values: list[float] = []
    for group in WHOLE_BODY_GROUPS:
        positions = _state_positions(payload, group)
        if positions is None:
            return None
        values.extend(positions)
    return np.asarray(values, dtype=np.float32)


def schema_state_from_whole_body(q23: np.ndarray, schema: RobotSchema) -> np.ndarray:
    """Convert physical q23 telemetry to a policy state in schema order."""

    q = np.asarray(q23, dtype=np.float32).reshape(-1)
    if q.size < WHOLE_BODY_DIM:
        raise ValueError(f"Expected {WHOLE_BODY_DIM} physical joints, got {q.size}.")

    values = [*q[LEFT_ARM_SLICE], *q[RIGHT_ARM_SLICE], q[LEFT_GRIPPER_INDEX], q[RIGHT_GRIPPER_INDEX]]
    group_names = {group.name for group in schema.groups}
    if "head" in group_names:
        values.extend(q[HEAD_SLICE])
    if "waist" in group_names:
        values.extend(q[WAIST_SLICE])
    elif "upper_waist" in group_names:
        values.extend(q[UPPER_WAIST_SLICE])
    state = np.asarray(values, dtype=np.float32)
    if state.size != schema.size:
        raise AssertionError(f"Schema conversion produced {state.size}, expected {schema.size}.")
    return state


def whole_body_from_schema_action(
    action: np.ndarray,
    measured_q23: np.ndarray,
    schema: RobotSchema,
) -> np.ndarray:
    """Apply a schema-ordered action onto measured q23 telemetry.

    Groups not enabled in ``schema`` remain at their measured positions, which
    prevents an arm-only policy from moving the head or waist.
    """

    values = np.asarray(action, dtype=np.float32).reshape(-1)
    if values.size < schema.size:
        raise ValueError(f"Policy returned {values.size} dims, expected {schema.size}.")
    q = np.asarray(measured_q23, dtype=np.float32).reshape(-1)
    if q.size < WHOLE_BODY_DIM:
        raise ValueError(f"Expected {WHOLE_BODY_DIM} physical joints, got {q.size}.")
    q = q[:WHOLE_BODY_DIM].copy()

    q[LEFT_ARM_SLICE] = values[: LEFT_ARM.size]
    q[RIGHT_ARM_SLICE] = values[LEFT_ARM.size : LEFT_ARM.size + RIGHT_ARM.size]
    q[LEFT_GRIPPER_INDEX] = values[LEFT_ARM.size + RIGHT_ARM.size]
    q[RIGHT_GRIPPER_INDEX] = values[LEFT_ARM.size + RIGHT_ARM.size + 1]

    offset = sum(group.size for group in BASE_GROUPS)
    group_names = {group.name for group in schema.groups}
    if "head" in group_names:
        q[HEAD_SLICE] = values[offset : offset + HEAD.size]
        offset += HEAD.size
    if "waist" in group_names:
        q[WAIST_SLICE] = values[offset : offset + WAIST.size]
    elif "upper_waist" in group_names:
        q[UPPER_WAIST_SLICE] = values[offset : offset + UPPER_WAIST.size]
    return q
