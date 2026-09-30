"""ROS-independent GR00T N1.7 wire and robot-schema helpers."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from typing import Any, Mapping, Optional


EXPECTED_FEATURES = (
    "left_shoulder_inner", "left_shoulder_outer", "left_upper_arm",
    "left_elbow", "left_forearm", "left_wrist_upper", "left_wrist_lower",
    "right_shoulder_inner", "right_shoulder_outer", "right_upper_arm",
    "right_elbow", "right_forearm", "right_wrist_upper", "right_wrist_lower",
    "left_gripper", "right_gripper", "neck_roll", "neck_pitch", "neck_yaw",
    "waist_pitch", "waist_yaw",
)
EXPECTED_IMAGES = (
    "observation.images.rgbd_head_color",
    "observation.images.hand_left",
    "observation.images.hand_right",
)
RUN_MODES = {
    "POLICY_ACTIVE", "POLICY_WARMUP", "FAILURE_HOLD",
    "EXPERT_RELEASE_REQUIRED", "EXPERT_READY", "EXPERT_ACTIVE",
}
# Manual completion mode has no policy handback. Keep GR00T alive only while
# it owns authority; inference during expert collection wastes CPU and can
# delay the teleoperation controller.
INFERENCE_MODES = {"POLICY_ACTIVE"}


def bridge_generation_boundary(old_mode: str, new_mode: str) -> bool:
    """Only arm/disarm boundaries restart inference; authority swaps stay live."""
    return (old_mode in INFERENCE_MODES) != (new_mode in INFERENCE_MODES)


class GrootProtocolError(RuntimeError):
    pass


@dataclass(frozen=True)
class GrootContract:
    state_features: tuple[str, ...]
    action_features: tuple[str, ...]
    image_shapes: dict[str, tuple[int, int, int]]
    chunk_size: int
    n_action_steps: int

    @classmethod
    def from_mapping(cls, raw: Mapping[str, Any]) -> "GrootContract":
        try:
            result = cls(
                tuple(str(x) for x in raw["state_feature_names"]),
                tuple(str(x) for x in raw["action_feature_names"]),
                {
                    str(key): tuple(int(x) for x in shape)
                    for key, shape in raw["image_features"].items()
                },
                int(raw["chunk_size"]),
                int(raw["n_action_steps"]),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise GrootProtocolError(f"invalid deployment contract: {exc}") from exc
        result.validate()
        return result

    def validate(self) -> None:
        if self.state_features != EXPECTED_FEATURES:
            raise GrootProtocolError("GR00T state feature order is incompatible with AutoLife q23")
        if self.action_features != EXPECTED_FEATURES:
            raise GrootProtocolError("GR00T action feature order is incompatible with AutoLife q23")
        if tuple(self.image_shapes) != EXPECTED_IMAGES:
            raise GrootProtocolError("GR00T camera keys/order differ from the three-camera contract")
        if any(len(shape) != 3 or shape[0] != 3 for shape in self.image_shapes.values()):
            raise GrootProtocolError("GR00T bridge supports RGB CHW camera contracts only")
        if self.chunk_size < 1 or not 1 <= self.n_action_steps <= self.chunk_size:
            raise GrootProtocolError("invalid GR00T action horizon")


def _positions(payload: Mapping[str, Any], key: str, size: int) -> Optional[list[float]]:
    nested = payload.get(key)
    if not isinstance(nested, Mapping):
        return None
    raw = nested.get("position")
    if not isinstance(raw, (list, tuple)) or len(raw) < size:
        return None
    try:
        values = [float(x) for x in raw[:size]]
    except (TypeError, ValueError):
        return None
    return values if all(math.isfinite(x) for x in values) else None


def parse_q23(payload: Mapping[str, Any]) -> Optional[list[float]]:
    """Parse physical q23: waist4, left7, right7, grippers2, neck3."""
    groups = (
        ("leg_waist_joint_state", 4),
        ("left_arm_joint_state", 7),
        ("right_arm_joint_state", 7),
        ("left_gripper_state", 1),
        ("right_gripper_state", 1),
        ("neck_joint_state", 3),
    )
    result: list[float] = []
    for key, size in groups:
        values = _positions(payload, key, size)
        if values is None:
            return None
        result.extend(values)
    return result


def policy_state_from_q23(q23: list[float]) -> list[float]:
    if len(q23) != 23 or not all(math.isfinite(float(x)) for x in q23):
        raise ValueError("whole-body state must be finite q23")
    return [float(x) for x in (
        q23[4:18] + q23[18:20] + q23[20:23] + q23[2:4]
    )]


def controller_groups_from_policy(
    action: list[float], measured_q23: list[float]
) -> dict[str, list[float]]:
    if len(action) != 21 or not all(math.isfinite(float(x)) for x in action):
        raise ValueError("GR00T policy action must be finite 21-D")
    if len(measured_q23) != 23:
        raise ValueError("GR00T controller mapping requires measured q23")
    return {
        "left_arm_target_joints_position": [float(x) for x in action[0:7]],
        "right_arm_target_joints_position": [float(x) for x in action[7:14]],
        "neck_target_joints_position": [float(x) for x in action[16:19]],
        "leg_waist_target_joints_position": [
            float(measured_q23[0]), float(measured_q23[1]),
            float(action[19]), float(action[20]),
        ],
    }


def array_digest_float32(rows: list[list[float]]) -> str:
    """Match NumPy's little-endian float32 array_digest used by the server."""
    import struct

    if not rows or not rows[0] or any(len(row) != len(rows[0]) for row in rows):
        raise ValueError("action prefix must be a non-empty rectangular array")
    shape = [len(rows), len(rows[0])]
    header = json.dumps(
        {"shape": shape, "dtype": "<f4"}, sort_keys=True, separators=(",", ":")
    ).encode("ascii")
    body = b"".join(struct.pack("<f", float(value)) for row in rows for value in row)
    return "sha256:" + hashlib.sha256(header + b"\0" + body).hexdigest()


def validated_actions(proposal: Mapping[str, Any], contract: GrootContract) -> list[list[float]]:
    raw = proposal.get("executable_actions")
    if not isinstance(raw, list) or len(raw) != contract.n_action_steps:
        raise GrootProtocolError("remote executable chunk length differs from contract")
    result: list[list[float]] = []
    for row in raw:
        if not isinstance(row, list) or len(row) != len(EXPECTED_FEATURES):
            raise GrootProtocolError("remote action row differs from 21-D contract")
        values = [float(x) for x in row]
        if not all(math.isfinite(x) for x in values):
            raise GrootProtocolError("remote action contains non-finite values")
        # Round through float32 so JSON ACK bytes match the server's executable array.
        import struct
        result.append([struct.unpack("<f", struct.pack("<f", x))[0] for x in values])
    if array_digest_float32(result) != str(proposal.get("executable_chunk_digest", "")):
        raise GrootProtocolError("remote executable chunk digest does not match its actions")
    return result
