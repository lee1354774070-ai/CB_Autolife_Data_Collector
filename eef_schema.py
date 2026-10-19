"""EEF telemetry and explicit joint/pose column layouts, without ROS imports.

The vendor's current/target_robot_eef_pose reports contain absolute poses in
the controller's robot frame: metres and quaternion XYZW. They are outputs,
not motion commands. No IK, FK, zero filling, or state-as-action fallback is
performed here. The two streams must be provided by the robot controller.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from time_sync import linear_interpolation_alpha, surrounding_samples

EEF_NAMES = tuple(f"{side}_eef_{axis}" for side in ("left", "right")
                  for axis in ("x", "y", "z", "qx", "qy", "qz", "qw"))
SCHEMA_PATH = "meta/collector_schema.json"
VECTOR_KEYS = ("observation.state", "action")
SELECTIONS = ("joint", "eef", "all", "eef+gripper")


def parse_eef_pose(payload: Any) -> np.ndarray | None:
    """Accept complete dual-arm vendor reports only; normalize valid rotations.

    Reject partial, nonfinite, and near-zero quaternion packets. Silently
    filling a missing arm from another generation would mislabel the sample.
    Commands with pos_*_in_robot fields intentionally are NOT this protocol.
    """
    if not isinstance(payload, dict):
        return None
    values = []
    for side in ("left", "right"):
        pose = payload.get(f"{side}_eef_pose")
        if not isinstance(pose, dict):
            return None
        for key, size in (("position", 3), ("rotation", 4)):
            raw = pose.get(key)
            if (not isinstance(raw, (list, tuple)) or len(raw) != size
                    or any(type(value) not in (int, float) for value in raw)):
                return None
            vector = np.asarray(raw, dtype=np.float64)
            if not np.isfinite(vector).all():
                return None
            if key == "rotation":
                norm = np.linalg.norm(vector)
                if not np.isfinite(norm) or norm < 1e-8:
                    return None
                vector /= norm
                # q and -q denote the same rotation. A deterministic hemisphere
                # prevents sign flips between equivalent incoming packets.
                pivot = 3 if abs(vector[3]) > 1e-8 else int(np.argmax(np.abs(vector)))
                if vector[pivot] < 0:
                    vector = -vector
            values.extend(vector)
    result = np.asarray(values, dtype=np.float32)
    return result if np.isfinite(result).all() else None


def interpolate_poses(before: np.ndarray, after: np.ndarray, alpha: float) -> np.ndarray:
    """Linear XYZ and shortest-arc SLERP for each unit quaternion.

    Linear interpolation of quaternion components is not a rotation-preserving
    operation. The near-parallel branch uses normalized lerp to avoid 0/0.
    """
    result = np.empty(len(EEF_NAMES), dtype=np.float32)
    for start in (0, 7):
        result[start:start + 3] = before[start:start + 3] * (1 - alpha) + after[start:start + 3] * alpha
        left = before[start + 3:start + 7].astype(np.float64)
        right = after[start + 3:start + 7].astype(np.float64)
        dot = float(np.dot(left, right))
        if dot < 0:
            right, dot = -right, -dot
        dot = min(1.0, dot)
        if dot > .9995:
            quat = left * (1 - alpha) + right * alpha
        else:
            angle = np.arccos(dot)
            quat = (np.sin((1 - alpha) * angle) * left + np.sin(alpha * angle) * right) / np.sin(angle)
        result[start + 3:start + 7] = quat / np.linalg.norm(quat)
    return result


class EefSyncError(ValueError):
    def __init__(self, reason: str, *, wait: bool = False):
        super().__init__(reason)
        self.wait = wait


def synchronized_pose(samples, anchor: float, now: float, *, max_gap: float, max_age: float):
    """Match an EEF state to the same image anchor as joints, without blocking.

    A not-yet-arrived right endpoint asks the recording loop to retain its
    reference image and retry. Missing history, a stalled stream, or an overly
    wide bracket invalidates the episode instead of silently skipping frames.
    """
    before, after = surrounding_samples(samples, anchor)
    if before is None:
        raise EefSyncError("eef_state_before_missing")
    if now - before.received_sec > max_age or (after is not None and now - after.received_sec > max_age):
        raise EefSyncError("eef_state_stale")
    if after is None:
        raise EefSyncError("eef_state_after_not_ready", wait=True)
    if after.stamp_sec - before.stamp_sec > max_gap:
        raise EefSyncError("eef_state_interpolation_gap")
    alpha = linear_interpolation_alpha(before.stamp_sec, after.stamp_sec, anchor)
    if alpha is None:
        raise EefSyncError("eef_state_interpolation_order")
    return (
        interpolate_poses(before.value, after.value, alpha),
        {f"eef_state_{key}": (sample.stamp_sec - anchor) * 1000 for key, sample in (("before", before), ("after", after))},
        {f"eef_state_{key}": (now - sample.received_sec) * 1000 for key, sample in (("before", before), ("after", after))},
    )


def capture_schema(joint_names, *, with_state=False, with_action=False,
                   state_topic="", action_topic="", action_mode="status_target") -> dict:
    """Describe array indices explicitly so conversion never guesses by width."""
    features = {}
    for key, enabled, topic in ((VECTOR_KEYS[0], with_state, state_topic), (VECTOR_KEYS[1], with_action, action_topic)):
        names = list(joint_names) + (list(EEF_NAMES) if enabled else [])
        feature = dict(names=names, groups={
            "joint": list(range(len(joint_names))),
            "gripper": [index for index, name in enumerate(joint_names) if name in ("left_gripper", "right_gripper")],
            "eef": list(range(len(joint_names), len(names))),
        })
        if enabled:
            feature["eef"] = dict(frame="robot", position_unit="m", rotation="quaternion_xyzw",
                                  source="controller_current_pose" if key == VECTOR_KEYS[0] else "controller_target_pose",
                                  topic=topic)
        features[key] = feature
    return dict(version=1, action_mode=action_mode, features=features)


def validate_layout(info: dict, layout: dict) -> None:
    """Fail closed on missing, ambiguous or edited index manifests."""
    if layout.get("version") != 1:
        raise ValueError("Unsupported or missing collector schema version")
    for key in VECTOR_KEYS:
        entry = layout.get("features", {}).get(key, {})
        names = entry.get("names", [])
        feature = info.get("features", {}).get(key, {})
        if (not names or feature.get("names") != names or feature.get("dtype") != "float32"
                or list(feature.get("shape", [])) != [len(names)] or len(set(names)) != len(names)):
            raise ValueError(f"{key}: collector schema disagrees with meta/info.json")
        groups = entry.get("groups", {})
        if set(groups) != {"joint", "eef", "gripper"}:
            raise ValueError(f"{key}: invalid column groups")
        for indices in groups.values():
            if (not isinstance(indices, list) or any(type(i) is not int or not 0 <= i < len(names) for i in indices)
                    or sorted(set(indices)) != indices):
                raise ValueError(f"{key}: invalid column indices")
        if sorted(groups["joint"] + groups["eef"]) != list(range(len(names))):
            raise ValueError(f"{key}: joint/EEF groups must partition the vector")
        if not set(groups["gripper"]).issubset(groups["joint"]):
            raise ValueError(f"{key}: grippers must belong to the joint group")
        expected_grippers = [i for i in groups["joint"] if names[i] in ("left_gripper", "right_gripper")]
        if groups["gripper"] != expected_grippers:
            raise ValueError(f"{key}: invalid gripper mapping")
        if groups["eef"]:
            description = entry.get("eef", {})
            if ([names[i] for i in groups["eef"]] != list(EEF_NAMES)
                    or description.get("rotation") != "quaternion_xyzw"
                    or description.get("position_unit") != "m" or description.get("frame") != "robot"):
                raise ValueError(f"{key}: unsupported EEF representation")


def select_indices(entry: dict, selection: str) -> list[int]:
    if selection not in SELECTIONS:
        raise ValueError(f"Unknown selection: {selection}")
    groups = entry["groups"]
    if selection == "all":
        return list(range(len(entry["names"])))
    required = ("eef", "gripper") if selection == "eef+gripper" else (selection,)
    if any(not groups[key] for key in required):
        raise ValueError(f"Selection {selection} requires data that was not recorded")
    if selection == "eef+gripper" and [entry["names"][i] for i in groups["gripper"]] != ["left_gripper", "right_gripper"]:
        raise ValueError("eef+gripper requires both recorded grippers in left/right order")
    return [index for key in required for index in groups[key]]
