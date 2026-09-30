"""Causal HG-DAgger provenance, independent of ROS and the policy framework.

The policy prefix is context, NOT an expert demonstration. Even after takeover,
an arm/gripper command from an earlier authority epoch must never become a
training target merely because a proxy republishes it with a newer timestamp.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math


FIELDS = ("control_source", "is_intervention", "intervention_id", "train_mask",
          "authority_epoch", "anchor_timestamp_ns", "arm_command_timestamp_ns",
          "gripper_command_timestamp_ns")
FEATURES = {f"dagger.{key}": {"dtype": "int64", "shape": (1,), "names": [key]} for key in FIELDS}


def timestamp_ns(value: object) -> int:
    # Refuse seconds, bools, strings, NaN, and implausible wall-clock stamps.
    if isinstance(value, bool) or not isinstance(value, int) or not 10**17 <= value < 2**63:
        raise ValueError("DAgger requires explicit Unix nanosecond timestamps")
    return value


@dataclass(frozen=True)
class ActionOrigin:
    effective_ns: int
    original_ns: int
    epoch: int


class DaggerLabels:
    def __init__(self, capacity: int = 64):
        self.labels: deque[dict] = deque(maxlen=capacity)
        self.arms: deque[ActionOrigin] = deque(maxlen=capacity)
        self.grippers: deque[ActionOrigin] = deque(maxlen=capacity)
        self.expert_frames = 0
        self.last_expert_frames = 0
        self.started_ns = 0
        self.ready = False
        self.trial_id = ""

    def reset(self) -> None:
        self.last_expert_frames = self.expert_frames
        self.expert_frames = 0
        self.ready = False
        self.trial_id = ""

    def ingest(self, packet: dict, *, gripper: bool = False) -> int:
        stamp = timestamp_ns(packet.get("timestamp_ns"))
        original = timestamp_ns(packet.get("original_command_timestamp_ns", stamp))
        epoch = packet.get("authority_epoch")
        if type(epoch) is not int or epoch < 0 or original > stamp:
            raise ValueError("invalid DAgger command origin")
        label = None if gripper else packet.get("dagger")
        if not gripper:
            if not isinstance(label, dict):
                raise ValueError("missing DAgger label")
            timestamp_ns(label.get("timestamp_ns"))
            boundary = timestamp_ns(label.get("authority_timestamp_ns"))
            if boundary > label["timestamp_ns"] or label.get("authority_epoch") != epoch:
                raise ValueError("invalid DAgger authority timestamp/epoch")
            if (type(label.get("control_source")) is not int or label["control_source"] not in (0, 1, 2)
                    or type(label.get("is_intervention")) is not int or label["is_intervention"] not in (0, 1)):
                raise ValueError("invalid DAgger source")
            if type(label.get("intervention_id")) is not int or label["intervention_id"] < 0:
                raise ValueError("invalid DAgger intervention id")
            if not isinstance(label.get("trial_id"), str) or not label["trial_id"]:
                raise ValueError("missing DAgger trial id")
        origin_epoch = packet.get("origin_authority_epoch", epoch)
        if type(origin_epoch) is not int or origin_epoch < -1:
            raise ValueError("invalid DAgger origin epoch")
        origin = ActionOrigin(stamp, original, origin_epoch)
        (self.grippers if gripper else self.arms).append(origin)
        if label is not None:
            self.labels.append(dict(label))
        return stamp

    def frame(self, anchor_sec: float, deltas_ms: dict[str, float], max_age_sec: float) -> dict[str, int]:
        anchor = round(anchor_sec * 1e9)
        label = max((x for x in self.labels if self.started_ns <= x["timestamp_ns"] <= anchor),
                    key=lambda x: x["timestamp_ns"], default=None)
        if label is None or anchor - label["timestamp_ns"] > max_age_sec * 1e9:
            raise ValueError("missing or stale DAgger label")
        if self.trial_id and label["trial_id"] != self.trial_id:
            raise ValueError("DAgger trial changed while recording")
        self.trial_id = label["trial_id"]
        origins = []
        for key, samples in (("action_body", self.arms), ("action_gripper", self.grippers)):
            delta = deltas_ms.get(key)
            if delta is None or not math.isfinite(delta) or delta > 0:
                raise ValueError("DAgger requires causal arm AND gripper targets")
            selected_ns = round((anchor_sec + delta / 1000) * 1e9)
            # The recorder currently stores seconds as float64; permit only its
            # sub-microsecond roundoff, not a different command generation.
            origin = next((x for x in reversed(samples) if abs(x.effective_ns - selected_ns) <= 512), None)
            if origin is None:
                raise ValueError("DAgger action origin was evicted")
            origins.append(origin)
        expert = (label["control_source"] == 1 and label["is_intervention"] == 1
                  and label["intervention_id"] > 0 and all(
                      x.epoch == label["authority_epoch"]
                      and label["authority_timestamp_ns"] <= x.original_ns <= anchor
                      for x in origins))
        values = {key: int(label[key]) for key in
                  ("control_source", "is_intervention", "intervention_id", "authority_epoch")}
        values.update(train_mask=int(expert), anchor_timestamp_ns=anchor,
                      arm_command_timestamp_ns=origins[0].original_ns,
                      gripper_command_timestamp_ns=origins[1].original_ns)
        return {f"dagger.{key}": value for key, value in values.items()}


def validate_features(features: dict, enabled: bool) -> None:
    present = {key for key in features if key.startswith("dagger.")}
    if present != (set(FEATURES) if enabled else set()):
        raise ValueError("DAgger schema differs; use a separate dataset directory")
    if enabled:
        for key in FEATURES:
            if features[key].get("dtype") != "int64" or list(features[key].get("shape", [])) != [1]:
                raise ValueError(f"incompatible DAgger feature: {key}")
