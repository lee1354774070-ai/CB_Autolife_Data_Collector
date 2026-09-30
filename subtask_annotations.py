"""Ordered, operator-confirmed subtask spans within a single LeRobot episode.

No ROS dependency: the launcher can validate configuration before starting any
process. Indices are local to each episode's plan, not global task_index values.
Only a confirmed span receives a label; unconfirmed frames remain -1.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path


SUBTASK_FEATURE = {"dtype": "int64", "shape": (1,), "names": ["subtask_index"]}


def parse_subtasks(value: str) -> tuple[str, ...]:
    try:
        plan = json.loads(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("Subtasks must be a JSON array of non-empty strings") from exc
    if not isinstance(plan, list) or any(not isinstance(s, str) or not s.strip() for s in plan):
        raise ValueError("Subtasks must be a JSON array of non-empty strings")
    # Repeated names are legal: the ordinal identifies a repeated step.
    return tuple(s.strip() for s in plan)


class SubtaskAnnotations:
    def __init__(self, plan: tuple[str, ...]):
        self.plan = plan
        self.reset()

    def reset(self) -> None:
        self.ends: list[int] = []

    @property
    def enabled(self) -> bool:
        return bool(self.plan)

    @property
    def complete(self) -> bool:
        return self.enabled and len(self.ends) == len(self.plan)

    def mark(self, frame_count: int) -> bool:
        """Confirm [previous_end, frame_count); reject empty/duplicate spans.

        frame_count is sampled by the recorder on command handling, after its
        completed add_frame calls. It is not the VR device's wall-clock time.
        """
        if not self.enabled or self.complete:
            raise ValueError("No unconfirmed subtask in the configured plan")
        if frame_count <= (self.ends[-1] if self.ends else 0):
            raise ValueError("No new recorded frames for this subtask; wait before marking")
        self.ends.append(frame_count)
        return self.complete

    def progress(self) -> dict:
        index = len(self.ends)
        return {"enabled": self.enabled, "confirmed": index, "total": len(self.plan),
                "complete": self.complete, "next_subtask": self.plan[index] if index < len(self.plan) else None}

    def manifest(self, episode_index: int, task: str, frames: int, fps: int, reason: str) -> dict:
        if frames <= 0 or fps <= 0 or (self.ends and self.ends[-1] > frames):
            raise ValueError("Annotation boundaries do not match the recorded episode")
        segments, start = [], 0
        for index, end in enumerate(self.ends):
            segments.append({"subtask_index": index, "text": self.plan[index],
                             "start_frame": start, "end_frame": end,
                             "start_sec": start / fps, "end_sec": end / fps})
            start = end
        complete = self.complete and start == frames
        return {"schema_version": 1, "episode_index": episode_index, "task": task,
                "fps": fps, "frames": frames, "subtasks": list(self.plan), "segments": segments,
                "unannotated_ranges": [{"start_frame": start, "end_frame": frames}] if start < frames else [],
                "annotation_complete": complete, "needs_review": not complete, "save_reason": reason,
                "boundary_basis": "recorder_accepted_frame_count", "interval_convention": "[start_frame,end_frame)"}

    def apply_to_buffer(self, dataset, frames: int) -> None:
        """Fill the custom column before LeRobot computes stats/writes Parquet.

        LeRobot builds expose either a separate writer or a dataset-owned
        buffer. Keep this narrow compatibility adapter here; fail rather than
        write misaligned labels.
        """
        import numpy as np

        owner = getattr(dataset, "writer", None) or dataset
        buffer = getattr(owner, "episode_buffer", None)
        if not isinstance(buffer, dict) or buffer.get("size") != frames:
            raise RuntimeError("Cannot access a matching LeRobot episode buffer for subtask labels")
        if len(buffer.get("subtask_index", [])) != frames:
            raise RuntimeError("LeRobot subtask_index buffer length differs from the recorded frame count")
        labels = np.full((frames, 1), -1, dtype=np.int64)
        start = 0
        for index, end in enumerate(self.ends):
            if not start < end <= frames:
                raise RuntimeError("Invalid subtask frame boundary")
            labels[start:end] = index
            start = end
        buffer["subtask_index"] = list(labels)


def check_pending_annotations(root: Path) -> None:
    pending = sorted((root / "annotations" / "subtasks").glob("*.pending.json"))
    if pending:
        raise RuntimeError(
            f"Unfinished episode save: {pending[0]}. Inspect dataset and annotation consistency "
            "before resuming; do not delete this recovery marker blindly."
        )


def prepare_annotation(root: Path, manifest: dict) -> tuple[Path, Path]:
    """Leave a durable recovery marker until LeRobot has accepted the episode.

    Two separate stores cannot be committed atomically. A crash/failure leaves
    *.pending.json and blocks resume instead of silently losing the annotation.
    This is a recovery aid, not a guarantee against power-loss data corruption.
    """
    directory = root / "annotations" / "subtasks"
    directory.mkdir(parents=True, exist_ok=True)
    stem = f"episode_{manifest['episode_index']:06d}"
    pending, final = directory / f"{stem}.pending.json", directory / f"{stem}.json"
    if final.exists():
        raise RuntimeError(f"Annotation already exists: {final}")
    with pending.open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    return pending, final


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Validate the ordered subtask JSON plan without ROS.")
    parser.add_argument("--subtasks-json", default="[]")
    parser.add_argument("--count", action="store_true", help="Print only the validated subtask count.")
    args = parser.parse_args()
    try:
        plan = parse_subtasks(args.subtasks_json)
    except ValueError as exc:
        parser.error(str(exc))
    if args.count:
        print(len(plan))
    elif plan:
        print("  subtasks    : " + " -> ".join(f"{i + 1}. {name}" for i, name in enumerate(plan)))
