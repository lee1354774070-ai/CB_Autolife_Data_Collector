#!/usr/bin/env python3
"""Control and inspect a running Autolife LeRobot collection session.

The recorder runs in the background, so keyboard commands need a small IPC
layer.  Commands travel through a named pipe and completed save/discard results
return through an atomically replaced JSON status file.  Keeping that protocol
here avoids duplicating Python snippets across the launcher and helper scripts.

This module intentionally uses only the Python standard library.  It can run
with the system ``python3`` even when ROS and LeRobot conda environments are not
active.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class SessionPaths:
    """Files shared by the launcher, recorder, and control commands."""

    base_dir: Path

    @property
    def pidfile(self) -> Path:
        return self.base_dir / ".official_recording_pids"

    @property
    def control_fifo(self) -> Path:
        return self.base_dir / ".official_recording_control"

    @property
    def status_file(self) -> Path:
        return self.base_dir / ".official_recording_status.json"


@dataclass
class QuantizedDeltaStats:
    """Track p95 and maximum deltas without retaining every frame value.

    Synchronization logs can contain millions of frames.  Values are counted in
    0.01 ms buckets, matching the report's displayed precision, so memory usage
    depends on the delta range instead of the dataset length.
    """

    bins: Counter[int] = field(default_factory=Counter)
    count: int = 0
    maximum: float = 0.0

    def add(self, value: float) -> None:
        value = abs(value)
        self.bins[round(value * 100.0)] += 1
        self.count += 1
        self.maximum = max(self.maximum, value)

    def percentile(self, fraction: float) -> float:
        target = max(1, math.ceil(self.count * fraction))
        seen = 0
        for bucket, count in sorted(self.bins.items()):
            seen += count
            if seen >= target:
                return bucket / 100.0
        return self.maximum


def read_recorder_pid(pidfile: Path) -> int:
    """Return the recorder PID from the launcher's three-column PID file."""

    if not pidfile.is_file():
        raise RuntimeError(f"PID file not found: {pidfile}")
    for line in pidfile.read_text(encoding="utf-8").splitlines():
        fields = line.split(maxsplit=2)
        if len(fields) >= 2 and fields[0] == "recorder":
            try:
                return int(fields[1])
            except ValueError as exc:
                raise RuntimeError(f"Invalid recorder PID in {pidfile}: {fields[1]!r}") from exc
    raise RuntimeError(f"Recorder entry not found in PID file: {pidfile}")


def process_is_running(pid: int) -> bool:
    """Check process existence without sending a state-changing signal."""

    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def send_fifo_command(fifo_path: Path, command: str, request_id: str | None = None) -> None:
    """Write one non-blocking command line to the recorder FIFO."""

    if not fifo_path.exists():
        raise RuntimeError(f"Control FIFO not found: {fifo_path}")
    line = command + (f" {request_id}" if request_id else "") + "\n"
    try:
        fd = os.open(fifo_path, os.O_WRONLY | os.O_NONBLOCK)
    except OSError as exc:
        raise RuntimeError(f"Could not open control FIFO {fifo_path}: {exc}") from exc
    try:
        os.write(fd, line.encode("utf-8"))
    finally:
        os.close(fd)


def wait_for_status(
    status_path: Path,
    request_id: str,
    recorder_pid: int,
    timeout_sec: float,
) -> dict[str, Any]:
    """Wait for the matching atomic acknowledgement from the recorder."""

    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        try:
            status = json.loads(status_path.read_text(encoding="utf-8"))
            if status.get("request_id") == request_id:
                return status
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            pass
        if not process_is_running(recorder_pid):
            raise RuntimeError("Recorder exited before acknowledging the command")
        time.sleep(0.1)
    raise TimeoutError(f"No recorder acknowledgement within {timeout_sec:g}s; check {status_path}")


def wait_for_ready_file(ready_path: Path, process_pid: int, timeout_sec: float) -> None:
    """Wait for an atomic recorder readiness marker while monitoring its PID."""

    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if ready_path.is_file():
            return
        if not process_is_running(process_pid):
            raise RuntimeError(f"Recorder exited before creating readiness file: {ready_path}")
        time.sleep(0.1)
    raise TimeoutError(f"Recorder did not become state-ready within {timeout_sec:g}s; check {ready_path}")


def format_status(status: dict[str, Any]) -> str:
    """Render a save/discard acknowledgement as concise English terminal text."""

    event = str(status.get("event", "operation"))
    success = bool(status.get("success"))
    task = str(status.get("task", "unknown"))
    frames = int(status.get("frames", 0))
    episode_index = status.get("episode_index")

    if event == "save" and success:
        lines = [f"[SAVED] Episode {episode_index} | {frames} frames | Task: {task}"]
    elif event == "discard" and success:
        lines = [f"[DISCARDED] {frames} frames | Task: {task} | Saved counts unchanged"]
        message = str(status.get("message", ""))
        if message.startswith("invalid episode discarded:"):
            lines.append(f"Reason: {message.removeprefix('invalid episode discarded:').strip()}")
    else:
        action = event.upper()
        lines = [f"[{action} NOT COMPLETED] {status.get('message', 'Unknown error')} | Task: {task}"]

    lines.append("Task episode counts:")
    for item in status.get("task_counts", []):
        lines.append(
            f"  - {item['task']}: total {item['total']}, "
            f"added this session {item['session_added']}, "
            f"discarded this session {item['session_discarded']}"
        )
    lines.append(
        f"Session totals: saved {status.get('session_saved_episodes', 0)}, "
        f"discarded {status.get('session_discarded_episodes', 0)}"
    )
    return "\n".join(lines)


def run_command(base_dir: Path, command: str, wait: bool, timeout_sec: float) -> dict[str, Any] | None:
    """Validate a session, send a command, and optionally await its result."""

    paths = SessionPaths(base_dir)
    recorder_pid = read_recorder_pid(paths.pidfile)
    if not process_is_running(recorder_pid):
        raise RuntimeError(f"Recorder is not running (PID {recorder_pid})")

    request_id = f"{os.getpid()}-{time.time_ns()}" if wait else None
    if wait:
        paths.status_file.unlink(missing_ok=True)
    send_fifo_command(paths.control_fifo, command, request_id)
    if not wait or request_id is None:
        return None
    return wait_for_status(paths.status_file, request_id, recorder_pid, timeout_sec)


def format_dataset_summary(dataset_root: Path) -> str:
    """Read the small ``info.json`` file and format a shutdown summary."""

    info_path = dataset_root / "meta" / "info.json"
    if not info_path.is_file():
        return f"Dataset metadata not found: {info_path}"
    info = json.loads(info_path.read_text(encoding="utf-8"))
    lines = [
        f"Dataset: {dataset_root}",
        f"Frames: {info.get('total_frames', 0)}",
        f"Episodes: {info.get('total_episodes', 0)}",
        f"FPS: {info.get('fps', 'unknown')}",
        "Recorded features:",
    ]
    for key, value in info.get("features", {}).items():
        if key.startswith("observation.images") or key in ("observation.state", "action"):
            lines.append(f"  - {key}: {value.get('shape')}")
    return "\n".join(lines)


def read_active_cameras(dataset_root: Path, recorder_log: Path) -> list[str]:
    """Return active camera names from metadata or the recorder startup log."""

    info_path = dataset_root / "meta" / "info.json"
    try:
        info = json.loads(info_path.read_text(encoding="utf-8"))
        cameras = sorted(
            key.removeprefix("observation.images.")
            for key in info.get("features", {})
            if key.startswith("observation.images.")
        )
        if cameras:
            return cameras
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        pass

    try:
        for line in reversed(recorder_log.read_text(encoding="utf-8", errors="replace").splitlines()):
            marker = "active cameras:"
            if marker in line:
                return [name.strip() for name in line.split(marker, 1)[1].split(",") if name.strip()]
    except OSError:
        pass
    return []


def wait_for_active_cameras(
    dataset_root: Path,
    recorder_log: Path,
    timeout_sec: float,
    recorder_pid: int | None = None,
) -> list[str]:
    """Poll until dataset metadata lists cameras or the recorder exits."""

    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        cameras = read_active_cameras(dataset_root, recorder_log)
        if cameras:
            return cameras
        if recorder_pid is not None and not process_is_running(recorder_pid):
            raise RuntimeError("Recorder exited before camera initialization completed")
        time.sleep(0.5)
    return []


def format_sync_report(sync_log: Path) -> str:
    """Summarize accepted-frame deltas, waits, and invalid episodes."""

    if not sync_log.is_file():
        return f"Synchronization log not found: {sync_log}"

    accepted_frames = 0
    waiting_ticks: Counter[str] = Counter()
    invalidated_episodes: Counter[str] = Counter()
    deltas_by_source: defaultdict[str, QuantizedDeltaStats] = defaultdict(QuantizedDeltaStats)
    # Stream the JSONL file instead of loading it as one large string; long
    # multi-task datasets can accumulate millions of synchronization events.
    with sync_log.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            if event.get("event") == "frame":
                accepted_frames += 1
                for source, delta in event.get("sync_deltas_ms", {}).items():
                    if isinstance(delta, (int, float)) and not isinstance(delta, bool):
                        deltas_by_source[source].add(float(delta))
            elif event.get("event") in {"wait", "drop"}:
                # ``drop`` is retained for reports from datasets created by
                # older collector versions. It represented a deferred timer
                # tick, not a discarded source frame.
                waiting_ticks[str(event.get("reason", "unknown"))] += 1
            elif event.get("event") == "episode_invalidated":
                invalidated_episodes[str(event.get("reason", "unknown"))] += 1

    lines = [
        "Synchronization report:",
        f"  accepted frames : {accepted_frames}",
        f"  waiting ticks   : {sum(waiting_ticks.values())}",
        f"  invalid episodes: {sum(invalidated_episodes.values())}",
    ]
    if deltas_by_source:
        lines.append("  absolute timestamp deltas:")
        for source, stats in sorted(deltas_by_source.items()):
            lines.append(
                f"    - {source}: p95={stats.percentile(0.95):.2f} ms, max={stats.maximum:.2f} ms"
            )
    if waiting_ticks:
        lines.append("  wait reasons:")
        for reason, count in waiting_ticks.most_common():
            lines.append(f"    - {reason}: {count}")
    if invalidated_episodes:
        lines.append("  episode invalidation reasons:")
        for reason, count in invalidated_episodes.most_common():
            lines.append(f"    - {reason}: {count}")
    return "\n".join(lines)


def consume_episode_event(event_path: Path) -> str:
    """Read, remove, and format one asynchronous recorder episode event."""

    event = json.loads(event_path.read_text(encoding="utf-8"))
    event_path.unlink(missing_ok=True)
    if event.get("event") == "episode_invalidated":
        return (
            f"[EPISODE INVALID] {event.get('buffered_frames', 0)} buffered frames | "
            f"Reason: {event.get('reason', 'unknown')}\n"
            "Recording paused. Press S or D to discard the entire episode."
        )
    return f"[RECORDER EVENT] {event}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="subcommand", required=True)

    command_parser = subparsers.add_parser("command", help="Send a command to a running recorder.")
    command_parser.add_argument("--base-dir", type=Path, required=True)
    command_parser.add_argument("--wait", action="store_true", help="Wait for save/discard completion.")
    command_parser.add_argument("--timeout", type=float, default=300.0)
    command_parser.add_argument("command", choices=("start", "save", "discard", "quit"))

    summary_parser = subparsers.add_parser("summary", help="Print dataset metadata summary.")
    summary_parser.add_argument("--dataset-root", type=Path, required=True)

    cameras_parser = subparsers.add_parser("cameras", help="Print cameras detected by the recorder.")
    cameras_parser.add_argument("--dataset-root", type=Path, required=True)
    cameras_parser.add_argument("--recorder-log", type=Path, required=True)
    cameras_parser.add_argument("--pid", type=int, default=None, help="Fail early if this recorder PID exits.")
    cameras_parser.add_argument("--timeout", type=float, default=15.0)

    sync_parser = subparsers.add_parser("sync-report", help="Summarize recorder synchronization quality.")
    sync_parser.add_argument("--sync-log", type=Path, required=True)

    ready_parser = subparsers.add_parser("wait-ready", help="Wait for a recorder readiness file.")
    ready_parser.add_argument("--path", type=Path, required=True)
    ready_parser.add_argument("--pid", type=int, required=True)
    ready_parser.add_argument("--timeout", type=float, default=10.0)

    event_parser = subparsers.add_parser("episode-event", help="Consume an asynchronous episode event.")
    event_parser.add_argument("--path", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    try:
        if args.subcommand == "summary":
            print(format_dataset_summary(args.dataset_root))
            return
        if args.subcommand == "cameras":
            cameras = wait_for_active_cameras(args.dataset_root, args.recorder_log, args.timeout, args.pid)
            if cameras:
                print(f"  recorded cameras  : {', '.join(cameras)}")
            else:
                raise RuntimeError(
                    f"No recorded camera metadata appeared within {args.timeout:g}s; "
                    f"check {args.recorder_log}"
                )
            return
        if args.subcommand == "sync-report":
            print(format_sync_report(args.sync_log))
            return
        if args.subcommand == "wait-ready":
            wait_for_ready_file(args.path, args.pid, args.timeout)
            print("  joint state       : ready and buffering")
            return
        if args.subcommand == "episode-event":
            print(consume_episode_event(args.path))
            return
        status = run_command(args.base_dir, args.command, args.wait, args.timeout)
        if status is not None:
            print(format_status(status))
            if not status.get("success"):
                raise SystemExit(1)
    except (RuntimeError, TimeoutError, OSError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}")
        raise SystemExit(1) from exc


if __name__ == "__main__":
    main()
