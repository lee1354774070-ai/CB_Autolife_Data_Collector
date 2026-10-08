"""Collector FIFO and intervention sidecar helpers."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import os
from pathlib import Path
import threading
import time
import uuid
from typing import Any, Deque, Dict, Optional


class TraceWriter:
    def __init__(self, root: str, ring_seconds: float = 5.0) -> None:
        self.root = Path(root).expanduser()
        self.ring_seconds = max(0.1, float(ring_seconds))
        self._ring: Deque[Dict[str, Any]] = deque()
        self._ring_lock = threading.Lock()
        self._directory: Optional[Path] = None
        self._files: Dict[str, Any] = {}

    def append_ring(self, record: Dict[str, Any]) -> None:
        with self._ring_lock:
            self._ring.append(record)
            cutoff = int(record["wall_ns"]) - int(self.ring_seconds * 1e9)
            while self._ring and int(self._ring[0]["wall_ns"]) < cutoff:
                self._ring.popleft()

    def prepare_root(self) -> Path:
        """Create the explicitly configured trace root before arming a session."""
        self.root.mkdir(parents=True, exist_ok=True)
        if not self.root.is_dir():
            raise RuntimeError(f"trace root is not a directory: {self.root}")
        return self.root

    def start(self, session_id: str, intervention_id: str, depth: bool, metadata: Dict[str, Any]) -> Path:
        with self._ring_lock:
            ring_snapshot = tuple(self._ring)
        directory = self.root / session_id / "interventions" / intervention_id
        directory.mkdir(parents=True, exist_ok=False)
        manifest = dict(metadata)
        manifest.update({
            "session_id": session_id,
            "intervention_id": intervention_id,
            "depth_enabled": bool(depth),
            "started_wall_ns": time.time_ns(),
            "status": "active",
        })
        self._write_json(directory / "manifest.json", manifest)
        self._directory = directory
        self._files["events"] = (directory / "events.jsonl").open("a", encoding="utf-8")
        self._files["vr"] = (directory / "vr_trace.jsonl").open("a", encoding="utf-8")
        self._files["policy"] = (directory / "policy_trace.jsonl").open("a", encoding="utf-8")
        with (directory / "pre_failure_trace.jsonl").open("w", encoding="utf-8") as handle:
            for record in ring_snapshot:
                handle.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
        return directory

    def write(self, stream: str, record: Dict[str, Any]) -> None:
        handle = self._files.get(stream)
        if handle is not None:
            handle.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
            handle.flush()

    def finish(
        self,
        status: str,
        frame_count: int,
        reason: str,
        collector_result: Optional[Dict[str, Any]] = None,
    ) -> None:
        if self._directory is None:
            return
        manifest_path = self._directory / "manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest.update({
            "status": status,
            "frame_count": int(frame_count),
            "finished_wall_ns": time.time_ns(),
            "finish_reason": reason,
        })
        if collector_result is not None:
            manifest["collector_result"] = dict(collector_result)
        self._write_json(manifest_path, manifest)
        for handle in self._files.values():
            handle.close()
        self._files.clear()
        self._directory = None

    @staticmethod
    def _write_json(path: Path, data: Dict[str, Any]) -> None:
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        os.replace(temporary, path)


@dataclass(frozen=True)
class CollectorCommandResult:
    acknowledged: bool
    success: bool
    event: str
    request_id: str
    message: str = ""
    episode_index: Optional[int] = None
    frames: int = 0
    expert_frames: int = 0

    def as_dict(self) -> Dict[str, Any]:
        return {
            "acknowledged": self.acknowledged,
            "success": self.success,
            "event": self.event,
            "request_id": self.request_id,
            "message": self.message,
            "episode_index": self.episode_index,
            "frames": self.frames,
            "expert_frames": self.expert_frames,
        }


class CollectorFifo:
    """Non-blocking best-effort writer for the collector command FIFO."""

    def __init__(self, rgb_fifo: str, rgbd_fifo: str) -> None:
        self.paths = {False: rgb_fifo, True: rgbd_fifo}

    def start_with_recovery(self, depth: bool, command: str, discard: str,
                            timeout: float) -> CollectorCommandResult:
        result = self.command_and_wait(depth, command, timeout)
        # Only the recorder's explicit invalid-episode latch is recoverable.
        # Never discard a live/valid episode or retry an ambiguous timeout.
        if (result.acknowledged and not result.success and result.event == "start"
                and result.message == "invalid episode must be discarded before starting another episode"):
            cleanup = self.command_and_wait(depth, discard, timeout)
            if not (cleanup.acknowledged and cleanup.success and cleanup.event == "discard"):
                return cleanup
            return self.command_and_wait(depth, command, timeout)
        return result

    def progress(self, depth: bool, trial_id: str) -> Dict[str, Any]:
        try:
            path = Path(self.paths[bool(depth)]).parent / ".dagger_progress.json"
            data = json.loads(path.read_text(encoding="utf-8"))
            if isinstance(data, dict) and data.get("trial_id") == trial_id:
                return data
        except (OSError, ValueError):
            pass
        return {}

    def invalid_episode_event(self, depth: bool, since_wall_ns: int) -> Dict[str, Any]:
        """Return only an invalidation from the current trial, never an older one."""
        if since_wall_ns <= 0:
            return {}
        path = Path(self.paths[bool(depth)]).parent / ".official_episode_event.json"
        try:
            event = json.loads(path.read_text(encoding="utf-8"))
            if (isinstance(event, dict)
                    and event.get("event") == "episode_invalidated"
                    and float(event.get("wall_time", 0)) * 1e9 >= since_wall_ns):
                return event
        except (OSError, ValueError, TypeError):
            pass
        return {}

    def command(
        self, depth: bool, command: str, request_id: Optional[str] = None
    ) -> bool:
        path = self.paths[bool(depth)]
        if not path or not os.path.exists(path):
            return False
        try:
            fd = os.open(path, os.O_WRONLY | os.O_NONBLOCK)
            try:
                line = command.rstrip()
                if request_id:
                    line += f" {request_id}"
                os.write(fd, (line + "\n").encode("utf-8"))
            finally:
                os.close(fd)
            return True
        except OSError:
            return False

    def command_and_wait(
        self, depth: bool, command: str, timeout: float = 3.0
    ) -> CollectorCommandResult:
        """Send a correlated command and require the collector's file ACK."""
        request_id = uuid.uuid4().hex
        if not self.command(depth, command, request_id):
            return CollectorCommandResult(
                False, False, command, request_id,
                "collector FIFO is unavailable or has no reader",
            )
        return self.wait_for_result(depth, command, request_id, timeout)

    def wait_for_result(self, depth: bool, command: str, request_id: str,
                        timeout: float = 3.0) -> CollectorCommandResult:
        """Reconcile an uncertain command without sending it a second time."""
        status_path = Path(self.paths[bool(depth)]).parent / ".official_recording_status.json"
        deadline = time.monotonic() + max(0.1, float(timeout))
        while time.monotonic() < deadline:
            try:
                payload = json.loads(status_path.read_text(encoding="utf-8"))
            except (FileNotFoundError, OSError, json.JSONDecodeError):
                time.sleep(0.02)
                continue
            if (
                isinstance(payload, dict)
                and payload.get("request_id") == request_id
            ):
                episode_index = payload.get("episode_index")
                return CollectorCommandResult(
                    True,
                    payload.get("success") is True,
                    str(payload.get("event", command)),
                    request_id,
                    str(payload.get("message", "")),
                    int(episode_index) if episode_index is not None else None,
                    int(payload.get("frames", 0) or 0),
                    int(payload.get("expert_frames", 0) or 0),
                )
            time.sleep(0.02)
        return CollectorCommandResult(
            False, False, command, request_id,
            f"collector did not acknowledge {command!r} within {timeout:.1f}s",
        )
