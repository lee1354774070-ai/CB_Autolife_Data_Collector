#!/usr/bin/env python3
"""Temporarily restrict the vendor vision service to the four collection streams."""
from __future__ import annotations
import argparse
import base64
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import stat
import struct
import subprocess
import sys
import tempfile
import time

KEEP_MODULES = ("mod_camera_hand_left", "mod_camera_hand_right", "mod_camera_rgbd_head")
STREAMS = ("hand_left", "hand_right", "rgbd_head_color", "rgbd_head_depth")
ALL_STREAMS = STREAMS + ("head_left", "head_right", "head_rear")
SETTINGS = Path.home() / ".config/autolife_collector/camera_session.json"
STATE_DIR = Path.home() / ".local/state/autolife_collector/cameras"

def digest(data):
    return hashlib.sha256(data).hexdigest()

def atomic_write(path, data, mode=0o600):
    path = Path(path)
    fd, name = tempfile.mkstemp(prefix="." + path.name + ".", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as file:
            os.fchmod(file.fileno(), mode)
            file.write(data)
            file.flush()
            os.fsync(file.fileno())
        os.replace(name, path)
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        if os.path.exists(name):
            os.unlink(name)

def four_stream_config(raw):
    data = json.loads(raw)
    modules = data.get("ENABLED_MODULES")
    if not isinstance(modules, list) or not all(isinstance(x, str) for x in modules):
        raise RuntimeError("视觉配置缺少有效的 ENABLED_MODULES")
    if not set(KEEP_MODULES).issubset(modules):
        raise RuntimeError("当前配置没有完整的左右手和头部RGBD模块；拒绝猜测相机映射")
    data["ENABLED_MODULES"] = [
        name for name in modules if not name.startswith("mod_camera_") or name in KEEP_MODULES
    ]
    return (json.dumps(data, ensure_ascii=False, indent=2) + "\n").encode()

def stream_stamp(name):
    stamps = []
    for suffix in ("", "_jpeg"):
        path = Path("/dev/shm/camera_metadata_struct_" + name + suffix)
        try:
            with path.open("rb") as file:
                raw = file.read(128)
            if raw[:4] == b"SHM2":
                header = struct.unpack_from("<4sIIIQQIIIIQIIQQQ", raw)
                stamp = header[15]
            else:
                stamp = struct.unpack_from("<qiiiii", raw)[0]
            if stamp > 0:
                stamps.append(stamp)
        except (OSError, struct.error):
            pass
    return max(stamps, default=0)

def recorder_pids():
    found = []
    for path in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            args = path.read_bytes().split(b"\0")
            if any(Path(arg.decode(errors="replace")).name == "record_lerobot_official.py"
                   for arg in args if arg):
                found.append(int(path.parent.name))
        except OSError:
            pass
    return found

class CameraSession:
    def __init__(self, settings=SETTINGS, state_dir=STATE_DIR):
        self.settings = Path(settings)
        self.state_dir = Path(state_dir)
        self.journal = self.state_dir / "pending.json"
        self.lock = None
        self.config = None
        self.service = None
        self.retain = False

    def _load(self):
        settings = json.loads(self.settings.read_text())
        self.config = Path(settings["vision_config"]).resolve(strict=True)
        self.retain = settings.get("retain_between_batches", False)
        if not isinstance(self.retain, bool):
            raise ValueError("retain_between_batches must be boolean")
        self.service = settings.get("service", "vision-service.service")
        if self.service != "vision-service.service":
            raise RuntimeError("相机管理只允许操作 vision-service.service")

    def _lock(self):
        self.state_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.lock = (self.state_dir / "session.lock").open("a")
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.lock.close()
            self.lock = None
            raise RuntimeError("其他采集会话正在管理相机；请先正常退出") from None

    def _idle(self):
        pids = recorder_pids()
        if pids:
            raise RuntimeError(f"录制进程仍存活 {pids}，暂不切换相机；请等待其退出后运行 recover")

    def _service_active(self):
        result = subprocess.run(["systemctl", "--user", "is-active", "--quiet", self.service])
        if result.returncode:
            raise RuntimeError("视觉服务未运行，拒绝自动启动；请检查原厂视觉服务")

    def _restart(self, streams):
        started = time.time_ns()
        subprocess.run(["systemctl", "--user", "restart", self.service], check=True, timeout=60)
        deadline = time.monotonic() + 90
        first = {}
        while time.monotonic() < deadline:
            now = time.time_ns()
            stamps = {name: stream_stamp(name) for name in streams}
            for name, stamp in stamps.items():
                if stamp >= started and now - stamp < 2_000_000_000:
                    first.setdefault(name, stamp)
            if all(name in first and stamps[name] > first[name] and
                   0 <= now - stamps[name] < 2_000_000_000 for name in streams):
                self._service_active()
                return
            time.sleep(0.25)
        raise RuntimeError("视觉服务切换后相机没有恢复持续出帧：" +
                           ", ".join(name for name in streams
                                     if time.time_ns() - stream_stamp(name) > 2_000_000_000))

    def _restore(self):
        if not self.journal.exists():
            return
        self._idle()
        state = json.loads(self.journal.read_text())
        config = Path(state["config"])
        if config.resolve() != self.config:
            raise RuntimeError("待恢复配置路径与当前设置不同；保留恢复记录，需人工检查")
        original = base64.b64decode(state["original"])
        current = config.read_bytes()
        if digest(current) not in (state["selected_sha256"], digest(original)):
            raise RuntimeError("相机配置被其他人修改，拒绝覆盖；原配置仍保存在 pending.json")
        print("相机模式：正在恢复采集前配置，画面会短暂重连…", flush=True)
        atomic_write(config, original, state["mode"])
        self._restart(state["previous_live_streams"])
        self.journal.unlink()
        print("相机模式：采集前配置已恢复，已确认原有活动画面恢复出帧。", flush=True)

    def acquire(self):
        if not self.settings.exists():
            return
        self._load()
        self._lock()
        # Reuse only our exact selected configuration; never overwrite an external edit.
        if self.retain and self.journal.exists():
            try:
                self._idle()
                state = json.loads(self.journal.read_text())
                if Path(state["config"]).resolve() != self.config or digest(self.config.read_bytes()) != state["selected_sha256"]:
                    raise RuntimeError("保留的四路配置已变化；请检查后显式 recover")
                self._service_active()
                first = {name: stream_stamp(name) for name in STREAMS}
                time.sleep(0.25)
                stamps = {name: stream_stamp(name) for name in STREAMS}
                now = time.time_ns()
                if not all(stamps[n] > first[n] and 0 <= now - stamps[n] < 2_000_000_000 for n in STREAMS):
                    raise RuntimeError("保留的相机没有持续出帧；未自动重启，请检查视觉服务")
                print("相机模式：复用四路相机，不重启视觉服务。", flush=True)
                return
            except BaseException:
                self._unlock()
                raise
        try:
            self._idle()
            self._restore()  # Recover a previous interrupted session before taking ownership.
            self._service_active()
            raw = self.config.read_bytes()
            selected = four_stream_config(raw)
            original_modules = json.loads(raw)["ENABLED_MODULES"]
            selected_modules = json.loads(selected)["ENABLED_MODULES"]
            if original_modules == selected_modules:
                print("相机模式：原配置已只有所需四路，无需切换。", flush=True)
                return
            stamps = {name: stream_stamp(name) for name in ALL_STREAMS}
            now = time.time_ns()
            live = [name for name, stamp in stamps.items() if 0 <= now - stamp < 2_000_000_000]
            state = dict(config=str(self.config), original=base64.b64encode(raw).decode(),
                         selected_sha256=digest(selected),
                         mode=stat.S_IMODE(self.config.stat().st_mode),
                         previous_live_streams=sorted(live))
            atomic_write(self.journal, json.dumps(state).encode())
            atomic_write(self.config, selected, state["mode"])
            print("相机模式：切换为左右手、头部RGB、头部深度；等待视觉服务重连…", flush=True)
            self._restart(STREAMS)
            print("相机模式：四路持续出帧已确认，可以开始采集。", flush=True)
        except BaseException as exc:
            print(f"相机切换失败：{exc}", file=sys.stderr, flush=True)
            try:
                if self.journal.exists():
                    self._restore()
            finally:
                self._unlock()
            raise

    def _unlock(self):
        if self.lock is not None:
            self.lock.close()
            self.lock = None

    def release(self):
        try:
            if self.lock is not None:
                if self.retain:
                    self._idle()
                    print("相机模式：本批已结束，保留四路。整场结束后运行 camera_session.py recover 恢复。", flush=True)
                else:
                    self._restore()
        except Exception as exc:
            print(f"相机恢复失败：{exc}；恢复记录已保留。", file=sys.stderr, flush=True)
            raise
        finally:
            self._unlock()

    def recover(self):
        self._load()
        self._lock()
        try:
            self._restore()
        finally:
            self._unlock()

def run_command(command):
    if command[:1] == ["--"]:
        command = command[1:]
    if not command:
        raise RuntimeError("run 需要采集启动命令")
    child = None
    received = []
    def handle(signum, _frame):
        if not received:
            received.append(signum)
            if child is not None and child.poll() is None:
                child.send_signal(signum)
    handlers = {s: signal.signal(s, handle) for s in (signal.SIGINT, signal.SIGTERM)}
    session = CameraSession()
    try:
        session.acquire()
        if received:
            return 128 + received[0]
        env = dict(os.environ, COLLECTOR_CAMERA_SESSION_ACTIVE="1")
        child = subprocess.Popen(command, env=env)
        return child.wait()
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
            child.wait()  # Never restart cameras underneath a surviving recorder.
        try:
            session.release()
        finally:
            for signum, handler in handlers.items():
                signal.signal(signum, handler)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("run", "status", "recover"))
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    try:
        if args.action == "run":
            return run_command(args.command)
        if args.action == "recover":
            CameraSession().recover()
            return 0
        configured = SETTINGS.exists()
        result = dict(configured=configured, recovery_pending=(STATE_DIR / "pending.json").exists())
        if configured:
            session = CameraSession()
            session._load()
            result["enabled_camera_modules"] = [
                name for name in json.loads(session.config.read_text())["ENABLED_MODULES"]
                if name.startswith("mod_camera_")]
        result["streams"] = {
            name: dict(age_sec=round((time.time_ns()-stamp)/1e9, 3) if stamp else None,
                       fresh=bool(stamp and 0 <= time.time_ns()-stamp < 2_000_000_000))
            for name in ALL_STREAMS for stamp in [stream_stamp(name)]}
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"相机切换失败：{exc}", file=sys.stderr, flush=True)
        return 1

if __name__ == "__main__":
    raise SystemExit(main())
