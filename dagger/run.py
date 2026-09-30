#!/usr/bin/env python3
"""Launch the integrated recorder and exclusive V4 DAgger stack.

Default is no hardware publishing. Setting DAGGER_PUBLISH=1 authorizes button-
initiated motion; it never starts an episode automatically. Existing sessions
are refused, not killed. Q/Ctrl+C stops only children created by this process.
"""

from __future__ import annotations

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import termios
import threading
import time
import tty

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from dagger.dependencies import conflicting_processes, validate
from dagger.runtime_copy import prepare_copy


def environment(task: str, text: str | None, source: dict[str, str]) -> dict[str, str]:
    env = dict(source)
    for name, default in (("DAGGER_PUBLISH", "0"), ("WITH_HEAD", "1"),
                          ("WITH_UPPER_WAIST", "1"), ("WITH_WAIST", "0"), ("WITH_DEPTH", "0")):
        env.setdefault(name, default)
        if env[name] not in ("0", "1"):
            raise ValueError(f"{name} must be 0 or 1")
    if (env["WITH_HEAD"], env["WITH_UPPER_WAIST"], env["WITH_WAIST"]) != ("1", "1", "0"):
        raise ValueError("This GR00T DAgger adapter requires 21-D: WITH_HEAD=1 WITH_UPPER_WAIST=1 WITH_WAIST=0")
    if env.get("ROBOT_ID", "300") != "300" or env.get("ROS_DOMAIN_ID", "0") != "0":
        raise ValueError("The inspected private DAgger stack only supports robot 300 / ROS domain 0")
    if not task or Path(task).name != task or task in (".", ".."):
        raise ValueError("task_name must be a single nonempty directory name")
    if env.get("ACTION_MODE", "joint") != "joint" or env.get("FALLBACK_ACTION_TO_STATE", "0") != "0":
        raise ValueError("DAgger requires ACTION_MODE=joint and FALLBACK_ACTION_TO_STATE=0")
    if not env.get("DAGGER_SERVER_URL", "").startswith(("http://", "https://")):
        raise ValueError("Set DAGGER_SERVER_URL to the Thor GR00T server, e.g. http://THOR_IP:8777")
    if env.get("CAMERA_ONLY", "0") != "0":
        raise ValueError("DAgger requires all three RGB cameras")
    expected_cameras = "4" if env["WITH_DEPTH"] == "1" else "3"
    if env.get("MIN_CAMERAS", expected_cameras) != expected_cameras:
        raise ValueError("DAgger requires all requested cameras; MIN_CAMERAS cannot relax the schema")
    env["MIN_CAMERAS"] = expected_cameras
    fps = float(env.get("COLLECT_FPS", "30"))
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError("COLLECT_FPS must be finite and positive")
    env.setdefault("DAGGER_DEPENDENCY_ROOT", "/home/ubuntu/ros2_ws/src")
    env.setdefault("DAGGER_TOOLS_ROOT", str(ROOT.parent) if (ROOT.parent / "deploy").is_dir()
                   else "/home/ubuntu/Autolife_VLA_Tools")
    env.setdefault("DAGGER_ROS_SETUP", "/home/ubuntu/ros2_ws/install/setup.bash")
    env.setdefault("DAGGER_TOKEN_FILE", "/home/ubuntu/.config/autolife_hg_dagger/groot_server.token")
    env.setdefault("OUTPUT_BASE_DIR", "/home/ubuntu/nas14")
    for name in ("DAGGER_DEPENDENCY_ROOT", "DAGGER_TOOLS_ROOT", "DAGGER_ROS_SETUP", "DAGGER_TOKEN_FILE", "OUTPUT_BASE_DIR"):
        env[name] = str(Path(env[name]).expanduser().resolve())
    env.setdefault("IMAGE_POLL_FPS", "60")
    env.setdefault("ENCODER_THREADS", "1")
    env.setdefault("DAGGER_WEB_PORT", "8447")
    if not 1 <= int(env["DAGGER_WEB_PORT"]) <= 65535:
        raise ValueError("invalid DAGGER_WEB_PORT")
    env.update(COLLECTOR_MODE="keyboard", VR_CONTROL="0", SUBTASKS_JSON="[]",
               _COLLECTOR_DAGGER_RECORDING="1", ACTION_MODE="joint", ROBOT_ID="300", ROS_DOMAIN_ID="0",
               START_HAND_PRODUCER="0", TASK_TEXT=env.get("TASK_TEXT", text or task),
               RMW_IMPLEMENTATION="rmw_cyclonedds_cpp")
    env["DAGGER_BASE_DIR"] = str(Path(env["OUTPUT_BASE_DIR"]).expanduser().resolve() / task)
    env["CYCLONEDDS_URI"] = env.get("COLLECT_CYCLONEDDS_URI", env.get("CYCLONEDDS_URI", (
        '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/>'
        '</Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery>'
        '<ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>'
        '</Discovery></Domain></CycloneDDS>')))
    env["COLLECT_CYCLONEDDS_URI"] = env["CYCLONEDDS_URI"]
    return env


def ros_command(env, *command):
    return ["bash", "-c", 'set -e; set +u; source "$1"; source "$2"; shift 2; exec "$@"',
            "dagger-env", "/opt/ros/jazzy/setup.bash", env["DAGGER_ROS_SETUP"], *command]


def stop_child(child, timeout=30, *, launch_managed=False):
    if child is None or child.poll() is not None:
        return
    if launch_managed:
        # ros2 launch forwards SIGINT to its nodes. Signalling the whole group
        # as well interrupts their cleanup a second time.
        child.send_signal(signal.SIGINT)
    else:
        os.killpg(child.pid, signal.SIGINT)
    try:
        child.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(child.pid, signal.SIGTERM)
        try:
            child.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGKILL)
            child.wait()


def wait_recorder(child, base, started, stop, timeout=90):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if stop.is_set():
            raise InterruptedError("DAgger startup cancelled")
        if child.poll() is not None:
            raise RuntimeError("Recorder launcher exited before readiness; inspect its log")
        try:
            status = json.loads((base / ".official_recording_status.json").read_text())
            if status.get("event") == "ready" and status.get("success") and status.get("wall_time", 0) >= started:
                return
        except (OSError, ValueError):
            pass
        time.sleep(0.1)
    raise RuntimeError("Timed out waiting for this recorder instance's ready acknowledgement")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("task_name")
    parser.add_argument("task_text", nargs="?")
    parser.add_argument("--check", action="store_true", help="Read-only dependency/process preflight; starts nothing")
    parser.add_argument('--serve', action='store_true', help='Keep the owned host alive without a terminal; VR/desktop controls remain active')
    args = parser.parse_args()
    backend = os.environ.get('DAGGER_BACKEND', 'owned')
    if backend not in ('owned', 'attach'):
        parser.error('DAGGER_BACKEND must be owned or attach')
    if backend == 'attach':
        if args.serve:
            parser.error('--serve owns a host; it cannot be combined with attach')
        # No dependency copies, model connection, recorder or controller launch.
        # The running host owns those lifecycles, even when this terminal exits.
        env = dict(os.environ)
        env.setdefault('DAGGER_ROS_SETUP', '/home/ubuntu/ros2_ws/install/setup.bash')
        env.setdefault('ROS_DOMAIN_ID', '0')
        env.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
        env.setdefault('CYCLONEDDS_URI', '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>')
        if env.get('COLLECT_CYCLONEDDS_URI'):
            env['CYCLONEDDS_URI'] = env['COLLECT_CYCLONEDDS_URI']
        command = [str(ROOT / 'dagger/attach.py'), args.task_name]
        if args.task_text is not None:
            command.append(args.task_text)
        if args.check:
            command.append('--check')
        result = subprocess.run(ros_command(env, env.get('DAGGER_ROS_PY', '/usr/bin/python3'), *command), env=env)
        raise SystemExit(result.returncode)
    try:
        env = environment(args.task_name, args.task_text, os.environ)
        validate(Path(env["DAGGER_DEPENDENCY_ROOT"]))
        conflicts = conflicting_processes()
        if conflicts:
            raise RuntimeError(f"Another control stack is running (PIDs {conflicts}); stop it manually before DAgger")
        if not Path(env["DAGGER_ROS_SETUP"]).is_file():
            raise RuntimeError("Missing ROS workspace setup script")
        if not Path(env["DAGGER_TOKEN_FILE"]).is_file() and not env.get("GROOT_REMOTE_TOKEN"):
            raise RuntimeError("Missing Thor token file; configure DAGGER_TOKEN_FILE")
        subprocess.run(ros_command(env, env.get("DAGGER_ROS_PY", "/usr/bin/python3"),
                                   str(ROOT / "dagger/runtime.py"), "check"), env=env, timeout=20, check=True)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        parser.error(str(exc))
    if args.check:
        print("Dependency, ROS graph and Thor protocol preflight passed; no recording or motion started.")
        return

    # Separate from the recorder's per-episode motion lock. This protects stack
    # startup too, including the interval before ROS participants are visible.
    lock = open("/tmp/collector_dagger_0_300.lock", "a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit("Another integrated DAgger launcher owns robot 300")
    base = Path(env["DAGGER_BASE_DIR"])
    base.mkdir(parents=True, exist_ok=True)
    recorder = stack = None
    terminal = None
    stop = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stop.set())
    command_worker = None
    try:
        env["DAGGER_V4_RUNTIME_ROOT"] = str(prepare_copy(
            Path(env["DAGGER_DEPENDENCY_ROOT"]), ROOT / ".runtime"))
        print(f"V4 runtime COPY: {env['DAGGER_V4_RUNTIME_ROOT']}; colleague source unchanged.", flush=True)
        started = time.time()
        recorder = subprocess.Popen(["bash", str(ROOT / "start_lerobot_official_collect.sh"), args.task_name],
                                    env=env, stdin=subprocess.PIPE, start_new_session=True)
        wait_recorder(recorder, base, started, stop)
        stack = subprocess.Popen(ros_command(env, "ros2", "launch", str(ROOT / "dagger/stack.launch.py")),
                                 env=env, stdin=subprocess.DEVNULL, start_new_session=True)
        print("DAgger running. VR: A start, GL/GR held takeover, B save, Y discard, X reset only. "
              "Terminal: C start, A save, X/D discard, R reset, Q exit.", flush=True)
        print(f"Hardware publishing: {env['DAGGER_PUBLISH']} (1=enabled). "
              "VR X / terminal R resets all body joints and grippers after B/Y receipt; keep workspace clear.", flush=True)
        if not args.serve and sys.stdin.isatty():
            terminal = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        while not stop.wait(0.05):
            if recorder.poll() is not None or stack.poll() is not None:
                raise RuntimeError("A DAgger child exited; closing the remaining session")
            if args.serve or not select.select([sys.stdin], [], [], 0)[0]:
                continue
            key = sys.stdin.read(1).lower()
            if key in ("", "q"):
                break
            service = {"c": "launcher/start", "x": "launcher/discard", "a": "launcher/finish",
                       "d": "launcher/discard", "r": "launcher/reset"}.get(key)
            if service and (command_worker is None or not command_worker.is_alive()):
                def command(service=service):
                    try:
                        subprocess.run(ros_command(env, "ros2", "service", "call", f"/hg_dagger/{service}",
                                                   "std_srvs/srv/Trigger", "{}"), env=env, timeout=10, check=False)
                    except subprocess.TimeoutExpired:
                        print("Command result unknown; inspect control_state. Not retried.", flush=True)
                command_worker = threading.Thread(target=command, daemon=True)
                command_worker.start()
    finally:
        if terminal is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal)
        # Revoke control first. DAgger recorder shutdown discards unconfirmed
        # frames, unlike ordinary keyboard mode's save-on-exit behavior.
        stop_child(stack, launch_managed=True)
        stop_child(recorder, timeout=330)
        if command_worker is not None:
            command_worker.join(timeout=12)
        lock.close()


if __name__ == "__main__":
    main()
