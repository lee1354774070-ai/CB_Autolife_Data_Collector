"""Release only inspected, idle user-space controllers before DAgger startup."""

import json
import os
from pathlib import Path
import select
import signal
import urllib.request


def _request(path, token=None, data=None):
    headers = {"Content-Type": "application/json"}
    if token:
        headers["Authorization"] = "Bearer " + token
    request = urllib.request.Request("http://127.0.0.1:8080/api/v1/" + path,
        data=None if data is None else json.dumps(data).encode(), headers=headers)
    with urllib.request.urlopen(request, timeout=3) as response:
        payload = json.load(response)
    if not isinstance(payload, dict) or payload.get("success") is not True:
        raise RuntimeError("Void-Cog状态未确认；请手动退出后台后重试")
    return payload["data"]


def _void_idle():
    token = _request("connect", data={})["token"]
    try:
        jobs = _request("jobs", token)
        if not isinstance(jobs, list) or any(not isinstance(j, dict) or j.get("status") not in ("done", "error", "failed", "cancelled") for j in jobs):
            raise RuntimeError("Void-Cog有执行中或排队任务；不会自动退出")
        if _request("laundry/tasks", token) != []:
            raise RuntimeError("Void-Cog有洗衣任务记录，需人工确认后退出后台")
        for path in ("motion/busy", "motion/host_loop", "tracking/watch"):
            state = _request(path, token)
            if (not isinstance(state, dict) or state.get("running") is not False
                    or (path == "motion/busy" and any(state.get(k) is not False for k in ("action", "speaking")))):
                raise RuntimeError("Void-Cog动作或循环未空闲；不会自动退出")
        recording = _request("motion/actions/record/status", token)
        sessions = _request("collect/sessions", token)
        if (not isinstance(recording, dict) or not isinstance(sessions, dict)
                or recording.get("state") != "idle" or recording.get("frames") != 0 or sessions.get("sessions") != []):
            raise RuntimeError("Void-Cog有录制或待处理采集；不会自动退出")
    finally:
        _request("disconnect", token, data={})


def _processes():
    result = {}
    for proc in Path("/proc").glob("[0-9]*"):
        try:
            if proc.stat().st_uid == os.getuid():
                result[int(proc.name)] = tuple(a for a in (proc / "cmdline").read_bytes().decode().split("\0") if a)
        except (OSError, UnicodeError):
            continue
    return result


def _stop(pid, expected):
    # A pidfd binds the signal to this process even if its PID is later reused.
    try:
        fd = os.pidfd_open(pid)
    except ProcessLookupError:
        return
    try:
        if select.select([fd], [], [], 0)[0]:
            return
        if _processes().get(pid) != expected:
            if select.select([fd], [], [], 0)[0]:
                return  # The watchdog can exit while /proc is being scanned.
            raise RuntimeError("冲突进程已变化，取消自动退出；请重新自检")
        stop_signal = signal.SIGTERM if any(a.endswith('/service/watch-backend.sh') for a in expected) else signal.SIGINT
        signal.pidfd_send_signal(fd, stop_signal)
        if not select.select([fd], [], [], 10)[0]:
            raise RuntimeError("冲突模块正常退出超时；未强杀，请手动退出后重试")
    finally:
        os.close(fd)


def _joint_idle(state):
    if (state.get("hardware_enabled") is not False
            or state.get("hardware_unlock_mode") != "locked"
            or state.get("action_running") is not False
            or state.get("active_goal_groups") != []
            or "hardware_prepare_stage" not in state or state["hardware_prepare_stage"] is not None):
        raise RuntimeError("控制中心关节模块未确认锁定空闲；不会自动退出")


def repair_idle_conflicts(conflicts, joint_status, robot, *, recheck=None):
    """No ARM/reset/enable calls; unknown or busy owners always block repair."""
    names = {n for group in conflicts["unknown_publishers"].values() for n in group}
    allowed = {"/whole_body_joint_bridge", f"/action_player_{robot.suffix}"}
    if conflicts["active_commands"] or not names or names - allowed:
        raise RuntimeError("有动作命令或未知控制程序；不会自动退出：" + ", ".join(sorted(names)))
    processes = _processes()
    targets = []
    stop_file = None
    if "/whole_body_joint_bridge" in names:
        _joint_idle(joint_status)
        launches = [(pid, args) for pid, args in processes.items()
                    if "control_center_joint_control" in args
                    and "whole_body_joint_control.launch.py" in args
                    and f"robot_id:={robot.robot_id}" in args and "launch" in args]
        if len(launches) != 1:
            raise RuntimeError("无法唯一确认控制中心关节模块进程；请在控制中心手动停止关节模块")
        targets += launches
    if f"/action_player_{robot.suffix}" in names:
        _void_idle()
        root = Path.home() / "code/void-cog"
        watchdogs = [(pid, args) for pid, args in processes.items()
                     if str(root / "scripts/service/watch-backend.sh") in args]
        backends = [(pid, args) for pid, args in processes.items()
                    if "-m" in args and "link.client" in args and "--real" in args
                    and ("--port", "8080") in tuple(zip(args, args[1:]))]
        if len(watchdogs) != 1 or len(backends) != 1:
            raise RuntimeError("无法唯一确认Void-Cog后台进程；请手动退出后台")
        for pid, _ in watchdogs + backends:
            if "/system.slice/void-cog-h5.service" not in Path(f"/proc/{pid}/cgroup").read_text():
                raise RuntimeError("Void-Cog进程来源不匹配；不会自动退出")
        # Preserve the frontend/service manager; only stop its idle backend.
        stop_file = root / "logs/backend.stop"
        targets = backends + watchdogs + targets
    if recheck is not None:
        current, state = recheck()
        current_names = {n for group in current["unknown_publishers"].values() for n in group}
        if current["active_commands"] or current_names != names:
            raise RuntimeError("检查期间控制接口发生变化或收到动作；取消自动退出")
        if "/whole_body_joint_bridge" in names:
            _joint_idle(state)
    if stop_file is not None:
        stop_file.touch()
    for pid, args in targets:
        print(f"INFO  自检修复：正常退出空闲控制模块 PID {pid}", flush=True)
        _stop(pid, args)
    return len(targets)
