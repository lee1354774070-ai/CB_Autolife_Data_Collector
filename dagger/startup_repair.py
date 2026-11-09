"""Release only inspected, idle user-space controllers before DAgger startup."""

import json
import os
from pathlib import Path
import select
import signal
import subprocess
import time
import math
import shlex
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


DASHBOARD_NODES = {"/dashboard_arm_io", "/dashboard_gv_io"}
DASHBOARD_UNIT = "dashboard-backend.service"


def _dashboard_unit():
    result = subprocess.run(['systemctl', '--user', 'show', DASHBOARD_UNIT,
        '-p', 'ActiveState', '-p', 'MainPID', '-p', 'Environment', '-p', 'ExecStart'],
        check=True, capture_output=True, text=True, timeout=5)
    return dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)


def _dashboard_json(path):
    with urllib.request.urlopen('http://127.0.0.1:9001/' + path, timeout=3) as response:
        return json.load(response)


def _dashboard_idle(robot, feedback):
    unit = _dashboard_unit()
    environment = shlex.split(unit.get('Environment', ''))
    if (unit.get('ActiveState') != 'active' or int(unit.get('MainPID', '0')) <= 1
            or 'autolife_robot_dashboard.server import main' not in unit.get('ExecStart', '')
            or f'ROBOT_ID={robot.robot_id}' not in environment
            or f'ROS_DOMAIN_ID={robot.domain}' not in environment):
        raise RuntimeError('原厂dashboard服务来源或机器人编号未确认；不会自动暂停')
    health = _dashboard_json('health')
    if (health.get('status') != 'healthy' or health.get('ros_ready') is not True
            or type(health.get('websocket_clients')) is not int
            or health['websocket_clients'] != 0):
        raise RuntimeError('原厂dashboard仍有网页连接或状态异常，请关闭原厂控制网页后重试')
    mapping = _dashboard_json('mapping/status')
    if (mapping.get('ok') is not True or mapping.get('enabled') is not False
            or mapping.get('exploring') is not False):
        raise RuntimeError('原厂dashboard建图或探索未确认空闲；不会自动暂停')
    flag = subprocess.run(['redis-cli', '--raw', 'GET',
        f'shared_params:{robot.domain}:{robot.robot_id}:is_navigating'],
        check=True, capture_output=True, text=True, timeout=3).stdout.strip().lower()
    if flag != 'false':
        raise RuntimeError('导航未确认空闲；不会自动暂停原厂dashboard')
    # A disconnected browser alone does not prove an earlier arm command ended.
    joints = feedback
    if not 0 <= time.monotonic() - float(joints.get('_received_at', 0)) <= .8:
        raise RuntimeError('实时关节反馈过期；不会自动暂停原厂dashboard')
    for group, size in (('left_arm', 7), ('right_arm', 7), ('neck', 3), ('leg_waist', 4)):
        state = joints[group + '_joint_state']
        position, speed = state['position'], state['speed']
        target = joints[group + '_target_joint_state']
        if (len(position) != size or len(speed) != size or len(target) != size
                or len(state['communication_lost']) != size
                or any(x is not False for x in state['communication_lost'])
                or not all(math.isfinite(x) for x in position + speed + target)
                or max(map(abs, speed)) > 3
                or max(abs(a-b) for a,b in zip(position, target)) > 2):
            raise RuntimeError('原厂dashboard仍有运动或关节状态异常；不会自动暂停')
    return unit['MainPID']


def _pause_dashboard(receipt, expected_pid):
    if _dashboard_unit().get('MainPID') != expected_pid:
        raise RuntimeError('原厂dashboard进程发生变化，请重新自检')
    # Record before stopping, so even a partial stop/error is restored by the host.
    receipt.write_text(json.dumps({'unit': DASHBOARD_UNIT, 'pid': expected_pid}))
    print('INFO  自检修复：正常暂停空闲原厂dashboard；退出DAgger后恢复', flush=True)
    subprocess.run(['systemctl', '--user', 'stop', DASHBOARD_UNIT],
                   check=True, capture_output=True, text=True, timeout=20)
    if _dashboard_unit().get('ActiveState') != 'inactive':
        raise RuntimeError('原厂dashboard尚未退出；不会启动第二套控制器')


def restore_dashboard(receipt):
    if not receipt.is_file():
        return
    saved = json.loads(receipt.read_text())
    if saved.get('unit') != DASHBOARD_UNIT:
        raise RuntimeError('原厂dashboard恢复记录不匹配')
    from .dependencies import conflicting_processes
    if conflicting_processes():
        raise RuntimeError('仍有控制进程，原厂dashboard暂不恢复；确认退出后执行 systemctl --user start dashboard-backend.service')
    unit = _dashboard_unit()
    if unit.get('ActiveState') == 'inactive':
        subprocess.run(['systemctl', '--user', 'start', DASHBOARD_UNIT],
                       check=True, capture_output=True, text=True, timeout=20)
    elif unit.get('ActiveState') != 'active':
        raise RuntimeError('原厂dashboard状态未稳定，请检查 systemctl --user status dashboard-backend.service')
    receipt.unlink()
    print('INFO  原厂dashboard已恢复', flush=True)


def repair_idle_conflicts(conflicts, joint_status, robot, *, recheck=None, robot_feedback=None):
    """No ARM/reset/enable calls; unknown or busy owners always block repair."""
    names = {n for group in conflicts["unknown_publishers"].values() for n in group}
    allowed = {"/whole_body_joint_bridge", f"/action_player_{robot.suffix}"} | DASHBOARD_NODES
    if conflicts["active_commands"] or not names or names - allowed:
        raise RuntimeError("有动作命令或未知控制程序；不会自动退出：" + ", ".join(sorted(names)))
    dashboard_pid = None
    if names & DASHBOARD_NODES:
        receipt_path = os.environ.get('DAGGER_DASHBOARD_RECEIPT')
        if not receipt_path:
            raise RuntimeError('请通过DAgger启动器启动，以便退出后恢复原厂dashboard')
        dashboard_pid = _dashboard_idle(robot, robot_feedback or {})
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
    if dashboard_pid is not None and _dashboard_idle(robot, robot_feedback or {}) != dashboard_pid:
        raise RuntimeError('原厂dashboard进程发生变化，取消自检修复')
    if recheck is not None:
        current, state = recheck()
        current_names = {n for group in current["unknown_publishers"].values() for n in group}
        if current["active_commands"] or current_names != names:
            raise RuntimeError("检查期间控制接口发生变化或收到动作；取消自动退出")
        if "/whole_body_joint_bridge" in names:
            _joint_idle(state)
    if dashboard_pid is not None:
        _pause_dashboard(Path(receipt_path), dashboard_pid)
    if stop_file is not None:
        stop_file.touch()
    for pid, args in targets:
        print(f"INFO  自检修复：正常退出空闲控制模块 PID {pid}", flush=True)
        _stop(pid, args)
    return len(targets) + int(dashboard_pid is not None)
