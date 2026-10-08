"""One exclusive V4 controller, one authority selector, one Thor client.

This file does not start a recorder or use the legacy factory VR controller.
The normal collector launcher owns the sole recorder and its FIFO lifecycle.
"""

import json
import os
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dagger.dependencies import ASSETS, interpreter_environment


def generate_launch_description():
    vr = Path(os.environ["DAGGER_V4_RUNTIME_ROOT"])
    hg = ASSETS
    here = Path(__file__).resolve().parent
    runtime = str(here / "runtime.py")
    base = Path(os.environ["DAGGER_BASE_DIR"])
    dry = "false" if os.environ["DAGGER_PUBLISH"] == "1" else "true"
    python = os.environ.get("DAGGER_ROS_PY", "/usr/bin/python3")
    vr_python = os.environ.get("DAGGER_VR_PY", "/home/ubuntu/ros2_ws/venvs/openarmx_v4_placo/bin/python")
    web_python = os.environ.get("DAGGER_WEB_PY", "/home/ubuntu/ros2_ws/venvs/hg_dagger_web/bin/python")
    prefix = "/openarmx_teleop_vr_306_v4"
    env = {"PYTHONPATH": os.pathsep.join((str(vr), str(hg), str(here.parent), os.environ.get("PYTHONPATH", "")))}

    def process(command, *, config=None, params=None, remaps=None):
        args = [*command, "--ros-args"]
        if config:
            args += ["--params-file", str(config)]
        for key, value in (params or {}).items():
            args += ["-p", f"{key}:={value}"]
        for source, target in (remaps or {}).items():
            args += ["-r", f"{source}:={target}"]
        process_env = {**env, **interpreter_environment(command[0], os.environ)}
        return ExecuteProcess(cmd=args, additional_env=process_env, output="screen",
                              sigterm_timeout="15", sigkill_timeout="5")

    supervisor = process([python, runtime, "supervisor"], config=hg / "config/hg_dagger.yaml", params={
        "trace_root": str(base / "dagger_trace"), "human_finish_only": "true",
        "default_depth_enabled": "true" if os.environ.get("WITH_DEPTH", "0") == "1" else "false",
        "rgb_collector_fifo": str(base / ".official_recording_control"),
        "rgbd_collector_fifo": str(base / ".official_recording_control"),
        "collector_action_rate_hz": str(float(os.environ.get("COLLECT_FPS", "30"))),
        "quick_reset_wait_timeout_sec": "65.0",
    })
    controller = process([vr_python, "-m", "openarmx_teleop_vr_306_v4.controller_node"],
                         config=vr / "config/controller.yaml", params={
        "topic_suffix": "0_300", "dry_run": dry,
        "gripper_max_position": "360.0",
        "sync_hold_only_enable": "true", "hardware_session_mode": "sync",
        "require_sync_session_service": "true", "require_teleop_heartbeat": "true",
        "reset_before_hardware_enable": "false", "quick_reset_after_hardware_enable": "false",
        "quick_reset_stable_sec": "1.0", "quick_reset_stable_delta_deg": "0.3",
        "body_height_control_enabled": "false", "allow_waist_in_ik": "false", "head_follow_enabled": "false",
        "follow_authority_topic": "/hg_dagger/control_state",
        "authority_heartbeat_topic": "/hg_dagger/authority_heartbeat",
        "input_target_topic": "/hg_dagger/selected/eef_target",
        "input_gripper_topic": "/hg_dagger/selected/gripper_target",
        "input_release_topic": "/hg_dagger/selected/release_hold",
        "input_joint_target_topic": "/hg_dagger/selected/joint_target",
        "input_body_height_topic": "/hg_dagger/disabled/body_height_command",
        "input_head_target_topic": "/hg_dagger/disabled/head_target",
        "urdf_path": str(vr / "urdf/robot_v2_2_simplified.urdf"),
        "srdf_path": str(vr / "urdf/robot_v2_2.srdf"),
    }, remaps={"__node": "independent_arm_controller_306_v4",
               f"{prefix}/set_hardware_enabled": "/hg_dagger/controller/set_hardware_enabled"})
    mapper = process([python, runtime, "mapper"],
                     config=vr / "config/teleop.yaml", params={
        "topic_suffix": "0_300", "dry_run": dry, "quick_reset_enabled": "false",
        "gripper_filter_alpha": "1.0", "gripper_max_step_per_cycle": "350.0", "gripper_closed_position": "360.0",
    }, remaps={**{f"{prefix}/{key}": f"/hg_dagger/expert/{key}"
                  for key in ("eef_target", "gripper_target", "release_hold")},
               f"{prefix}/head_target": "/hg_dagger/disabled/head_target",
               f"{prefix}/set_hardware_enabled": "/hg_dagger/set_session_enabled"})
    bridge = process([python, runtime, "bridge"], params={
        "server_url": os.environ["DAGGER_SERVER_URL"], "token_file": os.environ["DAGGER_TOKEN_FILE"],
        "task": json.dumps(os.environ["TASK_TEXT"]),
        "collector_root": str(here.parent),
        "joint_state_topic": "/topic_arm_whole_body_and_gripper_current_joints_status_0_300",
    })
    web = process([web_python, runtime, "web"], params={
        "host": "0.0.0.0", "https_port": os.environ.get("DAGGER_WEB_PORT", "8447"),
        "rgbd_camera_enabled": "true", "status_topic": "/hg_dagger/web_teleop_status",
    }, remaps={f"{prefix}/set_hardware_enabled": "/hg_dagger/set_session_enabled"})
    processes = [supervisor, controller, mapper, bridge, web]
    # Losing any component closes this entire stack; the launcher then closes
    # the recorder. No orphan controller should keep publishing alone.
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node, on_exit=[EmitEvent(event=Shutdown(reason="DAgger component exited"))]
    )) for node in processes]
    return LaunchDescription([*handlers, *processes])
