"""ROS 2 HG-DAgger supervisor.

This node is the sole authority selector upstream of the existing V4 controller.
It never publishes vendor hardware topics.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import threading
import time
import uuid
from typing import Any, Dict, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .core import (
    AuthorityStateMachine,
    Mode,
    action_jump_components,
    controller_hold_confirmed,
    debounce_elapsed,
    finite_vector,
    grip_snapshot,
    policy_to_controller,
    stamped_envelope,
)
from .recorder import CollectorFifo, TraceWriter


def compact(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, separators=(",", ":"))


class HgDaggerSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("hg_dagger_supervisor")
        self._declare_parameters()
        self._lock = threading.RLock()
        self._machine = AuthorityStateMachine()
        self._session_id = ""
        self._intervention_id = ""
        self._sequence: Dict[str, int] = {}
        self._controller_status: Dict[str, Any] = {}
        self._controller_status_received_ns = 0
        self._mapper_status: Dict[str, Any] = {}
        self._policy_status: Dict[str, Any] = {}
        self._last_policy_status_monotonic_ns = 0
        self._latest_grippers = [0.0, 0.0]
        self._vendor_grippers = [0.0, 0.0]
        self._expert_gripper_hold = [10.0, 10.0]
        self._expert_gripper_desired = [10.0, 10.0]
        self._expert_gripper_command = [10.0, 10.0]
        self._expert_gripper_pickup_pending = [False, False]
        self._expert_gripper_input_ns = 0
        self._expert_gripper_tick_ns = 0
        self._measured_grippers: Optional[list[float]] = None
        self._gripper_measurement_ns = 0
        self._gripper_tracking_side = -1
        self._gripper_tracking_since_ns = 0
        self._gripper_tracking_start_measurement = 0.0
        self._gripper_tracking_warning = ""
        self._latest_action: Optional[list[float]] = None
        self._last_policy_monotonic_ns = 0
        self._last_expert_monotonic_ns = 0
        self._last_vendor_command_monotonic_ns = 0
        self._last_vendor_command_wall_ns = 0
        self._last_gripper_command_wall_ns = 0
        self._last_gripper_command_monotonic_ns = 0
        self._hold_sent_monotonic_ns = 0
        self._depth_next = bool(self.get_parameter("default_depth_enabled").value)
        self._active_depth = self._depth_next
        self._face_buttons = {button: False for button in ("A", "B", "X", "Y")}
        self._face_button_down_ns: Dict[str, int] = {}
        self._y_click_times: list[int] = []
        self._launcher_control_active = False
        self._launcher_pending_action = None
        self._launcher_wait_for_release = False
        self._button_worker_active = False
        self._button_worker_name = ""
        self._session_start_pending = False
        self._grips = (False, False)
        self._expert_command_count = 0
        self._recorder_active = False
        self._collector_started_wall_ns = 0
        self._takeover_seen = False
        self._dagger_segment = 0
        self._authority_changed_wall_ns = time.time_ns()
        self._pending_collector_result = None
        self._reset_pending = False
        self._release_gate_started_ns = 0
        self._expert_command_pending = False
        self._expert_forwarded_monotonic_ns = 0
        self._policy_warmup_started_ns = 0
        self._policy_warmup_count = 0
        self._policy_warmup_last_action: Optional[list[float]] = None
        self._policy_warmup_proposal_id = ""
        self._hold_to_intervene = False
        self._grip_release_started_ns = 0
        self._last_grip_sample_ns = 0
        self._takeover_requested_ns = 0
        self._hold_confirmed_ns = 0
        self._handback_started_ns = 0
        self._policy_resume_pending = False
        self._timing_ms: Dict[str, float] = {}
        self._warmup_diagnostics: Dict[str, Any] = {}
        self._last_failure_reason = ""
        self._save_notice: Dict[str, Any] = {}
        self._last_episode_saved = False
        self._last_episode_disable_ok = False
        self._collection_exited = False

        latest = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        reliable = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self._selected_joint_pub = self.create_publisher(
            String, "/hg_dagger/selected/joint_target", reliable)
        self._selected_eef_pub = self.create_publisher(
            String, "/hg_dagger/selected/eef_target", latest)
        self._selected_gripper_pub = self.create_publisher(
            String, "/hg_dagger/selected/gripper_target", latest)
        self._selected_release_pub = self.create_publisher(
            String, "/hg_dagger/selected/release_hold", reliable)
        self._state_pub = self.create_publisher(String, "/hg_dagger/control_state", reliable)
        self._web_status_pub = self.create_publisher(
            String, "/hg_dagger/web_teleop_status", reliable)
        self._event_pub = self.create_publisher(String, "/hg_dagger/events", reliable)
        self._timed_vr_pub = self.create_publisher(String, "/hg_dagger/timing/vr_input", latest)
        self._timed_eef_pub = self.create_publisher(String, "/hg_dagger/timing/expert_eef", latest)
        self._timed_action_pub = self.create_publisher(String, "/hg_dagger/collector/action", reliable)
        self._collector_arm_pub = self.create_publisher(
            String, "/hg_dagger/collector/arm_action", reliable)
        self._collector_gripper_pub = self.create_publisher(
            String, "/hg_dagger/collector/gripper_action", reliable)
        self._policy_forward_ack_pub = self.create_publisher(
            String, "/hg_dagger/policy_forward_ack", reliable)

        self.create_subscription(
            String, "/openarmx_teleop_vr_306_v4/vr_input", self._on_vr_input, latest)
        self.create_subscription(
            String, "/hg_dagger/policy_action", self._on_policy_action, latest)
        self.create_subscription(
            String, "/hg_dagger/policy_status", self._on_policy_status, reliable)
        self.create_subscription(
            String, "/hg_dagger/expert/eef_target", self._on_expert_eef, latest)
        self.create_subscription(
            String, "/hg_dagger/expert/gripper_target", self._on_expert_gripper, latest)
        self.create_subscription(
            String, "/hg_dagger/expert/release_hold", self._on_expert_release, reliable)
        self.create_subscription(
            String, "/openarmx_teleop_vr_306_v4/status", self._on_controller_status, reliable)
        self.create_subscription(
            String,
            "/openarmx_teleop_vr_306_v4/teleop_status",
            self._on_mapper_status,
            reliable,
        )
        self.create_subscription(
            String, "/topic_arm_whole_body_and_gripper_current_joints_status_0_300",
            self._on_joint_feedback, latest,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("vendor_joint_command_topic").value),
            self._on_vendor_joint_command,
            latest,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("vendor_gripper_command_topic").value),
            self._on_vendor_gripper_command,
            latest,
        )

        service_group = ReentrantCallbackGroup()
        self._controller_enable = self.create_client(
            SetBool,
            "/hg_dagger/controller/set_hardware_enabled",
            callback_group=service_group,
        )
        self._full_body_reset = self.create_client(
            Trigger,
            "/openarmx_teleop_vr_306_v4/full_body_reset",
            callback_group=service_group,
        )
        self.create_service(
            SetBool,
            "/hg_dagger/set_session_enabled",
            self._on_set_session_enabled,
            callback_group=service_group,
        )
        self.create_service(Trigger, "/hg_dagger/request_failure", self._on_request_failure)
        self.create_service(Trigger, "/hg_dagger/request_takeover", self._on_request_takeover)
        self.create_service(Trigger, "/hg_dagger/request_resume", self._on_request_resume)
        self.create_service(Trigger, "/hg_dagger/finish_episode", self._on_finish_episode,
                            callback_group=service_group)

        for action in ("start", "finish", "discard", "reset", "quit"):
            self.create_service(
                Trigger, f"/hg_dagger/launcher/{action}",
                lambda request, response, action=action:
                    self._on_launcher_action(action, response))

        self._trace = TraceWriter(
            str(self.get_parameter("trace_root").value),
            float(self.get_parameter("pre_failure_ring_seconds").value),
        )
        self._collector = CollectorFifo(
            str(self.get_parameter("rgb_collector_fifo").value),
            str(self.get_parameter("rgbd_collector_fifo").value),
        )
        self._authority_heartbeat_pub = self.create_publisher(String, '/hg_dagger/authority_heartbeat', reliable)
        self.create_timer(0.1, lambda: self._authority_heartbeat_pub.publish(
            String(data='hg_dagger_alive')), callback_group=ReentrantCallbackGroup())
        self.create_timer(0.02, self._control_tick)
        self.create_timer(1.0 / float(self.get_parameter("collector_action_rate_hz").value), self._publish_collector_action)
        self.create_timer(0.2, self._publish_state)
        self.get_logger().info(
            "HG-DAgger supervisor ready; controller vendor topics are read-only")

    def _declare_parameters(self) -> None:
        defaults = {
            "default_depth_enabled": True,
            "rgbd_only": True,
            "human_finish_only": True,
            "button_double_click_window_sec": 5.0,
            "hold_confirmation_timeout_sec": 1.0,
            "policy_timeout_sec": 0.25,
            "policy_status_timeout_sec": 1.0,
            "expert_timeout_sec": 0.35,
            "expert_gripper_speed_deg_sec": 1000.0,
            "expert_gripper_rate_hz": 30.0,
            "grip_release_debounce_sec": 0.20,
            "policy_warmup_min_actions": 1,
            "policy_warmup_min_duration_sec": 0.0,
            "policy_resume_max_arm_jump_deg": 8.0,
            "policy_resume_max_gripper_jump_deg": 80.0,
            "policy_resume_max_observation_age_sec": 2.0,
            "minimum_episode_frames": 15,
            "collector_action_rate_hz": 30.0,
            "pre_failure_ring_seconds": 5.0,
            "trace_root": "/home/ubuntu/nas14/hg_dagger_traces",
            "rgb_collector_fifo": "/tmp/hg_dagger_rgb_collector.fifo",
            "rgbd_collector_fifo": "/tmp/hg_dagger_rgbd_collector.fifo",
            "collector_start_command": "start",
            "collector_save_command": "save",
            "collector_discard_command": "discard",
            "collector_command_timeout_sec": 4.0,
            "collector_save_timeout_sec": 120.0,
            "quick_reset_request_timeout_sec": 3.0,
            "quick_reset_wait_timeout_sec": 25.0,
            "vendor_joint_command_topic": "/topic_arm_whole_body_target_joints_position_0_300",
            "vendor_gripper_command_topic": "/topic_arm_gripper_target_joints_position_0_300",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _next_sequence(self, source: str) -> int:
        value = self._sequence.get(source, 0) + 1
        self._sequence[source] = value
        return value

    @staticmethod
    def _elapsed_ms(started_ns: int, finished_ns: int) -> float:
        return max(0.0, (finished_ns - started_ns) / 1e6)

    @staticmethod
    def _parse(message: String) -> Dict[str, Any]:
        data = json.loads(message.data)
        if not isinstance(data, dict):
            raise ValueError("payload must be a JSON object")
        return data

    def _timestamp(self, source: str, payload: Dict[str, Any], valid: bool = True, reason: str = "") -> Dict[str, Any]:
        source_timestamp_ns = payload.get("timestamp_ns")
        if source_timestamp_ns is None and isinstance(payload.get("timestamp"), (int, float)):
            # Browser Date.now() is milliseconds since epoch.
            source_timestamp_ns = int(float(payload["timestamp"]) * 1_000_000)
        try:
            source_timestamp_ns = None if source_timestamp_ns is None else int(source_timestamp_ns)
        except (TypeError, ValueError):
            source_timestamp_ns = None
        return stamped_envelope(
            source=source,
            sequence=int(payload.get("sequence", self._next_sequence(source))),
            source_timestamp_ns=source_timestamp_ns,
            receive_wall_ns=time.time_ns(),
            receive_monotonic_ns=time.monotonic_ns(),
            authority_epoch=self._machine.authority_epoch,
            valid=valid,
            reason=reason,
            payload=payload,
        )

    def _event(self, kind: str, reason: str, **extra: Any) -> None:
        if kind == "transition":
            self._authority_changed_wall_ns = time.time_ns()
            if extra.get("new") == Mode.EXPERT_ACTIVE.value:
                self._dagger_segment += 1
        record = {
            "kind": kind,
            "reason": reason,
            "wall_ns": time.time_ns(),
            "monotonic_ns": time.monotonic_ns(),
            "mode": self._machine.mode.value,
            "authority_epoch": self._machine.authority_epoch,
            "session_id": self._session_id,
            "intervention_id": self._intervention_id,
            **extra,
        }
        self._event_pub.publish(String(data=compact(record)))
        self._trace.append_ring(record)
        self._trace.write("events", record)

    def _publish_release_hold(self, reason: str) -> None:
        payload = {
            "sides": ["left", "right"],
            "source": "hg_dagger_authority_barrier",
            "authority_epoch": self._machine.authority_epoch,
            "reason": reason,
        }
        self._selected_release_pub.publish(String(data=compact(payload)))
        self._hold_sent_monotonic_ns = time.monotonic_ns()

    def _set_notice_locked(self, text: str, level: str = "info") -> None:
        self._save_notice = {
            "token": time.time_ns(),
            "level": level,
            "text": text,
        }

    def _spawn_button_worker_locked(self, name: str, target: Any) -> None:
        if self._button_worker_active or self._session_start_pending:
            self._event(
                "button_action_busy",
                f"ignored {name}; session startup is still running"
                if self._session_start_pending
                else f"ignored {name}; {self._button_worker_name} is still running",
            )
            return
        self._button_worker_active = True
        self._button_worker_name = name

        def run() -> None:
            try:
                target()
            except Exception as exc:
                with self._lock:
                    self._set_notice_locked(f"{name} 执行失败：{exc}", "error")
                    self._event("button_action_failed", str(exc), button=name)
                    self._publish_state()
            finally:
                with self._lock:
                    self._button_worker_active = False
                    self._button_worker_name = ""
                    self._launcher_control_active = False
                    pending = getattr(self, '_launcher_pending_action', None)
                    self._launcher_pending_action = None
                    if pending:
                        self._on_launcher_action(pending, Trigger.Response())

        threading.Thread(
            target=run,
            name=f"hg-dagger-button-{name.lower()}",
            daemon=True,
        ).start()

    def _update_face_button_locked(
        self, button: str, pressed: bool, now_ns: int
    ) -> None:
        button = str(button).upper()
        if button not in self._face_buttons:
            return
        pressed = bool(pressed)
        previous = self._face_buttons[button]
        if previous == pressed:
            return
        self._face_buttons[button] = pressed
        if pressed:
            self._face_button_down_ns[button] = int(now_ns)
            return
        self._face_button_down_ns.pop(button, None)
        self._on_face_button_click_locked(button, int(now_ns))

    def _on_launcher_action(self, action, response):
        with self._lock:
            if action not in ('start', 'finish', 'discard', 'reset', 'quit') or self._collection_exited:
                response.success, response.message = False, '操作无效或采集已结束'
                return response
            if self._button_worker_active or self._session_start_pending or self._reset_pending:
                if action == 'start' or getattr(self, '_launcher_pending_action', None):
                    response.success, response.message = False, '已有操作或启动器请求正在处理，请等待'
                    return response
                self._launcher_pending_action = action
                self._claim_launcher_locked()
                response.success, response.message = True, '启动器请求已优先排队，当前保存/复位完成后执行'
                return response
            if action == 'start' and self._machine.mode != Mode.DISARMED:
                response.success, response.message = False, '请先结束本条，再开始下一条'
                return response
            if action == 'reset' and self._intervention_id:
                response.success, response.message = False, '请先保存或丢弃本条，再单独复位'
                return response
            self._claim_launcher_locked()
            self._spawn_button_worker_locked('launcher ' + action,
                                             lambda: self._launcher_action_worker(action))
            response.success, response.message = True, '启动器已取得控制，请等待后台执行结果'
        return response

    def _claim_launcher_locked(self):
        self._launcher_control_active = True
        self._launcher_wait_for_release = True
        self._grips = (False, False)
        self._face_buttons = {key: False for key in ('A', 'B', 'X', 'Y')}
        self._face_button_down_ns.clear()
        self._expert_command_pending = False
        self._grip_release_started_ns = 0
        if self._machine.mode != Mode.DISARMED:
            self._publish_release_hold('launcher has priority over VR')

    def _launcher_action_worker(self, action):
        if action == 'start':
            self._start_session_worker()
            return
        if action == 'reset':
            self._mechanical_reset_worker('launcher')
            return
        if action == 'discard':
            self._reset_trial_worker('launcher')
            return
        with self._lock:
            has_trial = bool(self._intervention_id)
            if has_trial and self._machine.mode != Mode.DISARMED:
                transition = self._machine.disable()
                self._event('transition', 'launcher ends trial', old=transition.old.value,
                            new=transition.new.value)
                self._publish_state()
        if has_trial:
            self._save_episode_worker()  # Existing rule: no human takeover means discard.
        else:
            with self._lock:
                self._set_notice_locked('没有待保存的本条数据', 'info')
                self._publish_state()
        if action == 'quit':
            with self._lock:
                if self._intervention_id:
                    self._set_notice_locked('保存结果未确认，暂不退出；请查询保存结果', 'error')
                    self._publish_state()
                    return
            self._exit_collection_worker()

    def _on_face_button_click_locked(self, button: str, now_ns: int) -> None:
        if getattr(self, '_launcher_control_active', False) or getattr(self, '_launcher_wait_for_release', False):
            return
        if button == "A":
            if self._machine.mode == Mode.DISARMED:
                self._spawn_button_worker_locked(
                    "A start inference", self._start_session_worker
                )
            else:
                self._event(
                    "button_rejected",
                    f"A starts only from DISARMED (mode={self._machine.mode.value})",
                    button=button,
                )
            return

        if button == "B":
            if self._intervention_id and self._machine.mode == Mode.DISARMED:
                self._spawn_button_worker_locked("B reconcile save", self._save_episode_worker)
                return
            if self._machine.mode in (
                Mode.EXPERT_ACTIVE,
                Mode.EXPERT_READY,
                Mode.EXPERT_RELEASE_REQUIRED,
            ):
                self._spawn_button_worker_locked(
                    "B save", self._save_episode_worker
                )
            elif self._machine.mode in (Mode.POLICY_ACTIVE, Mode.POLICY_WARMUP):
                self._spawn_button_worker_locked(
                    "B discard", lambda: self._discard_session_worker(
                        "B pressed before human takeover"
                    )
                )
            else:
                self._event(
                    "button_rejected",
                    f"B has no active trial in {self._machine.mode.value}",
                    button=button,
                )
            return

        if button == "X":
            self._spawn_button_worker_locked(
                "X discard and reset", lambda: self._reset_trial_worker("X")
            )
            return

        if button == "Y":
            window_ns = int(
                float(self.get_parameter("button_double_click_window_sec").value)
                * 1e9
            )
            self._y_click_times = [
                timestamp
                for timestamp in self._y_click_times
                if now_ns - timestamp <= window_ns
            ]
            self._y_click_times.append(now_ns)
            if len(self._y_click_times) >= 2:
                self._y_click_times.clear()
                self._spawn_button_worker_locked(
                    "Y exit collection", self._exit_collection_worker
                )
            else:
                self._set_notice_locked("Y 再点按一次退出采集", "info")
                self._publish_state()

    def _start_session_worker(self) -> None:
        response = SetBool.Response()
        self._on_set_session_enabled(SetBool.Request(data=True), response)
        with self._lock:
            self._set_notice_locked(
                "VLA 推理与录制已开始" if response.success
                else f"VLA 启动失败：{response.message}",
                "success" if response.success else "error",
            )
            self._publish_state()

    def _close_trial_locked(self, reason: str) -> None:
        if self._machine.mode != Mode.DISARMED:
            self._publish_release_hold(reason)
            transition = self._machine.disable()
            self._expert_command_pending = False
            self._event(
                "transition",
                transition.reason,
                old=transition.old.value,
                new=transition.new.value,
            )
        if self._intervention_id:
            try:
                self._finish_intervention_locked(False, reason)
            except Exception:
                self._forward_enable(False)
                raise
        self._session_id = ""

    def _discard_session_worker(self, reason: str) -> None:
        with self._lock:
            self._close_trial_locked(reason)
            self._set_notice_locked("本条未接管，已丢弃，正在全身复位", "warning")
            self._publish_state()
        self._mechanical_reset_worker("B")

    def _save_episode_worker(self) -> None:
        with self._lock:
            if any(self._grips):
                self._set_notice_locked("保存前请先松开左右 Grip", "warning")
                self._publish_state()
                return
            if self._machine.mode not in (
                Mode.EXPERT_ACTIVE, Mode.EXPERT_READY, Mode.EXPERT_RELEASE_REQUIRED,
            ) and not (self._machine.mode == Mode.DISARMED and self._intervention_id):
                self._set_notice_locked("当前没有可保存的人工接管数据", "warning")
                self._publish_state()
                return
            saved, message = self._finish_episode_locked("B 保存接管纠错数据")
            if self._intervention_id:
                self._set_notice_locked("保存结果未确认，数据已保留；按 B 查询结果，暂不复位", "error")
                self._publish_state()
                return
            saved_ack = bool(getattr(self, "_last_episode_saved", saved))
            disabled = bool(getattr(self, "_last_episode_disable_ok", True))
            if not disabled:
                self._set_notice_locked(
                    "数据已保存，但硬件停用失败，已暂停自动复位" if saved_ack
                    else f"数据未保存且硬件停用失败：{message}", "error",
                )
                self._publish_state()
                return
            save_note = "本条已保存（双 Y 后文件封口）；" if saved_ack else "本条未保存；"
            self._set_notice_locked(save_note + "正在全身复位", "success" if saved_ack else "warning")
            self._publish_state()
        self._mechanical_reset_worker("B", save_note=save_note)

    def _exit_collection_worker(self) -> None:
        with self._lock:
            self._close_trial_locked("Y 双击退出采集")
            self._collection_exited = True
            depth = self._active_depth
            self._set_notice_locked("正在结束录制器并封口数据文件", "warning")
            self._publish_state()
        self._forward_enable(False)
        pid_path = Path(self._collector.paths[bool(depth)]).parent / ".hg_dagger_recorder.pid"
        try:
            recorder_pid = int(pid_path.read_text().strip())
        except (OSError, ValueError):
            recorder_pid = 0
        sent = self._collector.command(depth, "quit")
        closed = False
        if sent and recorder_pid:
            deadline = time.monotonic() + 30.0
            while time.monotonic() < deadline:
                try:
                    os.kill(recorder_pid, 0)
                except ProcessLookupError:
                    closed = True
                    break
                time.sleep(0.1)
        with self._lock:
            self._set_notice_locked(
                "采集已退出，数据文件已封口；请重启采集脚本"
                if closed else "采集已退出，但录制器封口未确认；请检查终端",
                "success" if closed else "error",
            )
            self._publish_state()

    def _forward_full_body_reset(self) -> tuple[bool, str]:
        timeout = float(self.get_parameter("quick_reset_request_timeout_sec").value)
        if not self._full_body_reset.wait_for_service(timeout_sec=2.0):
            return False, "full-body reset service is unavailable"
        future = self._full_body_reset.call_async(Trigger.Request())
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(max(0.1, timeout)):
            return False, "full-body reset request timed out"
        try:
            result = future.result()
        except Exception as exc:
            return False, f"full-body reset request failed: {exc}"
        return bool(result.success), str(result.message)

    def _wait_for_reset_enable(self, requested_ns: int) -> tuple[bool, str]:
        deadline = time.monotonic() + float(
            self.get_parameter("quick_reset_wait_timeout_sec").value
        )
        while time.monotonic() < deadline:
            with self._lock:
                status = dict(self._controller_status)
                received_ns = self._controller_status_received_ns
            fresh = received_ns > requested_ns and (
                time.monotonic_ns() - received_ns < 1_000_000_000
            )
            if fresh:
                if status.get("emergency_stop_latched") or status.get("state") in (
                    "FAULT", "E_STOP", "ESTOP"
                ):
                    return False, str(status.get("reason", "controller fault"))
                if status.get("hardware_ready") is True and not status.get(
                    "hardware_enable_pending", False
                ):
                    return True, "hardware ready"
            time.sleep(0.05)
        return False, "等待硬件使能就绪超时"

    def _wait_for_quick_reset(self) -> bool:
        deadline = time.monotonic() + float(
            self.get_parameter("quick_reset_wait_timeout_sec").value
        )
        observed_active = False
        while time.monotonic() < deadline:
            with self._lock:
                status = dict(self._controller_status)
                received_ns = self._controller_status_received_ns
            if time.monotonic_ns() - received_ns > 1_000_000_000:
                return False
            if status.get("emergency_stop_latched") or status.get("state") in (
                "FAULT", "E_STOP", "ESTOP", "DISARMED"
            ):
                return False
            active = bool(status.get("quick_reset_active")) or status.get(
                "state"
            ) == "RESETTING"
            observed_active = observed_active or active
            if (observed_active and not active
                    and status.get("hardware_ready") is True
                    and float(status.get("quick_reset_progress", 0.0)) >= 1.0):
                return True
            time.sleep(0.05)
        return False

    def _open_reset_grippers(self) -> tuple[bool, str]:
        # Robot 300: 10 motor degrees is fully open. Confirm feedback before
        # disabling SYNC so an accepted command is not mistaken for completion.
        started_ns = time.monotonic_ns()
        deadline = time.monotonic() + 5.0
        stable_since = None
        while time.monotonic() < deadline:
            now_ns = time.monotonic_ns()
            with self._lock:
                status = self._controller_status
                if (now_ns - self._controller_status_received_ns > 1_000_000_000
                        or status.get('hardware_ready') is not True
                        or status.get('emergency_stop_latched')
                        or status.get('state') in ('FAULT', 'ESTOP', 'E_STOP')):
                    return False, '夹爪张开中断：控制器未就绪或状态过期'
                measured = self._measured_grippers
                fresh = (self._gripper_measurement_ns > started_ns
                         and now_ns - self._gripper_measurement_ns < 250_000_000)
                if fresh and measured and all(abs(v - 10.0) <= 3.0 for v in measured):
                    if stable_since is None:
                        stable_since = now_ns
                    if now_ns - stable_since >= 200_000_000:
                        return True, '双夹爪已完全张开'
                else:
                    stable_since = None
                self._selected_gripper_pub.publish(String(data=compact({
                    'left_gripper_target_joints_position': [10.0],
                    'right_gripper_target_joints_position': [10.0],
                    'source': 'hg_dagger_full_reset',
                })))
            time.sleep(0.1)
        return False, '双夹爪张开未到位或反馈过期（目标 10°），请检查'

    def _mechanical_reset_worker(self, button: str, save_note: str = "") -> None:
        with self._lock:
            self._reset_pending = True
            self._set_notice_locked(save_note + "正在全身复位", "warning")
            self._publish_state()

        # Release the previous SYNC lease, then use the official whole-body reset.
        disabled, detail = self._forward_enable(False)
        reset_ok, reset_message = False, detail
        if disabled:
            reset_ok, reset_message = self._forward_full_body_reset()
            if reset_ok:
                reset_ok, reset_message = self._wait_for_reset_enable(time.monotonic_ns())
            if reset_ok:
                reset_ok, reset_message = self._open_reset_grippers()
        disabled, disable_message = self._forward_enable(False)
        if reset_ok and not disabled:
            reset_ok = False
            reset_message = f"复位完成但硬件停用失败：{disable_message}"
        with self._lock:
            self._reset_pending = False
            self._set_notice_locked(
                save_note + "全身复位完成，请按 A 开始下一轮"
                if reset_ok else f"{save_note}全身复位失败：{reset_message}",
                "success" if reset_ok else "error",
            )
            self._publish_state()

    def _reset_trial_worker(self, button: str) -> None:
        with self._lock:
            self._close_trial_locked(f"{button} discard before mechanical reset")
            self._set_notice_locked("当前数据已丢弃，正在全身复位", "warning")
            self._publish_state()
        self._mechanical_reset_worker(button)

    def _begin_failure_hold(
        self, reason: str, *, hold_to_intervene: bool = False
    ) -> None:
        self._last_failure_reason = str(reason)
        self._hold_to_intervene = bool(hold_to_intervene)
        self._grip_release_started_ns = 0
        self._takeover_requested_ns = (
            time.monotonic_ns() if self._hold_to_intervene else 0
        )
        self._hold_confirmed_ns = 0
        self._handback_started_ns = 0
        self._policy_resume_pending = False
        self._timing_ms = {}
        self._warmup_diagnostics = {}
        transition = self._machine.failure(reason)
        self._release_gate_started_ns = 0
        self._expert_command_pending = False
        self._publish_release_hold(reason)
        self._save_notice = {}
        measured_fresh = (
            self._measured_grippers is not None
            and time.monotonic_ns() - self._gripper_measurement_ns < 1_000_000_000
        )
        pickup = self._measured_grippers if measured_fresh else self._vendor_grippers
        held = [min(360.0, max(10.0, float(v))) for v in pickup]
        self._expert_gripper_hold = list(held)
        self._expert_gripper_desired = list(held)
        self._expert_gripper_command = list(held)
        self._expert_gripper_pickup_pending = [True, True]
        self._expert_gripper_input_ns = 0
        self._expert_gripper_tick_ns = 0
        self._event("transition", transition.reason, old=transition.old.value, new=transition.new.value)
        # Freeze the rolling pre-failure window at the failure boundary. Waiting
        # until the operator completes release/re-grip would let that window roll
        # forward and could evict the actual failure context.
        # Recording is opened when the session starts. Grip must only change
        # authority here; it must never wait for a FIFO ACK or video encoder.

    def _on_vr_input(self, message: String) -> None:
        try:
            packet = self._parse(message)
        except Exception as exc:
            self._event("invalid_vr_input", str(exc))
            return
        envelope = self._timestamp("vr_input", packet)
        self._timed_vr_pub.publish(String(data=compact(envelope)))
        self._trace.append_ring({"wall_ns": envelope["robot_receive_timestamp_ns"], "stream": "vr", "data": envelope})
        self._trace.write("vr", envelope)

        with self._lock:
            if getattr(self, '_launcher_control_active', False):
                return
            if getattr(self, '_launcher_wait_for_release', False):
                left, right = packet.get('leftController'), packet.get('rightController')
                if not isinstance(left, dict) or not isinstance(right, dict):
                    return
                if any(grip_snapshot(packet)) or any(left.get(k) for k in ('xButton','yButton')) or any(right.get(k) for k in ('aButton','bButton')):
                    return
                self._launcher_wait_for_release = False
            previous_grips = self._grips
            if "leftController" in packet or "rightController" in packet:
                self._grips = grip_snapshot(packet)
                self._last_grip_sample_ns = int(
                    envelope["robot_receive_monotonic_ns"]
                )
            grip_rising = any(now and not old for now, old in zip(self._grips, previous_grips))
            if any(self._grips):
                self._grip_release_started_ns = 0
            elif any(previous_grips):
                self._grip_release_started_ns = int(
                    envelope["robot_receive_monotonic_ns"]
                )
            if grip_rising and self._machine.mode in (
                Mode.POLICY_ACTIVE, Mode.POLICY_WARMUP
            ):
                self._begin_failure_hold(
                    "operator Grip requested takeover", hold_to_intervene=True
                )
            elif (
                self._machine.mode == Mode.EXPERT_RELEASE_REQUIRED
                and not any(self._grips)
                and envelope["robot_receive_monotonic_ns"]
                > self._release_gate_started_ns
            ):
                transition = self._machine.expert_release_confirmed()
                self._event(
                    "transition",
                    transition.reason,
                    old=transition.old.value,
                    new=transition.new.value,
                )
            elif grip_rising and self._machine.mode == Mode.EXPERT_READY:
                self._event(
                    "expert_regrip_detected",
                    "second Grip press accepted; waiting for re-anchored expert target",
                )

            now_ns = time.monotonic_ns()
            event_type = packet.get("type")
            if event_type in ("button_press", "button_release"):
                button = str(packet.get("button", "")).upper()
                if packet.get("cancelled"):
                    self._face_buttons[button] = False
                    self._face_button_down_ns.pop(button, None)
                    return
                self._update_face_button_locked(
                    button,
                    bool(packet.get("pressed", event_type == "button_press")),
                    now_ns,
                )
            for controller_key, button_keys in (
                (
                    "leftController",
                    (("xButton", "X"), ("yButton", "Y")),
                ),
                (
                    "rightController",
                    (("aButton", "A"), ("bButton", "B")),
                ),
            ):
                controller = packet.get(controller_key)
                if not isinstance(controller, dict):
                    continue
                for key, button in button_keys:
                    if key in controller:
                        self._update_face_button_locked(
                            button, bool(controller.get(key)), now_ns
                        )

    def _on_policy_action(self, message: String) -> None:
        try:
            payload = self._parse(message)
            controller, action = policy_to_controller(payload)
        except Exception as exc:
            self._event("policy_action_rejected", str(exc))
            return
        with self._lock:
            if getattr(self, '_launcher_control_active', False):
                return
            envelope = self._timestamp("policy", payload)
            self._trace.append_ring({"wall_ns": envelope["robot_receive_timestamp_ns"], "stream": "policy", "data": envelope})
            self._trace.write("policy", envelope)
            self._last_policy_monotonic_ns = time.monotonic_ns()
            if self._machine.mode == Mode.POLICY_WARMUP:
                proposal_id = str(payload.get("proposal_id", ""))
                observation_ns = payload.get("proposal_observation_timestamp_ns")
                try:
                    observation_age = (time.time_ns() - int(observation_ns)) / 1e9
                    chunk_step = int(payload.get("chunk_step", -1))
                except (TypeError, ValueError):
                    observation_age = float("inf")
                    chunk_step = -1
                if (
                    payload.get("shadow_only") is not True
                    or chunk_step != 0
                    or not proposal_id
                    or len(action) != 21
                    or observation_age < 0.0
                    or observation_age > float(self.get_parameter(
                        "policy_resume_max_observation_age_sec").value)
                ):
                    self._policy_warmup_count = 0
                    self._policy_warmup_proposal_id = ""
                    self._warmup_diagnostics = {
                        "proposal_id": proposal_id,
                        "rejection_reason": "missing_or_stale_shadow_proposal",
                    }
                    return
                reference = finite_vector(self._latest_action, 21)
                if reference is None:
                    self._policy_warmup_count = 0
                    return
                proposal_state = finite_vector(
                    payload.get("proposal_observation_state"), 21
                )
                measured_state = finite_vector(
                    payload.get("measured_policy_state"), 21
                )
                if proposal_state is None or measured_state is None:
                    self._policy_warmup_count = 0
                    self._policy_warmup_proposal_id = ""
                    self._warmup_diagnostics = {
                        "proposal_id": proposal_id,
                        "rejection_reason": "missing_observation_or_measured_state",
                    }
                    return
                action_jump = action_jump_components(action, reference)
                observation_drift = action_jump_components(
                    proposal_state, measured_state
                )
                self._warmup_diagnostics = {
                    "proposal_id": proposal_id,
                    "observation_age_ms": round(observation_age * 1000.0, 3),
                    "action_jump": action_jump,
                    "observation_drift": observation_drift,
                    "rejection_reason": "",
                }
                arm_limit = float(self.get_parameter(
                    "policy_resume_max_arm_jump_deg").value)
                gripper_limit = float(self.get_parameter(
                    "policy_resume_max_gripper_jump_deg").value)
                if (
                    action_jump["arm_deg"] > arm_limit
                    or action_jump["gripper_deg"] > gripper_limit
                    or action_jump["body_deg"] > arm_limit
                    or observation_drift["arm_deg"] > arm_limit
                    or observation_drift["gripper_deg"] > gripper_limit
                    or observation_drift["body_deg"] > arm_limit
                ):
                    self._policy_warmup_count = 0
                    self._policy_warmup_last_action = action
                    self._warmup_diagnostics["rejection_reason"] = (
                        "action_jump_or_observation_drift_exceeded"
                    )
                    return
                if proposal_id != self._policy_warmup_proposal_id:
                    self._policy_warmup_proposal_id = proposal_id
                    self._policy_warmup_count = 0
                    self._policy_warmup_last_action = None
                if self._policy_warmup_last_action is not None:
                    consecutive_jump = max(
                        abs(candidate - previous)
                        for candidate, previous in zip(
                            action[:14], self._policy_warmup_last_action[:14]
                        )
                    )
                    if consecutive_jump > float(self.get_parameter(
                            "policy_resume_max_arm_jump_deg").value):
                        self._policy_warmup_count = 0
                        self._policy_warmup_last_action = action
                        return
                self._policy_warmup_last_action = action
                self._policy_warmup_count += 1
                elapsed = (
                    time.monotonic_ns() - self._policy_warmup_started_ns
                ) / 1e9
                if (
                    self._policy_warmup_count < int(self.get_parameter(
                        "policy_warmup_min_actions").value)
                    or elapsed < float(self.get_parameter(
                        "policy_warmup_min_duration_sec").value)
                ):
                    return
                transition = self._machine.warmup_complete()
                now_ns = time.monotonic_ns()
                if self._handback_started_ns:
                    self._timing_ms["grip_release_to_policy_active"] = (
                        self._elapsed_ms(self._handback_started_ns, now_ns)
                    )
                self._event(
                    "transition",
                    transition.reason,
                    old=transition.old.value,
                    new=transition.new.value,
                    timing_ms=dict(self._timing_ms),
                    warmup=dict(self._warmup_diagnostics),
                )
                return
            if self._machine.mode != Mode.POLICY_ACTIVE:
                return
            if payload.get("shadow_only") is True:
                return
            controller.update({
                "authority_epoch": self._machine.authority_epoch,
                "source_sequence": envelope["source_sequence"],
                "source_timestamp_ns": envelope["source_timestamp_ns"],
                "robot_receive_timestamp_ns": envelope["robot_receive_timestamp_ns"],
            })
            self._selected_joint_pub.publish(String(data=compact(controller)))
            self._selected_gripper_pub.publish(String(data=compact({
                "left_gripper_target_joints_position": [action[14]],
                "right_gripper_target_joints_position": [action[15]],
                "source": "hg_dagger_policy",
                "authority_epoch": self._machine.authority_epoch,
            })))
            self._latest_grippers = action[14:16]
            if self._policy_resume_pending and self._handback_started_ns:
                self._timing_ms["grip_release_to_policy_forward"] = (
                    self._elapsed_ms(
                        self._handback_started_ns, time.monotonic_ns()
                    )
                )
                self._policy_resume_pending = False
                self._event(
                    "policy_resume_forwarded",
                    "first model command forwarded after handback",
                    timing_ms=dict(self._timing_ms),
                )
            if all(key in payload for key in (
                "proposal_id", "chunk_step", "bridge_generation"
            )):
                self._policy_forward_ack_pub.publish(String(data=compact({
                    "proposal_id": str(payload["proposal_id"]),
                    "chunk_step": int(payload["chunk_step"]),
                    "bridge_generation": int(payload["bridge_generation"]),
                    "authority_epoch": self._machine.authority_epoch,
                    "robot_forward_timestamp_ns": time.time_ns(),
                })))

    def _on_policy_status(self, message: String) -> None:
        try:
            status = self._parse(message)
        except Exception as exc:
            self._event("invalid_policy_status", str(exc))
            return
        with self._lock:
            self._policy_status = status
            self._last_policy_status_monotonic_ns = time.monotonic_ns()
            if (
                str(status.get("phase")) == "failed"
                and self._machine.mode in (Mode.POLICY_ACTIVE, Mode.POLICY_WARMUP)
            ):
                self._begin_failure_hold(str(status.get("detail", "VLA bridge failure")))

    def _on_expert_eef(self, message: String) -> None:
        try:
            payload = self._parse(message)
        except Exception as exc:
            self._event("expert_eef_rejected", str(exc))
            return
        with self._lock:
            if getattr(self, '_launcher_control_active', False) or getattr(self, '_launcher_wait_for_release', False):
                return
            envelope = self._timestamp("expert_eef", payload)
            self._timed_eef_pub.publish(String(data=compact(envelope)))
            self._trace.write("vr", envelope)
            has_pose = any(
                f"pos_{side}_in_robot" in payload or f"quat_{side}_in_robot" in payload
                for side in ("left", "right")
            )
            if self._machine.mode not in (Mode.EXPERT_READY, Mode.EXPERT_ACTIVE) or not has_pose:
                return
            if self._machine.mode == Mode.EXPERT_READY:
                if not any(self._grips):
                    return
            payload.update({
                "authority_epoch": self._machine.authority_epoch,
                "robot_receive_timestamp_ns": envelope["robot_receive_timestamp_ns"],
            })
            self._selected_eef_pub.publish(String(data=compact(payload)))
            self._takeover_seen = True
            self._last_expert_monotonic_ns = time.monotonic_ns()
            self._expert_command_pending = True
            self._expert_forwarded_monotonic_ns = self._last_expert_monotonic_ns

    def _on_expert_gripper(self, message: String) -> None:
        try:
            payload = self._parse(message)
        except Exception as exc:
            self._event("expert_gripper_rejected", str(exc))
            return
        with self._lock:
            if getattr(self, '_launcher_control_active', False) or getattr(self, '_launcher_wait_for_release', False):
                return
            if self._machine.mode not in (Mode.EXPERT_READY, Mode.EXPERT_ACTIVE) or not any(self._grips):
                return
            desired = list(self._expert_gripper_desired)
            for index, side in enumerate(("left", "right")):
                value = payload.get(f"{side}_gripper_target_joints_position")
                if not isinstance(value, list) or len(value) != 1:
                    continue
                try:
                    requested = float(value[0])
                except (TypeError, ValueError):
                    continue
                if not (10.0 <= requested <= 360.0):
                    continue
                if self._expert_gripper_pickup_pending[index]:
                    if requested + 2.0 < self._expert_gripper_hold[index]:
                        continue
                    self._expert_gripper_pickup_pending[index] = False
                    requested = max(requested, self._expert_gripper_hold[index])
                    self._event("expert_gripper_pickup", "VR Trigger caught held gripper", side=side)
                desired[index] = requested
            self._expert_gripper_desired = desired
            self._expert_gripper_input_ns = time.monotonic_ns()

    def _publish_expert_gripper_locked(self, now_ns: int) -> None:
        if (
            self._machine.mode != Mode.EXPERT_ACTIVE
            or not any(self._grips)
            or not self._expert_gripper_input_ns
            or (now_ns - self._expert_gripper_input_ns) / 1e9
            > float(self.get_parameter("expert_timeout_sec").value)
        ):
            self._expert_gripper_tick_ns = 0
            return
        previous = self._expert_gripper_tick_ns
        rate = float(self.get_parameter("expert_gripper_rate_hz").value)
        if rate <= 0 or (previous and (now_ns - previous) / 1e9 < 1.0 / rate):
            return
        self._expert_gripper_tick_ns = now_ns
        if not previous:
            return
        step = float(self.get_parameter("expert_gripper_speed_deg_sec").value) * min(
            (now_ns - previous) / 1e9, 0.05
        )
        if step <= 0:
            return
        command = [
            min(360.0, max(10.0, current + max(-step, min(step, target - current))))
            for current, target in zip(self._expert_gripper_command, self._expert_gripper_desired)
        ]
        if all(abs(a - b) < 0.5 for a, b in zip(command, self._expert_gripper_command)):
            return
        self._selected_gripper_pub.publish(String(data=compact({
            "left_gripper_target_joints_position": [command[0]],
            "right_gripper_target_joints_position": [command[1]],
            "source": "hg_dagger_expert_rate_limited",
            "authority_epoch": self._machine.authority_epoch,
        })))
        self._expert_gripper_command = command
        self._latest_grippers = list(command)

    def _on_expert_release(self, message: String) -> None:
        with self._lock:
            if getattr(self, '_launcher_control_active', False) or getattr(self, '_launcher_wait_for_release', False):
                return
            if self._machine.mode in (Mode.EXPERT_ACTIVE, Mode.EXPERT_READY):
                self._selected_release_pub.publish(message)

    def _on_controller_status(self, message: String) -> None:
        try:
            status = self._parse(message)
        except Exception:
            return
        with self._lock:
            self._controller_status = status
            self._controller_status_received_ns = time.monotonic_ns()
            if bool(status.get("emergency_stop_latched")) or status.get("state") == "FAULT":
                if self._machine.mode != Mode.ESTOP:
                    transition = self._machine.estop(str(status.get("reason", "controller fault")))
                    self._finish_intervention_locked(False, transition.reason)
                    self._event("transition", transition.reason, old=transition.old.value, new=transition.new.value)
                return
            if (
                self._machine.mode == Mode.FAILURE_HOLD
                and controller_hold_confirmed(status)
            ):
                if self._hold_to_intervene and any(self._grips):
                    transition = self._machine.held_grip_ready()
                    self._hold_confirmed_ns = time.monotonic_ns()
                    if self._takeover_requested_ns:
                        self._timing_ms["grip_press_to_hold"] = (
                            self._elapsed_ms(
                                self._takeover_requested_ns,
                                self._hold_confirmed_ns,
                            )
                        )
                elif self._hold_to_intervene:
                    return
                else:
                    transition = self._machine.hold_confirmed()
                    self._release_gate_started_ns = time.monotonic_ns()
                self._event(
                    "transition", transition.reason,
                    old=transition.old.value, new=transition.new.value,
                    timing_ms=dict(self._timing_ms),
                )
            elif (
                self._machine.mode == Mode.EXPERT_READY
                and self._expert_command_pending
                and status.get("state") == "ARMED"
                and status.get("target_source") == "cartesian_ik"
            ):
                transition = self._machine.expert_first_command()
                self._expert_command_pending = False
                now_ns = time.monotonic_ns()
                if self._takeover_requested_ns:
                    self._timing_ms["grip_press_to_expert_active"] = (
                        self._elapsed_ms(self._takeover_requested_ns, now_ns)
                    )
                if self._hold_confirmed_ns:
                    self._timing_ms["hold_to_expert_active"] = (
                        self._elapsed_ms(self._hold_confirmed_ns, now_ns)
                    )
                self._event(
                    "transition",
                    transition.reason,
                    old=transition.old.value,
                    new=transition.new.value,
                    timing_ms=dict(self._timing_ms),
                )

    def _on_joint_feedback(self, message: String) -> None:
        try:
            payload = self._parse(message)
            values = [
                float(payload[f"{side}_gripper_state"]["position"][0])
                for side in ("left", "right")
            ]
            if not all(-5.0 <= value <= 365.0 for value in values):
                return
        except (KeyError, IndexError, TypeError, ValueError):
            return
        with self._lock:
            self._measured_grippers = values
            self._gripper_measurement_ns = time.monotonic_ns()

    def _check_gripper_tracking_locked(self, now_ns: int) -> None:
        # Keep the stop reason visible until the operator takes over or resets.
        if self._machine.mode == Mode.FAILURE_HOLD and self._gripper_tracking_warning:
            return
        measured = self._measured_grippers
        policy_active = self._machine.mode == Mode.POLICY_ACTIVE
        expert_active = self._machine.mode == Mode.EXPERT_ACTIVE and any(self._grips)
        if (
            not (policy_active or expert_active)
            or measured is None
            or now_ns - self._gripper_measurement_ns > 1_000_000_000
            or self._controller_status.get("hardware_ready") is not True
        ):
            self._gripper_tracking_side = -1
            self._gripper_tracking_since_ns = 0
            self._gripper_tracking_warning = ""
            return
        target_grippers = (
            self._latest_grippers if policy_active else self._expert_gripper_command
        )
        errors = [
            abs(target - actual)
            for target, actual in zip(target_grippers, measured)
        ]
        side = max(range(2), key=errors.__getitem__)
        if errors[side] <= 30.0:
            self._gripper_tracking_side = -1
            self._gripper_tracking_since_ns = 0
            self._gripper_tracking_warning = ""
            return
        if (
            side != self._gripper_tracking_side
            or abs(measured[side] - self._gripper_tracking_start_measurement) >= 5.0
        ):
            self._gripper_tracking_side = side
            self._gripper_tracking_since_ns = now_ns
            self._gripper_tracking_start_measurement = measured[side]
            self._gripper_tracking_warning = ""
        elif now_ns - self._gripper_tracking_since_ns >= 2_000_000_000:
            warning = ("左" if side == 0 else "右") + "夹爪目标已变化，但实测未动；请检查"
            if not self._gripper_tracking_warning:
                self._event(
                    "gripper_tracking_stalled", warning,
                    target=target_grippers[side], measured=measured[side],
                )
                if policy_active:
                    self._begin_failure_hold(warning)
            self._gripper_tracking_warning = warning

    def _on_mapper_status(self, message: String) -> None:
        try:
            status = self._parse(message)
        except Exception:
            return
        with self._lock:
            self._mapper_status = status

    def _on_vendor_joint_command(self, message: String) -> None:
        # Read-only observation used to reconstruct the exact command label.
        try:
            payload = self._parse(message)
        except Exception:
            return
        with self._lock:
            left = payload.get("left_arm_target_joints_position")
            right = payload.get("right_arm_target_joints_position")
            neck = payload.get("neck_target_joints_position")
            waist = payload.get("leg_waist_target_joints_position")
            if (
                isinstance(left, list) and len(left) == 7
                and isinstance(right, list) and len(right) == 7
                and isinstance(neck, list) and len(neck) == 3
                and isinstance(waist, list) and len(waist) == 4
            ):
                try:
                    self._latest_action = [float(v) for v in (
                        left + right + list(self._vendor_grippers) + neck + waist[2:4]
                    )]
                    self._last_vendor_command_monotonic_ns = time.monotonic_ns()
                    self._last_vendor_command_wall_ns = time.time_ns()
                except (TypeError, ValueError):
                    pass

    def _on_vendor_gripper_command(self, message: String) -> None:
        try:
            payload = self._parse(message)
        except Exception:
            return
        with self._lock:
            for index, side in enumerate(("left", "right")):
                value = payload.get(f"{side}_gripper_target_joints_position")
                if isinstance(value, list) and value:
                    try:
                        self._vendor_grippers[index] = float(value[0])
                    except (TypeError, ValueError):
                        pass
            if self._latest_action is not None:
                self._latest_action[14:16] = self._vendor_grippers
                self._last_gripper_command_monotonic_ns = time.monotonic_ns()
                self._last_gripper_command_wall_ns = time.time_ns()

    def _prepare_intervention_locked(self) -> tuple[str, bool]:
        if self._intervention_id:
            raise RuntimeError("a trial is already active")
        self._intervention_id = f"int-{time.strftime('%Y%m%d-%H%M%S')}-{uuid.uuid4().hex[:8]}"
        self._active_depth = self._depth_next
        self._expert_command_count = 0
        self._takeover_seen = False
        self._recorder_active = False
        self._collector_started_wall_ns = time.time_ns()
        self._dagger_segment = 0
        self._trace.start(
            self._session_id,
            self._intervention_id,
            self._active_depth,
            {
                "schema": "groot-n1.7-21",
                "action_order": [
                    "left_arm_7", "right_arm_7", "left_gripper", "right_gripper",
                    "neck_roll_pitch_yaw", "waist_pitch_yaw",
                ],
                "collector_fifo": self._collector.paths[self._active_depth],
                "trial_mode": "continuous_recording_from_session_start",
            },
        )
        return self._intervention_id, self._active_depth

    def _start_collector_for_trial(
        self, intervention_id: str, depth: bool
    ) -> tuple[bool, str]:
        command = str(self.get_parameter("collector_start_command").value)
        try:
            result = self._collector.start_with_recovery(
                depth,
                command,
                str(self.get_parameter("collector_discard_command").value),
                float(self.get_parameter("collector_command_timeout_sec").value),
            )
        except Exception as exc:
            with self._lock:
                if self._intervention_id == intervention_id:
                    self._trace.finish("collector_start_failed", 0, str(exc), None)
                    self._intervention_id = ""
                    self._recorder_active = False
            return False, f"collector start failed: {exc}"
        with self._lock:
            if self._intervention_id != intervention_id:
                return False, "trial was closed while collector was starting"
            if not result.acknowledged or not result.success:
                self._trace.finish(
                    "collector_start_failed",
                    0,
                    result.message,
                    result.as_dict(),
                )
                self._intervention_id = ""
                self._recorder_active = False
                return False, result.message
            self._recorder_active = True
            self._event(
                "intervention_started",
                "collector acknowledged start before policy inference",
                depth=depth,
                collector_connected=True,
                collector_episode_index=result.episode_index,
                collector_request_id=result.request_id,
                pre_failure_frames=result.frames,
                recording_started_wall_ns=time.time_ns(),
            )
        return True, result.message

    def _finish_intervention_locked(self, requested_save: bool, reason: str) -> bool:
        if not self._intervention_id:
            return False
        # The collector counts accepted synchronized frames, not proxy publishes.
        save = requested_save
        command_name = "collector_save_command" if save else "collector_discard_command"
        result = None
        pending = getattr(self, "_pending_collector_result", None)
        if pending is not None:
            result = self._collector.wait_for_result(
                self._active_depth, pending.event, pending.request_id, 1.0)
            save = pending.event == "save"
        elif self._recorder_active:
            result = self._collector.command_and_wait(
                self._active_depth,
                str(self.get_parameter(command_name).value),
                float(self.get_parameter(
                    "collector_save_timeout_sec" if save else "collector_command_timeout_sec"
                ).value),
            )
        collector_ok = bool(
            result is not None and result.acknowledged and result.success
        )
        if not collector_ok:
            self._pending_collector_result = result
            raise RuntimeError("collector result unresolved; trial retained for reconciliation/recovery")
        self._pending_collector_result = None
        if save and collector_ok and result.event == "save":
            status = "saved"
        elif not save and collector_ok:
            status = "discarded"
        elif save and collector_ok and result.event == "discard":
            status = "discarded_invalid"
        else:
            status = "collector_command_failed"
        self._trace.finish(
            status,
            result.expert_frames,
            reason,
            None if result is None else result.as_dict(),
        )
        self._event(
            "intervention_finished",
            reason,
            status=status,
            frame_count=result.frames,
            expert_frame_count=result.expert_frames,
            collector_acknowledged=bool(result and result.acknowledged),
            collector_success=collector_ok,
            takeover_seen=self._takeover_seen,
            collector_episode_index=(
                None if result is None else result.episode_index
            ),
        )
        self._intervention_id = ""
        self._recorder_active = False
        self._takeover_seen = False
        return status == "saved"

    def _finish_episode_locked(self, reason: str) -> tuple[bool, str]:
        if any(self._grips) or self._machine.mode not in (
            Mode.EXPERT_ACTIVE, Mode.EXPERT_READY, Mode.EXPERT_RELEASE_REQUIRED, Mode.DISARMED
        ):
            return False, "release both Grips in expert mode before saving"
        self._publish_release_hold("finish episode; no policy handback")
        transition = self._machine.disable()
        self._expert_command_pending = False
        self._grip_release_started_ns = 0
        self._event("transition", reason, old=transition.old.value, new=transition.new.value)
        self._save_notice = {"token": time.time_ns(), "level": "info",
                             "text": "正在保存本条数据，请等待完成"}
        self._publish_state()
        saved = False
        error = ""
        try:
            saved = self._finish_intervention_locked(True, reason)
        except Exception as exc:
            error = str(exc)
        finally:
            disabled, detail = self._forward_enable(False)
        if not self._intervention_id:
            self._session_id = ""
        message = "episode saved; session ended" if saved else "episode NOT saved; inspect intervention_finished event"
        if not disabled:
            message += "; controller disable failed: " + detail
        if error:
            message += "; " + error
        self._last_episode_saved = bool(saved)
        self._last_episode_disable_ok = bool(disabled)
        self._save_notice = {
            "token": time.time_ns(),
            "level": "success" if saved and disabled else "error",
            "text": "本条已保存，会话结束" if saved and disabled else "保存或停用失败，请查看终端事件",
        }
        self._event("episode_save_result", message, saved=saved, controller_disabled=disabled)
        return saved and disabled, message

    def _on_finish_episode(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        with self._lock:
            response.success, response.message = self._finish_episode_locked("explicit finish episode")
        return response

    def _resume_locked(self, reason: str) -> None:
        if bool(self.get_parameter("human_finish_only").value):
            raise ValueError("policy handback disabled; use finish_episode to save")
        requested_ns = self._grip_release_started_ns or time.monotonic_ns()
        if self._machine.mode == Mode.FAILURE_HOLD:
            transition = self._machine.cancel_grip_takeover()
            save_intervention = False
        else:
            transition = self._machine.resume()
            save_intervention = True
        self._finish_intervention_locked(save_intervention, reason)
        self._publish_release_hold("policy warmup barrier")
        self._policy_warmup_started_ns = time.monotonic_ns()
        self._policy_warmup_count = 0
        self._policy_warmup_last_action = None
        self._policy_warmup_proposal_id = ""
        self._hold_to_intervene = False
        self._handback_started_ns = requested_ns
        self._policy_resume_pending = True
        self._grip_release_started_ns = 0
        self._warmup_diagnostics = {}
        self._timing_ms["grip_release_to_warmup"] = self._elapsed_ms(
            requested_ns, self._policy_warmup_started_ns
        )
        self._last_policy_monotonic_ns = self._policy_warmup_started_ns
        self._event(
            "transition", transition.reason,
            old=transition.old.value, new=transition.new.value,
            timing_ms=dict(self._timing_ms),
        )

    def _control_tick(self) -> None:
        with self._lock:
            now_ns = time.monotonic_ns()
            if getattr(self, '_launcher_control_active', False):
                return
            self._publish_expert_gripper_locked(now_ns)
            if (
                (
                    self._machine.mode == Mode.EXPERT_ACTIVE
                    or (
                        self._hold_to_intervene
                        and self._machine.mode in (
                            Mode.FAILURE_HOLD, Mode.EXPERT_READY
                        )
                    )
                )
                and not any(self._grips)
                and debounce_elapsed(
                    self._grip_release_started_ns,
                    self._last_grip_sample_ns,
                    float(self.get_parameter("grip_release_debounce_sec").value),
                )
            ):
                if bool(self.get_parameter("human_finish_only").value):
                    if self._machine.mode == Mode.EXPERT_ACTIVE:
                        self._publish_release_hold("expert paused; recording remains active")
                        transition = self._machine.pause_expert()
                        self._expert_command_pending = False
                        self._event("transition", transition.reason,
                                    old=transition.old.value, new=transition.new.value)
                    self._grip_release_started_ns = 0
                else:
                    self._resume_locked("both takeover Grips released after debounce")
                return
            if self._machine.mode == Mode.FAILURE_HOLD:
                timeout = float(self.get_parameter("hold_confirmation_timeout_sec").value)
                if self._hold_sent_monotonic_ns and (now_ns - self._hold_sent_monotonic_ns) / 1e9 > timeout:
                    self._publish_release_hold("repeat hold barrier until controller confirms")
            elif self._machine.mode == Mode.POLICY_ACTIVE and self._last_policy_monotonic_ns:
                timeout = float(self.get_parameter("policy_timeout_sec").value)
                if (now_ns - self._last_policy_monotonic_ns) / 1e9 > timeout:
                    status_age = (
                        float("inf") if not self._last_policy_status_monotonic_ns
                        else (now_ns - self._last_policy_status_monotonic_ns) / 1e9
                    )
                    waiting_phases = {
                        "connecting", "capturing", "inference", "retry", "holding",
                        "executing",
                    }
                    if (
                        status_age <= float(self.get_parameter(
                            "policy_status_timeout_sec").value)
                        and str(self._policy_status.get("phase")) in waiting_phases
                    ):
                        return
                    self._begin_failure_hold("policy action watchdog timeout")
            elif self._machine.mode == Mode.EXPERT_ACTIVE and self._last_expert_monotonic_ns:
                timeout = float(self.get_parameter("expert_timeout_sec").value)
                if (now_ns - self._last_expert_monotonic_ns) / 1e9 > timeout:
                    self._publish_release_hold("expert input watchdog timeout")
                    transition = self._machine.estop(
                        "expert input watchdog timeout"
                    )
                    if bool(self.get_parameter("human_finish_only").value):
                        self._set_notice_locked("手柄断流，已安全停止；本条保留，请用启动器保存或丢弃", "error")
                    else:
                        self._finish_intervention_locked(False, transition.reason)
                    self._event(
                        "transition",
                        transition.reason,
                        old=transition.old.value,
                        new=transition.new.value,
                    )

    def _publish_collector_action(self) -> None:
        with self._lock:
            if (
                self._machine.mode in (Mode.DISARMED, Mode.ESTOP)
                or self._latest_action is None
                or not self._session_id
            ):
                return
            held = (
                self._recorder_active
                and self._takeover_seen
                and self._machine.mode in (Mode.EXPERT_READY, Mode.EXPERT_RELEASE_REQUIRED)
                and controller_hold_confirmed(self._controller_status)
            )
            maximum_age = float(self.get_parameter("expert_timeout_sec").value)
            now_ns = time.monotonic_ns()
            if not held and (
                self._controller_status.get("hardware_ready") is not True
                or self._controller_status.get("state") not in ("ARMED", "HOLDING")
                or self._last_vendor_command_monotonic_ns <= 0
                or self._last_gripper_command_monotonic_ns <= 0
                or (now_ns - self._last_vendor_command_monotonic_ns) / 1e9 > maximum_age
            ):
                return
            # Vendor gripper commands are edge-triggered: a target stays active
            # after the last publication. Refresh only that latched target's
            # recording timestamp while a fresh joint output confirms hardware
            # is enabled; never refresh a stale joint action.
            gripper_latched = (
                not held
                and (now_ns - self._last_gripper_command_monotonic_ns) / 1e9 > maximum_age
            )
            label_timestamp_ns = time.time_ns() if held else self._last_vendor_command_wall_ns
            gripper_timestamp_ns = (
                time.time_ns() if held or gripper_latched
                else self._last_gripper_command_wall_ns
            )
            label_source = "hg_dagger_held_last_action_proxy" if held else "hg_dagger_expert_label_proxy"
            payload = {
                "action": list(self._latest_action),
                "units": "degrees",
                "source": "held_last_vendor_command" if held else "controller_vendor_command_observation",
                "source_timestamp_ns": label_timestamp_ns,
                "authority_epoch": self._machine.authority_epoch,
                "intervention_id": self._intervention_id,
                "control_mode": self._machine.mode.value,
            }
            self._timed_action_pub.publish(String(data=compact(payload)))
            dagger = {
                "timestamp_ns": time.time_ns(),
                "authority_timestamp_ns": self._authority_changed_wall_ns,
                "control_source": (1 if self._machine.mode == Mode.EXPERT_ACTIVE else
                                   0 if self._machine.mode == Mode.POLICY_ACTIVE else 2),
                "is_intervention": int(self._takeover_seen),
                "intervention_id": self._dagger_segment,
                "authority_epoch": self._machine.authority_epoch,
                "trial_id": self._intervention_id,
            }
            self._collector_arm_pub.publish(String(data=compact({
                "dagger": dagger,
                "left_arm_target_joints_position": list(self._latest_action[0:7]),
                "right_arm_target_joints_position": list(self._latest_action[7:14]),
                "neck_target_joints_position": list(self._latest_action[16:19]),
                # Collector upper-waist schema selects indices 2/3.
                "leg_waist_target_joints_position": [
                    0.0, 0.0, self._latest_action[19], self._latest_action[20]
                ],
                "source": label_source,
                "timestamp_ns": payload["source_timestamp_ns"],
                "authority_epoch": self._machine.authority_epoch,
            })))
            self._collector_gripper_pub.publish(String(data=compact({
                "left_gripper_target_joints_position": [self._latest_action[14]],
                "right_gripper_target_joints_position": [self._latest_action[15]],
                "source": (
                    "hg_dagger_latched_vendor_gripper_proxy"
                    if gripper_latched else label_source
                ),
                "timestamp_ns": gripper_timestamp_ns,
                "original_command_timestamp_ns": self._last_gripper_command_wall_ns,
                "authority_epoch": self._machine.authority_epoch,
            })))
            if (
                self._machine.mode == Mode.EXPERT_ACTIVE
                and self._intervention_id
                and self._recorder_active
            ):
                self._expert_command_count += 1

    def _publish_state(self) -> None:
        with self._lock:
            self._check_gripper_tracking_locked(time.monotonic_ns())
            progress = self._collector.progress(self._active_depth, self._intervention_id)
            invalid_event = (
                self._collector.invalid_episode_event(
                    self._active_depth, self._collector_started_wall_ns
                ) if self._intervention_id and self._recorder_active else {}
            )
            state = {
                "collector_fifo": str(self._collector.paths[self._depth_next]),
                "operation_busy": self._button_worker_active or self._session_start_pending,
                "launcher_pending": self._launcher_pending_action,
                "launcher_control_active": self._launcher_control_active,
                "mode": self._machine.mode.value,
                "authority_epoch": self._machine.authority_epoch,
                "session_id": self._session_id,
                "intervention_id": self._intervention_id,
                "depth_next": self._depth_next,
                "active_depth": self._active_depth,
                "controller_state": self._controller_status.get("state"),
                "controller_reason": self._controller_status.get("reason"),
                "quick_reset": {
                    "pending": self._reset_pending,
                    "active": bool(self._controller_status.get("quick_reset_active")),
                },
                "grips": {"left": self._grips[0], "right": self._grips[1]},
                "trial": {
                    "recording": self._recorder_active,
                    "human_takeover": self._takeover_seen,
                    "expert_command_samples": self._expert_command_count,
                    "expert_frames": int(progress.get("expert_frames", 0)),
                    "recorded_frames": int(progress.get("frames", 0)),
                    "save_pending": self._pending_collector_result is not None,
                    "invalid_reason": str(invalid_event.get("reason", "")),
                },
                "grip_release": {
                    "pending": bool(self._grip_release_started_ns),
                    "debounce_sec": float(self.get_parameter(
                        "grip_release_debounce_sec").value),
                },
                "policy_warmup": {
                    "accepted_actions": self._policy_warmup_count,
                    "required_actions": int(self.get_parameter(
                        "policy_warmup_min_actions").value),
                    "minimum_duration_sec": float(self.get_parameter(
                        "policy_warmup_min_duration_sec").value),
                    "diagnostics": dict(self._warmup_diagnostics),
                },
                "timing_ms": dict(self._timing_ms),
                "policy_bridge": dict(self._policy_status),
                "failure_reason": self._last_failure_reason,
                "human_finish_only": bool(self.get_parameter("human_finish_only").value),
                "collection_exited": self._collection_exited,
                "gripper_pickup_pending": dict(zip(("left", "right"), self._expert_gripper_pickup_pending)),
                "gripper_tracking_warning": self._gripper_tracking_warning,
                "gripper_measured": list(self._measured_grippers) if self._measured_grippers else [],
                "gripper_command": list(self._expert_gripper_command),
                "notice": dict(self._save_notice),
            }
            self._state_pub.publish(String(data=compact(state)))
            prompts = {
                Mode.POLICY_ACTIVE: "VLA 正在控制；Grip 接管，B 保存，Y 丢弃，X 仅复位",
                Mode.FAILURE_HOLD: (
                    "失败已触发：" + (self._last_failure_reason or "VLA/操作者请求")
                    + "；正在停止 VLA 并等待机器人保持"
                ),
                Mode.EXPERT_RELEASE_REQUIRED: "机器人已保持：请松开左右 Grip；B 保存，Y 丢弃，X 仅复位",
                Mode.EXPERT_READY: "机器人已保持；按 Grip 增量接管，B 保存，Y 丢弃，X 仅复位",
                Mode.EXPERT_ACTIVE: "手柄已接管；松 Grip 后按 B 保存，Y 丢弃，X 仅复位",
                Mode.POLICY_WARMUP: "VR 已交还；正在验证 VLA 连续输出",
                Mode.ESTOP: "HG-DAgger 已停止输出，请检查故障",
                Mode.DISARMED: (
                    "采集已退出；请重启 dagger_collector.sh"
                    if self._collection_exited else "未开始；点按 A 开始 VLA 与录制"
                ),
            }
            web_status = dict(self._mapper_status)
            web_status["hg_dagger"] = {
                **state,
                "prompt": (
                    "本条录制已失效，不能保存；按 Y 丢弃，X 仅复位后重试"
                    if invalid_event else prompts[self._machine.mode]
                ),
                "haptic_token": self._machine.authority_epoch,
            }
            self._web_status_pub.publish(String(data=compact(web_status)))

    def _forward_enable(self, enabled: bool) -> tuple[bool, str]:
        timeout = 12.0 if enabled else 5.0
        if not self._controller_enable.wait_for_service(timeout_sec=2.0):
            return False, "controller enable service is unavailable"
        future = self._controller_enable.call_async(SetBool.Request(data=enabled))
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout):
            return False, "controller enable request timed out"
        try:
            result = future.result()
        except Exception as exc:
            return False, f"controller enable request failed: {exc}"
        return bool(result.success), str(result.message)

    def _on_set_session_enabled(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if request.data:
            with self._lock:
                if self._collection_exited:
                    response.success = False
                    response.message = "collection exited; restart dagger_collector.sh"
                    return response
                if self._session_start_pending:
                    response.success = False
                    response.message = "session startup is already in progress"
                    return response
                if self._intervention_id:
                    response.success = False
                    response.message = "previous trial unresolved; reconcile save before starting"
                    return response
                if self._machine.mode != Mode.DISARMED:
                    response.success = False
                    response.message = (
                        "session enable requires DISARMED; disable the current "
                        f"session first (mode={self._machine.mode.value})"
                    )
                    return response
            try:
                self._trace.prepare_root()
            except Exception as exc:
                response.success = False
                response.message = f"data path is unavailable: {exc}"
                return response
            with self._lock:
                if self._machine.mode != Mode.DISARMED:
                    response.success = False
                    response.message = (
                        "session enable requires DISARMED; disable the current "
                        f"session first (mode={self._machine.mode.value})"
                    )
                    return response
                self._session_id = (
                    f"session-{time.strftime('%Y%m%d-%H%M%S')}-"
                    f"{uuid.uuid4().hex[:8]}"
                )
                self._session_start_pending = True
                self._save_notice = {}
                try:
                    intervention_id, depth = self._prepare_intervention_locked()
                except Exception as exc:
                    try:
                        self._trace.finish("trial_setup_failed", 0, str(exc))
                    except Exception as cleanup_exc:
                        self.get_logger().error(f"trial trace cleanup failed: {cleanup_exc}")
                    self._intervention_id = ""
                    self._recorder_active = False
                    self._session_start_pending = False
                    self._session_id = ""
                    response.success = False
                    response.message = f"trial setup failed: {exc}"
                    return response

            # Start the recorder before hardware authority is enabled. This
            # keeps the first policy observation and action inside the same
            # episode, while no Grip callback can be blocked by FIFO startup.
            recorder_ok, recorder_message = self._start_collector_for_trial(
                intervention_id, depth
            )
            if not recorder_ok:
                with self._lock:
                    self._session_start_pending = False
                    self._session_id = ""
                    self._set_notice_locked(
                        f"录制启动失败：{recorder_message}", "error"
                    )
                response.success = False
                response.message = recorder_message
                return response

            success, message = self._forward_enable(True)
            if not success:
                with self._lock:
                    self._session_start_pending = False
                    self._close_trial_locked(f"hardware enable failed: {message}")
                    self._set_notice_locked(f"硬件使能失败：{message}", "error")
                self._forward_enable(False)
                response.success = False
                response.message = message
                return response

            with self._lock:
                self._session_start_pending = False
                transition = self._machine.enable()
                self._timing_ms = {}
                self._warmup_diagnostics = {}
                self._grip_release_started_ns = 0
                self._handback_started_ns = 0
                self._policy_resume_pending = False
                self._last_policy_monotonic_ns = time.monotonic_ns()
                self._event(
                    "transition",
                    transition.reason,
                    old=transition.old.value,
                    new=transition.new.value,
                    recording_started=True,
                )
            response.success = True
            response.message = "VLA inference and recording started"
            return response

        with self._lock:
            self._session_start_pending = False
            self._close_trial_locked("session disabled")
            self._grip_release_started_ns = 0
            self._policy_resume_pending = False
            self._set_notice_locked("已停止推理，本条数据已丢弃", "info")
            self._publish_state()
        success, message = self._forward_enable(False)
        response.success = success
        response.message = message
        return response

    def _on_request_failure(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        with self._lock:
            try:
                self._begin_failure_hold("VLA failure service request")
                response.success = True
                response.message = "failure hold requested"
            except ValueError as exc:
                response.success = False
                response.message = str(exc)
        return response

    def _on_request_takeover(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        with self._lock:
            try:
                if self._machine.mode == Mode.POLICY_ACTIVE:
                    self._begin_failure_hold("explicit takeover service request")
                elif self._machine.mode != Mode.EXPERT_READY:
                    raise ValueError(f"takeover is invalid in {self._machine.mode.value}")
                response.success = True
                response.message = "takeover pending expert target"
            except ValueError as exc:
                response.success = False
                response.message = str(exc)
        return response

    def _on_request_resume(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        with self._lock:
            try:
                self._resume_locked("explicit resume service request")
                response.success = True
                response.message = "policy warmup entered"
            except ValueError as exc:
                response.success = False
                response.message = str(exc)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HgDaggerSupervisor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
