"""DAgger motion/session implementation with collection-only operator adapters.

Motion selection, clutch/gripper handling and policy forwarding come from the
pinned DAgger source. This module adds collection receipts, saved-pose reset,
operator attachment, explicit A/B/X/Y semantics and status/haptic presentation.
"""
import json
import os
import time
import threading
from pathlib import Path
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .control.core import Mode, finite_vector
from .control.supervisor_node import HgDaggerSupervisor
from .compat import CommandGate, CompatibilityPublisher, FEATURES, SERVICE, VERSION
from .status_cache import CachedCollector
from .trace import AsyncTrace
from vr_feedback import feedback_packet
from .robot_profile import resolve_robot

class ProvenancePublisher:
    """Attach controller-publication origins; never restamp an old held target."""
    def __init__(self, node, publisher, gripper):
        self.node, self.publisher, self.gripper = node, publisher, gripper

    def publish(self, message):
        packet = json.loads(message.data)
        origin = self.node._command_origins[self.gripper]
        if origin is None:
            return
        packet["original_command_timestamp_ns"] = origin[0]
        packet["origin_authority_epoch"] = origin[1]
        self.publisher.publish(String(data=json.dumps(packet, separators=(",", ":"))))

class CollectorDaggerSupervisor(HgDaggerSupervisor):
    def __init__(self):
        # Reject identity/model errors before the base creates ROS entities,
        # services and worker resources. A failed constructor cannot clean up
        # a partially owned control stack reliably.
        robot = resolve_robot(os.environ)
        self._cancel_start = False
        self._closing = False
        self._finish_outcome = None
        self._failure_button_start_ns = 0
        self._failure_button_last_ns = 0
        self._failure_button_released = False
        self._failure_button_trial = ""
        super().__init__()
        self._feedback_pub = self.create_publisher(String, "/collector/feedback", 10)
        self._speech_pub = self.create_publisher(String, f"/topic_tts_{robot.suffix}", 10)
        self._command_origins = {False: None, True: None}
        self._collector_arm_pub = ProvenancePublisher(self, self._collector_arm_pub, False)
        self._collector_gripper_pub = ProvenancePublisher(self, self._collector_gripper_pub, True)
        self.create_subscription(String, "/collector_dagger/controller_output", self._on_controller_output,
                                 QoSProfile(depth=8, reliability=ReliabilityPolicy.RELIABLE))
        self._trace = AsyncTrace(self._trace)
        self._collector = CachedCollector(self._collector)
        self._saved_reset = self.create_client(Trigger, '/openarmx_teleop_vr_306_v4/quick_reset')
        self.declare_parameter("controller_ready_timeout_sec", 20.0)
        from rcl_interfaces.srv import SetParametersAtomically
        self._operator_gate = CommandGate()
        self._state_pub = CompatibilityPublisher(self._state_pub, dict(
            version=VERSION, instance_id=self._operator_gate.instance_id, robot_id=robot.robot_id, robot_profile=robot.model,
            domain_id=os.environ.get('ROS_DOMAIN_ID', '0'),
            command_service=SERVICE, features=FEATURES,
            hardware_publish=os.environ.get('DAGGER_PUBLISH', '0') == '1',
            with_depth=os.environ.get('WITH_DEPTH', '0') == '1',
            with_eef_state=os.environ.get('WITH_EEF_STATE', '0') == '1',
            with_eef_action=os.environ.get('WITH_EEF_ACTION', '0') == '1',
            task_text=os.environ.get('TASK_TEXT', '')))
        self.create_service(SetParametersAtomically, SERVICE, self._on_attached_command)
        self.get_logger().info("AutoLife DAgger: A=start, GL/GR=takeover, B=save, Y tap=discard / hold 1s=failure save, X=reset")

    def _feedback(self, event, text):
        if hasattr(self, "_feedback_pub"):
            self._feedback_pub.publish(String(data=json.dumps(feedback_packet(event))))
            self._speech_pub.publish(String(data=json.dumps({"status": "play", "text": text}, ensure_ascii=False)))

    def _mechanical_reset_worker(self, button, save_note=''):
        with self._lock:
            self._reset_pending = True
            self._set_notice_locked(save_note + '按本轮任务姿态全身复位中', 'warning')
            self._publish_state()
        success, detail = False, ''
        enabled = False
        try:
            from .dependencies import validate_saved_reset
            from openarmx_teleop_vr_306_v4.reset_pose_loader import load_quick_reset_targets
            validate_saved_reset(Path(os.environ['DAGGER_DEPENDENCY_ROOT']))
            load_quick_reset_targets(resolve_robot(os.environ).robot_id,
                                     os.environ['DAGGER_RESET_STATE_PATH'])
            pose_name = os.environ.get('DAGGER_RESET_POSE_NAME', '本轮任务姿态')
            with self._lock:
                self._set_notice_locked(save_note + '全身复位目标：' + pose_name, 'warning')
                self._publish_state()
            # Claim a hold-only lease; the vendor HOME planner can fail while
            # reporting settled feedback, so it must not define this reset.
            self._wait_for_reset_heartbeat()
            requested = time.monotonic_ns()
            enabled = True
            accepted, detail = self._forward_enable(True)
            if not accepted:
                raise RuntimeError(detail)
            ready, detail = self._wait_for_reset_enable(requested)
            if not ready:
                raise RuntimeError(detail)
            if not self._saved_reset.wait_for_service(timeout_sec=2):
                raise RuntimeError('保存姿态复位接口不可用')
            future = self._saved_reset.call_async(Trigger.Request())
            done = threading.Event()
            future.add_done_callback(lambda _: done.set())
            if not done.wait(float(self.get_parameter('quick_reset_request_timeout_sec').value)):
                raise RuntimeError('复位请求结果未知；不会自动重试')
            result = future.result()
            if result is None or not result.success:
                raise RuntimeError(getattr(result, 'message', '复位没有回执'))
            if not self._wait_for_quick_reset():
                with self._lock:
                    reason = self._controller_status.get('reason', '到位未确认')
                raise RuntimeError('全身目标未到位：' + str(reason))
            success, detail = self._open_reset_grippers()
        except Exception as exc:
            detail = str(exc)
        finally:
            if enabled:
                disabled, message = self._forward_enable(False)
                if not disabled:
                    success, detail = False, '复位后停止输出未确认：' + message
            with self._lock:
                self._reset_pending = False
                self._set_notice_locked(save_note + ('全身复位完成，双夹爪已张开；按 A 开始' if success
                                        else '全身复位失败：' + detail), 'success' if success else 'error')
                self._publish_state()

    def _on_vendor_joint_command(self, message):
        pass  # Untagged DDS echoes do not prove which authority produced them.

    def _on_vendor_gripper_command(self, message):
        pass

    def _on_controller_output(self, message):
        try:
            packet = self._parse(message)
            stamp, epoch, gripper = packet["timestamp_ns"], packet["origin_authority_epoch"], packet["gripper"]
            if (type(stamp) is not int or type(epoch) is not int or type(gripper) is not bool
                    or not 0 <= time.time_ns() - stamp <= 500_000_000 or epoch < -1):
                return
            command = String(data=json.dumps(packet["command"]))
        except (KeyError, TypeError, ValueError):
            return
        with self._lock:
            if packet.get("session_id") != self._session_id:
                return
            previous = self._command_origins[gripper]
            if previous and stamp <= previous[0]:
                return
            if gripper:
                super()._on_vendor_gripper_command(command)
                self._last_gripper_command_monotonic_ns = time.monotonic_ns()
                self._last_gripper_command_wall_ns = stamp
            else:
                super()._on_vendor_joint_command(command)
                self._last_vendor_command_wall_ns = stamp
            self._command_origins[gripper] = (stamp, epoch)

    def _expert_packet_current(self, message):
        try:
            packet = self._parse(message)
        except (ValueError, TypeError):
            return False
        return (packet.get("authority_epoch") == self._machine.authority_epoch
                and packet.get("collector_session_id") == self._session_id)

    def _on_expert_eef(self, message):
        with self._lock:
            if self._expert_packet_current(message):
                super()._on_expert_eef(message)

    def _on_expert_gripper(self, message):
        with self._lock:
            if self._expert_packet_current(message):
                super()._on_expert_gripper(message)

    def _on_expert_release(self, message):
        with self._lock:
            if self._expert_packet_current(message):
                super()._on_expert_release(message)

    def _update_face_button_locked(self, button, pressed, now_ns):
        if button == "Y":
            return  # The tracked VR stream distinguishes tap from hold.
        # One command per press, independent of hold duration. Storage/RPC
        # waits run in the worker, never in this control callback.
        if button in ("B", "X"):
            previous = self._face_buttons.get(button, False)
            self._face_buttons[button] = bool(pressed)
            if pressed and not previous:
                self._on_face_button_click_locked(button, now_ns)
            return
        super()._update_face_button_locked(button, pressed, now_ns)

    def _on_policy_action(self, message):
        try:
            payload = self._parse(message)
        except (TypeError, ValueError):
            return
        with self._lock:
            if (self._cancel_start or self._closing
                    or self._machine.mode.value != "POLICY_ACTIVE"
                    or payload.get("collector_session_id") != self._session_id
                    or payload.get("authority_epoch") != self._machine.authority_epoch):
                self._reject_policy(payload, "authority revoked")
                return
            action = finite_vector(payload.get("action"), 21)
            measured = finite_vector(payload.get("measured_policy_state"), 21)
            if action is None or measured is None:
                self._reject_policy(payload, "invalid action/state")
                return
            # Tracking lag is not an adjacent-action jump. The downstream V4
            # controller owns joint bounds, trajectory limiting and collisions;
            # do not stop an episode merely for an 8/10-degree tracking error.
            if any(not 10 <= value <= 360 for value in action[14:16]):
                detail = f"policy gripper range: expected [10, 360], got {action[14:16]}"
                if self._machine.mode.value == "POLICY_ACTIVE":
                    self._begin_failure_hold(detail)
                    self._publish_state()
                self._reject_policy(payload, detail)
                return
            super()._on_policy_action(message)

    def _reject_policy(self, payload, reason):
        if all(key in payload for key in ("proposal_id", "chunk_step", "bridge_generation", "authority_epoch")):
            receipt = {key: payload[key] for key in (
                "proposal_id", "chunk_step", "bridge_generation", "authority_epoch")}
            receipt.update(accepted=False, reason=reason)
            self._policy_forward_ack_pub.publish(String(data=json.dumps(receipt)))

    def _on_policy_status(self, message):
        try:
            packet = self._parse(message)
        except (TypeError, ValueError):
            return
        with self._lock:
            if packet.get("collector_session_id") != self._session_id:
                return
            super()._on_policy_status(message)

    def _on_request_takeover(self, request, response):
        with self._lock:
            if not any(self._grips):
                response.success, response.message = False, "Press a Grip to take over"
            elif self._machine.mode in (Mode.POLICY_ACTIVE, Mode.POLICY_WARMUP):
                self._begin_failure_hold("explicit takeover request", hold_to_intervene=True)
                response.success, response.message = True, "Expert authority selected"
            else:
                response.success = self._machine.mode in (Mode.EXPERT_READY, Mode.EXPERT_ACTIVE)
                response.message = f"Current mode: {self._machine.mode.value}"
            return response

    def _on_controller_status(self, message):
        try:
            status = self._parse(message)
        except (TypeError, ValueError):
            return
        if status.get("emergency_stop_latched") or status.get("state") == "FAULT":
            with self._lock:
                self._controller_status = status
                self._controller_status_received_ns = time.monotonic_ns()
                if self._machine.mode.value == "ESTOP":
                    return
                transition = self._machine.estop(str(status.get("reason", "controller fault")))
                self._event("transition", transition.reason, old=transition.old.value, new=transition.new.value)
                self._set_notice_locked("Controller fault; output stopped; episode retained for B/Y", "error")
                self._publish_state()
            return  # Do not block a control callback on video encoding/FIFO.
        super()._on_controller_status(message)

    def _publish_state(self):
        with self._lock:
            if isinstance(self._collector, CachedCollector):
                self._collector.select(self._active_depth, self._intervention_id, self._collector_started_wall_ns)
            if (self._intervention_id and self._recorder_active
                    and self._machine.mode.value not in ("DISARMED", "ESTOP")):
                invalid = self._collector.invalid_episode_event(self._active_depth, self._collector_started_wall_ns)
                if (isinstance(self._collector, CachedCollector) and self._collector.age > 1.0
                        and time.time_ns() - self._collector_started_wall_ns > 1_000_000_000):
                    invalid = {"reason": "recorder status storage stalled"}
                if invalid:
                    self._cancel_start = True
                    self._publish_release_hold("recorder episode invalid; output revoked")
                    transition = self._machine.disable()
                    self._event("transition", "recorder episode invalid", old=transition.old.value,
                                new=transition.new.value)
                    self._set_notice_locked(f"Recording unavailable: {invalid.get('reason')}; output stopped", "error")
                    if not self._button_worker_active:
                        self._spawn_button_worker_locked("invalid-hold", lambda: self._forward_enable(False))
            super()._publish_state()

    def _begin_failure_hold(self, reason, *, hold_to_intervene=False):
        # DAgger initializes the current-pose/gripper pickup and revokes policy.
        super()._begin_failure_hold(reason, hold_to_intervene=hold_to_intervene)
        if hold_to_intervene:
            # The controller's atomic epoch handoff checks physical freshness.
            # HTTP cancellation and recorder I/O are never takeover barriers.
            transition = self._machine.select_expert_authority()
            self._last_expert_monotonic_ns = time.monotonic_ns()
            self._takeover_seen = True
            self._event("transition", transition.reason, old=transition.old.value, new=transition.new.value)
            self._publish_state()
            self._timing_ms["grip_to_authority_publish"] = self._elapsed_ms(
                self._takeover_requested_ns, time.monotonic_ns())

    def _wait_for_reset_heartbeat(self):
        """Wait for the controller's receipt, not just ROS service discovery."""
        requested = time.monotonic_ns()
        deadline = time.monotonic() + 10.
        while time.monotonic() < deadline:
            with self._lock:
                if self._closing or self._cancel_start:
                    raise RuntimeError('复位已取消')
                status = dict(self._controller_status)
                received = self._controller_status_received_ns
            age = time.monotonic_ns() - received
            if received > requested and 0 <= age < 500_000_000:
                if status.get('emergency_stop_latched') or status.get('state') in ('FAULT', 'ESTOP', 'E_STOP'):
                    raise RuntimeError(str(status.get('reason', '控制器故障')))
                if status.get('collector_heartbeat_ready') is True:
                    return
            time.sleep(.02)
        raise RuntimeError('复位尚未使能：等待控制器确认监管心跳超时')

    def _start_collector_for_trial(self, intervention_id, depth):
        # The recorder reads trace metadata to obtain this trial's identity.
        try:
            self._trace.barrier()
        except Exception as exc:
            try:
                self._trace.finish("collector_start_failed", 0, str(exc))
            except Exception as cleanup_exc:
                self.get_logger().error(f"trial trace cleanup failed: {cleanup_exc}")
            with self._lock:
                # No recorder command has been sent at this point.
                self._intervention_id = ""
                self._recorder_active = False
            return False, f"trace startup failed before recording: {exc}"
        return super()._start_collector_for_trial(intervention_id, depth)

    def _hold_locked(self, reason):
        self._publish_release_hold(reason)
        transition = self._machine.disable()
        self._expert_command_pending = False
        self._event("transition", reason, old=transition.old.value, new=transition.new.value)
        self._publish_state()

    def _wait_for_controller_ready(self, requested_ns):
        """A SetBool ACK only starts enabling; wait for fresh physical readiness.

        This runs on the start worker, never a ROS callback. Y/X/exit remain
        responsive, and policy authority stays DISARMED until SYNC is settled.
        """
        deadline = time.monotonic() + float(self.get_parameter("controller_ready_timeout_sec").value)
        while time.monotonic() < deadline:
            with self._lock:
                if self._cancel_start or self._closing:
                    return False, "start cancelled while waiting for controller readiness"
                status = dict(self._controller_status)
                received_ns = self._controller_status_received_ns
            age_ns = time.monotonic_ns() - received_ns
            if received_ns > requested_ns and 0 <= age_ns < 500_000_000:
                if status.get("emergency_stop_latched") or status.get("state") in ("FAULT", "E_STOP", "ESTOP"):
                    return False, str(status.get("reason", "controller fault"))
                if (status.get("hardware_enabled") is True and status.get("hardware_ready") is True
                        and status.get("state") == "ARMED" and not status.get("hardware_enable_pending")):
                    return True, "controller SYNC settled and hardware ready"
            time.sleep(.02)
        return False, "controller readiness timeout; policy not started"

    def _on_attached_command(self, request, response):
        from rcl_interfaces.msg import ParameterType, SetParametersResult
        from std_srvs.srv import Trigger
        params = request.parameters
        if (len(params) != 1 or params[0].name != 'collector_command'
                or params[0].value.type != ParameterType.PARAMETER_STRING):
            response.result = SetParametersResult(successful=False, reason='Expected one collector_command string')
            return response
        def execute(action):
            if self._closing:
                return False, 'Host closing'
            if action != 'reset' and (self._button_worker_active or self._session_start_pending or self._reset_pending):
                return False, 'Host busy; inspect state'
            if action == 'start' and (self._machine.mode != Mode.DISARMED or self._intervention_id):
                return False, 'Finish current episode and release Grips first'
            if action in ('finish', 'fail', 'discard') and not self._intervention_id:
                return False, 'No episode to finish'
            reply = self._on_launcher_action(action, Trigger.Response())
            return reply.success, reply.message
        with self._lock:
            success, reason = self._operator_gate.dispatch(params[0].value.string_value, execute)
        response.result = SetParametersResult(successful=success, reason=reason)
        return response

    def _event(self, kind, reason, **extra):
        super()._event(kind, reason, **extra)
        if kind == "transition" and extra.get("new") == "EXPERT_ACTIVE" and extra.get("old") != "EXPERT_ACTIVE":
            self._feedback("takeover", "人工接管")

    def _prepare_intervention_locked(self):
        result = super()._prepare_intervention_locked()
        self._command_origins = {False: None, True: None}
        self._collector.select(result[1], result[0], self._collector_started_wall_ns)
        return result

    def _forward_enable(self, enabled):
        if enabled and (self._closing or self._cancel_start):
            return False, "start cancelled"
        started = time.monotonic_ns()
        ok, detail = super()._forward_enable(enabled)
        if enabled and ok:
            ok, detail = self._wait_for_controller_ready(started)
            if self._closing or self._cancel_start:
                ok, detail = False, "start cancelled while enabling"
            if not ok:
                super()._forward_enable(False)
        return ok, detail

    def _on_face_button_click_locked(self, button, now_ns):
        if self._launcher_control_active or self._launcher_wait_for_release:
            return
        action = {"A": "start", "B": "finish", "X": "reset", "Y": "discard"}.get(button)
        if action:
            self._dispatch_operator(action, Trigger.Response(), launcher=False)

    def _update_vr_failure_button_locked(self, packet, now_ns):
        if packet.get("source") == "keyboard":
            if packet.get("button") == "Y" and packet.get("type") == "button_press" and not packet.get("cancelled"):
                self._on_face_button_click_locked("Y", now_ns)
            return
        if packet.get("button") == "Y" and packet.get("cancelled"):
            self._failure_button_start_ns = 0
            self._failure_button_released = False
        # Reliable controller edges have no pose; only tracked frames time Y.
        if "leftController" not in packet and "rightController" not in packet:
            return
        trial = self._intervention_id
        if trial != self._failure_button_trial or now_ns - self._failure_button_last_ns > 300_000_000:
            self._failure_button_start_ns = 0
            self._failure_button_released = False
        self._failure_button_trial = trial
        self._failure_button_last_ns = now_ns
        left, right = packet.get("leftController"), packet.get("rightController")
        tracked = all(isinstance(c, dict) and isinstance(c.get("position"), dict)
                      and finite_vector([c["position"].get(k) for k in ("x", "y", "z")], 3) is not None
                      for c in (left, right))
        if (not tracked or left.get("yButton") not in (False, True)
                or not trial or self._closing or self._button_worker_active
                or self._session_start_pending or self._reset_pending
                or self._launcher_control_active or self._launcher_wait_for_release
                or self._pending_collector_result is not None
                or left.get("xButton")
                or any(right.get(k) for k in ("aButton", "bButton"))):
            self._failure_button_start_ns = 0
            self._failure_button_released = False
            return
        if not left["yButton"]:
            started = self._failure_button_start_ns
            self._failure_button_start_ns = 0
            self._failure_button_released = True
            if started:
                action = "fail" if now_ns - started >= 1_000_000_000 else "discard"
                self._dispatch_operator(action, Trigger.Response(), launcher=False)
        elif self._failure_button_released:
            if not self._failure_button_start_ns:
                self._failure_button_start_ns = now_ns
            elif now_ns - self._failure_button_start_ns >= 1_000_000_000:
                # Consume until a fresh release, including busy/new-trial gaps.
                self._failure_button_start_ns = 0
                self._failure_button_released = False
                self._dispatch_operator("fail", Trigger.Response(), launcher=False)

    def _on_launcher_action(self, action, response):
        with self._lock:
            return self._dispatch_operator(action, response, launcher=True)

    def _dispatch_operator(self, action, response, *, launcher):
        if os.environ.get('DAGGER_RESET_ONLY') == '1' and action != 'reset':
            response.success, response.message = False, '独立复位模式不启动推理或录制'
            return response
        if action not in ("start", "finish", "fail", "discard", "reset") or self._closing:
            response.success, response.message = False, "操作无效；退出请用终端 Q"
            return response
        if self._button_worker_active or self._session_start_pending or self._reset_pending:
            response.success, response.message = False, "操作正在处理，请等待结果；没有排队"
        elif action == "reset" and (self._intervention_id or self._pending_collector_result is not None):
            response.success, response.message = False, "请先 B 保存或 Y 丢弃，确认后 X 仅复位；数据保留"
        elif action == "start" and (self._machine.mode != Mode.DISARMED or self._intervention_id or (not launcher and any(self._grips))):
            response.success, response.message = False, "请先结束当前条并松开握持键"
        elif action in ("finish", "fail", "discard") and not self._intervention_id:
            response.success, response.message = False, "没有待处理数据"
        else:
            if launcher:
                self._claim_launcher_locked()
            if action != "start":
                self._hold_locked("operator " + action)
            self._cancel_start = False
            target = {"start": self._start_session_worker,
                      "finish": lambda: self._finish_collection_trial(True),
                      "fail": lambda: self._finish_collection_trial(True, task_success=False),
                      "discard": lambda: self._finish_collection_trial(False),
                      "reset": lambda: self._mechanical_reset_worker("X")}[action]
            self._spawn_button_worker_locked(action, target)
            response.success, response.message = True, "已受理，请等待状态确认"
        if not response.success:
            self._set_notice_locked(response.message, "warning")
            self._publish_state()
        return response

    def _finish_collection_trial(self, save, *, task_success=True):
        # Close motion authority before storage waits, using the DAgger FIFO and
        # reconciliation logic. No implicit mechanical reset on either branch.
        with self._lock:
            pending = self._pending_collector_result
            if pending is not None:
                # Reconcile the original command; a later click cannot relabel
                # an episode whose save may already have committed.
                save = pending.event == "save"
                metadata = self._finish_outcome if save else None
                if save and metadata is None:
                    raise RuntimeError("原保存标签未知，请检查回执；不会重新保存或改写标签")
            else:
                metadata = {"trial_id": self._intervention_id, "task_success": task_success,
                            "human_takeover": bool(self._takeover_seen)} if save else None
                self._finish_outcome = metadata
            outcome_kind = ("任务失败" if not metadata["task_success"] else
                            "人工纠正成功" if metadata["human_takeover"] else "模型自主成功") if save else ""
            self._set_notice_locked("正在保存：" + outcome_kind if save else "正在丢弃", "info")
            self._publish_state()
        disabled, detail = self._forward_enable(False)
        # The one lifecycle worker owns the trial; authority is already
        # DISARMED. Do not hold the VR/control lock while encoding or waiting.
        reason = ("B success save" if metadata["task_success"] else "F failure save") if save else "Y discard"
        saved = self._finish_intervention_locked(save, reason, metadata=metadata)
        self._trace.barrier()
        with self._lock:
            if not self._intervention_id:
                self._session_id = ""
                self._finish_outcome = None
            message = "本条已保存：" + outcome_kind if saved else ("本条未保存，请检查录制错误" if save else "本条已丢弃")
            if not disabled:
                message += "；硬件停用未确认：" + detail
            self._set_notice_locked(message, "success" if disabled and (saved or not save) else "error")
            self._publish_state()

    def _on_finish_episode(self, request, response):
        return self._on_launcher_action("finish", response)

    def shutdown_session(self):
        with self._lock:
            self._closing = self._cancel_start = True
            self._hold_locked("collector exiting")
        self._forward_enable(False)
        deadline = time.monotonic() + 5
        while self._button_worker_active and time.monotonic() < deadline:
            time.sleep(.05)
        self._trace.close()
        self._collector.close()
