"""Adapt the inspected V4 authority stack, without patching its live installation.

Only this supervisor may feed the V4 controller. Policy/VR inputs stay upstream
of that controller's hold, re-anchor, stale-input and joint-limit protections.
The colleague's private package remains an explicit, version-checked runtime
dependency; the ordinary collector never imports it.
"""

from __future__ import annotations

import threading
import time
import uuid
import json
import os

from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy
from autolife_hg_dagger_mzj_300.supervisor_node import HgDaggerSupervisor
from autolife_hg_dagger_mzj_300.core import AuthorityStateMachine, Mode, finite_vector
from .trace import AsyncTrace
from .status_cache import CachedCollector
from vr_feedback import feedback_packet
from .compat import CommandGate, CompatibilityPublisher, FEATURES, SERVICE, VERSION


class ImmediateAuthority(AuthorityStateMachine):
    def take_over(self, reason):
        if self.mode not in (Mode.POLICY_ACTIVE, Mode.POLICY_WARMUP):
            raise ValueError(f"takeover is invalid in {self.mode.value}")
        return self._move(Mode.EXPERT_ACTIVE, reason)


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
        self._cancel_start = False
        self._closing = False
        self._pending_reset = False
        self._recording_start_sent = False
        self._work_thread = None
        super().__init__()
        self._machine = ImmediateAuthority()
        self._feedback_pub = self.create_publisher(String, "/collector/feedback", 10)
        self._speech_pub = self.create_publisher(String, "/topic_tts_0_300", 10)
        self._command_origins = {False: None, True: None}
        self._collector_arm_pub = ProvenancePublisher(self, self._collector_arm_pub, False)
        self._collector_gripper_pub = ProvenancePublisher(self, self._collector_gripper_pub, True)
        self.create_subscription(String, "/collector_dagger/controller_output", self._on_controller_output,
                                 QoSProfile(depth=8, reliability=ReliabilityPolicy.RELIABLE))
        self._trace = AsyncTrace(self._trace)
        self._collector = CachedCollector(self._collector)
        self.declare_parameter("controller_ready_timeout_sec", 20.0)
        from rcl_interfaces.srv import SetParametersAtomically
        self._operator_gate = CommandGate()
        self._state_pub = CompatibilityPublisher(self._state_pub, dict(
            version=VERSION, instance_id=self._operator_gate.instance_id, robot_id='300',
            domain_id=os.environ.get('ROS_DOMAIN_ID', '0'),
            command_service=SERVICE, features=FEATURES,
            hardware_publish=os.environ.get('DAGGER_PUBLISH', '0') == '1',
            with_depth=os.environ.get('WITH_DEPTH', '0') == '1',
            task_text=os.environ.get('TASK_TEXT', '')))
        self.create_service(SetParametersAtomically, SERVICE, self._on_attached_command)
        self.get_logger().info("Collector DAgger: A=start, GL/GR=takeover, B=save, Y=discard, X=reset")

    def _feedback(self, event, text):
        if hasattr(self, "_feedback_pub"):
            self._feedback_pub.publish(String(data=json.dumps(feedback_packet(event))))
            self._speech_pub.publish(String(data=json.dumps({"status": "play", "text": text}, ensure_ascii=False)))

    def _set_notice_locked(self, text, level="info"):
        super()._set_notice_locked(text, level)
        if level == "error":
            self._feedback("error", "操作异常，请检查终端")
        elif text == "Inference and recording started":
            self._feedback("start", "开始推理和录制")
        elif text == "Saving":
            self._feedback("saving", "正在保存")
        elif text == "Discarding episode":
            self._feedback("discarding", "正在丢弃")
        elif text.startswith("Episode saved:"):
            self._feedback("save", "已保存")
        elif text == "Episode discarded":
            self._feedback("discard", "已丢弃")
        elif text.startswith("Reset confirmed;"):
            self._feedback("reset", "复位完成")

    def _begin_failure_hold(self, reason, **kwargs):
        if not kwargs.get("hold_to_intervene"):
            return super()._begin_failure_hold(reason, **kwargs)
        # Called under the authority lock by the existing Grip edge handler.
        # Publish the new epoch immediately: neither HTTP, recorder ACK nor a
        # controller HOLDING status is a prerequisite for expert input.
        now = time.monotonic_ns()
        transition = self._machine.take_over(reason)
        self._takeover_requested_ns = now
        self._hold_confirmed_ns = 0
        self._hold_to_intervene = True
        self._grip_release_started_ns = 0
        self._release_gate_started_ns = 0
        self._handback_started_ns = 0
        self._policy_resume_pending = False
        self._expert_command_pending = False
        self._last_expert_monotonic_ns = now
        self._last_failure_reason = str(reason)
        self._takeover_seen = True
        self._timing_ms = {}
        self._warmup_diagnostics = {}
        self._save_notice = {}
        measured_fresh = (self._measured_grippers is not None
                          and now - self._gripper_measurement_ns < 1_000_000_000)
        pickup = self._measured_grippers if measured_fresh else self._vendor_grippers
        held = [min(330., max(10., float(value))) for value in pickup]
        self._expert_gripper_hold = list(held)
        self._expert_gripper_desired = list(held)
        self._expert_gripper_command = list(held)
        self._expert_gripper_pickup_pending = [True, True]
        self._expert_gripper_input_ns = self._expert_gripper_tick_ns = 0
        self._publish_state()
        self._timing_ms["grip_to_authority_publish"] = (time.monotonic_ns() - now) / 1e6
        self._event("transition", reason, old=transition.old.value, new=transition.new.value)
        self._feedback("takeover", "人工接管")

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
        # One command per press, independent of hold duration. Storage/RPC
        # waits run in the worker, never in this control callback.
        if button in ("B", "Y", "X"):
            previous = self._face_buttons.get(button, False)
            self._face_buttons[button] = bool(pressed)
            if pressed and not previous:
                self._on_face_button_click_locked(button, now_ns)
            return
        super()._update_face_button_locked(button, pressed, now_ns)

    def _hold_locked(self, reason):
        self._publish_release_hold(reason)
        transition = self._machine.disable()
        self._expert_command_pending = False
        self._event("transition", reason, old=transition.old.value, new=transition.new.value)
        self._publish_state()

    def _on_face_button_click_locked(self, button, now_ns):
        if self._closing:
            return
        mode = self._machine.mode.value
        if button == "X":
            if (self._intervention_id or self._recorder_active
                    or self._pending_collector_result is not None
                    or self._button_worker_active or self._session_start_pending
                    or self._reset_pending):
                self._set_notice_locked("请先按 B 保存或 Y 丢弃，等待结果确认后再按 X 复位；当前数据保留", "warning")
                return
            self._cancel_start = True
            self._pending_reset = True
            self._hold_locked("X reset requested; no episode is open")
            self._run_worker("reset", lambda: self._finish_trial(save=False, reset=True))
            return
        if self._button_worker_active or self._reset_pending or self._session_start_pending:
            self._set_notice_locked("Operation in progress; wait for confirmation", "warning")
            return
        if button == "A":
            if mode != "DISARMED" or self._intervention_id or any(self._grips):
                self._set_notice_locked("Release both Grips and finish the previous episode before A", "warning")
                return
            self._cancel_start = False
            self._run_worker("start", self._start_trial)
        elif button in ("B", "Y") and self._intervention_id:
            save = button == "B"
            self._hold_locked("B save requested" if save else "Y discard requested; no reset")
            self._run_worker("save" if save else "discard", lambda: self._finish_trial(save=save))

    def _run_worker(self, name, operation):
        if self._button_worker_active:
            return
        self._button_worker_active = True
        self._button_worker_name = name

        def run():
            try:
                operation()
            except Exception as exc:
                with self._lock:
                    self._hold_locked(f"DAgger {name} failed")
                    self._set_notice_locked(f"{name} failed: {exc}; inspect logs before retry", "error")
                self._forward_enable(False)
            finally:
                with self._lock:
                    self._button_worker_active = False
                    self._button_worker_name = ""
                    self._publish_state()

        self._work_thread = threading.Thread(target=run, name=f"collector-dagger-{name}", daemon=True)
        self._work_thread.start()

    def _start_trial(self):
        # All trace/FIFO/RPC waits are in this worker, outside the authority
        # lock. Never inherit the legacy enable-failure path's locked discard.
        try:
            with self._lock:
                self._session_start_pending = True
                self._session_id = f"session-{uuid.uuid4().hex}"
                self._command_origins = {False: None, True: None}
                self._recording_start_sent = False
                self._trace.prepare_root()
                trial, depth = self._prepare_intervention_locked()
                self._collector.select(depth, trial, self._collector_started_wall_ns)
            self._trace.barrier()
            if self._cancel_start or self._closing:
                return
            self._recording_start_sent = True
            result = self._collector.command_and_wait(depth, "start", float(
                self.get_parameter("collector_command_timeout_sec").value))
            with self._lock:
                if not result.acknowledged or not result.success:
                    self._pending_collector_result = result
                    raise RuntimeError(f"recorder start unresolved: {result.message}")
                self._recorder_active = True
            requested_ns = time.monotonic_ns()
            enabled, reason = self._forward_enable(True)
            if enabled:
                enabled, reason = self._wait_for_controller_ready(requested_ns)
            if not enabled:
                self._forward_enable(False)
            with self._lock:
                if not enabled or self._cancel_start or self._closing:
                    self._hold_locked(f"start not enabled: {reason}; B/Y closes the episode")
                    return
                transition = self._machine.enable()
                self._last_policy_monotonic_ns = time.monotonic_ns()
                self._event("transition", transition.reason, old=transition.old.value, new=transition.new.value)
                self._set_notice_locked("Inference and recording started", "success")
                self._publish_state()
        finally:
            with self._lock:
                self._session_start_pending = False

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

    def _forward_enable(self, enabled):
        if enabled and (self._cancel_start or self._closing):
            return False, "start cancelled before enabling hardware"
        result = super()._forward_enable(enabled)
        # A reset/exit arriving while the enable RPC is in flight must not
        # revive policy ownership after the request completes.
        if enabled and (self._cancel_start or self._closing):
            super()._forward_enable(False)
            return False, "start cancelled while enabling hardware"
        return result

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
            if any(not 10 <= value <= 330 for value in action[14:16]):
                detail = f"policy gripper range: expected [10, 330], got {action[14:16]}"
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
                        self._run_worker("invalid-hold", lambda: self._forward_enable(False))
            super()._publish_state()

    def _finish_trial(self, *, save, reset=False):
        """Stop action authority, close the episode with ACK, then optionally reset.

        Save/encode and service waits are outside the authority lock. A failed
        or ambiguous save stays latched: B/Y reconcile the original request ID,
        never replay save or discard an uncertain partially committed episode.
        """
        with self._lock:
            if reset and (self._intervention_id or self._recorder_active
                          or self._pending_collector_result is not None):
                self._pending_reset = False
                self._set_notice_locked("请先按 B 保存或 Y 丢弃，再按 X 复位；当前数据保留", "warning")
                return
            self._hold_locked("ending episode")
            trial, depth = self._intervention_id, self._active_depth
            pending = self._pending_collector_result
            self._set_notice_locked("Saving" if save else "Discarding episode", "info")
            if trial and not self._recording_start_sent:
                # Trace setup may fail/cancel before sending any recorder
                # command. There is no episode to reconcile in this case.
                try:
                    self._trace.finish("cancelled_before_recording", 0, "start cancelled")
                except RuntimeError as exc:
                    self._trace.error = str(exc)
                self._intervention_id = self._session_id = ""
                self._recorder_active = False
                trial = ""
        saved = False
        if trial:
            if pending is not None:
                result = self._collector.wait_for_result(depth, pending.event, pending.request_id, 1.0)
            else:
                command = "save" if save else "discard"
                result = self._collector.command_and_wait(
                    depth, command, float(self.get_parameter(
                        "collector_save_timeout_sec" if save else "collector_command_timeout_sec").value))
            if not result.acknowledged or not result.success:
                with self._lock:
                    self._pending_collector_result = result
                    self._set_notice_locked("Episode result unresolved; B/Y checks the same request; no reset", "error")
                self._forward_enable(False)
                return
            saved = result.event == "save"
            if result.event == "start":
                # A late start ACK is not a discard ACK. Close the now-known
                # recording before clearing trial IDs or permitting reset.
                with self._lock:
                    self._pending_collector_result = None
                return self._finish_trial(save=save, reset=reset)
            try:
                self._trace.finish("saved" if saved else "discarded", result.frames,
                                   "operator save" if saved else "operator discard", result.as_dict())
            except RuntimeError as exc:
                # Dataset ACK is authoritative. Diagnostic queue failure must
                # not make a later button replay an already committed save.
                self._trace.error = str(exc)
            with self._lock:
                self._pending_collector_result = None
                self._intervention_id = ""
                self._session_id = ""
                self._recorder_active = False
                self._takeover_seen = False
                self._set_notice_locked(
                    f"Episode saved: {result.frames} frames, {result.expert_frames} expert frames"
                    if saved else "Episode discarded", "success")
                self._publish_state()
        disabled, detail = self._forward_enable(False)
        if not disabled:
            with self._lock:
                self._pending_reset = False
                self._set_notice_locked(f"Controller disable not confirmed: {detail}; no reset", "error")
            return
        if reset and not self._closing:
            # Reset can move ALL body joints and grippers. It runs only after
            # recorder ACK, never while a training episode is still open.
            with self._lock:
                self._pending_reset = False
                self._feedback("resetting", "正在复位，请松开握持键")
            super()._mechanical_reset_worker("X")
            with self._lock:
                if self._save_notice.get("level") == "success":
                    self._set_notice_locked("Reset confirmed; release Grips, then A for a new episode", "success")

    def _on_launcher_action(self, action, response):
        # The optional desktop launcher must use the same workflow, not the
        # colleague's legacy save-and-reset mapping.
        mapping = {"start": "A", "finish": "B", "discard": "Y", "reset": "X"}
        with self._lock:
            if action not in mapping or self._closing:
                response.success, response.message = False, "Use collector terminal Q to exit"
            else:
                self._on_face_button_click_locked(mapping[action], time.monotonic_ns())
                response.success, response.message = True, "Request received; inspect control_state for outcome"
        return response

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
            if action == 'start' and (self._machine.mode != Mode.DISARMED or self._intervention_id or any(self._grips)):
                return False, 'Finish current episode and release Grips first'
            if action in ('finish', 'discard') and not self._intervention_id:
                return False, 'No episode to finish'
            reply = self._on_launcher_action(action, Trigger.Response())
            return reply.success, reply.message
        with self._lock:
            success, reason = self._operator_gate.dispatch(params[0].value.string_value, execute)
        response.result = SetParametersResult(successful=success, reason=reason)
        return response

    def _on_finish_episode(self, request, response):
        # Keep the legacy ROS endpoint, not its blocking save-under-lock path.
        return self._on_launcher_action("finish", response)

    def _on_set_session_enabled(self, request, response):
        # A browser hardware-toggle must not bypass episode bookkeeping.
        with self._lock:
            if request.data:
                self._on_face_button_click_locked("A", time.monotonic_ns())
                response.success, response.message = True, "Start requested; inspect control_state"
            else:
                self._cancel_start = True
                self._hold_locked("browser disabled session")
                if not self._button_worker_active:
                    self._run_worker("stop", lambda: self._finish_trial(save=False))
                response.success, response.message = True, "Policy revoked; stopping session without reset"
        return response

    def shutdown_session(self):
        with self._lock:
            self._closing = True
            self._cancel_start = True
            self._pending_reset = False
            self._hold_locked("collector exiting")
        self._forward_enable(False)
        thread = self._work_thread
        if thread is not None:
            thread.join(timeout=5.0)
        self._trace.close()
        self._collector.close()
