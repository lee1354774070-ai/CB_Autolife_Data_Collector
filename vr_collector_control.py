#!/usr/bin/env python3
"""VR episode controls with acknowledged storage and guarded V4 reset.

Button decoding and session logic are ROS-independent for deterministic tests.
ROS callbacks stay responsive while a single worker waits for a recorder receipt.
Speech uses the existing robot TTS service; this tool does not change its volume.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from cli_help import show_requested_parameter_help
from collector_control import SessionPaths, format_status, process_is_running, read_recorder_pid, run_command
from vr_feedback import feedback_packet


FACE_BUTTONS = {("r", 4): "start", ("r", 5): "save", ("l", 4): "reset", ("l", 5): "discard"}


def decode_controls(payload: str) -> tuple[bool, set[tuple[str, int]]]:
    """Read factory JSON: l/r.b[1].p is grip; indices 4/5 are X/Y or A/B.

    Incomplete or malformed packets are rejected, not interpreted as releases.
    In particular, the string 'false' must never count as a pressed button.
    """
    return _decode_controls(json.loads(payload))


def _decode_controls(data):
    if "leftController" in data or "rightController" in data:
        hands = [data[name] for name in ("leftController", "rightController")]
        faces = set()
        for hand, packet, names in zip(("l", "r"), hands, (("xButton", "yButton"), ("aButton", "bButton"))):
            for name in ("gripActive", *names):
                # V4 WebXR serializes face buttons as 0/1, Grips as bool.
                # Accept only those wire values; strings/floats are not presses.
                value = packet.get(name)
                if type(value) not in (bool, int) or value not in (0, 1):
                    raise ValueError("Incomplete V4 button packet")
            faces.update((hand, i) for i, name in zip((4, 5), names) if packet[name])
        return all(packet["gripActive"] for packet in hands), faces
    hands = [data[hand]["b"] for hand in ("l", "r")]
    for buttons in hands:
        if not isinstance(buttons, list) or len(buttons) < 6:
            raise ValueError("Incomplete VR button packet")
        for index in (1, 4, 5):
            value = buttons[index]["p"]
            if not isinstance(value, (bool, int)) or value not in (0, 1):
                raise ValueError("Button pressure must be boolean or 0/1")
    grips = all(buttons[1]["p"] for buttons in hands)
    faces = {(hand, i) for hand, buttons in zip(("l", "r"), hands) for i in (4, 5) if buttons[i]["p"]}
    return grips, faces


class ButtonGestures:
    """Require a neutral sample, then one face press/release with both grips held.

    Grips released, mixed face buttons, invalid input, or a stream gap cancel
    the gesture. Starting the process with A held cannot start recording.
    """

    def __init__(self, a_long_press_sec: float | None = None):
        self.a_long_press_sec = a_long_press_sec
        self.last_release = float("-inf")
        self.reset()

    def reset(self):
        self.armed = False
        self.candidate = None
        self.pressed_at = None

    def update(self, grips, faces, now):
        if len(faces) > 1:
            self.reset()
            return None
        if not grips:
            self.reset()
            # Neutral may precede the Grip chord; do not require an extra packet
            # with both Grips already held before accepting the face press.
            self.armed = not faces
            return None
        if not faces:
            command, self.candidate = self.candidate, None
            pressed_at, self.pressed_at = self.pressed_at, None
            self.armed = True
            # Filter contact/packet bounce; Y never doubles as an exit button.
            if command:
                if now - self.last_release < 0.2:
                    return None
                self.last_release = now
            if command == "save" and self.a_long_press_sec is not None:
                return "a_long" if now - pressed_at >= self.a_long_press_sec else "a_short"
            return command
        if self.armed:
            command = FACE_BUTTONS[next(iter(faces))]
            if self.candidate is not None and self.candidate != command:
                self.reset()
            else:
                if self.candidate is None:
                    self.pressed_at = now
                self.candidate = command
        return None


class VrSession:
    """One in-flight command; no delayed queue of accidental button presses."""

    def __init__(self, submit, speak, *, start_delay=3.0, input_timeout=0.75,
                 subtask_mode=False, a_long_press_sec=1.0, log=print, feedback=None,
                 reset_supported=True, subtask_b_rearm_sec=2.0):
        self.submit, self.speak, self.log = submit, speak, log
        self.feedback = feedback or (lambda event: None)
        self.reset_supported = reset_supported
        self.start_delay, self.input_timeout = start_delay, input_timeout
        self.subtask_mode = subtask_mode
        self.subtask_b_rearm_sec = subtask_b_rearm_sec
        self.subtask_progress = {}
        self.buttons = ButtonGestures(a_long_press_sec=a_long_press_sec if subtask_mode else None)
        self.last_input = None
        self.start_at = None
        self.countdown = None
        self.pending = None
        self.pending_command = None
        self.quit_after_pending = False
        self.state = "idle"
        self.status_key = None
        self.status_time = 0.0
        self.event_key = None
        self.grips_released = False
        self.last_busy_notice = float("-inf")
        self.last_controls = None
        self.subtask_b_locked = False
        self.subtask_b_released_at = None

    def on_payload(self, payload, now):
        try:
            data = json.loads(payload)
            grips, faces = _decode_controls(data)
        except (ValueError, TypeError, KeyError, IndexError):
            self.grips_released = False
            self.buttons.reset()
            self.subtask_b_released_at = None
            return
        if self.last_input is not None and now - self.last_input > self.input_timeout:
            self.buttons.reset()
            self.subtask_b_released_at = None
        self.last_input = now
        self.grips_released = (not any(data[name]["gripActive"] for name in ("leftController", "rightController"))
                               if "leftController" in data else not any(data[h]["b"][1]["p"] for h in ("l", "r")))
        controls = (bool(grips), tuple(sorted(faces)))
        controls_changed = controls != self.last_controls
        if controls_changed:
            self.last_controls = controls
            self.log(f"[VR input] GL+GR={bool(grips)}; faces={sorted(faces)}")
        if self.subtask_b_locked:
            # A retry burst may outlast the ACK; require fresh, quiet input
            # afterwards instead of allowing elapsed time or stale packets to rearm B.
            if self.pending is not None or ("r", 5) in faces:
                self.subtask_b_released_at = None
            elif self.subtask_b_released_at is None:
                self.subtask_b_released_at = now
            elif now - self.subtask_b_released_at >= self.subtask_b_rearm_sec:
                self.subtask_b_locked = False
                self.log("[VR] B ready for the next subtask mark")
            if ("r", 5) in faces:
                self.buttons.reset()
                if controls_changed:
                    self.log(f"[VR] Repeated B ignored: wait for acknowledgement, then release B for {self.subtask_b_rearm_sec:g}s")
                return
        command = self.buttons.update(grips, faces, now)
        if command:
            self.log(f"[VR] Button accepted: {command}")
            self.command(command, now)

    def command(self, command, now):
        if command == "a_short":
            if self.state != "recording":
                self.speak("请按A开始录制")
                return
            command = "mark_subtask"
        elif command == "a_long":
            if self.state == "idle" and self.start_at is None:
                self.speak("请按A开始录制")
                return
            command = "save"
        if self.pending is not None:
            if command == "quit":
                self.quit_after_pending = True
                self.speak("等待当前操作完成后退出")
            else:
                self.log(f"[VR] Command ignored: {self.pending_command} awaiting recorder acknowledgement")
                if now - self.last_busy_notice >= 2:
                    self.last_busy_notice = now
                    self.speak("操作处理中，请等待结果，不需要重复按键")
            return
        if self.subtask_b_locked and command in ("mark_subtask", "save"):
            self.log(f"[VR] Repeated B ignored: wait for acknowledgement, then release B for {self.subtask_b_rearm_sec:g}s")
            return
        if self.state == "exiting":
            return
        if command == "start":
            if self.start_at is not None:
                return
            if self.state != "idle":
                self.speak("当前无法开始，请检查终端或先丢弃当前数据")
                return
            self.start_at = now + self.start_delay
            self.countdown = None
            self.tick(now)
            return
        if self.start_at is not None:
            self.start_at = None
            if command not in ("quit", "reset"):
                self.speak("已取消倒计时")
                self.feedback("cancel")
                return
        if self.state == "unknown" and command != "quit":
            self.speak("操作结果未知，请检查终端并重新启动采集工具")
            self.feedback("error")
            return
        if command == "reset":
            # Reset never closes or discards an episode.
            if self.state != "idle":
                self.speak("请先按B保存或Y丢弃，再按X复位")
                self.feedback("error")
                return
            if not self.reset_supported:
                self.speak("原厂复位接口尚未接入，请使用原厂复位功能")
                self.feedback("error")
                return
            self._send("reset")
            return
        self._send(command)

    def _send(self, command):
        if self.subtask_mode and command == "mark_subtask":
            self.subtask_b_locked = True
            self.subtask_b_released_at = None
        self.pending_command = command
        self.pending_started_at = time.monotonic()
        self.pending = self.submit(command)
        self.log(f"[VR] Submitted: {command}; " + ("requesting recorder shutdown" if command == "quit" else "awaiting recorder result"))
        final_mark = command == "mark_subtask" and (
            self.subtask_progress.get("total", 0) > 0
            and self.subtask_progress.get("confirmed", 0) + 1 == self.subtask_progress["total"]
        )
        if command == "save" or final_mark:
            self.speak("保存中" if self.subtask_mode else "正在保存，请等待结果")
            self.feedback("saving")
        elif command == "reset":
            self.state = "resetting"
            self.speak("正在复位，请松开握持键")
            self.feedback("resetting")
        elif command == "discard":
            self.speak("正在丢弃")
            self.feedback("discarding")
        elif command == "quit":
            self.speak("正在退出采集")

    def tick(self, now):
        if self.pending is not None and self.pending.done():
            command = self.pending_command
            pending, self.pending = self.pending, None
            try:
                status = pending.result()
                if command == "quit":
                    self.state = "exiting"  # Sent, not a claim that finalization succeeded.
                elif command == "reset":
                    if not status.get("success"):
                        raise RuntimeError(status.get("message", "reset failed"))
                    self.state = "idle"
                    self.buttons.reset()
                    self.speak("复位完成")
                    self.feedback("reset")
                else:
                    self.log(f"[VR] Recorder acknowledgement latency: {(time.monotonic() - self.pending_started_at) * 1000:.1f} ms ({command})")
                    self.accept_status(status)
            except (OSError, RuntimeError, TimeoutError) as exc:
                self.state = "unknown"
                self.log(f"[VR] Command result unknown: {exc}. Do not retry automatically.")
                self.speak("操作结果未知，请检查采集终端")
                self.feedback("error")
            if self.quit_after_pending:
                self.quit_after_pending = False
                self._send("quit")
        if self.start_at is None:
            return
        if self.last_input is None or now - self.last_input > self.input_timeout:
            self.start_at = None
            self.buttons.reset()
            self.speak("VR信号中断，已取消倒计时")
            return
        remaining = max(0, math.ceil(self.start_at - now))
        if self.countdown is not None and remaining < self.countdown:
            # A delayed callback must not skip spoken numbers and start early.
            remaining = self.countdown - 1
            self.start_at = now + remaining
        if remaining == 0:
            self.start_at = None
            self._send("start")
        elif remaining != self.countdown:
            self.countdown = remaining
            self.log(f"[VR] Countdown: {remaining}")
            self.speak({3: "三", 2: "二", 1: "一"}.get(remaining, str(remaining)))
            self.feedback("countdown")

    def accept_status(self, status):
        """Consume receipts from either VR or keyboard without double speech."""
        if not isinstance(status, dict) or not status:
            return
        key = (status.get("request_id"), status.get("wall_time"))
        stamp = float(status.get("wall_time", 0))
        if key == self.status_key or stamp < self.status_time:
            return
        self.status_key = key
        # Do not carry a partially held gesture across a start/save/mark receipt.
        # A press begun while saving must never become a fresh idle-state start.
        self.buttons.reset()
        self.status_time = stamp
        self.start_at = None
        self.log(format_status(status))
        if not status.get("success"):
            event = status.get("event")
            message = str(status.get("message", ""))
            if (event in ("save", "discard") and message == f"no pending episode to {event}"
                    and status.get("recording") is False and not status.get("episode_invalid")):
                self.state = "idle"
                self.speak("当前没有待处理数据，可以按A开始下一条")
                self.feedback("cancel")
                return
            if status.get("publication_uncertain"):
                self.state = "unknown"
            elif status.get("episode_invalid"):
                self.state = "invalid"
            elif message.startswith(("save failed:", "discard failed:")):
                self.state = "unknown"
            elif status.get("recording"):
                self.state = "recording"
            self.speak("操作未完成，请检查采集终端")
            self.feedback("error")
            return
        event = status.get("event")
        subtasks = status.get("subtasks", {})
        self.subtask_progress = subtasks
        if event == "start":
            self.state = "recording"
            self.speak("开始" if self.subtask_mode else "开始录制")
            self.feedback("start")
        elif event == "save" and int(status.get("frames", 0)) > 0:
            self.state = "idle"
            self.feedback("save")
            if self.subtask_mode:
                self.speak("已保存" if subtasks.get("complete") else "已保存，标注未完成")
            else:
                self.speak(f"保存成功，共{status.get('total_saved_episodes', 0)}条，本条{status.get('frames', 0)}帧，可以开始下一次采集")
        elif event == "save_queued":
            self.state = "idle"
            self.speak("后台保存中，可以开始下一条")
        elif event == "seal":
            self.state = "idle"
            self.speak("本批已封口，可以开始下一条")
        elif event == "mark_subtask":
            if self.subtask_mode:
                self.subtask_b_locked = True
                self.subtask_b_released_at = None
            self.state = "recording"
            self.speak("下一步")
            self.feedback("mark")
        elif event == "discard":
            self.state = "idle"
            invalid = str(status.get("message", "")).startswith("invalid episode discarded:")
            self.speak("无效数据已丢弃，未保存" if invalid else "已丢弃，可以开始下一次采集")
            self.feedback("discard")
        else:
            self.state = "unknown"
            self.speak("收到异常确认，请检查采集终端")

    def accept_background(self, status):
        if not status:
            return
        seen = getattr(self, '_background_seen', set())
        for result in status.get('completed', []):
            if result['job_id'] not in seen:
                seen.add(result['job_id'])
                self.log(f"[SAVED BACKGROUND] Episode {result['episode_index']} | {result['frames']} frames")
                self.speak(f"第{result['episode_index'] + 1}条已保存")
        self._background_seen = seen
        if status.get('error') and status['error'] != getattr(self, '_background_error', None):
            self._background_error = status['error']
            self.log('BACKGROUND SAVE FAILED: ' + status['error'])
            self.speak('后台保存失败，暂存数据保留，请检查终端')
        # Completion of a previous episode must never reset the current gesture,
        # countdown, recording state, or pending command.

    def accept_event(self, event):
        if not isinstance(event, dict) or event.get("event") != "episode_invalidated":
            return
        key = (event.get("wall_time"), event.get("reason"))
        if key == self.event_key or float(event.get("wall_time", 0)) <= self.status_time:
            return
        self.event_key = key
        # A slow acknowledgement must not undo a newer invalidation or cause
        # a stale success cue after the operator has been told to discard.
        self.status_time = float(event.get("wall_time", 0))
        self.buttons.reset()
        self.state = "invalid"
        self.start_at = None
        self.log(f"[EPISODE INVALID] {event.get('reason')}; discard before starting again")
        self.speak("录制异常，请停止遥操并丢弃")
        self.feedback("error")


def read_json(path):
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


class ReceiptCache:
    """Poll atomic recorder files off the ROS thread, parsing only changes.

    NAS stat/read calls can block indefinitely. The daemon owns those calls;
    callbacks only copy an immutable snapshot under a short lock. A failed read
    never replaces an accepted receipt with a fabricated empty acknowledgement.
    """

    def __init__(self, paths, interval=.1):
        self.paths = tuple(paths)
        self.interval = interval
        self._values = tuple({} for _ in self.paths)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._run, name="vr-receipts", daemon=True)
        self._worker.start()

    def snapshot(self):
        with self._lock:
            return self._values

    def _run(self):
        signatures = [None] * len(self.paths)
        values = list(self._values)
        while not self._stop.is_set():
            for index, path in enumerate(self.paths):
                try:
                    stat = path.stat()
                    signature = (stat.st_ino, stat.st_mtime_ns, stat.st_size)
                    if signature == signatures[index]:
                        continue
                    value = json.loads(path.read_text(encoding="utf-8"))
                    if not isinstance(value, dict):
                        continue
                    signatures[index], values[index] = signature, value
                except (OSError, ValueError):
                    continue
            with self._lock:
                self._values = tuple(values)
            self._stop.wait(self.interval)

    def close(self):
        self._stop.set()
        self._worker.join(timeout=.2)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--base-dir", type=Path, required=True, help="Session directory containing the recorder FIFO and PID file.")
    parser.add_argument("--topic", required=True, help="Factory String VR input topic, normally /control_topic_0_<robot-id>.")
    parser.add_argument("--tts-topic", required=True, help="Existing robot TTS String topic, normally /topic_tts_0_<robot-id>.")
    parser.add_argument("--start-delay", type=float, default=3, help="Countdown seconds before sending start; 0 disables countdown.")
    parser.add_argument("--command-timeout", type=float, default=300, help="Maximum seconds waiting for a real recorder acknowledgement.")
    parser.add_argument("--no-speech", action="store_true", help="Do not publish TTS messages; keep terminal status output.")
    parser.add_argument("--subtask-mode", action="store_true", help="A starts; while recording short B marks a subtask, long B saves.")
    parser.add_argument("--reset-prefix", default="/openarmx_teleop_vr_306_v4", help="Inspected V4 controller namespace for guarded reset and status. No direct vendor-reset fallback.")
    parser.add_argument("--feedback-topic", default="/collector/feedback", help="Feedback events consumed by the collector VR web extension.")
    parser.add_argument("--motion-lock-file", type=Path, help="Recorder's shared motion lock, required for X reset.")
    parser.add_argument("--a-long-press-sec", type=float, default=1.0, help="Minimum B hold time (legacy option name) for early save in subtask mode; fires only on release.")
    parser.add_argument("--subtask-b-rearm-sec", type=float, default=2.0, help="Continuous B release seconds after a subtask acknowledgement before another B mark or early save; finite and positive.")
    parser.add_argument("--check-config", action="store_true", help="Validate arguments without importing ROS or starting any process.")
    show_requested_parameter_help(parser)
    args = parser.parse_args()
    if not math.isfinite(args.start_delay) or not 0 <= args.start_delay <= 30:
        parser.error("--start-delay must be finite and between 0 and 30")
    if not math.isfinite(args.command_timeout) or args.command_timeout <= 0:
        parser.error("--command-timeout must be finite and positive")
    if not math.isfinite(args.a_long_press_sec) or not 0.2 <= args.a_long_press_sec <= 10:
        parser.error("--a-long-press-sec must be finite and between 0.2 and 10")
    if not math.isfinite(args.subtask_b_rearm_sec) or args.subtask_b_rearm_sec <= 0:
        parser.error("--subtask-b-rearm-sec must be finite and positive")
    return args


def main():
    args = parse_args()
    if args.check_config:
        return
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    # Bash background jobs may inherit SIGINT=ignored. Restore handlers so
    # launcher cleanup cancels acknowledgement waits and releases the IPC lock.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    rclpy.init()
    node = Node("vr_collector_control")
    paths = SessionPaths(args.base_dir)
    pid = read_recorder_pid(paths.pidfile)
    publisher = node.create_publisher(String, args.tts_topic, 10) if not args.no_speech else None
    cancel = threading.Event()
    worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="recorder_ack")
    feedback_pub = node.create_publisher(String, args.feedback_topic, 10)

    def speak(text):
        print(f"[VR speech{' disabled' if publisher is None else ''}] {text}", flush=True)
        if publisher is not None:
            publisher.publish(String(data=json.dumps({"status": "play", "text": text}, ensure_ascii=False)))

    def submit(command):
        if command == "reset":
            return worker.submit(reset)
        return worker.submit(run_command, args.base_dir, command, command != "quit", args.command_timeout, cancel)

    session = VrSession(submit, speak, start_delay=args.start_delay, subtask_mode=args.subtask_mode,
                        a_long_press_sec=args.a_long_press_sec, subtask_b_rearm_sec=args.subtask_b_rearm_sec,
                        log=lambda text: print(text, flush=True),
                        reset_supported=not args.topic.startswith("/control_topic_"),
                        feedback=lambda event: feedback_pub.publish(String(data=json.dumps(feedback_packet(event)))))
    from vr_reset import GuardedReset
    reset = GuardedReset(node, args.reset_prefix,
                         lambda: session.grips_released and session.last_input is not None
                         and time.monotonic() - session.last_input < session.input_timeout, cancel,
                         motion_lock=args.motion_lock_file)
    # Ignore receipts left over from a previous session. The launcher creates
    # this helper only after recorder/camera startup, before accepting controls.
    previous = read_json(paths.status_file)
    session.status_key = (previous.get("request_id"), previous.get("wall_time"))
    session.status_time = float(previous.get("wall_time", 0))
    receipts = ReceiptCache((paths.status_file, args.base_dir / ".official_episode_event.json",
                             args.base_dir / ".official_background_saves.json"))
    ready_at = time.monotonic() + 1.0
    input_started_at = time.monotonic()
    input_missing = False

    def monitor():
        nonlocal ready_at, input_missing
        now = time.monotonic()
        status, event, background = receipts.snapshot()
        session.accept_background(background)
        session.accept_status(status)
        session.accept_event(event)
        if not process_is_running(pid):
            if session.state != "exiting":
                speak("采集进程已退出，请检查终端")
            rclpy.shutdown()
            return
        if ready_at is not None and now >= ready_at:
            ready_at = None
            if publisher is not None and publisher.get_subscription_count() == 0:
                print(f"[VR] WARNING: no TTS subscriber on {args.tts_topic}; check the robot speech service", flush=True)
            if session.start_at is None and session.pending is None and session.state == "idle":
                speak("采集工具已就绪")
        latest_input = session.last_input or input_started_at
        if now - latest_input > 5 and not input_missing:
            input_missing = True
            print(f"[VR] WARNING: no recent valid input on {args.topic}; check headset and VR service", flush=True)
            speak("未收到手柄信号，请检查原厂遥操连接和采集输入话题")
        elif session.last_input is not None and now - session.last_input <= session.input_timeout and input_missing:
            input_missing = False
            print("[VR] Input resumed", flush=True)
            if session.start_at is None and session.pending is None:
                speak("手柄连接已恢复")

    # Accept either reliable or best-effort factory publishers, and reject stale
    # gestures using our own heartbeat instead of building a DDS backlog.
    qos = QoSProfile(depth=1 if args.subtask_mode else 10, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(String, args.topic, lambda msg: session.on_payload(msg.data, time.monotonic()), qos)
    # Acknowledge a completed command promptly without reading files at 50 Hz.
    # The existing monitor checks external/keyboard receipts only at 10 Hz.
    node.create_timer(0.02, lambda: session.tick(time.monotonic()))
    node.create_timer(0.1, monitor)
    print(f"[VR] Listening: {args.topic}; speech: {args.tts_topic if publisher else 'off'}", flush=True)
    print("[VR] Hold GL+GR; tap/release A=start, B=save, X=reset only, Y=discard only. Exit in terminal.", flush=True)
    if session.reset_supported:
        print("[VR] X requires the guarded V4 reset service. Release both Grips after X; unavailable service blocks reset.", flush=True)
    else:
        print("[VR] Factory input: X reset is not integrated; use the factory reset control. No V4 reset will be sent.", flush=True)
    if args.subtask_mode:
        print(f"[VR] While recording: short B=mark next subtask (last mark saves); "
              f"hold B >= {args.a_long_press_sec:g}s then release=save early", flush=True)
        print(f"[VR] 防连按：标记确认后，B连续松开{args.subtask_b_rearm_sec:g}秒才能再次使用；期间重复点击/长按会忽略并重新计时，不必松GL+GR。", flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cancel.set()
        receipts.close()
        worker.shutdown(wait=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
