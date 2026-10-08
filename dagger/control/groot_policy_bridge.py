"""GR00T N1.7 remote-policy bridge behind the HG-DAgger authority gate.

The bridge never publishes a vendor command. Each action step is offered to
the supervisor and is counted as executed only after a correlated forward ACK.
"""

from __future__ import annotations

import base64
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Mapping, Optional

import cv2
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from .groot_bridge_core import (
    GrootContract,
    GrootProtocolError,
    INFERENCE_MODES,
    array_digest_float32,
    bridge_generation_boundary,
    parse_q23,
    policy_state_from_q23,
    validated_actions,
)


def compact(value: Mapping[str, Any]) -> str:
    return json.dumps(value, separators=(",", ":"), ensure_ascii=False, allow_nan=False)


class RemoteClient:
    def __init__(self, url: str, token: str, timeout: float) -> None:
        self.url = url.rstrip("/")
        self.token = token
        self.timeout = float(timeout)
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self.lock = threading.Lock()

    def request(self, endpoint: str, payload: Optional[Mapping[str, Any]] = None) -> dict[str, Any]:
        raw = None if payload is None else compact(payload).encode("utf-8")
        request = urllib.request.Request(
            self.url + endpoint,
            data=raw,
            method="GET" if raw is None else "POST",
            headers={"Authorization": f"Bearer {self.token}", "Content-Type": "application/json"},
        )
        started = time.monotonic()
        try:
            with self.lock:
                with self.opener.open(request, timeout=self.timeout) as response:
                    result = json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise GrootProtocolError(f"remote HTTP {exc.code}: {detail}") from exc
        except (OSError, TimeoutError, urllib.error.URLError, json.JSONDecodeError) as exc:
            raise GrootProtocolError(f"remote transport failed: {exc}") from exc
        if not isinstance(result, dict) or result.get("error"):
            raise GrootProtocolError(str(result.get("error", "invalid remote response")))
        result["bridge_round_trip_ms"] = (time.monotonic() - started) * 1000.0
        return result


class GrootPolicyBridge(Node):
    def __init__(self) -> None:
        super().__init__("hg_dagger_groot_policy_bridge")
        defaults = {
            "server_url": "http://192.168.8.224:8777",
            "token_file": "/home/ubuntu/.config/autolife_hg_dagger/groot_server.token",
            "task": "Pick the laundry bag.",
            "request_timeout_sec": 180.0,
            "max_inference_latency_sec": 2.0,
            "shadow_refresh_sec": 0.35,
            "max_shadow_candidate_age_sec": 2.0,
            "action_rate_hz": 30.0,
            "forward_ack_timeout_sec": 0.25,
            "max_state_age_sec": 0.5,
            "max_image_age_sec": 0.5,
            "max_image_delta_sec": 0.04,
            "camera_wait_timeout_sec": 5.0,
            "jpeg_quality": 90,
            "collector_root": str(Path(__file__).resolve().parents[2]),
            "joint_state_topic": "/topic_arm_whole_body_and_gripper_current_joints_status_0_300",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._condition = threading.Condition()
        self._mode = "DISARMED"
        self._authority_epoch = 0
        self._generation = 0
        self._q23: Optional[list[float]] = None
        self._q23_received = 0.0
        self._forward_acks: set[tuple[str, int, int]] = set()
        self._shutdown = False
        self._phase = "idle"
        self._detail = "waiting for HG-DAgger policy authority"
        self._remote: Optional[RemoteClient] = None
        self._contract: Optional[GrootContract] = None
        self._health: dict[str, Any] = {}
        self._session_id = ""
        self._proposal: Optional[dict[str, Any]] = None
        self._executed: list[list[float]] = []
        self._ack_sequence = 0
        self._observation_sequence = 0
        self._policy_sequence = 0

        self._policy_pub = self.create_publisher(String, "/hg_dagger/policy_action", 10)
        self._status_pub = self.create_publisher(String, "/hg_dagger/policy_status", 10)
        self.create_subscription(String, "/hg_dagger/control_state", self._on_control_state, 10)
        self.create_subscription(
            String, str(self.get_parameter("joint_state_topic").value), self._on_joint_state, 10
        )
        self.create_subscription(
            String, "/hg_dagger/policy_forward_ack", self._on_forward_ack, 20
        )
        self.create_timer(0.2, self._publish_status)
        self._load_camera_reader()
        self._worker = threading.Thread(target=self._run, name="groot-policy-worker", daemon=True)
        self._worker.start()
        self.get_logger().info("GR00T bridge ready behind HG-DAgger authority")

    def _load_camera_reader(self) -> None:
        root = Path(str(self.get_parameter("collector_root").value))
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))
        try:
            from camera_config import camera_shm_candidates
            from shm_camera import frame_to_hwc, read_shm_frame, shm_timestamp_sec
        except Exception as exc:
            raise RuntimeError(f"collector SHM camera reader is unavailable: {exc}") from exc
        self._camera_shm_candidates = camera_shm_candidates
        self._frame_to_hwc = frame_to_hwc
        self._read_shm_frame = read_shm_frame
        self._shm_timestamp_sec = shm_timestamp_sec

    def _set_phase(self, phase: str, detail: str) -> None:
        with self._condition:
            self._phase, self._detail = phase, detail

    def _on_control_state(self, message: String) -> None:
        try:
            packet = json.loads(message.data)
            mode = str(packet["mode"])
            epoch = int(packet.get("authority_epoch", 0))
        except Exception:
            return
        with self._condition:
            if mode != self._mode or epoch != self._authority_epoch:
                if bridge_generation_boundary(self._mode, mode):
                    self._generation += 1
                self._mode, self._authority_epoch = mode, epoch
                self._condition.notify_all()

    def _on_joint_state(self, message: String) -> None:
        try:
            packet = json.loads(message.data)
            q23 = parse_q23(packet)
        except Exception:
            return
        if q23 is not None:
            with self._condition:
                self._q23 = q23
                self._q23_received = time.time()
                self._condition.notify_all()

    def _on_forward_ack(self, message: String) -> None:
        try:
            packet = json.loads(message.data)
            key = (
                str(packet["proposal_id"]), int(packet["chunk_step"]),
                int(packet["bridge_generation"]),
            )
        except Exception:
            return
        with self._condition:
            self._forward_acks.add(key)
            self._condition.notify_all()

    def _snapshot(self) -> tuple[str, int, int]:
        with self._condition:
            return self._mode, self._authority_epoch, self._generation

    def _generation_valid(self, generation: int, required_mode: Optional[str] = None) -> bool:
        with self._condition:
            return (
                not self._shutdown and self._generation == generation
                and self._mode in INFERENCE_MODES
                and (required_mode is None or self._mode == required_mode)
            )

    def _fresh_q23(self) -> list[float]:
        with self._condition:
            q23 = None if self._q23 is None else list(self._q23)
            age = time.time() - self._q23_received
        if q23 is None or age > float(self.get_parameter("max_state_age_sec").value):
            raise RuntimeError("no fresh complete q23 telemetry")
        return q23

    def _token(self) -> str:
        inline = os.getenv("GROOT_REMOTE_TOKEN", "").strip()
        if inline:
            return inline
        path = Path(str(self.get_parameter("token_file").value)).expanduser()
        try:
            value = path.read_text(encoding="utf-8").strip()
        except OSError as exc:
            raise RuntimeError(f"GR00T token file is unavailable: {path}: {exc}") from exc
        if not value:
            raise RuntimeError(f"GR00T token file is empty: {path}")
        return value

    def _connect(self) -> None:
        self._set_phase("connecting", "validating THOR health and deployment contract")
        remote = RemoteClient(
            str(self.get_parameter("server_url").value), self._token(),
            float(self.get_parameter("request_timeout_sec").value),
        )
        health = remote.request("/health")
        if not health.get("ready") or health.get("policy_type") != "groot":
            raise GrootProtocolError("THOR GR00T service is not ready")
        if health.get("mode") not in {"policy_only_baseline", "policy_only_frame"}:
            raise GrootProtocolError(
                "this bridge currently requires baseline/frame policy-only mode; "
                "SOMA causal Outcome history is not yet supported"
            )
        if health.get("outcome_history_offsets") or health.get("outcome_visual_history_required"):
            raise GrootProtocolError("remote policy unexpectedly requires causal Outcome history")
        contract = GrootContract.from_mapping(health.get("contract", {}))
        self._remote, self._contract, self._health = remote, contract, health

    def _camera_frame(self, name: str) -> tuple[Any, float]:
        for spec in self._camera_shm_candidates(name):
            frame = self._read_shm_frame(spec)
            if frame is None:
                continue
            try:
                image = self._frame_to_hwc(frame, False, rgb=False)
            except ValueError:
                continue
            return image, float(self._shm_timestamp_sec(frame.timestamp_ns))
        raise RuntimeError(f"camera SHM frame is unavailable: {name}")

    def _capture_images(self) -> dict[str, str]:
        assert self._contract is not None
        deadline = time.monotonic() + float(self.get_parameter("camera_wait_timeout_sec").value)
        last_error = "camera frames are unavailable"
        while time.monotonic() < deadline:
            try:
                frames: dict[str, tuple[Any, float]] = {}
                for key in self._contract.image_shapes:
                    frames[key] = self._camera_frame(key.rsplit(".", 1)[-1])
                stamps = [value[1] for value in frames.values()]
                if time.time() - min(stamps) > float(self.get_parameter("max_image_age_sec").value):
                    raise RuntimeError("camera frame is stale")
                if max(stamps) - min(stamps) > float(self.get_parameter("max_image_delta_sec").value):
                    raise RuntimeError("three camera frames are not synchronized")
                encoded: dict[str, str] = {}
                for key, (image, _) in frames.items():
                    _, height, width = self._contract.image_shapes[key]
                    resized = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
                    ok, jpeg = cv2.imencode(
                        ".jpg", resized,
                        [cv2.IMWRITE_JPEG_QUALITY, int(self.get_parameter("jpeg_quality").value)],
                    )
                    if not ok:
                        raise RuntimeError(f"JPEG encoding failed: {key}")
                    encoded[key] = base64.b64encode(jpeg.tobytes()).decode("ascii")
                return encoded
            except RuntimeError as exc:
                last_error = str(exc)
                time.sleep(0.01)
        raise RuntimeError(last_error)

    def _observation(self) -> tuple[dict[str, Any], int, list[float]]:
        self._set_phase("capturing", "capturing synchronized q23 and three-camera observation")
        observation_wall_ns = time.time_ns()
        q23 = self._fresh_q23()
        observation_state = policy_state_from_q23(q23)
        payload = {
            "sequence_id": self._observation_sequence,
            "state": observation_state,
            "images": self._capture_images(),
        }
        self._observation_sequence += 1
        return payload, observation_wall_ns, observation_state

    def _post(self, endpoint: str, payload: Mapping[str, Any]) -> dict[str, Any]:
        if self._remote is None:
            raise RuntimeError("GR00T remote client is not connected")
        return self._remote.request(endpoint, payload)

    def _resolve_proposal(self, *, cancel: bool) -> None:
        proposal, session_id = self._proposal, self._session_id
        if proposal is None or not session_id:
            return
        try:
            if self._executed and cancel:
                self._ack_sequence += 1
                self._post("/cancel", {
                    "session_id": session_id,
                    "ack": self._execution_ack(proposal, self._executed),
                    "actual_executed_prefix": self._executed,
                    "observed_state": policy_state_from_q23(self._fresh_q23()),
                })
            elif self._executed:
                self._ack_sequence += 1
                self._post("/ack", {
                    "session_id": session_id,
                    "ack": self._execution_ack(proposal, self._executed),
                    "actual_executed_prefix": self._executed,
                    "observed_state": policy_state_from_q23(self._fresh_q23()),
                })
            else:
                self._post("/discard", {
                    "session_id": session_id, "proposal_id": str(proposal["proposal_id"])
                })
        finally:
            self._proposal = None
            self._executed = []

    def _execution_ack(self, proposal: Mapping[str, Any], actions: list[list[float]]) -> dict[str, Any]:
        return {
            "ack_sequence_id": self._ack_sequence,
            "proposal_id": str(proposal["proposal_id"]),
            "context_digest": str(proposal["context_digest"]),
            "executable_chunk_digest": str(proposal["executable_chunk_digest"]),
            "executed_steps": len(actions),
            "executed_prefix_digest": array_digest_float32(actions),
        }

    def _close_session(self) -> None:
        if not self._session_id:
            return
        session_id = self._session_id
        try:
            self._post("/close", {"session_id": session_id})
        except Exception as exc:
            self.get_logger().warning(f"remote close failed: {exc}")
        else:
            self._session_id = ""

    def _track_response_proposal(self, response: Mapping[str, Any]) -> None:
        """Remember a live proposal before checks that may reject the response."""
        proposal = response.get("proposal")
        if isinstance(proposal, dict):
            self._proposal, self._executed = proposal, []

    def _request_failure(self, reason: str) -> None:
        self._set_phase("failed", reason)
        self.get_logger().error(reason)
        # Publish immediately rather than waiting for the 5 Hz status timer;
        # the supervisor turns this correlated failure detail into FAILURE_HOLD.
        self._publish_status()

    def _next_response(self, observation: Mapping[str, Any]) -> dict[str, Any]:
        self._set_phase("inference", "waiting for a fresh GR00T proposal")
        if not self._session_id:
            response = self._post("/start", {
                "task": str(self.get_parameter("task").value), **observation,
            })
            self._session_id = str(response["session_id"])
        else:
            response = self._post("/infer", {"session_id": self._session_id, **observation})
        self._track_response_proposal(response)
        self._check_inference_latency(response)
        while str(response.get("decision")) == "reinfer":
            self._set_phase("retry", str(response.get("reason", "action rejected")))
            response = self._post("/retry", {"session_id": self._session_id})
            self._track_response_proposal(response)
            self._check_inference_latency(response)
        return response

    def _check_inference_latency(self, response: Mapping[str, Any]) -> None:
        maximum_ms = float(self.get_parameter("max_inference_latency_sec").value) * 1000.0
        observed_ms = float(response.get("bridge_round_trip_ms", float("inf")))
        if maximum_ms > 0 and observed_ms >= maximum_ms:
            raise RuntimeError(
                f"VLA inference latency {observed_ms:.1f} ms reached limit {maximum_ms:.1f} ms"
            )

    def _wait_forward_ack(self, proposal_id: str, step: int, generation: int) -> bool:
        key = (proposal_id, step, generation)
        deadline = time.monotonic() + float(self.get_parameter("forward_ack_timeout_sec").value)
        with self._condition:
            while key not in self._forward_acks:
                if (
                    self._shutdown
                    or self._generation != generation
                    or self._mode != "POLICY_ACTIVE"
                ):
                    return False
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._condition.wait(remaining)
            self._forward_acks.discard(key)
            return True

    def _policy_payload(
        self,
        action: list[float],
        proposal: Mapping[str, Any],
        step: int,
        generation: int,
        observation_wall_ns: int,
        observation_state: list[float],
        *,
        shadow_only: bool,
    ) -> dict[str, Any]:
        measured = self._fresh_q23()
        self._policy_sequence += 1
        return {
            "action": action,
            "units": "degrees",
            "sequence": self._policy_sequence,
            "timestamp_ns": time.time_ns(),
            "proposal_observation_timestamp_ns": observation_wall_ns,
            "proposal_observation_state": observation_state,
            "measured_policy_state": policy_state_from_q23(measured),
            "measured_leg_waist": measured[:4],
            "groot_session_id": self._session_id,
            "proposal_id": str(proposal["proposal_id"]),
            "context_digest": proposal["context_digest"],
            "executable_chunk_digest": proposal["executable_chunk_digest"],
            "chunk_step": step,
            "bridge_generation": generation,
            "shadow_only": shadow_only,
            "warmup_only": shadow_only,
        }

    def _execute(
        self,
        proposal: dict[str, Any],
        generation: int,
        observation_wall_ns: int,
        observation_state: list[float],
    ) -> bool:
        assert self._contract is not None
        actions = validated_actions(proposal, self._contract)
        self._proposal, self._executed = proposal, []
        proposal_id = str(proposal["proposal_id"])
        period = 1.0 / float(self.get_parameter("action_rate_hz").value)
        deadline = time.monotonic()
        for step, action in enumerate(actions):
            if not self._generation_valid(generation, "POLICY_ACTIVE"):
                self._resolve_proposal(cancel=True)
                return False
            payload = self._policy_payload(
                action, proposal, step, generation, observation_wall_ns,
                observation_state,
                shadow_only=False,
            )
            self._policy_pub.publish(String(data=compact(payload)))
            if not self._wait_forward_ack(proposal_id, step, generation):
                if self._generation_valid(generation, "POLICY_ACTIVE"):
                    raise RuntimeError(f"supervisor did not ACK policy step {step}")
                self._resolve_proposal(cancel=True)
                return False
            self._executed.append(action)
            self._set_phase(
                "executing",
                f"proposal={proposal_id} step={step + 1}/{len(actions)}",
            )
            deadline += period
            remaining = deadline - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            else:
                deadline = time.monotonic()
        self._resolve_proposal(cancel=False)
        return True

    def _shadow_proposal(
        self,
        proposal: dict[str, Any],
        generation: int,
        observation_wall_ns: int,
        observation_state: list[float],
    ) -> bool:
        """Refresh in expert mode, then reuse a fresh proposal at Grip release."""
        assert self._contract is not None
        actions = validated_actions(proposal, self._contract)
        self._proposal, self._executed = proposal, []
        proposal_id = str(proposal["proposal_id"])
        refresh_deadline = (
            time.monotonic()
            + float(self.get_parameter("shadow_refresh_sec").value)
        )
        period = 1.0 / float(self.get_parameter("action_rate_hz").value)
        while self._generation_valid(generation):
            mode, _, _ = self._snapshot()
            age = (time.time_ns() - observation_wall_ns) / 1e9
            if age > float(self.get_parameter("max_shadow_candidate_age_sec").value):
                self._resolve_proposal(cancel=False)
                return False
            if mode == "POLICY_ACTIVE":
                return self._execute(
                    proposal, generation, observation_wall_ns, observation_state
                )
            if mode == "POLICY_WARMUP":
                payload = self._policy_payload(
                    actions[0], proposal, 0, generation, observation_wall_ns,
                    observation_state,
                    shadow_only=True,
                )
                self._policy_pub.publish(String(data=compact(payload)))
                self._set_phase(
                    "warmup",
                    f"validating shadow proposal={proposal_id} before policy release",
                )
            else:
                self._set_phase(
                    "shadow",
                    f"expert has authority; cached proposal={proposal_id}",
                )
                if time.monotonic() >= refresh_deadline:
                    self._resolve_proposal(cancel=False)
                    return False
            with self._condition:
                self._condition.wait(timeout=period)
        return False

    def _run_generation(self, generation: int) -> None:
        self._connect()
        self._observation_sequence = 0
        self._ack_sequence = 0
        self._policy_sequence = 0
        while self._generation_valid(generation):
            observation, observation_wall_ns, observation_state = self._observation()
            response = self._next_response(observation)
            if not self._generation_valid(generation):
                proposal = response.get("proposal")
                if isinstance(proposal, dict):
                    self._proposal, self._executed = proposal, []
                break
            decision = str(response.get("decision"))
            if decision == "execute" and isinstance(response.get("proposal"), dict):
                mode, _, _ = self._snapshot()
                if mode == "POLICY_ACTIVE":
                    self._execute(
                        response["proposal"], generation, observation_wall_ns,
                        observation_state,
                    )
                else:
                    self._shadow_proposal(
                        response["proposal"], generation, observation_wall_ns,
                        observation_state,
                    )
                continue
            if decision == "hold_reobserve":
                self._set_phase("holding", str(response.get("reason", decision)))
                time.sleep(0.05)
                continue
            raise RuntimeError(
                f"GR00T requested {decision}: {response.get('reason', 'no reason')}"
            )

    def _cleanup_generation(self) -> None:
        try:
            if self._proposal is not None:
                self._resolve_proposal(cancel=True)
        except Exception as exc:
            self.get_logger().error(f"remote proposal cancellation failed: {exc}")
        self._close_session()

    def _run(self) -> None:
        while True:
            with self._condition:
                while not self._shutdown and self._mode not in INFERENCE_MODES:
                    self._phase = "idle"
                    self._detail = "waiting for HG-DAgger policy authority"
                    self._condition.wait(0.5)
                if self._shutdown:
                    return
                generation = self._generation
            try:
                self._run_generation(generation)
            except Exception as exc:
                if self._generation_valid(generation):
                    self._request_failure(f"GR00T bridge failure: {exc}")
            finally:
                self._cleanup_generation()
            time.sleep(0.02)

    def _publish_status(self) -> None:
        mode, epoch, generation = self._snapshot()
        status = {
            "phase": self._phase,
            "detail": self._detail,
            "hg_mode": mode,
            "authority_epoch": epoch,
            "bridge_generation": generation,
            "server_url": str(self.get_parameter("server_url").value),
            "server_mode": self._health.get("mode"),
            "session_id": self._session_id,
            "proposal_id": None if self._proposal is None else self._proposal.get("proposal_id"),
            "timestamp_ns": time.time_ns(),
        }
        self._status_pub.publish(String(data=compact(status)))

    def destroy_node(self) -> bool:
        with self._condition:
            self._shutdown = True
            self._generation += 1
            self._condition.notify_all()
        if self._worker.is_alive():
            self._worker.join(timeout=3.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GrootPolicyBridge()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
