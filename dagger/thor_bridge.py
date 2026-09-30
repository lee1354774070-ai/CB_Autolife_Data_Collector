"""Our Thor protocol + SHM reader, upstream of the sole V4 motion controller.

HTTP, JPEG and camera waits never run in ROS callbacks. One worker owns the
remote session; there is no request backlog or automatic POST replay. The
supervisor fences every target by local session and authority epoch, so X/B
revoke output even while Thor is still computing an obsolete chunk.
"""

from __future__ import annotations

import json
import os
import threading
import time

import cv2
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from deploy.common.robot_io import DirectShmCameraSet, image_to_policy_chw
from deploy.groot_n1_7.auth import read_token
from deploy.groot_n1_7.robot_client import GrootRemoteClient
from deploy.groot_n1_7.robot_mapping import policy_state_from_q23
from deploy.groot_n1_7.protocol import ProtocolError, array_digest
from robot_schema import parse_whole_body_state


class Revoked(RuntimeError):
    """Local authority changed; never publish the old worker's result."""


class CollectorThorBridge(Node):
    def __init__(self, *, start_worker=True):
        super().__init__("collector_thor_bridge")
        defaults = dict(server_url="", token_file="", task="", request_timeout_sec=10.0,
                        action_rate_hz=30.0, receipt_timeout_sec=.15,
                        camera_wait_sec=1.0, max_image_age_sec=.25, max_image_delta_sec=.04,
                        max_state_age_sec=.35, max_proposal_age_sec=2.0,
                        joint_state_topic="/topic_arm_whole_body_and_gripper_current_joints_status_0_300")
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        for key in defaults:
            if isinstance(defaults[key], float):
                value = self.get_parameter(key).value
                if not np.isfinite(value) or value <= 0:
                    raise ValueError(f"{key} must be finite and positive")
        self._condition = threading.Condition()
        self._stopped = False
        self._generation = 0
        self._identity = ("", -1, "DISARMED")
        self._state_received = 0.
        self._q23 = None
        self._joints_received = 0.
        self._waiting_receipt = None
        self._receipt = False
        self._receipt_accepted = False
        self._phase = "idle"
        self._detail = ""
        self._phase_session = ""
        self._timings = {}
        self._sequence = 0
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._action_pub = self.create_publisher(String, "/hg_dagger/policy_action", reliable)
        self._status_pub = self.create_publisher(String, "/hg_dagger/policy_status", reliable)
        self.create_subscription(String, "/hg_dagger/control_state", self._on_control_state, reliable)
        self.create_subscription(String, "/hg_dagger/policy_forward_ack", self._on_receipt, reliable)
        self.create_subscription(String, self.get_parameter("joint_state_topic").value, self._on_joints, latest)
        self.create_timer(.1, self._publish_status)
        cv2.setNumThreads(1)  # JPEG/resize workers must not consume all robot cores.
        self._worker = threading.Thread(target=self._run, name="collector-thor-http", daemon=True)
        if start_worker:
            self._worker.start()

    def _on_control_state(self, message):
        try:
            data = json.loads(message.data)
            identity = (data["session_id"], int(data["authority_epoch"]), data["mode"])
        except (ValueError, TypeError, KeyError):
            return
        with self._condition:
            if identity != self._identity:
                self._generation += 1
            self._identity = identity
            self._state_received = time.monotonic()
            self._condition.notify_all()

    def _on_joints(self, message):
        try:
            packet = json.loads(message.data)
            q23 = parse_whole_body_state(packet) if isinstance(packet, dict) else None
        except (ValueError, TypeError, KeyError):
            return
        if q23 is not None and np.isfinite(q23).all():
            with self._condition:
                self._q23, self._joints_received = q23, time.monotonic()

    def _on_receipt(self, message):
        try:
            data = json.loads(message.data)
            key = (data["proposal_id"], data["chunk_step"], data["bridge_generation"], data["authority_epoch"])
        except (ValueError, TypeError, KeyError):
            return
        with self._condition:
            if key == self._waiting_receipt:
                self._receipt = True
                self._receipt_accepted = data.get("accepted", True) is True
                self._condition.notify_all()

    def _valid(self, generation):
        return (not self._stopped and generation == self._generation
                and bool(self._identity[0]) and self._identity[2] == "POLICY_ACTIVE"
                and time.monotonic() - self._state_received < .6)

    def _check(self, generation):
        with self._condition:
            if not self._valid(generation):
                raise Revoked("local authority revoked or heartbeat stale")

    def _fresh_q23(self):
        with self._condition:
            if (self._q23 is None or time.monotonic() - self._joints_received
                    > self.get_parameter("max_state_age_sec").value):
                raise RuntimeError("fresh complete q23 telemetry required")
            return self._q23.copy()

    def _set_phase(self, phase, detail=""):
        with self._condition:
            self._phase, self._detail = phase, detail

    def _publish_status(self):
        with self._condition:
            value = dict(phase=self._phase, detail=self._detail,
                         collector_session_id=self._phase_session, timing_ms=dict(self._timings),
                         receipt_scope="controller_submission")
        self._status_pub.publish(String(data=json.dumps(value, separators=(",", ":"))))

    def _capture(self, client, contract, cameras, generation):
        self._set_phase("capturing")
        deadline = time.monotonic() + self.get_parameter("camera_wait_sec").value
        while time.monotonic() < deadline:
            self._check(generation)
            cameras.refresh()
            frames = cameras.synchronized_latest("hand_left", self.get_parameter("max_image_delta_sec").value,
                                                 self.get_parameter("max_image_age_sec").value)
            if frames is not None:
                q23 = self._fresh_q23()
                state = policy_state_from_q23(q23)
                observation_ns = time.time_ns()
                images = {key: image_to_policy_chw(frames[key.rsplit(".", 1)[-1]].image_hwc, shape)
                          for key, shape in contract.image_shapes.items()}
                self._sequence += 1
                encoded = client.observation_payload(self._sequence, state, images, contract)
                self._check(generation)
                return encoded, state, observation_ns
            with self._condition:
                self._condition.wait(.005)
        raise TimeoutError("three fresh synchronized RGB cameras unavailable")

    def _submit(self, proposal, contract, generation, state, observation_ns):
        actions = np.asarray(proposal["executable_actions"], dtype=np.float32)
        if (actions.ndim != 2 or actions.shape[1] != contract.action_dim
                or not 0 < len(actions) <= contract.chunk_size
                or not np.isfinite(actions).all()
                or array_digest(actions) != proposal["executable_chunk_digest"]):
            raise ProtocolError("invalid/digest-mismatched Thor action chunk")
        prefix = []
        period = 1.0 / self.get_parameter("action_rate_hz").value
        deadline = time.monotonic()
        for step, action in enumerate(actions[:contract.n_action_steps]):
            with self._condition:
                while self._valid(generation) and time.monotonic() < deadline:
                    self._condition.wait(deadline - time.monotonic())
                if not self._valid(generation):
                    break
                session, epoch, _ = self._identity
            if (time.time_ns() - observation_ns) / 1e9 > self.get_parameter("max_proposal_age_sec").value:
                break
            q23 = self._fresh_q23()
            payload = dict(action=action.astype(float).tolist(), units="degrees", shadow_only=False,
                           measured_leg_waist=q23[:4].astype(float).tolist(),
                           measured_policy_state=policy_state_from_q23(q23).astype(float).tolist(),
                           proposal_observation_state=state.astype(float).tolist(),
                           proposal_observation_timestamp_ns=observation_ns,
                           proposal_id=proposal["proposal_id"], chunk_step=step,
                           source_timestamp_ns=time.time_ns(), bridge_generation=generation,
                           collector_session_id=session, authority_epoch=epoch)
            with self._condition:
                if not self._valid(generation):
                    break
                self._waiting_receipt = (proposal["proposal_id"], step, generation, epoch)
                self._receipt = False
                self._action_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))
                receipt_deadline = time.monotonic() + self.get_parameter("receipt_timeout_sec").value
                # After send, even X cannot tell us whether the last target was
                # forwarded. Reconcile its receipt; no physical output waits on
                # this thread. Missing receipt latches an ambiguous session.
                while not self._receipt and time.monotonic() < receipt_deadline:
                    self._condition.wait(receipt_deadline - time.monotonic())
                if not self._receipt:
                    raise RuntimeError("controller submission receipt missing; no automatic replay or remote close")
                self._waiting_receipt = None
                if not self._receipt_accepted:
                    break
            prefix.append(action)
            deadline += period
            # Do not burst a backlog of commands after scheduler stalls.
            if deadline < time.monotonic():
                deadline = time.monotonic() + period
        return prefix

    def _session(self, client, generation):
        self._set_phase("connecting")
        health, contract = client.health()
        if (health.get("mode") not in ("policy_only_baseline", "policy_only_frame") or health.get("outcome_history_offsets")
                or not health.get("controller_submission_receipts")):
            raise ProtocolError("Update our Thor baseline/frame server; controller_submission receipts required")
        cameras = DirectShmCameraSet(tuple(key.rsplit(".", 1)[-1] for key in contract.image_shapes), set())
        remote_session = None
        while True:
            try:
                self._check(generation)
                payload, state, observation_ns = self._capture(client, contract, cameras, generation)
            except Revoked:
                break
            except Exception:
                # At a capture boundary no proposal is outstanding. Closing is
                # safe here; unlike an ambiguous /infer or receipt timeout it
                # does not discard an unknown execution prefix.
                if remote_session:
                    client._request("/close", {"session_id": remote_session})
                raise
            self._set_phase("inference")
            started = time.monotonic()
            response = (client.start(self.get_parameter("task").value, payload) if remote_session is None
                        else client.infer(remote_session, payload))
            remote_session = response["session_id"]
            with self._condition:
                self._timings["inference_round_trip"] = (time.monotonic() - started) * 1000
            proposal = response.get("proposal")
            if response.get("decision") != "execute" or not isinstance(proposal, dict):
                raise ProtocolError("Expected policy-only execute proposal")
            self._set_phase("executing")
            prefix = self._submit(proposal, contract, generation, state, observation_ns)
            self._set_phase("holding")
            if prefix:
                client.finish_controller_submission(remote_session, proposal, np.stack(prefix))
            else:
                client.discard_unexecuted(remote_session, proposal)
            if not prefix:
                with self._condition:
                    if not self._valid(generation):
                        break
                client._request("/close", {"session_id": remote_session})
                raise RuntimeError("proposal expired before any target could be submitted")
        if remote_session:
            # Do not use close(), which intentionally suppresses ProtocolError
            # for legacy callers. DAgger must expose uncertain lifecycle I/O.
            client._request("/close", {"session_id": remote_session})

    def _run(self):
        failed_session = None
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._stopped or (
                    self._valid(self._generation) and self._identity[0] != failed_session))
                if self._stopped:
                    return
                generation, local_session = self._generation, self._identity[0]
                self._phase_session = local_session
            client = None
            try:
                client = GrootRemoteClient(self.get_parameter("server_url").value,
                                           read_token(os.environ.get("GROOT_REMOTE_TOKEN"),
                                                      self.get_parameter("token_file").value),
                                           timeout_sec=self.get_parameter("request_timeout_sec").value)
                self._session(client, generation)
                self._set_phase("idle")
            except Revoked:
                # A response/receipt already reconciled; do not restart this
                # same local session if its heartbeat simply reappears.
                failed_session = local_session
                self._set_phase("idle", "authority revoked")
            except Exception as exc:
                failed_session = local_session
                self._set_phase("failed", str(exc))
            finally:
                if client is not None:
                    client.close_connections()

    def destroy_node(self):
        with self._condition:
            self._stopped = True
            self._condition.notify_all()
        if self._worker.is_alive():
            self._worker.join(timeout=self.get_parameter("request_timeout_sec").value + 1)
        return super().destroy_node()
