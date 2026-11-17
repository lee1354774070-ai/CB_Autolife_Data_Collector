"""DAgger GR00T client, retaining its capture, chunk execution and cancellation.

The sole protocol addition reports a controller-submission receipt, never
claims physical completion from a ROS forward acknowledgement.
"""
import json
import os
import cv2
import time
from std_msgs.msg import String
from .control.groot_policy_bridge import GrootPolicyBridge
from .control.groot_bridge_core import array_digest_float32, policy_state_from_q23
from .robot_profile import resolve_robot
from .inference_options import validate_selection


class CollectorThorBridge(GrootPolicyBridge):
    def __init__(self):
        cv2.setNumThreads(1)  # Preserve the collection CPU limit for JPEG/resize.
        self._collector_session_id = ""
        self._control_received = 0.0
        self._proposal_authority = (-1, "", -1)
        self._v5_policy_pacing = os.environ.get("DAGGER_TELEOP_BACKEND", "v4") == "v5"
        from .local_policy_execution import enabled
        self._local_execution = enabled()
        self._local_capability = None
        self._local_pending = None
        self._local_answers = {}
        super().__init__()
        if self._local_execution:
            from .local_policy_execution import POLICY_TOPIC
            self._local_query_pub = self.create_publisher(String, POLICY_TOPIC, 16)

    def _on_control_state(self, message):
        try:
            packet = json.loads(message.data)
            session, epoch = packet["session_id"], packet["authority_epoch"]
            if not isinstance(session, str) or type(epoch) is not int or epoch < 0:
                return
        except (KeyError, TypeError, ValueError):
            return
        with self._condition:
            if session == self._collector_session_id and epoch < self._authority_epoch:
                return
            identity_changed = (session, epoch) != (self._collector_session_id, self._authority_epoch)
            generation = self._generation
            super()._on_control_state(message)
            if identity_changed and generation == self._generation:
                self._generation += 1
            self._collector_session_id = session
            self._control_received = time.monotonic()
            compatibility = packet.get('collector_compat', {})
            self._local_capability = (compatibility.get('local_policy_execution')
                                      if isinstance(compatibility, dict) else None)
            self._condition.notify_all()

    def _generation_valid(self, generation, required_mode=None):
        with self._condition:
            return (0 <= time.monotonic() - self._control_received <= .5
                    and bool(self._collector_session_id)
                    and super()._generation_valid(generation, required_mode))

    def _run_generation(self, generation):
        with self._condition:
            if not self._generation_valid(generation, "POLICY_ACTIVE"):
                return
            self._proposal_authority = (generation, self._collector_session_id, self._authority_epoch)
        # Retry an unresolved cleanup before creating another remote proposal.
        if self._proposal is not None:
            self._resolve_proposal(cancel=True)
        if self._session_id:
            self._close_session()
            if self._session_id:
                raise RuntimeError("previous Thor session has not closed")
        super()._run_generation(generation)

    def _wait_policy_tracking(self, action, generation):
        # The pinned V5 controller faults at 22 degrees (arms and neck). Keep
        # 4 degrees of scheduling margin without clipping the proposal/ACK.
        # Grippers can stop against an object and must never gate body motion.
        deadline = time.monotonic() + 2.0
        indices = (*range(14), 16, 17, 18)
        with self._condition:
            while True:
                if not self._generation_valid(generation, "POLICY_ACTIVE"):
                    raise RuntimeError("policy authority changed while waiting for tracking")
                measured = policy_state_from_q23(self._fresh_q23())
                index = max(indices, key=lambda i: abs(action[i] - measured[i]))
                error = abs(action[index] - measured[index])
                if error <= 18.0:
                    return
                detail = (f"waiting for body tracking: action[{index}] error {error:.1f} deg; "
                          "next policy step withheld (18 deg lead limit)")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError(detail + "; tracking wait exceeded 2 seconds")
                self._set_phase("holding", detail)
                # Joint feedback / Grip / epoch changes wake this immediately.
                # Condition.wait releases the lock; takeover never waits for us.
                self._condition.wait(min(.02, remaining))

    def _policy_payload(self, action, proposal, step, generation, observation_wall_ns,
                        observation_state, *, shadow_only):
        local = getattr(self, '_local_execution', False) and not shadow_only
        if local:
            capability = self._local_capability
            if (not isinstance(capability, dict) or capability.get('version') != 1
                    or type(capability.get('timestamp_ns')) is not int
                    or not 0 <= time.time_ns() - capability['timestamp_ns'] <= 500_000_000):
                raise RuntimeError('bounded controller capability is missing or stale')
            if self._local_pending is not None:
                raise RuntimeError('previous controller submission is unresolved')
        # Local receipts confirm acceptance; they no longer imply servo smoothing.
        if self._v5_policy_pacing and not shadow_only:
            self._wait_policy_tracking(action, generation)
        payload = super()._policy_payload(action, proposal, step, generation,
            observation_wall_ns, observation_state, shadow_only=shadow_only)
        owner_generation, session, epoch = self._proposal_authority
        if generation != owner_generation:
            raise RuntimeError("proposal authority changed")
        payload.update(collector_session_id=session, authority_epoch=epoch)
        if local:
            from .local_policy_execution import binding
            self._local_pending = {'payload': dict(payload), 'binding': binding(payload)}
        return payload  # Action values and digest remain byte-for-byte compatible.

    def _on_forward_ack(self, message):
        if getattr(self, '_local_execution', False):
            from .local_policy_execution import matches, identity
            try:
                receipt = json.loads(message.data)
                with self._condition:
                    pending = self._local_pending
                    if pending and matches(receipt, pending['binding']):
                        self._local_answers[identity(pending['binding'])] = receipt
                        self._condition.notify_all()
            except (KeyError, TypeError, ValueError):
                pass
            return
        try:
            if json.loads(message.data).get("accepted") is False:
                return
        except (TypeError, ValueError):
            return
        super()._on_forward_ack(message)

    def _await_local_receipt(self, timeout):
        from .local_policy_execution import identity
        pending = self._local_pending
        key = identity(pending['binding'])
        deadline = time.monotonic() + timeout
        with self._condition:
            while key not in self._local_answers:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(min(.02, remaining))
            return self._local_answers.pop(key)

    def _query_local_receipt(self):
        self._local_query_pub.publish(String(data=json.dumps(
            dict(self._local_pending['payload'], query_only=True))))
        return self._await_local_receipt(.5)

    def _wait_forward_ack(self, proposal_id, step, generation):
        if not getattr(self, '_local_execution', False):
            return super()._wait_forward_ack(proposal_id, step, generation)
        pending = self._local_pending
        if pending is None or (pending['binding']['proposal_id'],
                pending['binding']['chunk_step'], pending['binding']['bridge_generation']) != (
                proposal_id, step, generation):
            raise RuntimeError('controller submission identity mismatch')
        result = self._await_local_receipt(.25)
        if result is None:
            result = self._query_local_receipt()  # Read-only; never resubmit motion.
        if result is None:
            raise RuntimeError('controller submission outcome unknown; preserving proposal')
        self._local_pending = None
        if result['accepted']:
            return True
        if self._generation_valid(generation, 'POLICY_ACTIVE'):
            raise RuntimeError('controller rejected policy: ' + str(result.get('reason')))
        return False

    def _recover_local_receipt(self):
        pending = getattr(self, '_local_pending', None)
        if pending is None:
            return
        result = self._await_local_receipt(0.) or self._query_local_receipt()
        if result is None:
            raise RuntimeError('controller outcome remains unknown; proposal not retired')
        if result['accepted']:
            if (self._proposal is None
                    or str(self._proposal['proposal_id']) != pending['binding']['proposal_id']
                    or len(self._executed) != pending['binding']['chunk_step']):
                raise RuntimeError('recovered receipt does not match the retained prefix')
            self._executed.append(pending['payload']['action'])
        self._local_pending = None

    def _close_session(self):
        if getattr(self, '_local_pending', None) is not None:
            self.get_logger().error('retaining Thor session: controller outcome is unknown')
            return
        super()._close_session()

    def _connect(self):
        super()._connect()
        resolve_robot(os.environ).validate_health(self._health)
        from .v5_runtime import validate_server_head_limit
        validate_server_head_limit(self._health)
        validate_selection(self._health, os.environ['TASK_TEXT'], os.environ.get('DAGGER_GRIPPER_PHASE', 'auto'),
                           os.environ.get('DAGGER_EXPECTED_CHECKPOINT', ''))

    def _post(self, endpoint, payload):
        if endpoint == '/start':
            phase = validate_selection(self._health, payload['task'],
                os.environ.get('DAGGER_GRIPPER_PHASE', 'auto'), os.environ.get('DAGGER_EXPECTED_CHECKPOINT', ''))
            payload = dict(payload)
            if phase in self._health.get('gripper_phases', ()):
                payload['gripper_phase'] = phase
            expected = os.environ.get('DAGGER_EXPECTED_CHECKPOINT', '') or self._health.get('checkpoint', '')
            if expected:
                payload['expected_checkpoint'] = expected
        return super()._post(endpoint, payload)

    def _publish_status(self):
        # Tag the producer generation, not whichever new trial is active while
        # an old HTTP request is finishing. Stale errors cannot stop a new trial.
        generation, session, epoch = self._proposal_authority
        self._status_pub.publish(String(data=json.dumps({
            "phase": self._phase, "detail": self._detail,
            "collector_session_id": session, "authority_epoch": epoch,
            "bridge_generation": generation, "hg_mode": self._mode,
            "server_url": str(self.get_parameter("server_url").value),
            "server_mode": self._health.get("mode"), "session_id": self._session_id,
            "proposal_id": None if self._proposal is None else self._proposal.get("proposal_id"),
            "timestamp_ns": time.time_ns(),
        }, separators=(",", ":"))))

    def _resolve_proposal(self, *, cancel):
        if getattr(self, '_local_execution', False):
            self._recover_local_receipt()
        proposal, session_id = self._proposal, self._session_id
        if proposal is None or not session_id:
            return
        if self._executed:
            self._post("/controller_ack", {
                "session_id": session_id,
                "receipt_scope": "controller_submission",
                "proposal_id": str(proposal["proposal_id"]),
                "context_digest": str(proposal["context_digest"]),
                "executable_chunk_digest": str(proposal["executable_chunk_digest"]),
                "submitted_prefix": self._executed,
                "submitted_prefix_digest": array_digest_float32(self._executed),
            })
        else:
            self._post("/discard", {
                "session_id": session_id, "proposal_id": str(proposal["proposal_id"])
            })
        # Preserve unresolved proposal on error; do not claim it was retired.
        self._proposal = None
        self._executed = []
