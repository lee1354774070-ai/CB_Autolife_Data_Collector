"""MZJ GR00T client, retaining its capture, chunk execution and cancellation.

The sole protocol addition reports a controller-submission receipt, never
claims physical completion from a ROS forward acknowledgement.
"""
import json
import cv2
import time
from std_msgs.msg import String
from .mzj_base.groot_policy_bridge import GrootPolicyBridge
from .mzj_base.groot_bridge_core import array_digest_float32


class CollectorThorBridge(GrootPolicyBridge):
    def __init__(self):
        cv2.setNumThreads(1)  # Preserve the collection CPU limit for JPEG/resize.
        self._collector_session_id = ""
        self._control_received = 0.0
        self._proposal_authority = (-1, "", -1)
        super().__init__()

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

    def _policy_payload(self, action, proposal, step, generation, observation_wall_ns,
                        observation_state, *, shadow_only):
        payload = super()._policy_payload(action, proposal, step, generation,
            observation_wall_ns, observation_state, shadow_only=shadow_only)
        owner_generation, session, epoch = self._proposal_authority
        if generation != owner_generation:
            raise RuntimeError("proposal authority changed")
        payload.update(collector_session_id=session, authority_epoch=epoch)
        return payload  # Action values and digest remain byte-for-byte compatible.

    def _on_forward_ack(self, message):
        try:
            if json.loads(message.data).get("accepted") is False:
                return
        except (TypeError, ValueError):
            return
        super()._on_forward_ack(message)

    def _connect(self):
        super()._connect()
        if not self._health.get("controller_submission_receipts"):
            raise RuntimeError("Thor controller_submission receipts are required")

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
