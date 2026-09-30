"""Extend the existing V4 mapper, without a second hardware command path.

Every expert target carries the authority under which its clutch was anchored.
The supervisor must not relabel a queued pre-takeover target as a fresh one.
"""

import json
import time

from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from openarmx_teleop_vr_306_v4.vr_mapper_node import IndependentVrMapper


class EpochPublisher:
    def __init__(self, node, publisher):
        self.node, self.publisher = node, publisher

    def publish(self, message):
        with self.node._lock:
            if not self.node._expert_authority_fresh():
                return
            session, epoch, _ = self.node._collector_authority
            packet = json.loads(message.data)
            packet.update(collector_session_id=session, authority_epoch=epoch)
            self.publisher.publish(String(data=json.dumps(packet, separators=(",", ":"))))


class CollectorVrMapper(IndependentVrMapper):
    def __init__(self):
        super().__init__()
        self._collector_authority = ("", -1, "DISARMED")
        self._collector_authority_time = 0.
        for name in ("_eef_target_pub", "_gripper_pub", "_release_pub"):
            setattr(self, name, EpochPublisher(self, getattr(self, name)))
        self.create_subscription(String, "/hg_dagger/control_state", self._on_authority,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

    def _expert_authority_fresh(self):
        session, _, mode = self._collector_authority
        return (bool(session) and mode in ("EXPERT_READY", "EXPERT_ACTIVE")
                and 0 <= time.monotonic() - self._collector_authority_time < .5)

    def _on_authority(self, message):
        try:
            packet = json.loads(message.data)
            identity = (str(packet["session_id"]), int(packet["authority_epoch"]), str(packet["mode"]))
        except (ValueError, TypeError, KeyError):
            return
        with self._lock:
            previous = self._collector_authority
            if identity[0] == previous[0] and identity[1] < previous[1]:
                return
            if identity[:2] != previous[:2]:
                # The next V4 tick latches current hand pose against its latest
                # measured EEF pose. No release/re-grip or hold ACK is needed.
                self._release_all(require_release=False, notify_controller=False)
            self._collector_authority = identity
            self._collector_authority_time = time.monotonic()

    def _control_tick(self):
        with self._lock:
            if not self._expert_authority_fresh():
                self._release_all(require_release=False, notify_controller=False)
                return
            super()._control_tick()
