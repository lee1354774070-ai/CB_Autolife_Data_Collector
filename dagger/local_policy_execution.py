"""Bounded controller execution of immutable model submissions.

The requested 21-D action is never rewritten in a proposal or a Thor receipt.
Only the actuator reference is projected/rate limited, and the receipt records
both representations. A positive receipt confirms controller acceptance only.
"""
import collections
import json
import math
import os
import time

import numpy as np

from .control.groot_bridge_core import array_digest_float32


IDENTITY = ('collector_session_id', 'authority_epoch', 'proposal_id',
            'chunk_step', 'bridge_generation')
POLICY_TOPIC = '/hg_dagger/controller/policy_submission'
RECEIPT_TOPIC = '/hg_dagger/controller/policy_receipt'


def enabled():
    return (os.environ.get('DAGGER_LOCAL_POLICY_EXECUTION') == '1'
            and os.environ.get('DAGGER_TELEOP_BACKEND') == 'v5')


def binding(packet):
    result = {k: packet[k] for k in IDENTITY}
    for k in ('collector_session_id', 'proposal_id'):
        if not isinstance(result[k], str) or not result[k]:
            raise ValueError('missing submission identity: ' + k)
    for k in ('authority_epoch', 'chunk_step', 'bridge_generation'):
        if type(result[k]) is not int or result[k] < 0:
            raise ValueError('invalid submission identity: ' + k)
    action = packet.get('action')
    if not isinstance(action, list) or len(action) != 21:
        raise ValueError('policy action must have 21 values')
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in action):
        raise ValueError('policy action must be finite numeric values')
    result['action_digest'] = array_digest_float32([action])
    return result


def identity(packet):
    return tuple(packet[k] for k in IDENTITY)


def matches(receipt, expected):
    return (all(receipt.get(k) == v for k, v in expected.items())
            and receipt.get('receipt_scope') == 'controller_submission'
            and receipt.get('physical_execution_confirmed') is False
            and type(receipt.get('accepted')) is bool)


def project_configuration(q, lower, upper, tolerance_deg=2.0):
    q = np.asarray(q, dtype=float)
    lower, upper = np.asarray(lower), np.asarray(upper)
    if (q.shape != lower.shape or q.shape != upper.shape
            or not np.isfinite(q).all() or not 0 <= tolerance_deg <= 2.0):
        raise ValueError('invalid bounded policy configuration')
    projected = np.clip(q, lower, upper)
    violation = np.rad2deg(np.abs(q - projected))
    if float(violation.max()) > tolerance_deg + 1e-6:
        raise ValueError('policy target exceeds soft-limit tolerance: '
                         f'{float(violation.max()):.3f} deg > {tolerance_deg:.3f} deg')
    return projected, violation


class PolicyMotion:
    """Independent history for 100-Hz policy motion, reset at every epoch."""
    def __init__(self, limiter_type, lower, upper):
        # Same conservative arm speed/acceleration as the reviewed V5 servo;
        # waist and neck use the existing conservative reset envelopes.
        speed = np.r_[np.full(4, 5.), np.full(14, 80.), np.full(3, 30.)]
        accel = np.r_[np.full(4, 12.), np.full(14, 300.), np.full(3, 83.333)]
        self.lead = np.r_[np.full(4, 5.), np.full(14, 12.), np.full(3, 8.)]
        self.limiter = limiter_type(lower, upper, speed, accel)
        self.epoch = None
        self.history = collections.deque(maxlen=260)

    def step(self, desired, measured, dt, epoch, now):
        desired, measured = np.asarray(desired), np.asarray(measured)
        if self.epoch != epoch:
            self.limiter.reset(measured)
            self.epoch = epoch
            self.history.clear()
        goal = np.clip(desired, measured - self.lead, measured + self.lead)
        command = self.limiter.step(goal, min(float(dt), .02))
        error = np.abs(command - measured)
        if np.any(error > self.lead + .1):
            raise RuntimeError('policy feedback moved outside the bounded execution window')
        self.history.append((now, measured.copy(), error.copy()))
        while len(self.history) > 1 and now - self.history[1][0] >= 2.0:
            self.history.popleft()
        if self.history and now - self.history[0][0] >= 2.0:
            motion = np.ptp(np.asarray([x[1] for x in self.history]), axis=0)
            sustained = np.min(np.asarray([x[2] for x in self.history]), axis=0)
            stalled = np.flatnonzero((motion < .3) & (sustained > 8.0))
            if stalled.size:
                raise RuntimeError(f'policy actuator stalled for 2 seconds: body axis {int(stalled[0])}')
        return command


class LocalPolicyExecutionMixin:
    """Installed only in the checked, private V5 controller runtime copy."""
    def _init_local_policy_execution(self):
        self._local_policy_enabled = enabled()
        self._local_policy_motion = None
        self._local_policy_cache = collections.OrderedDict()
        if not self._local_policy_enabled:
            return
        if self._output_control_mode != 'position':
            raise ValueError('bounded policy execution requires the reviewed position-control configuration')
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos = QoSProfile(depth=16, reliability=ReliabilityPolicy.RELIABLE)
        self._local_policy_receipt_pub = self.create_publisher(String, RECEIPT_TOPIC, qos)
        self.create_subscription(String, POLICY_TOPIC, self._on_local_policy_submission, qos)
        self.create_timer(.1, self._publish_local_policy_capability)

    def _publish_local_policy_capability(self):
        self._publish_local_policy_receipt(dict(kind='capability', version=1,
            enabled=True, timestamp_ns=time.time_ns()))

    def _publish_local_policy_receipt(self, packet):
        from std_msgs.msg import String
        self._local_policy_receipt_pub.publish(String(data=json.dumps(packet, separators=(',', ':'))))

    def _on_local_policy_submission(self, message):
        try:
            packet = json.loads(message.data)
            bound = binding(packet)
        except (KeyError, TypeError, ValueError):
            return  # No unbound receipt can acknowledge an action.
        key = identity(bound)
        with self._lock:
            previous = self._local_policy_cache.get(key)
            if packet.get('query_only') is True:
                if previous and previous['action_digest'] == bound['action_digest']:
                    self._publish_local_policy_receipt(dict(previous, recovered=True))
                return  # Querying never applies or reapplies a command.
            receipt = dict(bound, receipt_scope='controller_submission',
                physical_execution_confirmed=False, timestamp_ns=time.time_ns())
            try:
                if not self._local_policy_enabled:
                    raise ValueError('bounded policy execution is disabled')
                if packet.get('units') != 'degrees':
                    raise ValueError('policy submission must use degrees')
                if (packet.get('collector_session_id') != self._collector_session_id
                        or not self._collector_target_allowed(packet, policy_only=True)):
                    raise ValueError('policy authority revoked or stale')
                if previous:
                    if previous['action_digest'] != bound['action_digest']:
                        raise ValueError('submission identity reused with different action')
                    self._publish_local_policy_receipt(previous)
                    return
                stamp = packet.get('timestamp_ns')
                if type(stamp) is not int or not 0 <= time.time_ns() - stamp <= 500_000_000:
                    raise ValueError('policy submission timestamp is missing or stale')
                if (not self._hardware_enabled or self._state == 'FAULT'
                        or self._reset_active or self._enable_pending or self._estop_latched):
                    reason = (getattr(self, '_last_hardware_fault_reason', None)
                              or self._reason or 'controller is not enabled for policy submissions')
                    receipt.update(controller_state=self._state,
                                   hardware_enabled=bool(self._hardware_enabled))
                    raise ValueError(reason)
                if self._feedback is None or time.monotonic() - self._feedback_time > float(
                        self.get_parameter('feedback_timeout_sec').value):
                    raise ValueError('joint feedback is stale')
                action = np.asarray(packet['action'], dtype=float)
                if np.any(action[14:16] < 10.) or np.any(action[14:16] > 360.):
                    raise ValueError('gripper target outside [10, 360]')
                groups = self._feedback.as_dict()
                groups = {k: np.asarray(v, dtype=float).copy() for k, v in groups.items()}
                groups.update(left_arm=action[:7].copy(), right_arm=action[7:14].copy(),
                              neck=action[16:19].copy())
                groups['leg_waist'][2:4] = action[19:21]
                q = self._kinematics.q_from_feedback(groups, clip=False)
                try:
                    q, violation = project_configuration(q, self._kinematics.lower,
                                                         self._kinematics.upper)
                except ValueError as exc:
                    index = int(np.argmax(np.abs(q - np.clip(
                        q, self._kinematics.lower, self._kinematics.upper))))
                    name = next((n for n, i in self._kinematics.joint_q.items()
                                 if i == index), str(index))
                    raise ValueError(str(exc) + '; joint ' + name) from exc
                bounded = self._kinematics.groups_from_q_deg(q, groups)
                guard, _ = self._configuration_guard_error(bounded, 'bounded policy target')
                if guard:
                    raise ValueError(guard)
                # Commit body + both grippers under one authority/feedback lock.
                self._grip_ik.reset()
                self._latest_target_mailbox.reset()
                self._collector_body_origin_epoch = bound['authority_epoch']
                self._collector_gripper_origin_epoch = bound['authority_epoch']
                self._target_groups = bounded
                self._target_time = time.monotonic()
                self._target_source = 'joint_space'
                self._target_velocity_estimate.fill(0.)
                self._velocity_active_sides = {'left', 'right'}
                for side, value in zip(('left', 'right'), action[14:16]):
                    self._gripper_targets[side] = float(value)
                    self._gripper_dirty[side] = True
                self._state = 'ARMED'
                self._reason = 'bounded policy submission accepted'
                effective = [*bounded['left_arm'], *bounded['right_arm'],
                             *action[14:16], *bounded['neck'], *bounded['leg_waist'][2:4]]
                limited = {name: float(violation[index])
                           for name, index in self._kinematics.joint_q.items()
                           if violation[index] > 1e-6}
                receipt.update(accepted=True, reason=self._reason,
                    requested_action=packet['action'], bounded_target=effective,
                    limited_joints=limited)
            except Exception as exc:
                receipt.update(accepted=False, reason=str(exc))
            # Never replace an earlier accepted result with a stale/reused query.
            if previous is None:
                self._local_policy_cache[key] = receipt
                while len(self._local_policy_cache) > 128:
                    self._local_policy_cache.popitem(last=False)
            self._publish_local_policy_receipt(receipt)

    def _bounded_local_policy_command(self, desired, measured, dt, now):
        if self._local_policy_motion is None:
            self._local_policy_motion = PolicyMotion(type(self._limiter),
                                                     self._limiter.lower, self._limiter.upper)
        return self._local_policy_motion.step(desired, measured, dt,
            (self._collector_session_id, self._collector_body_origin_epoch), now)
