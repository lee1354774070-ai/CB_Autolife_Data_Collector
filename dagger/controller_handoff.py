"""Atomic authority handoff inside the existing V4 controller's lock.

No service calls, status acknowledgement, sleeps, or second motor publisher.
The checked V4 copy calls this before installing its new authority mode.
"""

import time


def apply_authority(controller, payload, mode):
    epoch = payload.get("authority_epoch")
    session = payload.get("session_id")
    if type(epoch) is not int or epoch < 0 or not isinstance(session, str):
        return False
    old_epoch = controller._collector_authority_epoch
    old_session = controller._collector_session_id
    if session == old_session and epoch < old_epoch:
        return False
    controller._collector_reset_pending = bool(payload.get('quick_reset', {}).get('pending'))
    old_mode = controller._follow_authority_mode
    entering_expert = (mode in ("EXPERT_READY", "EXPERT_ACTIVE")
                       and old_mode in ("POLICY_ACTIVE", "POLICY_WARMUP"))
    stopping = (epoch != old_epoch and old_mode in (
        "POLICY_ACTIVE", "POLICY_WARMUP", "EXPERT_READY", "EXPERT_ACTIVE")
        and mode in ("DISARMED", "ESTOP", "FAILURE_HOLD", "EXPERT_RELEASE_REQUIRED", "EXPERT_READY"))
    if entering_expert or stopping or getattr(controller, '_collector_handoff_pending', False):
        controller._collector_handoff_pending = True
        # Fence immediately, including any IK already running off-lock. Its
        # commit rechecks the epoch and mailbox sequence before accepting it.
        controller._collector_revoked_epoch = max(controller._collector_revoked_epoch, old_epoch)
        controller._latest_target_mailbox.reset()
        now = time.monotonic()
        fresh = (controller._feedback is not None
                 and 0 <= now - controller._feedback_time
                 <= float(controller.get_parameter('feedback_timeout_sec').value))
        if stopping and controller._reset_active:
            # A commanded reset is a separate owner; a late DISARMED heartbeat
            # must never rebase the reset trajectory midway through its move.
            controller._collector_handoff_pending = False
            controller._collector_authority_epoch = epoch
            controller._collector_session_id = session
            return True
        if (not fresh or controller._reset_active or controller._enable_pending
                or controller._estop_latched):
            # Do not claim expert readiness or refresh authority on bad data.
            # The ordinary controller watchdog remains responsible for halt.
            controller._follow_authority_mode = ''
            controller._follow_authority_time = 0.
            controller._target_time = 0.
            if not controller._reset_active:
                controller._target_groups = None
                controller._gripper_dirty = {'left': False, 'right': False}
            controller._reason = 'authority handoff rejected: feedback/state unavailable'
            return False
        # Reuse the vendor's clutch/trajectory reset, but never its hold-ACK
        # state machine. Replace all outstanding policy goals, including waist,
        # neck and grippers, with measured values in this same critical section.
        controller._begin_fresh_teleop_session_locked(controller._feedback.as_dict())
        controller._gripper_targets = {
            'left': float(controller._feedback.left_gripper[0]),
            'right': float(controller._feedback.right_gripper[0]),
        }
        controller._gripper_dirty = {'left': True, 'right': True}
        controller._collector_body_origin_epoch = -1
        controller._collector_gripper_origin_epoch = -1
        controller._reason = 'authority changed; current-pose anchor'
        controller._collector_handoff_pending = False
    controller._collector_authority_epoch = epoch
    controller._collector_session_id = session
    return True
