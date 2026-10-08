"""Safety gate for operating both controllers while the headset is stationary.

The gate deliberately contains no ROS dependencies so its reference-frame
rules can be exercised with deterministic unit tests.  It never creates a
motion target; it only decides whether a fresh WebXR frame is stable enough to
permit a new clutch anchor.
"""

import numpy as np


def _vector3(value, label):
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError(f'{label} must contain three finite values')
    return vector


def _angular_distance_deg(current, reference):
    delta = (_vector3(current, 'head rotation')
             - _vector3(reference, 'head rotation') + 180.0) % 360.0 - 180.0
    return float(np.linalg.norm(delta))


def world_to_operator_yaw_rotation(yaw_deg):
    """Return the WebXR world-to-operator translation basis.

    The shortcut is pressed while the operator is still wearing the headset.
    Freezing that yaw keeps physical forward mapped to robot forward after the
    headset is placed on a table in any orientation.
    """
    yaw = float(yaw_deg)
    if not np.isfinite(yaw):
        raise ValueError('operator yaw must be finite')
    angle = np.deg2rad(-yaw)
    cosine = float(np.cos(angle))
    sine = float(np.sin(angle))
    return np.array([
        [cosine, 0.0, sine],
        [0.0, 1.0, 0.0],
        [-sine, 0.0, cosine],
    ], dtype=float)


class DesktopModeGate:
    """Require a stationary HMD/controllers before desktop teleoperation.

    Once ready, normal controller motion is permitted.  The locked HMD remains
    a reference-space sentinel: moving it, rotating it, or an implausible
    one-frame controller jump closes the gate and requires a fresh settle.
    """

    def __init__(
            self, *, settle_seconds=1.0, head_stability_m=0.015,
            controller_stability_m=0.035, head_stability_deg=3.0,
            head_jump_m=0.08, head_jump_deg=10.0,
            controller_jump_m=0.18):
        self.settle_seconds = max(0.1, float(settle_seconds))
        self.head_stability_m = max(0.001, float(head_stability_m))
        self.controller_stability_m = max(
            0.001, float(controller_stability_m)
        )
        self.head_stability_deg = max(0.1, float(head_stability_deg))
        self.head_jump_m = max(self.head_stability_m, float(head_jump_m))
        self.head_jump_deg = max(
            self.head_stability_deg, float(head_jump_deg)
        )
        self.controller_jump_m = max(
            self.controller_stability_m, float(controller_jump_m)
        )
        self.enabled = False
        self.ready = False
        self.reason = 'desktop mode is off'
        self._stable_since = None
        self._stable_elapsed = 0.0
        self._settle_anchor = None
        self._last = None
        self._locked_head_position = None
        self._locked_head_rotation = None

    @property
    def stable_elapsed(self):
        return 0.0 if self._stable_since is None else self._stable_elapsed

    def set_enabled(self, enabled, now):
        self.enabled = bool(enabled)
        self.reset_tracking(now)
        self.reason = (
            'waiting for stationary headset and both controllers'
            if self.enabled else 'desktop mode is off'
        )

    def reset_tracking(self, now, reason=None):
        self.ready = False
        self._stable_since = float(now)
        self._stable_elapsed = 0.0
        self._settle_anchor = None
        self._last = None
        self._locked_head_position = None
        self._locked_head_rotation = None
        if reason:
            self.reason = str(reason)

    @staticmethod
    def _frame(head_position, head_rotation_deg, controllers):
        if not isinstance(controllers, dict):
            raise ValueError('both controller positions are required')
        return {
            'head_position': _vector3(head_position, 'head position'),
            'head_rotation': _vector3(head_rotation_deg, 'head rotation'),
            'left': _vector3(controllers.get('left'), 'left controller'),
            'right': _vector3(controllers.get('right'), 'right controller'),
        }

    def observe(self, *, head_position, head_rotation_deg, controllers, now):
        if not self.enabled:
            return False, 'desktop mode is off'
        now = float(now)
        try:
            frame = self._frame(
                head_position, head_rotation_deg, controllers
            )
        except (TypeError, ValueError) as exc:
            self.reset_tracking(now, f'desktop tracking incomplete: {exc}')
            return False, self.reason

        if self.ready:
            head_shift = float(np.linalg.norm(
                frame['head_position'] - self._locked_head_position
            ))
            head_turn = _angular_distance_deg(
                frame['head_rotation'], self._locked_head_rotation
            )
            controller_jump = max(
                float(np.linalg.norm(frame[side] - self._last[side]))
                for side in ('left', 'right')
            )
            if (
                head_shift > self.head_jump_m
                or head_turn > self.head_jump_deg
                or controller_jump > self.controller_jump_m
            ):
                reason = (
                    'desktop reference moved; release both grips and keep the '
                    'headset/controllers still'
                )
                self.reset_tracking(now, reason)
                self._settle_anchor = frame
                self._last = frame
                return False, reason
            self._last = frame
            self.reason = 'desktop reference is stable'
            return True, self.reason

        if self._settle_anchor is None:
            self._settle_anchor = frame
            self._last = frame
            self._stable_since = now
            self._stable_elapsed = 0.0
            self.reason = 'keep the headset and both controllers still'
            return False, self.reason

        head_motion = float(np.linalg.norm(
            frame['head_position'] - self._settle_anchor['head_position']
        ))
        head_turn = _angular_distance_deg(
            frame['head_rotation'], self._settle_anchor['head_rotation']
        )
        controller_motion = max(
            float(np.linalg.norm(frame[side] - self._settle_anchor[side]))
            for side in ('left', 'right')
        )
        if (
            head_motion > self.head_stability_m
            or head_turn > self.head_stability_deg
            or controller_motion > self.controller_stability_m
        ):
            self._settle_anchor = frame
            self._stable_since = now
            self._stable_elapsed = 0.0
            self.reason = 'tracking is moving; restart the one-second settle'
            self._last = frame
            return False, self.reason

        self._stable_elapsed = max(0.0, now - self._stable_since)
        self._last = frame
        if self._stable_elapsed < self.settle_seconds:
            self.reason = 'checking stationary desktop tracking'
            return False, self.reason

        self.ready = True
        self._locked_head_position = frame['head_position'].copy()
        self._locked_head_rotation = frame['head_rotation'].copy()
        self.reason = 'desktop reference is stable'
        return True, self.reason

    def status(self):
        return {
            'enabled': bool(self.enabled),
            'ready': bool(self.ready),
            'stable_elapsed': round(float(self._stable_elapsed), 3),
            'settle_seconds': float(self.settle_seconds),
            'reason': str(self.reason),
        }
