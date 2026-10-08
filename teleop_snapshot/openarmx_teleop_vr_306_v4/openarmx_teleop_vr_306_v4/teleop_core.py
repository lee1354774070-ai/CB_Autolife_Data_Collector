"""Pure pose mapping and safety helpers for VR arm teleoperation."""

from dataclasses import dataclass
import math

import numpy as np


VR_TO_ROBOT_ROT = np.array(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
    dtype=np.float64,
)


def vr_input_is_fresh(vr_age, maximum_age):
    """Return true only for a finite, non-negative, recent VR sample age."""
    try:
        age = float(vr_age)
        limit = float(maximum_age)
    except (TypeError, ValueError):
        return False
    return (
        math.isfinite(age)
        and math.isfinite(limit)
        and age >= 0.0
        and limit > 0.0
        and age <= limit
    )


def forward_reach_to_waist_pitch(
        forward_x_m,
        neutral_pitch_deg,
        start_x_m,
        full_x_m,
        maximum_lean_deg,
        upright_pitch_deg=0.0):
    """Map forward hand reach to a bounded forward waist pitch.

    Robot 306's physical waist leans forward in the positive pitch direction.
    The helper adds a normal-workspace dead zone, grows linearly, then
    saturates before the mechanical limit. It is pure so the mapping can be
    tested off-robot.
    """
    values = [
        float(forward_x_m),
        float(neutral_pitch_deg),
        float(start_x_m),
        float(full_x_m),
        float(maximum_lean_deg),
        float(upright_pitch_deg),
    ]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('waist forward-assist parameters must be finite')
    forward_x, neutral, start_x, full_x, maximum_lean, upright = values
    if full_x <= start_x:
        raise ValueError('waist forward-assist full reach must exceed start reach')
    if maximum_lean < 0.0:
        raise ValueError('waist forward-assist maximum lean must be non-negative')
    linear_ratio = min(1.0, max(0.0, (forward_x - start_x) / (full_x - start_x)))
    ratio = linear_ratio * linear_ratio * (3.0 - 2.0 * linear_ratio)
    # The neutral is the current coordinated body pose, not an absolute motor
    # zero. Reverse crouching needs a negative waist pitch to keep the torso
    # upright. Clamping it to zero bends the torso while the legs stay crouched.
    # Only the additional reach contribution is non-negative.
    return neutral + ratio * maximum_lean


def forward_limit_with_wrist_reserve(
        maximum_forward_m,
        orientation_angle_rad,
        minimum_reserve_m,
        maximum_reserve_m,
        reserve_start_angle_rad,
        reserve_full_angle_rad):
    """Keep the smallest smooth reach reserve needed for wrist dexterity.

    An exactly straight seven-axis arm is kinematically singular: Cartesian
    wrist rotation can require an impossible end-effector position. This
    helper reduces only the forward cap as the hand rotates away from its
    clutch orientation, avoiding alternation between incompatible IK modes.
    """
    values = [
        float(maximum_forward_m),
        float(orientation_angle_rad),
        float(minimum_reserve_m),
        float(maximum_reserve_m),
        float(reserve_start_angle_rad),
        float(reserve_full_angle_rad),
    ]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('wrist reach-reserve parameters must be finite')
    maximum_forward, angle, minimum, maximum, start, full = values
    if maximum_forward <= 0.0:
        raise ValueError('maximum forward reach must be positive')
    if minimum < 0.0 or maximum < minimum or maximum >= maximum_forward:
        raise ValueError('wrist reach-reserve distance envelope is invalid')
    if start < 0.0 or full <= start:
        raise ValueError('wrist reach-reserve angle envelope is invalid')
    ratio = min(1.0, max(0.0, (abs(angle) - start) / (full - start)))
    ratio = ratio * ratio * (3.0 - 2.0 * ratio)
    reserve = minimum + ratio * (maximum - minimum)
    return maximum_forward - reserve


def vr_tracking_state(vr_age, pause_timeout, disable_timeout):
    """Return active, paused or disabled for a controller-tracking age."""
    try:
        age = float(vr_age)
        pause = float(pause_timeout)
        disable = float(disable_timeout)
    except (TypeError, ValueError):
        return 'disabled'
    if not all(math.isfinite(value) for value in (age, pause, disable)):
        return 'disabled'
    if age < 0.0 or pause <= 0.0 or disable < pause:
        return 'disabled'
    if age <= pause:
        return 'active'
    if age <= disable:
        return 'paused'
    return 'disabled'


def grip_release_state(raw_active, control_latched, false_since, now, delay):
    """Classify a noisy WebXR grip sample without masking a real release.

    A single false sample is common when a WebXR gamepad frame is late or the
    squeeze button crosses its browser ``pressed`` threshold.  Once an arm is
    latched, hold its last target for a short, bounded interval and only
    release after false has remained continuous for ``delay`` seconds.
    """
    timestamp = float(now)
    debounce = max(0.0, float(delay))
    if bool(raw_active):
        return 'active', None
    if not bool(control_latched):
        return 'released', None
    started = timestamp if false_since is None else float(false_since)
    if timestamp - started < debounce:
        return 'pending', started
    return 'released', started


def ik_rejected_sides(status):
    """Return the arm sides implicated by a live Cartesian IK rejection.

    The mapper uses this signal to re-clutch at the measured pose.  Without a
    re-clutch, an unreachable target keeps moving while the controller holds
    the previous solution, then produces a visible catch-up jump when IK
    becomes reachable again.
    """
    if not isinstance(status, dict):
        return []
    reason = str(status.get('reason', ''))
    if status.get('state') != 'HOLDING' or not reason.startswith('IK rejected:'):
        return []
    ik = status.get('ik')
    if not isinstance(ik, dict) or bool(ik.get('success', False)):
        return []
    sides = set()
    for key in ('position_error_m', 'orientation_error_rad'):
        errors = ik.get(key)
        if isinstance(errors, dict):
            sides.update(side for side in errors if side in ('left', 'right'))
    return [side for side in ('left', 'right') if side in sides]


def normalized_quaternion(value):
    quaternion = np.asarray(value, dtype=np.float64)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise ValueError('quaternion must contain four finite values')
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-9:
        raise ValueError('quaternion norm is zero')
    return quaternion / norm


def wrap_degrees(value):
    """Wrap one angle or an array of angles into [-180, 180)."""
    values = np.asarray(value, dtype=np.float64)
    wrapped = (values + 180.0) % 360.0 - 180.0
    return float(wrapped) if wrapped.ndim == 0 else wrapped


def quaternion_to_matrix(value):
    x, y, z, w = normalized_quaternion(value)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def webxr_yaw_deg_from_quaternion(value):
    """Return headset yaw around WebXR +Y without Euler cross-axis coupling.

    WebXR looks along local -Z. Positive yaw turns that forward vector toward
    world -X, matching the sign used by ``world_to_operator_yaw_rotation``.
    Pitch and roll do not contaminate the horizontal forward projection unless
    the headset is pointed almost vertically, where the last clutch basis is a
    safer fallback at the mapper layer.
    """
    rotation = quaternion_to_matrix(value)
    forward = rotation @ np.asarray([0.0, 0.0, -1.0], dtype=np.float64)
    horizontal = math.hypot(float(forward[0]), float(forward[2]))
    if horizontal < 1.0e-6:
        raise ValueError('headset forward direction is vertical')
    return math.degrees(math.atan2(-float(forward[0]), -float(forward[2])))


def matrix_to_quaternion(matrix):
    matrix = np.asarray(matrix, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            [
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        axis = int(np.argmax(np.diag(matrix)))
        if axis == 0:
            scale = math.sqrt(max(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2], 0.0)) * 2.0
            quaternion = np.array(
                [0.25 * scale, (matrix[0, 1] + matrix[1, 0]) / scale,
                 (matrix[0, 2] + matrix[2, 0]) / scale, (matrix[2, 1] - matrix[1, 2]) / scale]
            )
        elif axis == 1:
            scale = math.sqrt(max(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2], 0.0)) * 2.0
            quaternion = np.array(
                [(matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale,
                 (matrix[1, 2] + matrix[2, 1]) / scale, (matrix[0, 2] - matrix[2, 0]) / scale]
            )
        else:
            scale = math.sqrt(max(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1], 0.0)) * 2.0
            quaternion = np.array(
                [(matrix[0, 2] + matrix[2, 0]) / scale,
                 (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale,
                 (matrix[1, 0] - matrix[0, 1]) / scale]
            )
    return normalized_quaternion(quaternion)


def slerp(start, end, fraction):
    start = normalized_quaternion(start)
    end = normalized_quaternion(end)
    dot = float(np.dot(start, end))
    if dot < 0.0:
        end = -end
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    fraction = float(np.clip(fraction, 0.0, 1.0))
    if dot > 0.9995:
        return normalized_quaternion(start + fraction * (end - start))
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    return normalized_quaternion(
        math.sin((1.0 - fraction) * theta) / sin_theta * start
        + math.sin(fraction * theta) / sin_theta * end
    )


def quaternion_angle(start, end):
    dot = abs(float(np.dot(normalized_quaternion(start), normalized_quaternion(end))))
    return 2.0 * math.acos(float(np.clip(dot, 0.0, 1.0)))


def scale_quaternion_rotation(value, gain):
    """Scale a relative rotation on its shortest axis-angle path."""
    quaternion = normalized_quaternion(value)
    factor = float(gain)
    if not math.isfinite(factor) or factor <= 0.0:
        raise ValueError('rotation gain must be positive and finite')
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    vector = quaternion[:3]
    magnitude = float(np.linalg.norm(vector))
    if magnitude < 1e-9:
        return np.asarray([0.0, 0.0, 0.0, 1.0])
    angle = 2.0 * math.atan2(magnitude, float(quaternion[3]))
    scaled = min(math.pi, angle * factor)
    axis = vector / magnitude
    return normalized_quaternion(np.concatenate((
        axis * math.sin(0.5 * scaled),
        [math.cos(0.5 * scaled)],
    )))


def parse_eef_feedback(payload):
    result = {}
    for side in ('left', 'right'):
        nested = payload.get(f'{side}_eef_pose') or {}
        position = payload.get(f'pos_{side}_in_robot') or nested.get('position')
        orientation = (
            payload.get(f'quat_{side}_in_robot')
            or nested.get('rotation')
            or nested.get('orientation')
        )
        if len(position or []) == 3 and len(orientation or []) == 4:
            result[side] = {
                'position': np.asarray(position, dtype=np.float64),
                'orientation': normalized_quaternion(orientation),
            }
    return result


@dataclass
class ControllerSample:
    position: np.ndarray
    orientation: np.ndarray
    grip_active: bool
    trigger: float

    @classmethod
    def from_packet(cls, value):
        if not isinstance(value, dict):
            raise ValueError('controller is not an object')
        position = value.get('position') or {}
        orientation = value.get('quaternion') or {}
        position_array = np.asarray(
            [position.get('x'), position.get('y'), position.get('z')], dtype=np.float64
        )
        orientation_array = np.asarray(
            [orientation.get('x'), orientation.get('y'), orientation.get('z'), orientation.get('w')],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(position_array)):
            raise ValueError('controller position is invalid')
        trigger = float(value.get('trigger', 0.0))
        if not math.isfinite(trigger):
            raise ValueError('controller trigger is invalid')
        return cls(
            position=position_array,
            orientation=normalized_quaternion(orientation_array),
            grip_active=bool(value.get('gripActive', False)),
            trigger=trigger,
        )


@dataclass
class HeadSample:
    """Finite WebXR headset pose in the active reference space."""

    rotation: np.ndarray
    position: np.ndarray = None
    quaternion: np.ndarray = None

    @classmethod
    def from_packet(cls, value):
        if not isinstance(value, dict):
            raise ValueError('head is not an object')
        rotation = value.get('rotation')
        if not isinstance(rotation, dict):
            raise ValueError('head rotation is missing')
        angles = np.asarray(
            [rotation.get('x'), rotation.get('y'), rotation.get('z')],
            dtype=np.float64,
        )
        if angles.shape != (3,) or not np.all(np.isfinite(angles)):
            raise ValueError('head rotation must contain finite x/y/z angles')
        position = value.get('position')
        position_array = None
        if position is not None:
            if not isinstance(position, dict):
                raise ValueError('head position must be an object')
            position_array = np.asarray(
                [position.get('x'), position.get('y'), position.get('z')],
                dtype=np.float64,
            )
            if position_array.shape != (3,) or not np.all(
                    np.isfinite(position_array)):
                raise ValueError('head position must contain finite x/y/z values')
        quaternion = value.get('quaternion')
        quaternion_array = None
        if quaternion is not None:
            if not isinstance(quaternion, dict):
                raise ValueError('head quaternion must be an object')
            quaternion_array = np.asarray(
                [
                    quaternion.get('x'), quaternion.get('y'),
                    quaternion.get('z'), quaternion.get('w'),
                ],
                dtype=np.float64,
            )
            if quaternion_array.shape != (4,) or not np.all(
                    np.isfinite(quaternion_array)):
                raise ValueError(
                    'head quaternion must contain finite x/y/z/w values'
                )
            norm = float(np.linalg.norm(quaternion_array))
            if norm < 1.0e-8:
                raise ValueError('head quaternion has zero length')
            quaternion_array = quaternion_array / norm
        return cls(
            rotation=angles,
            position=position_array,
            quaternion=quaternion_array,
        )


def quaternion_multiply_xyzw(left, right):
    """Hamilton product for normalized WebXR quaternions in XYZW order."""
    lx, ly, lz, lw = np.asarray(left, dtype=np.float64)
    rx, ry, rz, rw = np.asarray(right, dtype=np.float64)
    return np.asarray([
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ], dtype=np.float64)


def relative_quaternion_rotation_vector_deg(origin, current):
    """Return the shortest origin-relative WebXR rotation vector in degrees.

    A rotation vector avoids the Euler-axis exchange that made combined head
    pitch/yaw/roll feel coupled.  Components remain WebXR X/Y/Z so the existing
    explicit robot Z/X/Y permutation stays visible at the mapping boundary.
    """
    origin = np.asarray(origin, dtype=np.float64)
    current = np.asarray(current, dtype=np.float64)
    origin = origin / np.linalg.norm(origin)
    current = current / np.linalg.norm(current)
    conjugate = np.asarray(
        [-origin[0], -origin[1], -origin[2], origin[3]], dtype=np.float64
    )
    delta = quaternion_multiply_xyzw(conjugate, current)
    delta = delta / np.linalg.norm(delta)
    if delta[3] < 0.0:
        delta = -delta
    vector_norm = float(np.linalg.norm(delta[:3]))
    if vector_norm < 1.0e-10:
        return np.zeros(3, dtype=np.float64)
    angle = 2.0 * math.atan2(vector_norm, float(delta[3]))
    return np.rad2deg(delta[:3] * (angle / vector_norm))


def quaternion_distance_deg(left, right):
    """Unsigned shortest angular distance between two XYZW quaternions."""
    left = np.asarray(left, dtype=np.float64)
    right = np.asarray(right, dtype=np.float64)
    left = left / np.linalg.norm(left)
    right = right / np.linalg.norm(right)
    dot = float(np.clip(abs(np.dot(left, right)), 0.0, 1.0))
    return math.degrees(2.0 * math.acos(dot))


class HeadMapper:
    """Map WebXR head orientation to neck Roll/Pitch/Yaw targets.

    A-Frame reports pitch around X, yaw around Y and roll around Z.  Robot neck
    order is Roll, Pitch, Yaw, hence the explicit [Z, X, Y] permutation.

    ``relative_neutral_quaternion`` captures the operator's current forward
    direction but maps it onto the calibrated robot neutral pose.  A brief
    tracking outage therefore cannot turn an off-centre measured neck pose
    into the next zero reference.  ``absolute_reference`` treats the WebXR
    reference-space orientation as an absolute direction.  The older relative
    modes remain available for compatibility.
    """

    def __init__(
        self,
        filter_alpha,
        maximum_step_deg,
        axis_gain,
        maximum_offset_deg,
        joint_minimum_deg,
        joint_maximum_deg,
        tracking_mode='relative_measured',
        neutral_neck_deg=(0.0, 0.0, 0.0),
        deadband_deg=(0.0, 0.0, 0.0),
        reference_jump_deg=35.0,
    ):
        self.filter_alpha = float(filter_alpha)
        self.maximum_step = np.broadcast_to(
            np.asarray(maximum_step_deg, dtype=np.float64), (3,)
        ).copy()
        self.axis_gain = np.broadcast_to(
            np.asarray(axis_gain, dtype=np.float64), (3,)
        ).copy()
        self.maximum_offset = np.broadcast_to(
            np.asarray(maximum_offset_deg, dtype=np.float64), (3,)
        ).copy()
        self.joint_minimum = np.asarray(joint_minimum_deg, dtype=np.float64)
        self.joint_maximum = np.asarray(joint_maximum_deg, dtype=np.float64)
        self.tracking_mode = str(tracking_mode).strip().lower()
        self.neutral_neck = np.asarray(neutral_neck_deg, dtype=np.float64)
        self.deadband = np.broadcast_to(
            np.asarray(deadband_deg, dtype=np.float64), (3,)
        ).copy()
        self.reference_jump = float(reference_jump_deg)
        arrays = (
            self.maximum_step, self.axis_gain, self.maximum_offset,
            self.joint_minimum, self.joint_maximum, self.neutral_neck,
            self.deadband,
        )
        if any(array.shape != (3,) or not np.all(np.isfinite(array)) for array in arrays):
            raise ValueError('head mapper parameters must contain three finite values')
        if (
            np.any(self.maximum_step <= 0.0)
            or np.any(self.maximum_offset <= 0.0)
            or np.any(self.deadband < 0.0)
            or not math.isfinite(self.reference_jump)
            or self.reference_jump <= 0.0
            or np.any(self.joint_minimum >= self.joint_maximum)
        ):
            raise ValueError('head mapper limits are invalid')
        if self.tracking_mode not in (
            'relative_measured', 'absolute_reference', 'relative_quaternion',
            'relative_neutral_quaternion',
        ):
            raise ValueError('unsupported head tracking mode')
        if np.any(self.neutral_neck < self.joint_minimum) or np.any(
                self.neutral_neck > self.joint_maximum):
            raise ValueError('neutral neck pose is outside configured limits')
        self.release()

    @property
    def latched(self):
        return self.head_origin is not None

    def release(self):
        self.head_origin = None
        self.head_origin_quaternion = None
        self.last_quaternion = None
        self.neck_origin = None
        self.filtered = None
        self.reference_rebases = 0
        self.recalibration_required = False

    def latch(self, sample, measured_neck_deg):
        measured = np.asarray(measured_neck_deg, dtype=np.float64)
        if measured.shape != (3,) or not np.all(np.isfinite(measured)):
            raise ValueError('measured neck pose must contain three finite values')
        self.head_origin = sample.rotation.copy()
        self.head_origin_quaternion = (
            None if sample.quaternion is None else sample.quaternion.copy()
        )
        self.last_quaternion = (
            None if sample.quaternion is None else sample.quaternion.copy()
        )
        self.neck_origin = measured.copy()
        self.filtered = measured.copy()

    def map(self, sample):
        if not self.latched:
            raise RuntimeError('head origin has not been latched')
        quaternion_mode = self.tracking_mode in (
            'relative_quaternion', 'relative_neutral_quaternion'
        )
        if (
            quaternion_mode
            and sample.quaternion is not None
            and self.head_origin_quaternion is not None
        ):
            if (
                self.tracking_mode == 'relative_neutral_quaternion'
                and self.recalibration_required
            ):
                return self.filtered.copy()
            # A Quest recenter/reference-space replacement presents as a single
            # implausibly large frame jump.  The neutral-relative mode holds
            # the last command until the operator explicitly resets or toggles
            # head follow; it must never silently turn the crooked pose into a
            # new forward reference.
            if (
                self.last_quaternion is not None
                and quaternion_distance_deg(
                    self.last_quaternion, sample.quaternion
                ) > self.reference_jump
            ):
                if self.tracking_mode == 'relative_neutral_quaternion':
                    self.recalibration_required = True
                    return self.filtered.copy()
                self.head_origin = sample.rotation.copy()
                self.head_origin_quaternion = sample.quaternion.copy()
                self.last_quaternion = sample.quaternion.copy()
                self.neck_origin = self.filtered.copy()
                self.reference_rebases += 1
                return self.filtered.copy()
            source_xyz = relative_quaternion_rotation_vector_deg(
                self.head_origin_quaternion, sample.quaternion
            )
            self.last_quaternion = sample.quaternion.copy()
            base_neck = (
                self.neutral_neck
                if self.tracking_mode == 'relative_neutral_quaternion'
                else self.neck_origin
            )
        elif self.tracking_mode == 'absolute_reference':
            # WebXR viewer Euler angles are already expressed in the active
            # reference space.  Mapping those angles directly makes looking
            # forward return the robot to its configured neutral pose instead
            # of preserving whatever neck offset existed at enable time.
            source_xyz = wrap_degrees(sample.rotation)
            base_neck = self.neutral_neck
        else:
            source_xyz = wrap_degrees(sample.rotation - self.head_origin)
            base_neck = self.neck_origin
        # Remove tiny headset orientation noise continuously rather than using a
        # discontinuous threshold.  Values beyond the deadband retain their
        # excess motion, so deliberate small head motion remains responsive.
        source_xyz = np.sign(source_xyz) * np.maximum(
            np.abs(source_xyz) - self.deadband, 0.0
        )
        # WebXR XYZ -> robot neck Roll/Pitch/Yaw = Z/X/Y.
        head_neck = source_xyz[[2, 0, 1]] * self.axis_gain
        head_neck = np.clip(
            head_neck, -self.maximum_offset, self.maximum_offset
        )
        # A calibrated robot can report a logical neck zero outside the generic
        # URDF range (306 currently does this on pitch).  Grandfather the latch
        # pose as a one-sided boundary: enabling never jumps, and motion may
        # return toward the configured range but may not travel farther out.
        safe_minimum = np.minimum(self.joint_minimum, self.neck_origin)
        safe_maximum = np.maximum(self.joint_maximum, self.neck_origin)
        target = np.clip(
            base_neck + head_neck,
            safe_minimum,
            safe_maximum,
        )
        alpha = float(np.clip(self.filter_alpha, 0.0, 1.0))
        filtered = self.filtered + alpha * (target - self.filtered)
        step = np.clip(filtered - self.filtered, -self.maximum_step, self.maximum_step)
        self.filtered = self.filtered + step
        return self.filtered.copy()


def linear_gripper_position(trigger, open_position, closed_position, deadzone=0.0):
    """Map an analogue trigger value to a gripper position."""
    value = float(np.clip(float(trigger), 0.0, 1.0))
    deadzone = float(np.clip(float(deadzone), 0.0, 0.95))
    if value <= deadzone:
        normalized = 0.0
    else:
        normalized = (value - deadzone) / (1.0 - deadzone)
    return float(open_position) + normalized * (
        float(closed_position) - float(open_position)
    )


class ArmMapper:
    """Maps relative VR motion onto one 306 robot end effector."""

    def __init__(
        self,
        side,
        position_scale,
        filter_alpha,
        maximum_position_step,
        maximum_orientation_step,
        maximum_anchor_displacement,
        workspace_minimum,
        workspace_maximum,
        left_y_minimum,
        right_y_maximum,
        forbidden_box_minimum,
        forbidden_box_maximum,
        forward_position_scale=0.0,
        maximum_forward_displacement=0.0,
        wrist_orientation_gain=1.0,
        wrist_reach_reserve_enabled=False,
        wrist_reach_reserve_minimum_m=0.015,
        wrist_reach_reserve_maximum_m=0.070,
        wrist_reach_reserve_start_angle_deg=0.0,
        wrist_reach_reserve_full_angle_deg=70.0,
    ):
        self.side = side
        self.position_scale = float(position_scale)
        self.forward_position_scale = float(forward_position_scale)
        if self.forward_position_scale <= 0.0:
            self.forward_position_scale = self.position_scale
        self.maximum_forward_displacement = float(maximum_forward_displacement)
        if not all(math.isfinite(value) and value > 0.0 for value in (
                self.position_scale, self.forward_position_scale)):
            raise ValueError('position scales must be positive and finite')
        if not math.isfinite(self.maximum_forward_displacement):
            raise ValueError('maximum forward displacement must be finite')
        self.wrist_orientation_gain = float(wrist_orientation_gain)
        if not math.isfinite(self.wrist_orientation_gain) or self.wrist_orientation_gain <= 0.0:
            raise ValueError('wrist orientation gain must be positive and finite')
        self.wrist_reach_reserve_enabled = bool(wrist_reach_reserve_enabled)
        self.wrist_reach_reserve_minimum_m = float(
            wrist_reach_reserve_minimum_m
        )
        self.wrist_reach_reserve_maximum_m = float(
            wrist_reach_reserve_maximum_m
        )
        self.wrist_reach_reserve_start_angle = math.radians(float(
            wrist_reach_reserve_start_angle_deg
        ))
        self.wrist_reach_reserve_full_angle = math.radians(float(
            wrist_reach_reserve_full_angle_deg
        ))
        if self.wrist_reach_reserve_enabled:
            forward_limit_with_wrist_reserve(
                self.maximum_forward_displacement,
                0.0,
                self.wrist_reach_reserve_minimum_m,
                self.wrist_reach_reserve_maximum_m,
                self.wrist_reach_reserve_start_angle,
                self.wrist_reach_reserve_full_angle,
            )
        self.filter_alpha = float(filter_alpha)
        self.maximum_position_step = float(maximum_position_step)
        self.maximum_orientation_step = float(maximum_orientation_step)
        self.maximum_anchor_displacement = float(maximum_anchor_displacement)
        self.workspace_minimum = np.asarray(workspace_minimum, dtype=np.float64)
        self.workspace_maximum = np.asarray(workspace_maximum, dtype=np.float64)
        self.left_y_minimum = float(left_y_minimum)
        self.right_y_maximum = float(right_y_maximum)
        self.forbidden_box_minimum = np.asarray(forbidden_box_minimum, dtype=np.float64)
        self.forbidden_box_maximum = np.asarray(forbidden_box_maximum, dtype=np.float64)
        self.release()

    @property
    def latched(self):
        return self.vr_origin_position is not None

    def release(self):
        self.vr_origin_position = None
        self.vr_origin_orientation = None
        self.body_origin_position = None
        self.last_body_position = None
        self.robot_origin_position = None
        self.robot_origin_orientation = None
        self.filtered_position = None
        self.filtered_orientation = None

    @staticmethod
    def _body_position(value):
        if value is None:
            return None
        position = np.asarray(value, dtype=np.float64).reshape(-1)
        if position.size != 3 or not np.all(np.isfinite(position)):
            raise ValueError('body reference position must contain three finite values')
        return position.copy()

    def latch(self, sample, current_pose, body_position=None):
        self.vr_origin_position = sample.position.copy()
        self.vr_origin_orientation = sample.orientation.copy()
        self.body_origin_position = self._body_position(body_position)
        self.last_body_position = (
            None if self.body_origin_position is None
            else self.body_origin_position.copy()
        )
        self.robot_origin_position = np.asarray(current_pose['position'], dtype=np.float64).copy()
        self.robot_origin_orientation = normalized_quaternion(current_pose['orientation'])
        self.filtered_position = self.robot_origin_position.copy()
        self.filtered_orientation = self.robot_origin_orientation.copy()

    def _constrain_position(self, position):
        """Project a target onto safety boundaries instead of freezing motion.

        A rejected Cartesian sample makes an inward hand motion appear as
        latency: the arm remains at the last valid sample until the controller
        leaves the forbidden volume.  Projection preserves tangential motion
        along the boundary while the URDF/SRDF collision check remains the
        final physical guard.
        """
        constrained = np.asarray(position, dtype=np.float64).copy()
        reasons = []
        clipped = np.clip(
            constrained, self.workspace_minimum, self.workspace_maximum
        )
        if not np.allclose(clipped, constrained):
            reasons.append('target limited at the configured robot workspace')
            constrained = clipped
        if self.side == 'left' and constrained[1] < self.left_y_minimum:
            constrained[1] = self.left_y_minimum
            reasons.append('left arm target limited at the configured centre boundary')
        if self.side == 'right' and constrained[1] > self.right_y_maximum:
            constrained[1] = self.right_y_maximum
            reasons.append('right arm target limited at the configured centre boundary')

        inside_forbidden = np.all(
            constrained >= self.forbidden_box_minimum
        ) and np.all(constrained <= self.forbidden_box_maximum)
        if inside_forbidden:
            previous = (
                self.filtered_position
                if self.filtered_position is not None
                else self.robot_origin_position
            )
            candidates = []
            # Prefer the face from which the arm entered the box.  This avoids
            # switching between faces as the hand moves along the torso.
            for axis in range(3):
                if previous[axis] < self.forbidden_box_minimum[axis]:
                    candidates.append((axis, self.forbidden_box_minimum[axis]))
                elif previous[axis] > self.forbidden_box_maximum[axis]:
                    candidates.append((axis, self.forbidden_box_maximum[axis]))
            if not candidates:
                # Defensive recovery if configuration/re-anchoring starts
                # inside the box: leave through the nearest face.
                for axis in range(3):
                    candidates.extend((
                        (axis, self.forbidden_box_minimum[axis]),
                        (axis, self.forbidden_box_maximum[axis]),
                    ))
            axis, boundary = min(
                candidates,
                key=lambda item: abs(float(constrained[item[0]] - item[1])),
            )
            constrained[axis] = boundary
            reasons.append('target sliding along the torso exclusion boundary')
        return constrained, '; '.join(reasons)

    def map(self, sample, body_position=None):
        if not self.latched:
            raise RuntimeError('controller origin has not been latched')
        vr_delta = sample.position - self.vr_origin_position
        current_body = self._body_position(body_position)
        if self.body_origin_position is None and current_body is not None:
            # Begin compensation without a target jump if headset position was
            # temporarily absent when the Grip session was latched.
            self.body_origin_position = current_body.copy()
            self.last_body_position = current_body.copy()
        elif self.body_origin_position is not None:
            if current_body is None:
                current_body = self.last_body_position
            else:
                self.last_body_position = current_body.copy()
            if current_body is not None:
                # WebXR hand coordinates are room-relative.  Remove the HMD's
                # common translation so walking/stepping with the same arm
                # posture does not masquerade as an arm command.  Genuine
                # hand motion relative to the operator remains unchanged.
                vr_delta = vr_delta - (
                    current_body - self.body_origin_position
                )
        robot_delta = VR_TO_ROBOT_ROT @ vr_delta
        robot_delta = self.position_scale * robot_delta
        # Human and robot arm extension ranges are not equal.  Preserve the
        # established lateral/vertical feel while mapping operator-forward to
        # the robot's usable arm span with its own calibrated ratio.
        robot_delta[0] *= self.forward_position_scale / self.position_scale

        initial_rotation = quaternion_to_matrix(self.vr_origin_orientation)
        current_rotation = quaternion_to_matrix(sample.orientation)
        relative_vr_rotation = current_rotation @ initial_rotation.T
        relative_robot_rotation = VR_TO_ROBOT_ROT @ relative_vr_rotation @ VR_TO_ROBOT_ROT.T
        relative_robot_rotation = quaternion_to_matrix(
            scale_quaternion_rotation(
                matrix_to_quaternion(relative_robot_rotation),
                self.wrist_orientation_gain,
            )
        )
        raw_orientation = matrix_to_quaternion(
            relative_robot_rotation @ quaternion_to_matrix(self.robot_origin_orientation)
        )

        forward_limit = self.maximum_forward_displacement
        reserve_active = False
        if (
            self.wrist_reach_reserve_enabled
            and self.maximum_forward_displacement > 0.0
        ):
            orientation_angle = quaternion_angle(
                self.robot_origin_orientation, raw_orientation
            )
            forward_limit = forward_limit_with_wrist_reserve(
                self.maximum_forward_displacement,
                orientation_angle,
                self.wrist_reach_reserve_minimum_m,
                self.wrist_reach_reserve_maximum_m,
                self.wrist_reach_reserve_start_angle,
                self.wrist_reach_reserve_full_angle,
            )
            reserve_active = robot_delta[0] > forward_limit
        if (
            forward_limit > 0.0
            and robot_delta[0] > forward_limit
        ):
            robot_delta[0] = forward_limit
        raw_position = self.robot_origin_position + robot_delta
        constraint_reasons = []
        if reserve_active:
            constraint_reasons.append(
                'forward reach smoothly reserved for wrist dexterity'
            )
        elif (
            self.maximum_forward_displacement > 0.0
            and (self.forward_position_scale * (VR_TO_ROBOT_ROT @ vr_delta)[0])
            > self.maximum_forward_displacement
        ):
            constraint_reasons.append(
                'target proportionally limited at forward arm extension'
            )
        displacement = raw_position - self.robot_origin_position
        displacement_norm = float(np.linalg.norm(displacement))
        if (
            self.maximum_anchor_displacement > 0.0
            and displacement_norm > self.maximum_anchor_displacement
        ):
            # Do not drop the target at the clutch-radius boundary.  Dropping it
            # freezes the robot while the VR hand continues to move and causes a
            # catch-up jump when it re-enters the sphere.  Radial projection
            # keeps tangential movement continuous and bounded.
            raw_position = self.robot_origin_position + displacement * (
                self.maximum_anchor_displacement / displacement_norm
            )
            constraint_reasons.append(
                'target sliding along maximum clutch displacement boundary'
            )
        raw_position, position_reason = self._constrain_position(raw_position)
        if position_reason:
            constraint_reasons.append(position_reason)
        constraint_reason = '; '.join(constraint_reasons)

        alpha = float(np.clip(self.filter_alpha, 0.0, 1.0))
        filtered_position = (1.0 - alpha) * self.filtered_position + alpha * raw_position
        filtered_orientation = slerp(self.filtered_orientation, raw_orientation, alpha)

        position_delta = filtered_position - self.filtered_position
        distance = float(np.linalg.norm(position_delta))
        if distance > self.maximum_position_step > 0.0:
            filtered_position = self.filtered_position + position_delta * (
                self.maximum_position_step / distance
            )
        angle = quaternion_angle(self.filtered_orientation, filtered_orientation)
        if angle > self.maximum_orientation_step > 0.0:
            filtered_orientation = slerp(
                self.filtered_orientation,
                filtered_orientation,
                self.maximum_orientation_step / angle,
            )

        self.filtered_position = filtered_position
        self.filtered_orientation = filtered_orientation
        return {
            'position': filtered_position.copy(),
            'orientation': filtered_orientation.copy(),
        }, constraint_reason


def vendor_pose_payload(targets, current_poses):
    poses = {}
    for side in ('left', 'right'):
        pose = targets.get(side) or current_poses[side]
        poses[side] = {
            'position': [float(value) for value in pose['position']],
            'orientation': [float(value) for value in pose['orientation']],
        }
    return {
        'pos_left_in_robot': poses['left']['position'],
        'quat_left_in_robot': poses['left']['orientation'],
        'pos_right_in_robot': poses['right']['position'],
        'quat_right_in_robot': poses['right']['orientation'],
    }
