"""Adaptive position feed-forward for smooth low-latency teleoperation."""

import numpy as np


def _smoothstep(value, lower, upper):
    if upper <= lower:
        raise ValueError('adaptive error upper bound must exceed lower bound')
    ratio = np.clip((value - lower) / (upper - lower), 0.0, 1.0)
    return ratio * ratio * (3.0 - 2.0 * ratio)


class DynamicCommandLead:
    """Joint-wise measured-pose lead guard for responsive position control.

    The guard expands only while there is sustained motion demand and the
    previous command is still tracking acceptably.  A growing following error
    shrinks the permitted lead immediately; expansion is filtered so a noisy
    velocity estimate cannot produce a command jump.
    """

    def __init__(self, size, active_indices):
        self.size = int(size)
        self.active = np.zeros(self.size, dtype=bool)
        self.active[np.asarray(list(active_indices), dtype=int)] = True
        self.lead = np.zeros(self.size, dtype=float)

    def reset(self, nominal=0.0):
        nominal = float(nominal)
        if not np.isfinite(nominal) or nominal < 0.0:
            raise ValueError('dynamic lead reset value is invalid')
        self.lead.fill(nominal)
        self.lead[~self.active] = 0.0
        return self.lead.copy()

    def update(
        self,
        measured,
        desired,
        measured_velocity,
        previous_command,
        dt,
        *,
        minimum,
        nominal,
        maximum,
        speed_low,
        speed_high,
        target_error_low,
        target_error_high,
        following_error_soft,
        following_error_hard,
        expansion_filter_tau,
    ):
        arrays = [
            np.asarray(value, dtype=float).reshape(-1)
            for value in (measured, desired, measured_velocity, previous_command)
        ]
        if any(value.size != self.size for value in arrays):
            raise ValueError('dynamic lead array size mismatch')
        if not all(np.all(np.isfinite(value)) for value in arrays):
            raise ValueError('dynamic lead inputs must be finite')
        measured, desired, measured_velocity, previous_command = arrays
        values = [
            dt, minimum, nominal, maximum, speed_low, speed_high,
            target_error_low, target_error_high, following_error_soft,
            following_error_hard, expansion_filter_tau,
        ]
        values = [float(value) for value in values]
        if not all(np.isfinite(value) for value in values):
            raise ValueError('dynamic lead parameters must be finite')
        (
            dt, minimum, nominal, maximum, speed_low, speed_high,
            target_error_low, target_error_high, following_error_soft,
            following_error_hard, expansion_filter_tau,
        ) = values
        if dt <= 0.0 or minimum <= 0.0:
            raise ValueError('dynamic lead timing/minimum is invalid')
        if not minimum <= nominal <= maximum:
            raise ValueError('dynamic lead bounds are invalid')
        if speed_low < 0.0 or speed_high <= speed_low:
            raise ValueError('dynamic lead speed thresholds are invalid')
        if target_error_low < 0.0 or target_error_high <= target_error_low:
            raise ValueError('dynamic lead target thresholds are invalid')
        if following_error_soft < 0.0 or following_error_hard <= following_error_soft:
            raise ValueError('dynamic lead tracking thresholds are invalid')
        if expansion_filter_tau < 0.0:
            raise ValueError('dynamic lead filter time constant is invalid')

        speed_demand = _smoothstep(
            np.abs(measured_velocity), speed_low, speed_high
        )
        position_demand = _smoothstep(
            np.abs(desired - measured), target_error_low, target_error_high
        )
        demand = np.maximum(speed_demand, position_demand)
        following_error = np.abs(previous_command - measured)
        tracking_health = 1.0 - _smoothstep(
            following_error, following_error_soft, following_error_hard
        )
        # Healthy tracking starts at the nominal allowance and may expand
        # with real motion demand.  Once the actuator falls behind, contract
        # all the way toward the minimum instead of retaining a stale nominal
        # window that would make the eventual catch-up abrupt.
        requested = minimum + tracking_health * (
            (nominal - minimum) + (maximum - nominal) * demand
        )
        requested = np.clip(requested, minimum, maximum)

        alpha = 1.0 if expansion_filter_tau == 0.0 else 1.0 - np.exp(
            -min(dt, 0.1) / expansion_filter_tau
        )
        expanded = self.lead + alpha * (requested - self.lead)
        # Expansion is smooth; a deteriorating tracking margin contracts in the
        # current cycle so the hard 22-degree watchdog is never approached by a
        # stale large allowance.
        self.lead = np.where(requested < self.lead, requested, expanded)
        self.lead = np.clip(self.lead, minimum, maximum)
        self.lead[~self.active] = 0.0
        return self.lead.copy(), following_error.copy()


class AdaptivePositionFeedforward:
    """Estimate actuator delay and tune lookahead/acceleration per joint.

    The controller still publishes position targets.  Measured command lag is
    divided by planned joint speed to estimate the useful lookahead duration.
    Acceleration rises smoothly only on joints that are actually falling
    behind; inactive body joints retain the nominal acceleration.
    """

    def __init__(self, size, active_indices):
        self.size = int(size)
        self.active = np.zeros(self.size, dtype=bool)
        self.active[np.asarray(list(active_indices), dtype=int)] = True
        self.lookahead = np.zeros(self.size, dtype=float)
        self.following_error = np.zeros(self.size, dtype=float)
        self.velocity_limit = np.zeros(self.size, dtype=float)
        self.acceleration = np.zeros(self.size, dtype=float)

    @staticmethod
    def _vector(value, size, name):
        try:
            result = np.broadcast_to(np.asarray(value, dtype=float), (size,)).copy()
        except ValueError as exc:
            raise ValueError(f'{name} cannot be broadcast to controller size') from exc
        if not np.all(np.isfinite(result)):
            raise ValueError(f'{name} must contain finite values')
        return result

    def reset(self):
        self.lookahead.fill(0.0)
        self.following_error.fill(0.0)
        self.velocity_limit.fill(0.0)
        self.acceleration.fill(0.0)

    def update(
        self,
        measured,
        previous_command,
        planned_velocity,
        goal,
        dt,
        *,
        minimum_lookahead,
        maximum_lookahead,
        filter_tau,
        velocity_floor,
        minimum_velocity,
        maximum_velocity,
        nominal_acceleration,
        maximum_acceleration,
        error_low,
        error_high,
    ):
        measured = self._vector(measured, self.size, 'measured')
        previous_command = self._vector(
            previous_command, self.size, 'previous_command'
        )
        planned_velocity = self._vector(
            planned_velocity, self.size, 'planned_velocity'
        )
        goal = self._vector(goal, self.size, 'goal')
        dt = float(dt)
        minimum_lookahead = float(minimum_lookahead)
        maximum_lookahead = float(maximum_lookahead)
        filter_tau = float(filter_tau)
        velocity_floor = float(velocity_floor)
        minimum_velocity = float(minimum_velocity)
        maximum_velocity = float(maximum_velocity)
        nominal_acceleration = float(nominal_acceleration)
        maximum_acceleration = float(maximum_acceleration)
        error_low = float(error_low)
        error_high = float(error_high)
        scalars = (
            dt, minimum_lookahead, maximum_lookahead, filter_tau,
            velocity_floor, minimum_velocity, maximum_velocity,
            nominal_acceleration, maximum_acceleration,
            error_low, error_high,
        )
        if not all(np.isfinite(value) for value in scalars):
            raise ValueError('adaptive position parameters must be finite')
        if dt <= 0.0 or minimum_lookahead < 0.0:
            raise ValueError('adaptive timing parameters are invalid')
        if maximum_lookahead < minimum_lookahead or filter_tau < 0.0:
            raise ValueError('adaptive lookahead bounds are invalid')
        if velocity_floor <= 0.0 or minimum_velocity <= 0.0:
            raise ValueError('adaptive velocity/acceleration must be positive')
        if maximum_velocity < minimum_velocity or nominal_acceleration <= 0.0:
            raise ValueError('adaptive velocity bounds are invalid')
        if maximum_acceleration < nominal_acceleration:
            raise ValueError('adaptive maximum acceleration is below nominal')

        signed_lag = previous_command - measured
        self.following_error = np.abs(signed_lag)
        speed = np.abs(planned_velocity)
        moving_toward_command = signed_lag * planned_velocity > 0.0
        valid_delay = self.active & moving_toward_command & (speed >= velocity_floor)

        requested = np.zeros(self.size, dtype=float)
        requested[valid_delay] = np.clip(
            self.following_error[valid_delay] / speed[valid_delay],
            minimum_lookahead,
            maximum_lookahead,
        )
        # A stopped or reversing joint must not retain stale prediction.
        alpha = 1.0 if filter_tau == 0.0 else 1.0 - np.exp(
            -min(max(dt, 1.0e-4), 0.1) / filter_tau
        )
        self.lookahead += alpha * (requested - self.lookahead)
        self.lookahead[~self.active] = 0.0

        tracking_ratio = _smoothstep(self.following_error, error_low, error_high)
        goal_ratio = _smoothstep(np.abs(goal - measured), error_low, error_high)
        velocity_demand = np.maximum(tracking_ratio, goal_ratio)
        velocity_limit = minimum_velocity + (
            maximum_velocity - minimum_velocity
        ) * velocity_demand
        velocity_limit[~self.active] = maximum_velocity
        self.velocity_limit = velocity_limit
        # A joint receives extra acceleration only while there is both a real
        # goal and measured following lag.  This avoids globally stiffening
        # stationary or lightly loaded joints.
        demand = np.sqrt(tracking_ratio * goal_ratio)
        acceleration = nominal_acceleration + (
            maximum_acceleration - nominal_acceleration
        ) * demand
        acceleration[~self.active] = nominal_acceleration
        self.acceleration = acceleration
        return (
            self.lookahead.copy(),
            velocity_limit.copy(),
            acceleration.copy(),
            self.following_error.copy(),
        )


class JointTargetVelocityEstimator:
    """Latest-only filtered derivative of accepted IK joint targets."""

    def __init__(self, size, active_indices):
        self.size = int(size)
        self.active = np.zeros(self.size, dtype=bool)
        self.active[np.asarray(list(active_indices), dtype=int)] = True
        self.previous_target = None
        self.previous_time = None
        self.velocity = np.zeros(self.size, dtype=float)

    def reset(self, target=None, timestamp=None):
        self.velocity.fill(0.0)
        self.previous_target = None if target is None else np.asarray(
            target, dtype=float
        ).reshape(self.size).copy()
        self.previous_time = None if timestamp is None else float(timestamp)
        return self.velocity.copy()

    def reset_indices(self, indices, target):
        indices = np.asarray(list(indices), dtype=int).reshape(-1)
        target = np.asarray(target, dtype=float).reshape(-1)
        if indices.size != target.size:
            raise ValueError('velocity estimator indices and target size differ')
        if self.previous_target is not None and indices.size:
            self.previous_target[indices] = target
        self.velocity[indices] = 0.0

    def update(self, target, timestamp, *, filter_tau, maximum_velocity):
        target = np.asarray(target, dtype=float).reshape(-1)
        timestamp = float(timestamp)
        filter_tau = float(filter_tau)
        maximum_velocity = float(maximum_velocity)
        if target.size != self.size or not np.all(np.isfinite(target)):
            raise ValueError('IK target velocity input is invalid')
        if not all(np.isfinite(value) for value in (
                timestamp, filter_tau, maximum_velocity)):
            raise ValueError('IK target velocity parameters must be finite')
        if filter_tau < 0.0 or maximum_velocity <= 0.0:
            raise ValueError('IK target velocity parameters are invalid')
        if self.previous_target is None or self.previous_time is None:
            return self.reset(target, timestamp)
        dt = timestamp - self.previous_time
        if dt <= 1.0e-4 or dt > 0.25:
            return self.reset(target, timestamp)
        raw = np.clip(
            (target - self.previous_target) / dt,
            -maximum_velocity,
            maximum_velocity,
        )
        raw[~self.active] = 0.0
        alpha = 1.0 if filter_tau == 0.0 else 1.0 - np.exp(-dt / filter_tau)
        self.velocity += alpha * (raw - self.velocity)
        self.velocity[~self.active] = 0.0
        self.previous_target = target.copy()
        self.previous_time = timestamp
        return self.velocity.copy()


def predict_latest_target(target, velocity, lookahead, lower, upper):
    """Predict beyond the newest IK sample while preserving hard bounds."""
    target = np.asarray(target, dtype=float)
    velocity = np.asarray(velocity, dtype=float)
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)
    try:
        lookahead = np.broadcast_to(np.asarray(lookahead, dtype=float), target.shape)
    except ValueError as exc:
        raise ValueError('prediction lookahead shape mismatch') from exc
    if not (target.shape == velocity.shape == lower.shape == upper.shape):
        raise ValueError('prediction arrays must have matching shapes')
    if not all(np.all(np.isfinite(value)) for value in (
            target, velocity, lookahead, lower, upper)):
        raise ValueError('prediction inputs must be finite')
    if np.any(lookahead < 0.0) or np.any(upper <= lower):
        raise ValueError('prediction bounds are invalid')
    return np.clip(target + velocity * lookahead, lower, upper)


def project_position(position, velocity, target, lookahead):
    """Project each joint without overshooting the latest position target."""
    position = np.asarray(position, dtype=float)
    velocity = np.asarray(velocity, dtype=float)
    target = np.asarray(target, dtype=float)
    try:
        lookahead = np.broadcast_to(
            np.asarray(lookahead, dtype=float), position.shape
        )
    except ValueError as exc:
        raise ValueError('lookahead cannot be broadcast to position shape') from exc
    if not (position.shape == velocity.shape == target.shape):
        raise ValueError('position projection arrays must have matching shapes')
    if not all(np.all(np.isfinite(value)) for value in (
            position, velocity, target, lookahead)):
        raise ValueError('position projection inputs must be finite')
    if np.any(lookahead < 0.0):
        raise ValueError('lookahead must be non-negative')
    projected = position + velocity * lookahead
    return np.minimum(
        np.maximum(projected, np.minimum(position, target)),
        np.maximum(position, target),
    )
