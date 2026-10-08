import numpy as np


def bounded_target_by_feedback(current, target, maximum_lead):
    """Bound a target around measured joints, matching the proven old teleop."""
    current = np.asarray(current, dtype=float)
    target = np.asarray(target, dtype=float)
    maximum_lead = np.asarray(maximum_lead, dtype=float)
    try:
        maximum_lead = np.broadcast_to(maximum_lead, current.shape)
    except ValueError as exc:
        raise ValueError('maximum lead cannot be broadcast to the joint shape') from exc
    if target.shape != current.shape:
        raise ValueError('current and target shapes must match')
    if not all(np.all(np.isfinite(values)) for values in (
            current, target, maximum_lead)):
        raise ValueError('feedback lead inputs must contain only finite values')
    if np.any(maximum_lead <= 0.0):
        raise ValueError('maximum lead must be positive')
    return np.clip(target, current - maximum_lead, current + maximum_lead)


class TrajectoryLimiter:
    """Discrete velocity/acceleration limiter with hard joint bounds.

    The next velocity is selected from the intersection of three feasible
    intervals: the configured velocity bound, the acceleration-reachable
    interval, and the velocity that can be integrated for one cycle without
    crossing a hard joint bound.  A braking envelope additionally reduces the
    requested speed early enough to approach a stationary target without an
    instantaneous terminal velocity reset.
    """

    def __init__(self, lower, upper, max_velocity, max_acceleration):
        self.lower = np.asarray(lower, dtype=float)
        self.upper = np.asarray(upper, dtype=float)
        try:
            self.max_velocity = np.broadcast_to(
                np.asarray(max_velocity, dtype=float), self.lower.shape
            ).copy()
            self.max_acceleration = np.broadcast_to(
                np.asarray(max_acceleration, dtype=float), self.lower.shape
            ).copy()
        except ValueError as exc:
            raise ValueError(
                'trajectory limit arrays must have the same shape'
            ) from exc
        if not (self.lower.shape == self.upper.shape == self.max_velocity.shape == self.max_acceleration.shape):
            raise ValueError('trajectory limit arrays must have the same shape')
        if not all(np.all(np.isfinite(values)) for values in (
                self.lower, self.upper, self.max_velocity, self.max_acceleration)):
            raise ValueError('trajectory limits must contain only finite values')
        if np.any(self.upper <= self.lower):
            raise ValueError('invalid joint bounds')
        if np.any(self.max_velocity <= 0.0) or np.any(self.max_acceleration <= 0.0):
            raise ValueError('velocity and acceleration limits must be positive')
        self.position = None
        self.velocity = np.zeros_like(self.lower)

    def reset(self, position):
        position = np.asarray(position, dtype=float)
        if position.shape != self.lower.shape:
            raise ValueError('position shape mismatch')
        if not np.all(np.isfinite(position)):
            raise ValueError('position must contain only finite values')
        self.position = np.clip(position, self.lower, self.upper)
        self.velocity = np.zeros_like(position)
        return self.position.copy()

    def reset_indices(self, indices, position):
        """Immediately hold selected joints at measured positions.

        Grip release is a control-mode transition, not a new motion target.
        Resetting only the released arm prevents its old trajectory from
        continuing while leaving the other arm's velocity state untouched.
        """
        if self.position is None:
            raise RuntimeError('trajectory limiter has not been initialized')
        indices = np.asarray(list(indices), dtype=int).reshape(-1)
        position = np.asarray(position, dtype=float).reshape(-1)
        if indices.size == 0 or indices.size != position.size:
            raise ValueError('indices and position must have the same non-zero size')
        if np.any(indices < 0) or np.any(indices >= self.position.size):
            raise ValueError('trajectory limiter index is out of range')
        if len(set(int(index) for index in indices)) != indices.size:
            raise ValueError('trajectory limiter indices must be unique')
        if not np.all(np.isfinite(position)):
            raise ValueError('position must contain only finite values')
        held = np.clip(position, self.lower[indices], self.upper[indices])
        self.position[indices] = held
        self.velocity[indices] = 0.0
        return held.copy()

    def step(self, target, dt, max_velocity=None, max_acceleration=None):
        target = np.asarray(target, dtype=float)
        if target.shape != self.lower.shape:
            raise ValueError('target shape mismatch')
        if not np.all(np.isfinite(target)):
            raise ValueError('target must contain only finite values')
        try:
            dt = float(dt)
        except (TypeError, ValueError) as exc:
            raise ValueError('dt must be a finite scalar') from exc
        if not np.isfinite(dt):
            raise ValueError('dt must be a finite scalar')
        target = np.clip(target, self.lower, self.upper)
        if self.position is None:
            return self.reset(target)
        if not np.all(np.isfinite(self.position)) or not np.all(np.isfinite(self.velocity)):
            raise RuntimeError('trajectory limiter state is not finite')

        dt = float(np.clip(dt, 1e-4, 0.1))
        velocity_limit = self.max_velocity
        acceleration_limit = self.max_acceleration
        if max_velocity is not None:
            velocity_limit = np.minimum(
                velocity_limit,
                np.broadcast_to(
                    np.asarray(max_velocity, dtype=float), target.shape
                ),
            )
        if max_acceleration is not None:
            acceleration_limit = np.minimum(
                acceleration_limit,
                np.broadcast_to(
                    np.asarray(max_acceleration, dtype=float), target.shape
                ),
            )
        if (
            not np.all(np.isfinite(velocity_limit))
            or not np.all(np.isfinite(acceleration_limit))
            or np.any(velocity_limit <= 0.0)
            or np.any(acceleration_limit <= 0.0)
        ):
            raise ValueError('trajectory limit overrides must be finite and positive')
        error = target - self.position

        # If v is the next commanded speed, this conservative envelope solves
        #
        #   v * dt + v**2 / (2*a) <= remaining_distance
        #
        # so one commanded cycle plus continuous maximum braking still fits
        # before the target.  It deliberately starts braking slightly earlier
        # than a continuous time-optimal profile, which is appropriate for a
        # sampled command stream.
        acceleration_step = acceleration_limit * dt
        distance = np.abs(error)
        braking_speed = np.sqrt(
            acceleration_step * acceleration_step
            + 2.0 * acceleration_limit * distance
        ) - acceleration_step
        desired_velocity = np.sign(error) * np.minimum(
            velocity_limit,
            np.maximum(braking_speed, 0.0),
        )

        acceleration_lower = self.velocity - acceleration_step
        acceleration_upper = self.velocity + acceleration_step
        bound_lower = (self.lower - self.position) / dt
        bound_upper = (self.upper - self.position) / dt
        feasible_lower = np.maximum.reduce((
            -velocity_limit,
            acceleration_lower,
            bound_lower,
        ))
        feasible_upper = np.minimum.reduce((
            velocity_limit,
            acceleration_upper,
            bound_upper,
        ))
        numerical_tolerance = 1e-12
        if np.any(feasible_lower > feasible_upper + numerical_tolerance):
            raise RuntimeError(
                'trajectory state cannot satisfy acceleration and hard bounds'
            )
        # Collapse a possible sub-epsilon interval inversion caused by floating
        # point rounding at an exact hard limit.
        feasible_lower = np.minimum(feasible_lower, feasible_upper)
        new_velocity = np.clip(
            desired_velocity, feasible_lower, feasible_upper
        )
        new_position = self.position + new_velocity * dt

        self.position = np.clip(new_position, self.lower, self.upper)
        self.velocity = new_velocity
        return self.position.copy()
