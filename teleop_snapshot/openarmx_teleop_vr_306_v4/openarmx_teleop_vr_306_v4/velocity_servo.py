"""Fail-closed joint velocity servo for low-latency VR teleoperation."""

import numpy as np


def lookahead_reference(position, velocity, target, lookahead_sec):
    """Project a smooth position trajectory without overshooting its target."""
    position = np.asarray(position, dtype=float)
    velocity = np.asarray(velocity, dtype=float)
    target = np.asarray(target, dtype=float)
    if position.shape != velocity.shape or position.shape != target.shape:
        raise ValueError('lookahead reference arrays must have matching shapes')
    if not all(np.all(np.isfinite(value)) for value in (
            position, velocity, target)):
        raise ValueError('lookahead reference arrays must be finite')
    lookahead_sec = float(lookahead_sec)
    if not np.isfinite(lookahead_sec) or lookahead_sec < 0.0:
        raise ValueError('lookahead duration must be finite and non-negative')
    projected = position + velocity * lookahead_sec
    return np.minimum(
        np.maximum(projected, np.minimum(position, target)),
        np.maximum(position, target),
    )


class VelocityServo:
    """Convert joint position goals into bounded velocity commands.

    All public values use degrees and seconds because the vendor ROS velocity
    interface is documented in deg/s.  Only explicitly active indices can
    move; every other entry is forced to zero on every cycle.
    """

    def __init__(
        self,
        lower,
        upper,
        maximum_velocity,
        maximum_acceleration,
        position_gain,
        feedforward_gain,
        limit_gain,
        maximum_jerk=4000.0,
        goal_rate_filter_tau=0.04,
    ):
        self.lower = np.asarray(lower, dtype=float)
        self.upper = np.asarray(upper, dtype=float)
        self.maximum_velocity = np.broadcast_to(
            np.asarray(maximum_velocity, dtype=float), self.lower.shape
        ).copy()
        self.maximum_acceleration = np.broadcast_to(
            np.asarray(maximum_acceleration, dtype=float), self.lower.shape
        ).copy()
        self.maximum_jerk = np.broadcast_to(
            np.asarray(maximum_jerk, dtype=float), self.lower.shape
        ).copy()
        if not (
            self.lower.shape
            == self.upper.shape
            == self.maximum_velocity.shape
            == self.maximum_acceleration.shape
            == self.maximum_jerk.shape
        ):
            raise ValueError('velocity servo arrays must have matching shapes')
        if not all(np.all(np.isfinite(value)) for value in (
            self.lower, self.upper, self.maximum_velocity,
            self.maximum_acceleration, self.maximum_jerk,
        )):
            raise ValueError('velocity servo limits must be finite')
        if np.any(self.upper <= self.lower):
            raise ValueError('velocity servo joint bounds are invalid')
        if np.any(self.maximum_velocity <= 0.0):
            raise ValueError('maximum velocity must be positive')
        if np.any(self.maximum_acceleration <= 0.0):
            raise ValueError('maximum acceleration must be positive')
        if np.any(self.maximum_jerk <= 0.0):
            raise ValueError('maximum jerk must be positive')
        self.position_gain = float(position_gain)
        self.feedforward_gain = float(feedforward_gain)
        self.limit_gain = float(limit_gain)
        self.goal_rate_filter_tau = float(goal_rate_filter_tau)
        if not all(np.isfinite(value) and value >= 0.0 for value in (
            self.position_gain, self.feedforward_gain, self.limit_gain,
        )):
            raise ValueError('velocity servo gains must be finite and non-negative')
        if not np.isfinite(self.goal_rate_filter_tau) or self.goal_rate_filter_tau < 0.0:
            raise ValueError('goal rate filter time constant must be finite and non-negative')
        self.velocity = np.zeros_like(self.lower)
        self.acceleration = np.zeros_like(self.lower)
        self.filtered_goal_rate = np.zeros_like(self.lower)
        self.previous_goal = None

    def reset(self, goal=None):
        self.velocity.fill(0.0)
        self.acceleration.fill(0.0)
        self.filtered_goal_rate.fill(0.0)
        self.previous_goal = (
            None if goal is None else np.asarray(goal, dtype=float).copy()
        )
        if self.previous_goal is not None and self.previous_goal.shape != self.lower.shape:
            raise ValueError('velocity servo reset goal shape mismatch')
        return self.velocity.copy()

    def stop(self):
        """Return an immediate all-zero command for watchdog/release paths."""
        self.velocity.fill(0.0)
        self.acceleration.fill(0.0)
        self.filtered_goal_rate.fill(0.0)
        return self.velocity.copy()

    def stop_indices(self, indices):
        indices = np.asarray(list(indices), dtype=int).reshape(-1)
        if indices.size:
            self.velocity[indices] = 0.0
            self.acceleration[indices] = 0.0
            self.filtered_goal_rate[indices] = 0.0
        return self.velocity.copy()

    def rebase_indices(self, indices, goal):
        """Remove stale feed-forward history when a hand is re-clutched."""
        indices = np.asarray(list(indices), dtype=int).reshape(-1)
        goal = np.asarray(goal, dtype=float)
        if goal.shape != self.lower.shape:
            raise ValueError('velocity servo rebase goal shape mismatch')
        if np.any(indices < 0) or np.any(indices >= goal.size):
            raise ValueError('velocity servo rebase index is out of range')
        if self.previous_goal is None:
            self.previous_goal = goal.copy()
        elif indices.size:
            self.previous_goal[indices] = goal[indices]
        if indices.size:
            self.velocity[indices] = 0.0
            self.acceleration[indices] = 0.0
            self.filtered_goal_rate[indices] = 0.0
        return self.velocity.copy()

    def _bounded_override(self, configured, override, name):
        if override is None:
            return configured
        try:
            value = np.broadcast_to(
                np.asarray(override, dtype=float), configured.shape
            ).copy()
        except ValueError as exc:
            raise ValueError(f'{name} override shape mismatch') from exc
        if not np.all(np.isfinite(value)) or np.any(value <= 0.0):
            raise ValueError(f'{name} override must be finite and positive')
        return np.minimum(configured, value)

    def step(
        self,
        current,
        goal,
        dt,
        active_indices,
        maximum_velocity=None,
        maximum_acceleration=None,
        maximum_jerk=None,
    ):
        current = np.asarray(current, dtype=float)
        goal = np.asarray(goal, dtype=float)
        if current.shape != self.lower.shape or goal.shape != self.lower.shape:
            raise ValueError('velocity servo state shape mismatch')
        if not np.all(np.isfinite(current)) or not np.all(np.isfinite(goal)):
            raise ValueError('velocity servo state must be finite')
        dt = float(dt)
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError('velocity servo dt must be positive and finite')
        dt = float(np.clip(dt, 1e-4, 0.1))
        active = np.asarray(list(active_indices), dtype=int).reshape(-1)
        if np.any(active < 0) or np.any(active >= current.size):
            raise ValueError('velocity servo active index is out of range')
        active_mask = np.zeros(current.shape, dtype=bool)
        active_mask[active] = True
        velocity_limit = self._bounded_override(
            self.maximum_velocity, maximum_velocity, 'velocity'
        )
        acceleration_limit = self._bounded_override(
            self.maximum_acceleration, maximum_acceleration, 'acceleration'
        )
        jerk_limit = self._bounded_override(
            self.maximum_jerk, maximum_jerk, 'jerk'
        )

        if self.previous_goal is None:
            goal_rate = np.zeros_like(goal)
        else:
            goal_rate = (goal - self.previous_goal) / dt
        self.previous_goal = goal.copy()
        goal_rate = np.clip(
            goal_rate, -velocity_limit, velocity_limit
        )
        alpha = (
            1.0 if self.goal_rate_filter_tau <= 0.0
            else dt / (self.goal_rate_filter_tau + dt)
        )
        self.filtered_goal_rate += alpha * (
            goal_rate - self.filtered_goal_rate
        )
        self.filtered_goal_rate[~active_mask] = 0.0
        requested = (
            self.position_gain * (goal - current)
            + self.feedforward_gain * self.filtered_goal_rate
        )
        requested = np.clip(
            requested, -velocity_limit, velocity_limit
        )

        # Soft-limit velocity envelope.  This reaches zero at the configured
        # hard bound and also respects the acceleration-limited stopping speed.
        lower_distance = np.maximum(current - self.lower, 0.0)
        upper_distance = np.maximum(self.upper - current, 0.0)
        lower_speed = np.minimum(
            self.limit_gain * lower_distance,
            np.sqrt(2.0 * acceleration_limit * lower_distance),
        )
        upper_speed = np.minimum(
            self.limit_gain * upper_distance,
            np.sqrt(2.0 * acceleration_limit * upper_distance),
        )
        requested = np.clip(requested, -lower_speed, upper_speed)
        requested[~active_mask] = 0.0

        desired_acceleration = np.clip(
            (requested - self.velocity) / dt,
            -acceleration_limit,
            acceleration_limit,
        )
        acceleration_step = jerk_limit * dt
        next_acceleration = np.clip(
            desired_acceleration,
            self.acceleration - acceleration_step,
            self.acceleration + acceleration_step,
        )
        next_acceleration = np.clip(
            next_acceleration, -acceleration_limit, acceleration_limit
        )
        command = self.velocity + next_acceleration * dt
        command = np.minimum(
            np.maximum(command, np.minimum(self.velocity, requested)),
            np.maximum(self.velocity, requested),
        )
        next_acceleration = (command - self.velocity) / dt
        # Release/watchdog paths are not deceleration trajectories: inactive
        # joints must receive zero in this very packet.
        command[~active_mask] = 0.0
        next_acceleration[~active_mask] = 0.0
        self.acceleration = next_acceleration
        self.velocity = command
        return command.copy()
