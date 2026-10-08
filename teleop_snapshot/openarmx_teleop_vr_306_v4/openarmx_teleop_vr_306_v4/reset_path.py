"""Continuous X+A reset path from measured joints to the system pose.

The path deliberately contains no intermediate shoulder detour.  It is one
quintic sweep whose complete sampled path is validated by the controller
before any command is committed.
"""

import numpy as np


def smoothstep5(progress):
    """Quintic zero-velocity/zero-acceleration interpolation in ``[0, 1]``."""
    progress = float(np.clip(progress, 0.0, 1.0))
    return progress ** 3 * (10.0 + progress * (-15.0 + 6.0 * progress))


def continuous_direct_reset(start, goal, progress):
    """Return one sample of the direct, zero-end-velocity reset sweep."""
    start = np.asarray(start, dtype=float).reshape(-1)
    goal = np.asarray(goal, dtype=float).reshape(-1)
    if start.shape != goal.shape or start.size != 21:
        raise ValueError('reset start and goal must both contain 21 joints')
    if not np.all(np.isfinite(start)) or not np.all(np.isfinite(goal)):
        raise ValueError('reset path contains NaN or Inf')
    blend = smoothstep5(progress)
    return start + (goal - start) * blend


def conservative_reset_duration(
    start,
    goal,
    arm_velocity_deg_sec,
    neck_velocity_deg_sec,
    minimum_duration_sec=1.0,
):
    """Choose a duration that keeps the moving reference below speed limits.

    The quintic blend peaks at 1.875 times the average rate.  Runtime
    trajectory limiting remains the final acceleration/velocity guard.
    """
    start = np.asarray(start, dtype=float).reshape(-1)
    goal = np.asarray(goal, dtype=float).reshape(-1)
    if start.shape != goal.shape or start.size != 21:
        raise ValueError('reset start and goal must both contain 21 joints')
    arm_velocity = float(arm_velocity_deg_sec)
    neck_velocity = float(neck_velocity_deg_sec)
    minimum = float(minimum_duration_sec)
    if not all(np.isfinite(value) and value > 0.0 for value in (
            arm_velocity, neck_velocity, minimum)):
        raise ValueError('reset duration limits must be finite and positive')

    arm_delta = np.abs(goal[:18] - start[:18])
    arm_path_rate = 1.875 * arm_delta
    neck_path_rate = 1.875 * np.abs(goal[18:21] - start[18:21])
    return max(
        minimum,
        float(np.max(arm_path_rate)) / arm_velocity,
        float(np.max(neck_path_rate)) / neck_velocity,
    )


def feedback_bounded_reset_progress(start, goal, measured, previous, proposed, lead):
    """Limit the shared waist/leg path phase by its slowest measured joint.

    All three height joints keep one path parameter. Do not independently
    accelerate the ankle/waist past a slow knee, or advance time during a stall.
    Arm-only reset retains its original clock.
    """
    start, goal, measured = [np.asarray(v, dtype=float) for v in (start, goal, measured)]
    if any(v.shape != (21,) or not np.all(np.isfinite(v)) for v in (start, goal, measured)):
        raise ValueError('reset feedback must contain 21 finite joints')
    if not all(np.isfinite(v) for v in (previous, proposed, lead)) or lead <= 0:
        raise ValueError('reset progress and lead must be finite; lead must be positive')
    previous = float(np.clip(previous, 0, 1))
    proposed = float(np.clip(proposed, previous, 1))
    delta = goal[:3] - start[:3]
    moving = np.abs(delta) > 1.e-6
    if not np.any(moving):
        return proposed
    allowed_blend = np.min(((measured[:3] - start[:3]) / np.where(moving, delta, 1)
                            + lead / np.maximum(np.abs(delta), 1.e-6))[moving])
    if smoothstep5(proposed) <= allowed_blend:
        return proposed
    low, high = previous, proposed
    for _ in range(24):
        mid = (low + high) * .5
        if smoothstep5(mid) <= allowed_blend:
            low = mid
        else:
            high = mid
    return low
