"""Coordinated S2 waist/leg profile for vertical body-height control."""

import math


MAX_BODY_LOWERING_M = 0.56
MAX_REVERSE_BODY_LOWERING_M = 0.53
BODY_HEIGHT_JOINT_NAMES = (
    'Joint_Ankle',
    'Joint_Knee',
    'Joint_Waist_Pitch',
)
BODY_HEIGHT_MAXIMUM_DEG = (76.0, 155.0, 81.0)


def measured_reverse_squat(measured_deg, previous=False):
    """Choose the manual branch from legs, never from reach-assist waist pitch.

    Retain the selected branch inside a 2 mm standing dead band. The caller
    latches it for a complete key press, including travel all the way to zero.
    """
    measured = tuple(float(v) for v in measured_deg)
    if len(measured) < 2 or not all(math.isfinite(v) for v in measured[:2]):
        raise ValueError('finite ankle and knee feedback is required')
    a, k = BODY_HEIGHT_MAXIMUM_DEG[:2]
    height = (measured[0]*a + measured[1]*k)/(a*a+k*k)*MAX_BODY_LOWERING_M
    return bool(previous) if abs(height) <= .002 else height < 0.


def body_height_joint_targets(lowering_m):
    """Return ankle, knee and waist-pitch targets for a lowering distance."""
    lowering = min(MAX_BODY_LOWERING_M, max(0.0, float(lowering_m)))
    scale = lowering / MAX_BODY_LOWERING_M
    return tuple(maximum * scale for maximum in BODY_HEIGHT_MAXIMUM_DEG)


def estimate_body_lowering(leg_waist_deg, *, signed=False):
    """Project a four-joint leg/waist pose onto the coordinated height curve."""
    values = tuple(float(value) for value in leg_waist_deg)
    if len(values) != 4:
        raise ValueError('leg_waist_deg must contain four joints')
    denominator = sum(value * value for value in BODY_HEIGHT_MAXIMUM_DEG)
    scale = sum(
        values[index] * maximum
        for index, maximum in enumerate(BODY_HEIGHT_MAXIMUM_DEG)
    ) / denominator
    minimum = -MAX_REVERSE_BODY_LOWERING_M / MAX_BODY_LOWERING_M if signed else 0.0
    return min(1.0, max(minimum, scale)) * MAX_BODY_LOWERING_M


def coordinated_body_height_targets(
    lowering_m,
    measured_deg,
    previous_progress,
    command_lead_deg,
    *, allow_reverse=False,
):
    """Advance ankle, knee and waist pitch with one feedback-bounded progress.

    The vendor position loops do not track the three joints at exactly the same
    rate. Sending a moving target independently lets a fast leg joint get ahead
    of waist pitch. A single normalized progress value keeps the body-height
    geometry intact and pauses all three targets when one joint is catching up.

    ``previous_progress`` also makes the command monotonic in the requested
    direction, so small encoder noise cannot make the body rock back and forth.
    """
    measured = tuple(float(value) for value in measured_deg)
    if len(measured) != len(BODY_HEIGHT_MAXIMUM_DEG):
        raise ValueError('measured_deg must contain ankle, knee and waist pitch')
    if not all(math.isfinite(value) for value in measured):
        raise ValueError('measured_deg must contain only finite values')

    minimum = -MAX_REVERSE_BODY_LOWERING_M / MAX_BODY_LOWERING_M if allow_reverse else 0.0
    desired = min(
        1.0,
        max(minimum, float(lowering_m) / MAX_BODY_LOWERING_M),
    )
    previous = min(1.0, max(minimum, float(previous_progress)))
    lead = max(0.1, float(command_lead_deg))
    if not all(math.isfinite(value) for value in (desired, previous, lead)):
        raise ValueError('body-height progress inputs must be finite')

    epsilon = 1.0e-9
    if desired > previous + epsilon:
        feedback_limit = min(
            value / maximum + lead / maximum
            for value, maximum in zip(measured, BODY_HEIGHT_MAXIMUM_DEG)
        )
        progress = max(previous, min(desired, feedback_limit))
    elif desired < previous - epsilon:
        feedback_limit = max(
            value / maximum - lead / maximum
            for value, maximum in zip(measured, BODY_HEIGHT_MAXIMUM_DEG)
        )
        progress = min(previous, max(desired, feedback_limit))
    else:
        progress = previous

    progress = min(1.0, max(minimum, progress))
    return (
        tuple(maximum * progress for maximum in BODY_HEIGHT_MAXIMUM_DEG),
        progress,
    )
