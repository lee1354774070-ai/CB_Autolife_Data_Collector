"""Pure state machines for repeatable VR clutch and quick-reset handling.

This module deliberately has no ROS dependency.  Keeping the transition logic
separate makes the two safety properties easy to test:

* an unanswered quick-reset service call expires and an old callback cannot
  mutate a later request; and
* a reset/session boundary requires a newly observed Grip release before an arm
  can be clutched again.
"""

from dataclasses import dataclass, field
import math


QUICK_BUTTON_KEYS = ('left_x', 'right_a')
ARM_SIDES = ('left', 'right')


@dataclass
class ClutchedTrigger:
    """Re-clutching alone must not drop an object held at the last target."""
    anchor: float | None = None
    armed: bool = False

    def release(self):
        self.anchor = None
        self.armed = False

    def observe(self, value):
        value = float(value)
        if not math.isfinite(value) or not 0 <= value <= 1:
            return None
        if self.anchor is None:
            self.anchor = value
            return None
        if abs(value - self.anchor) >= 0.025:
            self.armed = True
        return value if self.armed else None


def _finite_time(value, name):
    timestamp = float(value)
    if not math.isfinite(timestamp):
        raise ValueError(f'{name} must be finite')
    return timestamp


@dataclass
class QuickResetState:
    """Track the X+A gesture and one generation-guarded service request."""

    request_timeout_sec: float = 2.0
    button_keys: tuple = QUICK_BUTTON_KEYS
    buttons: dict = field(default_factory=lambda: {
        'left_x': False,
        'right_a': False,
    })
    full_input_time: float = 0.0
    chord_started: float | None = None
    chord_consumed: bool = False
    generation: int = 0
    pending_generation: int | None = None
    request_deadline: float | None = None

    def __post_init__(self):
        self.buttons = {key: False for key in self.button_keys}
        timeout = float(self.request_timeout_sec)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError('request_timeout_sec must be finite and positive')
        self.request_timeout_sec = timeout

    @property
    def pending(self):
        return self.pending_generation is not None

    def update_button_event(self, key, pressed):
        """Apply a reliable one-button edge without claiming a full snapshot."""
        if key not in self.button_keys:
            raise ValueError(f'unsupported quick-reset button: {key}')
        self.buttons[key] = bool(pressed)

    def update_full_snapshot(self, left_x, right_a, now, both_tracked=True):
        """Apply one authoritative two-controller button snapshot.

        A consumed X+A chord is re-armed as soon as a *fresh* snapshot shows
        that either button has been released.  Requiring both buttons to be
        false made a missed release edge permanently suppress later resets.
        """
        timestamp = _finite_time(now, 'now')
        if not bool(both_tracked):
            self.full_input_time = 0.0
            self.chord_started = None
            return
        self.buttons[self.button_keys[0]] = bool(left_x)
        self.buttons[self.button_keys[1]] = bool(right_a)
        self.full_input_time = timestamp
        if not self.both_pressed:
            self.chord_started = None
            self.chord_consumed = False

    @property
    def both_pressed(self):
        return all(self.buttons[key] for key in self.button_keys)

    def input_is_fresh(self, now, maximum_age):
        timestamp = _finite_time(now, 'now')
        limit = float(maximum_age)
        return (
            math.isfinite(limit)
            and limit > 0.0
            and self.full_input_time > 0.0
            and 0.0 <= timestamp - self.full_input_time <= limit
        )

    def gesture_state(self, now, maximum_age, hold_seconds):
        """Return ``idle``, ``holding``, ``consumed``, ``pending`` or ``ready``."""
        timestamp = _finite_time(now, 'now')
        if not self.input_is_fresh(timestamp, maximum_age):
            self.chord_started = None
            return 'idle'
        if not self.both_pressed:
            self.chord_started = None
            self.chord_consumed = False
            return 'idle'
        if self.chord_started is None:
            self.chord_started = timestamp
        if self.pending:
            return 'pending'
        if self.chord_consumed:
            return 'consumed'
        hold = max(0.0, float(hold_seconds))
        if timestamp - self.chord_started < hold:
            return 'holding'
        return 'ready'

    def consume_chord(self):
        """Require a fresh combination-release snapshot before another try."""
        self.chord_consumed = True

    def begin_request(self, now):
        """Start a request and return the callback generation token."""
        if self.pending:
            raise RuntimeError('a quick-reset request is already pending')
        timestamp = _finite_time(now, 'now')
        self.generation += 1
        self.pending_generation = self.generation
        self.request_deadline = timestamp + self.request_timeout_sec
        self.chord_consumed = True
        return self.pending_generation

    def complete_request(self, generation):
        """Accept only the callback belonging to the current pending request."""
        if generation != self.pending_generation:
            return False
        self.pending_generation = None
        self.request_deadline = None
        return True

    def expire_request(self, now):
        """Invalidate and return a timed-out request generation, if any."""
        timestamp = _finite_time(now, 'now')
        if (
            not self.pending
            or self.request_deadline is None
            or timestamp < self.request_deadline
        ):
            return None
        expired = self.pending_generation
        self.generation += 1
        self.pending_generation = None
        self.request_deadline = None
        return expired

    def begin_boundary(self):
        """Invalidate old async work and require X+A release after a boundary."""
        invalidated = self.pending_generation
        self.generation += 1
        self.pending_generation = None
        self.request_deadline = None
        self.full_input_time = 0.0
        self.chord_started = None
        self.chord_consumed = True
        self.buttons = {key: False for key in self.button_keys}
        return invalidated


@dataclass
class GripRearmState:
    """Require a continuous fresh Grip release after each session boundary."""

    waiting: dict = field(default_factory=lambda: {
        'left': True,
        'right': True,
    })
    false_since: dict = field(default_factory=lambda: {
        'left': None,
        'right': None,
    })
    generation: int = 0

    def begin_boundary(self):
        self.generation += 1
        for side in ARM_SIDES:
            self.waiting[side] = True
            self.false_since[side] = None
        return self.generation

    def set_required(self, side, required):
        if side not in ARM_SIDES:
            raise ValueError(f'unsupported arm side: {side}')
        self.waiting[side] = bool(required)
        self.false_since[side] = None

    def observe(self, side, active, now, release_debounce_sec=0.0):
        """Observe a fresh Grip sample; return true on the re-arm transition.

        This method is intended to run directly in the VR input callback.  It
        therefore continues to process release samples even while the control
        timer is temporarily suppressing motion for an X+A gesture.
        """
        if side not in ARM_SIDES:
            raise ValueError(f'unsupported arm side: {side}')
        timestamp = _finite_time(now, 'now')
        if not self.waiting[side]:
            return False
        if bool(active):
            self.false_since[side] = None
            return False
        started = self.false_since[side]
        if started is None:
            self.false_since[side] = timestamp
            started = timestamp
        delay = max(0.0, float(release_debounce_sec))
        if timestamp - started < delay:
            return False
        self.waiting[side] = False
        self.false_since[side] = None
        return True
