"""Per-hand tactile policy for the *owner* of the gripper command queue.

The Collector is an observer, not that owner. This module never publishes a
position. A hardware integration must run tick() on its own watchdog clock,
even with no new operator command, and implement the cancellation contract.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Integral
from typing import Callable, Collection, NoReturn, Sequence


OPTION = "GRIPPER_TACTILE_LIMIT_N"


def positive_limit(value) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        number = math.nan
    if isinstance(value, bool) or not math.isfinite(number) or number <= 0:
        raise ValueError(f"{OPTION} must be a finite positive force in newtons; unset it to disable")
    return number


def reject_unverified_factory_limit(value) -> NoReturn:
    """Do not arm based on an installed SDK, a SHM path, or a transport ACK."""
    limit = positive_limit(value)
    raise ValueError(f"{OPTION}={limit:g} N requested, but no verified factory gripper "
                     "cancel-and-hold integration is available. Recording was not started; "
                     "WITH_TACTILE only records observations and does not limit closing force.")


class Intent(Enum):
    CLOSE = "close"
    OPEN = "open"
    IDLE = "idle"


@dataclass(frozen=True)
class Touch:
    normal_n: Sequence[float]
    status: Sequence[int]
    stamp_s: Sequence[float]  # per-point SDK host receipt timestamps, not camera times


@dataclass(frozen=True)
class Trip:
    side: str
    wall_time: float | None
    reason: str
    limit_n: float
    peak_n: float | None
    point: int | None


@dataclass(frozen=True)
class Decision:
    # None suppresses a new position command. A latched stop is effected by
    # cancel_and_hold itself, not by suppressing a future close command.
    forward: Intent | None
    latched: bool
    hold_confirmed: bool
    trip: Trip | None
    hold_error: str | None = None
    sample_issue: str | None = None


class HandLimit:
    """One instance per hand. Only the exclusive owner may consume decisions.

    OPEN means a verified opening relative to the measured aperture, replacing
    any queued close; a smaller operator target is not necessarily an opening.
    cancel_and_hold(side) must atomically cancel pending/in-flight closing and
    hold the measured aperture, without interrupting an existing opening move.
    Only True acknowledges completion, not merely transport delivery. False
    means retry the same idempotent operation at the next watchdog tick.
    """

    def __init__(self, side: str, limit_n: float, *, max_age_s: float,
                 healthy_status: Collection[int]):
        if side not in ("left", "right") or not math.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError("invalid side or tactile watchdog interval")
        if not healthy_status or not all(isinstance(s, Integral) and not isinstance(s, bool)
                                         for s in healthy_status):
            raise ValueError("verified healthy tactile status codes required")
        self.side = side
        self._limit = positive_limit(limit_n)
        self._max_age = max_age_s
        self._healthy = frozenset(healthy_status)
        self._stamps = self._deadlines = ()
        self._last_tick = -math.inf
        self._latched = self._held = False

    def _inspect(self, touch, now, tick):
        if not math.isfinite(tick) or tick < self._last_tick or not math.isfinite(now):
            return "invalid_clock", None, None
        self._last_tick = tick
        if touch is None or not all(len(v) == 8 for v in (touch.normal_n, touch.status, touch.stamp_s)):
            return "missing_points", None, None
        stamps = tuple(touch.stamp_s)
        if not all(math.isfinite(t) and 0 <= now - t <= self._max_age for t in stamps):
            return "invalid_or_stale_timestamp", None, None
        if self._stamps and any(t < old for t, old in zip(stamps, self._stamps)):
            return "out_of_order_timestamp", None, None
        # A repeat from SDK 'latest' must not renew even one point's deadline,
        # including when the system wall clock stalls while monotonic advances.
        deadlines = tuple(self._deadlines[i] if self._stamps and t == self._stamps[i]
                          else tick + self._max_age - (now - t) for i, t in enumerate(stamps))
        self._stamps, self._deadlines = stamps, deadlines
        if tick >= min(deadlines):
            return "stale_point", None, None
        if any(not isinstance(s, Integral) or isinstance(s, bool) or s not in self._healthy
               for s in touch.status):
            return "invalid_status", None, None
        if not all(math.isfinite(n) and n >= 0 for n in touch.normal_n):
            return "invalid_normal_force", None, None
        point = max(range(8), key=touch.normal_n.__getitem__)
        peak = float(touch.normal_n[point])
        return ("force_limit" if peak >= self._limit else None), peak, point

    def tick(self, intent: Intent, touch: Touch | None, *, now_s: float, monotonic_s: float,
             cancel_and_hold: Callable[[str], bool]) -> Decision:
        try:
            reason, peak, point = self._inspect(touch, now_s, monotonic_s)
        except (AttributeError, TypeError, ValueError, OverflowError):
            reason, peak, point = "invalid_sample", None, None
        if not isinstance(intent, Intent):
            reason = "invalid_intent"
        trip = None
        if reason and not self._latched:
            trip = Trip(self.side, now_s if isinstance(now_s, (int, float)) and math.isfinite(now_s)
                        else None, reason, self._limit, peak, point)
            self._latched = True
        if self._latched:
            hold_error = None
            if not self._held:
                try:
                    self._held = cancel_and_hold(self.side) is True
                except Exception as exc:
                    hold_error = f"cancel-and-hold failed: {exc}"
                if not self._held and hold_error is None:
                    hold_error = "cancel-and-hold not acknowledged"
            if intent is Intent.OPEN:
                # Do not trap an object on a sensor/ACK fault. Do not re-arm
                # for closing until both the cancellation and recovery are known.
                if self._held and reason is None:
                    self._latched = self._held = False
                return Decision(Intent.OPEN, self._latched, False, trip, hold_error, reason)
            return Decision(None, True, self._held, trip, hold_error, reason)
        return Decision(None if intent is Intent.IDLE else intent, False, False, None)
