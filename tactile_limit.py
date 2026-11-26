"""Per-hand tactile policy for the *owner* of the gripper command queue.

The Collector is an observer, not that owner. This module never publishes a
position. A hardware integration must run tick() on its own watchdog clock,
even with no new operator command, and implement the cancellation contract.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math
from numbers import Integral
from threading import Lock
import time
from typing import Callable, Collection, NoReturn, Sequence


OPTION = "GRIPPER_TACTILE_LIMIT_N"
EVENT_OPTION = "TACTILE_EVENT_THRESHOLD_N"


def positive_limit(value, name=OPTION) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        number = math.nan
    if isinstance(value, bool) or not math.isfinite(number) or number <= 0:
        raise ValueError(f"{name} must be a finite positive force in newtons; unset it to disable")
    return number


def event_threshold(value, with_tactile, mode):
    if value is None:
        return None
    if not with_tactile or mode == "dagger":
        raise ValueError(f"{EVENT_OPTION} requires WITH_TACTILE=1 in keyboard, vr or subtask mode")
    return positive_limit(value, EVENT_OPTION)


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


class _TouchThreshold:
    """Shared validation of point health, freshness and monotonicity for one hand."""

    def __init__(self, limit_n, max_age_s, healthy_status):
        if not math.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError("invalid tactile watchdog interval")
        if not healthy_status or not all(isinstance(s, Integral) and not isinstance(s, bool)
                                         for s in healthy_status):
            raise ValueError("verified healthy tactile status codes required")
        self.limit = positive_limit(limit_n)
        self._max_age = max_age_s
        self._healthy = frozenset(healthy_status)
        self._stamps = self._deadlines = ()
        self._last_tick = -math.inf

    def inspect(self, touch, now, tick):
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
        return ("force_limit" if peak >= self.limit else None), peak, point


class TactileEvents:
    """Passively associate observed crossings with the first causal, accepted image.

    The SDK reader may see a crossing between two dataset frames. Only small
    event records cross the thread boundary, never a sensor buffer or a file.
    An episode index can be reused after discard; start time identifies the attempt.
    """

    capacity = 128

    def __init__(self, limit_n, max_age_s, max_delta_s, healthy_status):
        self.config = (limit_n, max_age_s, healthy_status)
        self.max_delta = max_delta_s
        self.lock = Lock()
        self.pending = deque()
        self.attempt = None
        self.dropped = 0

    def begin(self, index, started):
        with self.lock:
            if self.attempt is not None:
                raise RuntimeError("previous tactile event window is still open")
            self.attempt = dict(episode_index=index, episode_start_sec=started)
            self.sensors = {s: _TouchThreshold(*self.config) for s in ("left", "right")}
            self.above = set()

    def observe(self, side, touch):
        with self.lock:
            if self.attempt is None:
                return
            # Timestamp the actual check under the same lock used by frame/end.
            now = time.time()
            sensor = self.sensors[side]
            reason, peak, point = sensor.inspect(touch, now, time.monotonic())
            if reason is None:
                self.above.discard(side)
            elif reason == "force_limit" and side not in self.above:
                stamp = float(touch.stamp_s[point])
                if stamp < self.attempt["episode_start_sec"]:
                    return
                self.above.add(side)
                if len(self.pending) == self.capacity:
                    self.dropped += 1
                    return
                self.pending.append(dict(event="tactile_threshold_observed", side=side,
                    hand_point_index=point, threshold_n=sensor.limit, peak_n=peak,
                    source_stamp_sec=stamp, detected_stamp_sec=now, **self.attempt))

    def drain(self, *, frame_index=None, anchor=None, end=None):
        """Emit unmatched records on expiry/end, never silently relabel another attempt."""
        with self.lock:
            if self.attempt is None:
                return ()
            ready = []
            if self.dropped:
                ready.append(dict(event="tactile_event_overflow", unmatched_reason="queue_full",
                                  frame_index=None, count=self.dropped, **self.attempt))
                self.dropped = 0
            for _ in range(len(self.pending)):
                event = self.pending.popleft()
                if end is not None:
                    ready.append(dict(event, frame_index=None, unmatched_reason=end))
                    continue
                delay = anchor - event["detected_stamp_sec"]
                if delay < 0:
                    self.pending.append(event)
                    continue
                source_delta = anchor - event["source_stamp_sec"]
                reason = ("detection_outside_sync_window" if delay > self.max_delta else
                          "source_outside_sync_window" if source_delta > self.max_delta else None)
                ready.append(dict(event, frame_index=frame_index if reason is None else None,
                    candidate_frame_index=frame_index, anchor_stamp_sec=anchor,
                    anchor_minus_detection_ms=delay*1000, anchor_minus_source_ms=source_delta*1000,
                    unmatched_reason=reason))
            if end is not None:
                self.attempt = None
            return ready


class HandLimit:
    """The exclusive command owner must cancel pending closes and hold measured aperture.

    Only a completed, idempotent hold may return True; a delivery ACK is not enough.
    An opening must remain possible, including during a sensor or actuator fault.
    """

    def __init__(self, side: str, limit_n: float, *, max_age_s: float,
                 healthy_status: Collection[int]):
        if side not in ("left", "right"):
            raise ValueError("invalid tactile hand")
        self.side = side
        self.sensor = _TouchThreshold(limit_n, max_age_s, healthy_status)
        self._latched = self._held = False

    def tick(self, intent: Intent, touch: Touch | None, *, now_s: float, monotonic_s: float,
             cancel_and_hold: Callable[[str], bool]) -> Decision:
        try:
            reason, peak, point = self.sensor.inspect(touch, now_s, monotonic_s)
        except (AttributeError, TypeError, ValueError, OverflowError):
            reason, peak, point = "invalid_sample", None, None
        if not isinstance(intent, Intent):
            reason = "invalid_intent"
        trip = None
        if reason and not self._latched:
            trip = Trip(self.side, now_s if isinstance(now_s, (int, float)) and math.isfinite(now_s)
                        else None, reason, self.sensor.limit, peak, point)
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
