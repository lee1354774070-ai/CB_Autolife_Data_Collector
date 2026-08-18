#!/usr/bin/env python3
"""Timestamp normalization and nearest-neighbor selection for the collector.

Autolife camera messages carry a ROS ``header.stamp``, while robot telemetry is
currently transported as JSON in ``std_msgs/String``.  Some firmware versions
include a top-level timestamp and others do not.  This module accepts common
epoch units when present, rejects implausible values, and otherwise lets the
recorder fall back to the local ROS callback time.

The functions use only the Python standard library so synchronization behavior
can be unit-tested without ROS, cameras, or LeRobot.
"""

from __future__ import annotations

import math
from collections.abc import Iterable
from typing import Any, Protocol, TypeVar


EPOCH_SEC_MIN = 946_684_800.0  # 2000-01-01 UTC
EPOCH_SEC_MAX = 4_102_444_800.0  # 2100-01-01 UTC


class Timestamped(Protocol):
    """Structural type required by ``nearest_sample``."""

    stamp_sec: float


class ReceivedTimestamped(Timestamped, Protocol):
    """Timestamped sample that also records local callback arrival time."""

    received_sec: float


TimestampedT = TypeVar("TimestampedT", bound=Timestamped)
ReceivedTimestampedT = TypeVar("ReceivedTimestampedT", bound=ReceivedTimestamped)


def normalize_epoch_seconds(value: Any) -> float | None:
    """Convert a numeric epoch timestamp in s/ms/us/ns to seconds.

    Magnitude identifies the unit.  Values outside 2000-2100 are rejected so
    monotonic clocks, frame counters, and unrelated numeric fields cannot enter
    the same time domain as ROS wall-clock image stamps.
    """

    if isinstance(value, str):
        try:
            value = float(value)
        except ValueError:
            return None
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return None

    timestamp = float(value)
    if not math.isfinite(timestamp):
        return None
    absolute = abs(timestamp)
    if absolute >= 1e17:
        timestamp /= 1e9
    elif absolute >= 1e14:
        timestamp /= 1e6
    elif absolute >= 1e11:
        timestamp /= 1e3
    return timestamp if EPOCH_SEC_MIN <= timestamp <= EPOCH_SEC_MAX else None


def _stamp_mapping_to_seconds(value: Any) -> float | None:
    """Parse ROS-style ``{sec, nanosec}`` timestamp dictionaries."""

    if not isinstance(value, dict):
        return None
    seconds = value.get("sec", value.get("secs", value.get("seconds")))
    nanoseconds = value.get("nanosec", value.get("nsecs", value.get("nanoseconds", 0)))
    if not isinstance(seconds, (int, float)) or not isinstance(nanoseconds, (int, float)):
        return None
    return normalize_epoch_seconds(float(seconds) + float(nanoseconds) * 1e-9)


def payload_timestamp_sec(payload: Any, received_sec: float, max_clock_skew_sec: float = 5.0) -> float:
    """Return a trustworthy top-level JSON timestamp or ``received_sec``.

    Supported keys are ``timestamp``, ``stamp``, ``timestamp_ns``,
    ``timestamp_us``, and ``timestamp_ms``.  A parsed timestamp must also be
    close to the callback's wall clock.  This protects synchronization if a
    publisher exposes device uptime instead of Unix epoch time.
    """

    if not isinstance(payload, dict):
        return received_sec

    for key in ("timestamp", "stamp", "timestamp_ns", "timestamp_us", "timestamp_ms"):
        if key not in payload:
            continue
        value = payload[key]
        timestamp = _stamp_mapping_to_seconds(value)
        if timestamp is None:
            timestamp = normalize_epoch_seconds(value)
        if timestamp is not None and abs(timestamp - received_sec) <= max_clock_skew_sec:
            return timestamp
    return received_sec


def nearest_sample(samples: Iterable[TimestampedT], target_sec: float) -> tuple[TimestampedT, float] | None:
    """Return the sample nearest ``target_sec`` and its absolute delta."""

    nearest: TimestampedT | None = None
    nearest_delta = math.inf
    for sample in samples:
        delta = abs(sample.stamp_sec - target_sec)
        if delta < nearest_delta:
            nearest = sample
            nearest_delta = delta
    if nearest is None:
        return None
    return nearest, nearest_delta


def oldest_ready_sample(
    samples: Iterable[ReceivedTimestampedT],
    now_sec: float,
    wait_sec: float,
    after_stamp_sec: float | None,
) -> ReceivedTimestampedT | None:
    """Return the oldest unconsumed sample old enough for future peers to arrive.

    Waiting one synchronization window before matching an anchor allows packets
    immediately after the anchor to enter their buffers.  Without this delay,
    callback ordering can force matching against the previous camera frame.
    Choosing the oldest ready sample preserves FIFO order under temporary load
    instead of silently skipping an image in favor of the newest one.
    """

    ready = (
        sample
        for sample in samples
        if now_sec - sample.received_sec >= wait_sec
        and (after_stamp_sec is None or sample.stamp_sec > after_stamp_sec)
    )
    return min(ready, key=lambda sample: sample.stamp_sec, default=None)


def bracketing_samples(
    samples: Iterable[TimestampedT],
    target_sec: float,
) -> tuple[TimestampedT, TimestampedT] | None:
    """Return the closest samples immediately before and after ``target_sec``."""

    before, after = surrounding_samples(samples, target_sec)
    if before is None or after is None:
        return None
    return before, after


def surrounding_samples(
    samples: Iterable[TimestampedT],
    target_sec: float,
) -> tuple[TimestampedT | None, TimestampedT | None]:
    """Return the closest samples on each side of ``target_sec``.

    Unlike :func:`bracketing_samples`, this function preserves which side is
    missing. A recorder can therefore wait when a future sample has not arrived
    yet, while rejecting an anchor whose older sample has already fallen out of
    the buffer.
    """

    before: TimestampedT | None = None
    after: TimestampedT | None = None
    for sample in samples:
        if sample.stamp_sec <= target_sec and (before is None or sample.stamp_sec > before.stamp_sec):
            before = sample
        if sample.stamp_sec >= target_sec and (after is None or sample.stamp_sec < after.stamp_sec):
            after = sample
    return before, after


def latest_at_or_before(samples: Iterable[TimestampedT], target_sec: float) -> TimestampedT | None:
    """Return the newest causal sample whose timestamp does not exceed target."""

    candidates = (sample for sample in samples if sample.stamp_sec <= target_sec)
    return max(candidates, key=lambda sample: sample.stamp_sec, default=None)


def linear_interpolation_alpha(before_sec: float, after_sec: float, target_sec: float) -> float | None:
    """Return the interpolation weight for a target inside a time bracket."""

    if after_sec < before_sec or target_sec < before_sec or target_sec > after_sec:
        return None
    if after_sec == before_sec:
        return 0.0
    return (target_sec - before_sec) / (after_sec - before_sec)


def frame_interval_error_ratio(previous_sec: float, current_sec: float, fps: float) -> float:
    """Return absolute reference-frame interval error relative to ``1 / fps``."""

    expected = 1.0 / fps
    return abs((current_sec - previous_sec) - expected) / expected
