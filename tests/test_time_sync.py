#!/usr/bin/env python3
"""Unit tests for timestamp normalization and nearest-neighbor matching."""

from __future__ import annotations

import unittest
from dataclasses import dataclass

from time_sync import (
    bracketing_samples,
    frame_interval_error_ratio,
    latest_at_or_before,
    linear_interpolation_alpha,
    nearest_sample,
    normalize_epoch_seconds,
    oldest_ready_sample,
    payload_timestamp_sec,
    surrounding_samples,
)


@dataclass(frozen=True)
class Sample:
    stamp_sec: float
    value: str
    received_sec: float = 0.0


class TimeSyncTest(unittest.TestCase):
    def test_epoch_units_are_normalized(self) -> None:
        expected = 1_750_000_000.125
        self.assertAlmostEqual(normalize_epoch_seconds(expected), expected)
        self.assertAlmostEqual(normalize_epoch_seconds(expected * 1e3), expected)
        self.assertAlmostEqual(normalize_epoch_seconds(expected * 1e6), expected)
        self.assertAlmostEqual(normalize_epoch_seconds(expected * 1e9), expected)

    def test_payload_supports_ros_stamp_mapping(self) -> None:
        received = 1_750_000_000.2
        payload = {"stamp": {"sec": 1_750_000_000, "nanosec": 150_000_000}}
        self.assertAlmostEqual(payload_timestamp_sec(payload, received), 1_750_000_000.15)

    def test_invalid_or_distant_payload_timestamp_falls_back(self) -> None:
        received = 1_750_000_000.0
        self.assertEqual(payload_timestamp_sec({"timestamp": 123.0}, received), received)
        self.assertEqual(payload_timestamp_sec({"timestamp": received - 10.0}, received), received)

    def test_payload_respects_configured_clock_skew(self) -> None:
        received = 1_750_000_000.0
        payload = {"timestamp": received - 0.2}
        self.assertEqual(payload_timestamp_sec(payload, received, max_clock_skew_sec=0.15), received)
        self.assertAlmostEqual(
            payload_timestamp_sec(payload, received, max_clock_skew_sec=0.25),
            received - 0.2,
        )

    def test_nearest_sample_reports_delta(self) -> None:
        samples = [Sample(10.000, "old"), Sample(10.032, "nearest"), Sample(10.070, "new")]
        result = nearest_sample(samples, 10.030)
        self.assertIsNotNone(result)
        sample, delta = result
        self.assertEqual(sample.value, "nearest")
        self.assertAlmostEqual(delta, 0.002)

    def test_reference_waits_for_sync_window_and_is_not_reused(self) -> None:
        samples = [
            Sample(10.000, "consumed", 10.005),
            Sample(10.033, "ready", 10.038),
            Sample(10.066, "too-new", 10.071),
        ]
        selected = oldest_ready_sample(samples, now_sec=10.075, wait_sec=0.03, after_stamp_sec=10.000)
        self.assertIsNotNone(selected)
        self.assertEqual(selected.value, "ready")

    def test_fifo_selection_keeps_oldest_ready_reference(self) -> None:
        samples = [Sample(10.033, "first", 10.035), Sample(10.066, "second", 10.068)]
        selected = oldest_ready_sample(samples, now_sec=10.100, wait_sec=0.02, after_stamp_sec=10.000)
        self.assertIsNotNone(selected)
        self.assertEqual(selected.value, "first")

    def test_bracketing_and_causal_hold(self) -> None:
        samples = [Sample(10.00, "before"), Sample(10.04, "after"), Sample(10.08, "future")]
        before, after = bracketing_samples(samples, 10.03)
        self.assertEqual((before.value, after.value), ("before", "after"))
        self.assertEqual(latest_at_or_before(samples, 10.03).value, "before")
        self.assertAlmostEqual(linear_interpolation_alpha(10.00, 10.04, 10.03), 0.75)

    def test_surrounding_samples_identifies_missing_side(self) -> None:
        before, after = surrounding_samples([Sample(10.00, "before")], 10.03)
        self.assertEqual(before.value, "before")
        self.assertIsNone(after)

        before, after = surrounding_samples([Sample(10.04, "after")], 10.03)
        self.assertIsNone(before)
        self.assertEqual(after.value, "after")

    def test_frame_interval_gate_detects_one_missing_frame(self) -> None:
        self.assertAlmostEqual(frame_interval_error_ratio(10.0, 10.0 + 1 / 30, 30), 0.0)
        self.assertAlmostEqual(frame_interval_error_ratio(10.0, 10.0 + 2 / 30, 30), 1.0)


if __name__ == "__main__":
    unittest.main()
