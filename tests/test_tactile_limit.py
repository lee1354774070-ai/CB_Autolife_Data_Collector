"""Owner-side replay and fail-closed entry tests; no robot or motor publishers."""

from dataclasses import replace
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

from tactile_limit import EVENT_OPTION, HandLimit, Intent, OPTION, TactileEvents, Touch, positive_limit


ROOT = Path(__file__).resolve().parents[1]


def reading(stamp, peak=0.4, point=3):
    values = [0.0] * 8
    values[point] = peak
    return Touch(values, (0,) * 8, (stamp,) * 8)


def hand(side="left"):
    # Illustrative replay settings, NOT a hardware deadline or a cup rating.
    return HandLimit(side, 1.5, max_age_s=0.04, healthy_status={0})


class OwnerReplayTest(unittest.TestCase):
    def test_stops_existing_motion_without_waiting_for_next_close_and_stays_latched(self):
        guard = hand()
        pending = []
        def stop(side):
            self.assertEqual(side, "left")
            pending.clear()
            return True
        cancel = Mock(side_effect=stop)
        def step(t, intent, force):
            decision = guard.tick(intent, reading(t, force), now_s=t, monotonic_s=t,
                                  cancel_and_hold=cancel)
            if decision.forward:
                pending.append(decision.forward)
            return decision

        self.assertEqual(step(1, Intent.CLOSE, 1.49).forward, Intent.CLOSE)
        self.assertEqual(pending, [Intent.CLOSE])
        decision = step(1.01, Intent.IDLE, 1.5)
        self.assertEqual((decision.trip.peak_n, decision.trip.point, decision.trip.side),
                         (1.5, 3, "left"))
        self.assertEqual(pending, [])
        self.assertTrue(decision.hold_confirmed)
        self.assertIsNone(step(1.02, Intent.CLOSE, 5).trip)
        self.assertIsNone(step(1.03, Intent.CLOSE, 0.1).forward)
        cancel.assert_called_once_with("left")
        self.assertEqual(step(1.04, Intent.OPEN, 0.1).forward, Intent.OPEN)
        self.assertEqual(step(1.05, Intent.CLOSE, 0.1).forward, Intent.CLOSE)

    def test_faults_deny_closing_but_never_trap_an_object(self):
        good = reading(1)
        cases = (None, replace(good, normal_n=[0.1]*7),
                 replace(good, normal_n=[0.1]*7 + [float("nan")]),
                 replace(good, normal_n=["bad"]*8), replace(good, normal_n=[-1]*8),
                 replace(good, status=(0,)*7 + (1,)), replace(good, status=(0.0,)*8),
                 replace(good, stamp_s=(1.0,)*7 + (.90,)),
                 replace(good, stamp_s=(1.0,)*7 + (1.01,)))
        for sample in cases:
            with self.subTest(sample=sample):
                guard = hand()
                stop = Mock(return_value=True)
                step = lambda intent,row: guard.tick(intent, row, now_s=1, monotonic_s=2, cancel_and_hold=stop)
                result = step(Intent.CLOSE, sample)
                self.assertTrue(result.latched)
                self.assertIsNone(result.forward)
                self.assertIsNotNone(result.trip)
                self.assertEqual(step(Intent.OPEN, sample).forward, Intent.OPEN)
                self.assertIsNone(step(Intent.CLOSE, good).forward)
                self.assertEqual(step(Intent.OPEN, good).forward, Intent.OPEN)
                self.assertEqual(step(Intent.CLOSE, good).forward, Intent.CLOSE)

    def test_one_old_point_cannot_be_rearmed_by_repoll_or_clock_regression(self):
        guard = hand()
        stop = Mock(return_value=True)
        def step(s, now, tick):
            return guard.tick(Intent.CLOSE, s, now_s=now, monotonic_s=tick, cancel_and_hold=stop)
        self.assertFalse(step(reading(10), 10, 5).latched)
        self.assertFalse(step(reading(10), 10, 5.02).latched)
        s = replace(reading(10.01), stamp_s=(10,)+(10.01,)*7)
        self.assertEqual(step(s, 10.01, 5.041).trip.reason, "stale_point")
        for s, now, tick, reason in ((reading(9.99), 10, 6, "out_of_order_timestamp"),
                                     (reading(10), 10, 4, "invalid_clock")):
            guard = hand()
            self.assertFalse(step(reading(10), 10, 5).latched)
            self.assertEqual(step(s, now, tick).trip.reason, reason)

    def test_unacknowledged_or_failed_hold_does_not_pass_close(self):
        guard = hand()
        cancel = Mock(side_effect=[OSError("transport offline"), 1, True])
        for index in range(3):
            result = guard.tick(Intent.CLOSE, reading(1, 2), now_s=1, monotonic_s=1,
                                cancel_and_hold=cancel)
            self.assertIsNone(result.forward)
            self.assertEqual(result.hold_confirmed, index == 2)
            self.assertEqual(result.hold_error is None, index == 2)
            self.assertEqual(result.trip is not None, index == 0)
        self.assertEqual(cancel.call_count, 3)

    def test_hands_independent_and_open_always_replaces_close(self):
        stop = Mock(return_value=True)
        left, right = hand(), hand("right")
        def step(g, intent, peak):
            return g.tick(intent, reading(1, peak), now_s=1, monotonic_s=1, cancel_and_hold=stop)
        self.assertTrue(step(left, Intent.CLOSE, 3).latched)
        self.assertFalse(step(right, Intent.CLOSE, .2).latched)
        self.assertEqual(step(left, Intent.OPEN, 3).forward, Intent.OPEN)
        self.assertTrue(step(left, Intent.CLOSE, .1).latched)
        # No second hold cancels the opening move, even if the button changes again.
        self.assertEqual(stop.call_count, 1)
        self.assertTrue(step(right, "close", .1).latched)

    def test_open_can_escape_with_unconfirmed_stop_but_does_not_reenable_closing(self):
        guard = hand()
        cancel = Mock(side_effect=[False, False, True])
        def step(intent, sample):
            return guard.tick(intent, sample, now_s=1, monotonic_s=1, cancel_and_hold=cancel)
        opening = step(Intent.OPEN, reading(1, 9))
        self.assertEqual(opening.forward, Intent.OPEN)
        self.assertIsNotNone(opening.hold_error)
        self.assertTrue(step(Intent.OPEN, reading(1)).latched)
        self.assertIsNone(step(Intent.CLOSE, reading(1)).forward)
        self.assertFalse(step(Intent.OPEN, reading(1)).latched)

    def test_invalid_config_never_silently_disables_protection(self):
        for value in ("", None, True, -1, 0, "nan", "inf", "bad"):
            with self.assertRaises(ValueError):
                positive_limit(value)
        for codes in ([], [True], ["0"]):
            with self.assertRaises(ValueError):
                HandLimit("left", 1, max_age_s=.04, healthy_status=codes)


class EventAlignmentTest(unittest.TestCase):
    def setUp(self):
        self.events = TactileEvents(1.5, .15, .03, {0})
        self.events.begin(4, 10.)

    def observe(self, now, sample, hand="left"):
        with patch("tactile_limit.time.time", return_value=now), patch("tactile_limit.time.monotonic", return_value=now):
            self.events.observe(hand, sample)

    def test_first_accepted_causal_frame_and_actual_point_stamp(self):
        sample = replace(reading(10.01, 1.5), stamp_s=[10.01]*3 + [10.012] + [10.009]*4)
        self.observe(10.016, sample)
        self.observe(10.017, sample)
        self.assertFalse(self.events.drain(frame_index=0, anchor=10.015))
        event, = self.events.drain(frame_index=1, anchor=10.024)
        self.assertEqual((event["episode_index"], event["frame_index"], event["hand_point_index"]), (4, 1, 3))
        self.assertAlmostEqual(event["anchor_minus_source_ms"], 12)
        self.assertAlmostEqual(event["anchor_minus_detection_ms"], 8)
        self.assertIsNone(event["unmatched_reason"])
        self.observe(10.025, reading(10.025, 2))
        self.assertFalse(self.events.drain(frame_index=2, anchor=10.04))
        self.observe(10.05, reading(10.05, .2))
        self.observe(10.06, reading(10.06, 2))
        event, = self.events.drain(frame_index=3, anchor=10.06)
        self.assertEqual(event["anchor_minus_detection_ms"], 0.)

    def test_source_age_and_detection_delay_are_checked_separately(self):
        self.observe(10.1, reading(10.02, 2))
        self.observe(10.05, reading(10.05, 3), "right")
        events = self.events.drain(frame_index=0, anchor=10.11)
        self.assertEqual([e["unmatched_reason"] for e in events],
                         ["source_outside_sync_window", "detection_outside_sync_window"])
        self.assertTrue(all(e["frame_index"] is None and e["candidate_frame_index"] == 0 for e in events))

    def test_discard_and_reused_index_never_cross_attempts(self):
        self.observe(10.01, reading(9.999, 3))  # late SDK packet from before start
        self.observe(10.02, replace(reading(10.02, 4), status=(1,)*8))
        self.observe(10.03, reading(10.001, 2))  # out-of-order after the unhealthy packet
        self.observe(10.04, reading(10.04, 5))
        event, = self.events.drain(end="episode_discarded")
        self.assertEqual((event["peak_n"], event["frame_index"], event["unmatched_reason"]), (5, None, "episode_discarded"))
        self.observe(10.06, reading(10.06, 5))
        self.events.begin(4, 11.)
        self.assertFalse(self.events.drain(frame_index=0, anchor=11.01))

    def test_backlog_is_bounded_and_loss_is_explicit(self):
        self.events.capacity = 1
        for hand in ("right", "left"):
            self.observe(10.01, reading(10.01, 2), hand)
        overflow, first = self.events.drain(end="shutdown")
        self.assertEqual((overflow["count"], overflow["unmatched_reason"], overflow["frame_index"]), (1, "queue_full", None))
        self.assertEqual(first["side"], "right")


class EntryTest(unittest.TestCase):
    def test_passive_threshold_requires_tactile_and_never_implies_a_hard_limit(self):
        from recorder_config import parse_args
        with patch.dict(os.environ, {EVENT_OPTION: "1.25"}):
            args = parse_args(["--output-dir", "/unused", "--with-tactile"])
            self.assertEqual((args.fps, args.tactile_event_threshold_n, args.gripper_tactile_limit_n), (30, 1.25, None))
            for value, tactile in (("1", "0"), ("nan", "1")):
                env = dict(os.environ, WITH_TACTILE=tactile, TACTILE_EVENT_THRESHOLD_N=value)
                result = subprocess.run(["bash", str(ROOT / "start_lerobot_official_collect.sh"), "cup"],
                                        env=env, text=True, capture_output=True, timeout=5)
                self.assertEqual(result.returncode, 2)
                self.assertIn(EVENT_OPTION, result.stderr)

    def test_launcher_rejects_before_camera_hook_dataset_or_mode_dispatch(self):
        with tempfile.TemporaryDirectory() as directory:
            home = Path(directory)
            settings = home / ".config/autolife_collector/camera_session.json"
            settings.parent.mkdir(parents=True)
            settings.write_text("{}")
            out = home / "must-not-exist"
            for mode in ("keyboard", "vr", "subtask", "dagger"):
                env = dict(os.environ, HOME=directory, OUTPUT_BASE_DIR=str(out), COLLECTOR_MODE=mode,
                           SUBTASKS_JSON='["grasp"]' if mode == "subtask" else "[]", VR_CONTROL="",
                           GRIPPER_TACTILE_LIMIT_N="1.5")
                result = subprocess.run(["bash", str(ROOT / "start_lerobot_official_collect.sh"), "cup"],
                                        env=env, capture_output=True, text=True, timeout=5)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn("no verified factory", result.stderr)
                self.assertFalse(out.exists())

    def test_direct_recorder_refuses_before_ros_import_and_help_is_available(self):
        env = {k: v for k,v in os.environ.items() if k != OPTION}
        cmd = [sys.executable, "-B", str(ROOT / "record_lerobot_official.py"), "--output-dir", "/unused"]
        for extra, settings in ((["--gripper-tactile-limit-n", "1"], env),
                                ([], dict(env, GRIPPER_TACTILE_LIMIT_N="2"))):
            result = subprocess.run(cmd + extra, env=settings, text=True, capture_output=True)
            self.assertEqual(result.returncode, 2)
            self.assertIn("no verified factory", result.stderr)
        result = subprocess.run(["bash", str(ROOT / "start_lerobot_official_collect.sh"), OPTION, "--help"],
                                env=env, text=True, capture_output=True, check=True)
        self.assertIn("not force protection", result.stdout)

    def test_misplaced_limit_is_not_ignored_as_task_text(self):
        for arg in ("GRIPPER_TACTILE_LIMIT_N=1", "--gripper-tactile-limit-n=1", "--gripper-tactile-limit-n"):
            result = subprocess.run(["bash", str(ROOT / "start_lerobot_official_collect.sh"), "cup", arg],
                                    text=True, capture_output=True, timeout=5)
            self.assertEqual(result.returncode, 2)
            self.assertIn("before bash", result.stderr)
