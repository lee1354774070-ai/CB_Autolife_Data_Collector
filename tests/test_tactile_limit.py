"""Owner-side replay and fail-closed entry tests; no robot or motor publishers."""

from dataclasses import replace
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock

from tactile_limit import HandLimit, Intent, OPTION, Touch, positive_limit


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


class EntryTest(unittest.TestCase):
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
