"""VR gesture and acknowledgement tests without ROS, hardware, or sleeping."""

import json
import subprocess
import unittest
import tempfile
import threading
import time
from unittest.mock import patch
from concurrent.futures import Future
from pathlib import Path

from vr_collector_control import ButtonGestures, ReceiptCache, VrSession, decode_controls


def payload(*faces, grips=True):
    data = {h: {"b": [{"p": False} for _ in range(6)]} for h in ("l", "r")}
    for hand in ("l", "r"):
        data[hand]["b"][1]["p"] = grips
    for hand, index in faces:
        data[hand]["b"][index]["p"] = True
    return json.dumps(data)


class GestureTest(unittest.TestCase):
    def test_all_bindings_fire_only_on_release(self):
        for button, expected in [(('r', 5), 'save'), (('l', 4), 'reset'), (('l', 5), 'discard'), (('r', 4), 'start')]:
            with self.subTest(button=button):
                gestures = ButtonGestures()
                self.assertIsNone(gestures.update(True, set(), 1))
                for _ in range(10):
                    self.assertIsNone(gestures.update(True, {button}, 2))
                self.assertEqual(gestures.update(True, set(), 3), expected)
                self.assertIsNone(gestures.update(True, set(), 4))

    def test_grips_and_face_may_arrive_together_after_neutral(self):
        gestures = ButtonGestures()
        self.assertIsNone(gestures.update(False, set(), 1))
        self.assertIsNone(gestures.update(True, {('r', 4)}, 1.1))
        self.assertEqual(gestures.update(True, set(), 1.2), 'start')
        # A face held across Grip loss still cannot complete a stale gesture.
        gestures.update(True, {('r', 4)}, 2)
        gestures.update(False, {('r', 4)}, 2.1)
        gestures.update(True, {('r', 4)}, 2.2)
        self.assertIsNone(gestures.update(True, set(), 2.3))

    def test_held_button_at_startup_cannot_fire(self):
        gestures = ButtonGestures()
        gestures.update(True, {('r', 5)}, 1)
        self.assertIsNone(gestures.update(True, set(), 2))

    def test_grip_loss_and_multiple_faces_cancel(self):
        for grips, faces in [(False, set()), (True, {('r', 5), ('r', 4)})]:
            gestures = ButtonGestures()
            gestures.update(True, set(), 1)
            gestures.update(True, {('r', 5)}, 2)
            gestures.update(grips, faces, 3)
            self.assertIsNone(gestures.update(True, set(), 4))

    def test_switching_face_without_neutral_cancels(self):
        gestures = ButtonGestures()
        gestures.update(True, set(), 1)
        gestures.update(True, {('r', 5)}, 2)
        gestures.update(True, {('l', 4)}, 3)
        self.assertIsNone(gestures.update(True, set(), 4))

    def test_double_y_never_quits(self):
        for release_grips, second_time, expected in [(False, 4, 'discard'), (False, 10, 'discard'), (True, 4, 'discard')]:
            gestures = ButtonGestures()
            gestures.update(True, set(), 1)
            gestures.update(True, {('l', 5)}, 2)
            self.assertEqual(gestures.update(True, set(), 2.1), 'discard')
            if release_grips:
                gestures.update(False, set(), 3)
                gestures.update(True, set(), 3.1)
            gestures.update(True, {('l', 5)}, second_time)
            self.assertEqual(gestures.update(True, set(), second_time + .1), expected)

    def test_decoder_rejects_malformed_and_string_boolean(self):
        invalid = json.loads(payload())
        invalid['r']['b'][1]['p'] = 'false'
        for text in ['null', '{}', '[1]', '{', json.dumps(invalid)]:
            with self.assertRaises((ValueError, TypeError, KeyError)):
                decode_controls(text)
        self.assertEqual(decode_controls(payload(('r', 5))), (True, {('r', 5)}))

    def test_v4_webxr_numeric_faces_reach_all_four_gestures(self):
        packet = {
            'leftController': {'gripActive': True, 'xButton': 0, 'yButton': 0},
            'rightController': {'gripActive': True, 'aButton': 0, 'bButton': 0},
        }
        for hand, face, expected in (
                ('rightController', 'aButton', 'start'), ('rightController', 'bButton', 'save'),
                ('leftController', 'xButton', 'reset'), ('leftController', 'yButton', 'discard')):
            gestures = ButtonGestures()
            self.assertIsNone(gestures.update(*decode_controls(json.dumps(packet)), 1))
            packet[hand][face] = 1
            self.assertIsNone(gestures.update(*decode_controls(json.dumps(packet)), 2))
            packet[hand][face] = 0
            self.assertEqual(gestures.update(*decode_controls(json.dumps(packet)), 3), expected)

    def test_v4_rejects_missing_or_nonbinary_values(self):
        for value in (None, 'false', '1', 1.0, 0.5, -1, 2, [], {}):
            packet = {
                'leftController': {'gripActive': True, 'xButton': 0, 'yButton': 0},
                'rightController': {'gripActive': True, 'aButton': value, 'bButton': 0},
            }
            with self.subTest(value=value), self.assertRaises(ValueError):
                decode_controls(json.dumps(packet))
        del packet['rightController']['aButton']
        with self.assertRaises(ValueError):
            decode_controls(json.dumps(packet))

    def test_y_packet_bounce_does_not_quit(self):
        gestures = ButtonGestures()
        gestures.update(True, set(), 1)
        gestures.update(True, {('l', 5)}, 1.1)
        self.assertEqual(gestures.update(True, set(), 1.2), 'discard')
        gestures.update(True, {('l', 5)}, 1.21)
        self.assertIsNone(gestures.update(True, set(), 1.22))


class ReceiptCacheTest(unittest.TestCase):
    def test_unchanged_files_are_not_parsed_again_and_atomic_replace_refreshes(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'status.json'
            path.write_text('{"request_id":"old"}')
            with patch('vr_collector_control.json.loads', wraps=json.loads) as loads:
                cache = ReceiptCache((path,), interval=.005)
                try:
                    deadline = time.monotonic() + 1
                    while not cache.snapshot()[0] and time.monotonic() < deadline:
                        time.sleep(.005)
                    self.assertEqual(cache.snapshot()[0]['request_id'], 'old')
                    time.sleep(.03)
                    self.assertEqual(loads.call_count, 1)
                    replacement = Path(directory) / 'next.json'
                    replacement.write_text('{"request_id":"new"}')
                    replacement.replace(path)
                    deadline = time.monotonic() + 1
                    while cache.snapshot()[0].get('request_id') != 'new' and time.monotonic() < deadline:
                        time.sleep(.005)
                    self.assertEqual(cache.snapshot()[0]['request_id'], 'new')
                    self.assertEqual(loads.call_count, 2)
                finally:
                    cache.close()

    def test_blocked_storage_does_not_block_callbacks(self):
        blocked, release = threading.Event(), threading.Event()
        def slow(*args, **kwargs):
            blocked.set()
            release.wait(2)
            return '{}'
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'status.json'
            path.write_text('{}')
            with patch.object(Path, 'read_text', slow):
                cache = ReceiptCache((path,))
                try:
                    self.assertTrue(blocked.wait(1))
                    started = time.monotonic()
                    for _ in range(1000):
                        self.assertEqual(cache.snapshot(), ({},))
                    self.assertLess(time.monotonic() - started, .1)
                finally:
                    release.set()
                    cache.close()


class SessionTest(unittest.TestCase):
    def setUp(self):
        self.sent, self.speech, self.logs = [], [], []

        def submit(command):
            future = Future()
            self.sent.append((command, future))
            return future

        self.session = VrSession(submit, self.speech.append, log=self.logs.append)

    def tap(self, button, now):
        self.session.on_payload(payload(), now)
        self.session.on_payload(payload(button), now + .1)
        self.session.on_payload(payload(), now + .2)

    def receipt(self, event, stamp=100, **kwargs):
        return dict(event=event, success=True, request_id=f'req-{stamp}', wall_time=stamp,
                    frames=30, total_saved_episodes=4, session_saved_episodes=2, **kwargs)

    def test_countdown_does_not_claim_recording_before_ack(self):
        self.tap(('r', 4), 1)
        for now in [1.5, 2, 2.2, 2.5, 3, 3.2, 3.5, 4, 4.3]:
            self.session.on_payload(payload(), now)
            self.session.tick(now)
        self.assertEqual([c for c, _ in self.sent], ['start'])
        self.assertEqual(self.session.state, 'idle')
        self.sent[0][1].set_result(self.receipt('start'))
        self.session.tick(4.4)
        self.assertEqual(self.session.state, 'recording')
        self.assertIn('开始录制', self.speech)

    def test_delayed_timer_still_speaks_three_two_one_before_start(self):
        self.tap(('r', 4), 1)
        for now in (5, 6):
            self.session.on_payload(payload(), now)
            self.session.tick(now)
            self.assertFalse(self.sent)
        self.assertEqual(self.speech, ['三', '二', '一'])
        self.session.on_payload(payload(), 7)
        self.session.tick(7)
        self.assertEqual([c for c, _ in self.sent], ['start'])

    def test_each_round_has_countdown_and_confirmed_save_or_discard_feedback(self):
        for index, finish in enumerate(('save', 'discard')):
            start = 10 + index * 10
            self.tap(('r', 4), start)
            for offset in (.5, 1, 1.3, 1.8, 2.4, 2.9, 3.5):
                self.session.on_payload(payload(), start + offset)
                self.session.tick(start + offset)
            self.assertEqual(self.sent[-1][0], 'start')
            self.sent[-1][1].set_result(self.receipt('start', stamp=start))
            self.session.tick(start + 3.6)
            self.session.command(finish, start + 4)
            before = len(self.speech)
            self.sent[-1][1].set_result(self.receipt(finish, stamp=start + 4))
            self.session.tick(start + 4.1)
            self.assertEqual(len(self.speech), before + 1)
            self.assertEqual(self.session.state, 'idle')
        for spoken in ('三', '二', '一', '开始录制'):
            self.assertEqual(self.speech.count(spoken), 2)
        self.assertTrue(any('保存成功' in s and '30帧' in s for s in self.speech))
        self.assertIn('已丢弃，可以开始下一次采集', self.speech)

    def test_no_pending_episode_is_ready_feedback_not_a_failed_session(self):
        for event in ('save', 'discard'):
            status = self.receipt(event, stamp=100 if event == 'save' else 101,
                                  message=f'no pending episode to {event}')
            status.update(success=False, recording=False, frames=0)
            self.session.accept_status(status)
            self.assertEqual(self.session.state, 'idle')
            self.assertEqual(self.speech[-1], '当前没有待处理数据，可以按A开始下一条')

    def test_repeated_a_does_not_extend_countdown(self):
        self.tap(('r', 4), 1)
        deadline = self.session.start_at
        self.tap(('r', 4), 1.4)
        self.assertEqual(self.session.start_at, deadline)

    def test_stale_input_cancels_countdown(self):
        self.tap(('r', 4), 1)
        self.session.tick(5)
        self.assertIsNone(self.session.start_at)
        self.assertFalse(self.sent)

    def test_malformed_packet_cannot_complete_gesture(self):
        self.session.on_payload(payload(), 1)
        self.session.on_payload(payload(('r', 5)), 1.1)
        self.session.on_payload('{}', 1.2)
        self.session.on_payload(payload(), 1.3)
        self.assertIsNone(self.session.start_at)

    def test_b_or_y_cancels_countdown_without_sending(self):
        for command in ('save', 'discard'):
            self.session.last_input = 1
            self.session.command('start', 1)
            self.session.command(command, 2)
            self.assertIsNone(self.session.start_at)
            self.assertFalse(self.sent)

    def test_save_wait_keeps_buttons_responsive_and_never_queues_start(self):
        self.session.state = 'recording'
        self.session.command('save', 1)
        self.tap(('r', 5), 2)
        self.assertEqual([c for c, _ in self.sent], ['save'])
        self.sent[0][1].set_result(self.receipt('save'))
        self.session.tick(3)
        self.assertEqual(self.session.state, 'idle')
        self.assertIsNone(self.session.start_at)

    def test_quit_waits_for_pending_save(self):
        self.session.command('save', 1)
        self.session.command('quit', 2)
        self.assertEqual(len(self.sent), 1)
        self.sent[0][1].set_result(self.receipt('save'))
        self.session.tick(3)
        self.assertEqual([c for c, _ in self.sent], ['save', 'quit'])

    def test_timeout_does_not_retry_or_start_again(self):
        self.session.command('save', 1)
        self.sent[0][1].set_exception(TimeoutError('test'))
        self.session.tick(2)
        self.session.command('start', 3)
        self.assertEqual(self.session.state, 'unknown')
        self.assertEqual(len(self.sent), 1)

    def test_invalid_save_receipt_is_announced_as_discard(self):
        self.session.accept_status(self.receipt('discard', message='invalid episode discarded: gap'))
        self.assertEqual(self.session.state, 'idle')
        self.assertIn('未保存', self.speech[-1])
        self.assertFalse(any('保存成功' in text for text in self.speech))

    def test_receipt_is_announced_once_and_keyboard_updates_state(self):
        receipt = self.receipt('start')
        self.session.accept_status(receipt)
        self.session.accept_status(receipt)
        self.assertEqual(self.speech, ['开始录制'])
        self.assertEqual(self.session.state, 'recording')

    def test_invalid_event_is_announced_once_and_old_events_ignored(self):
        self.session.accept_status(self.receipt('start', stamp=100))
        event = dict(event='episode_invalidated', wall_time=101, reason='gap')
        self.session.accept_event(event)
        self.session.accept_event(event)
        self.assertEqual(self.session.state, 'invalid')
        self.assertEqual(len(self.speech), 2)
        self.session.accept_status(self.receipt('discard', stamp=102))
        self.session.accept_event(event)
        self.assertEqual(self.session.state, 'idle')

    def test_failed_save_blocks_new_episode(self):
        receipt = self.receipt('save', message='save failed: writer broken')
        receipt['success'] = False
        self.session.accept_status(receipt)
        self.session.command('start', 1)
        self.assertEqual(self.session.state, 'unknown')
        self.assertFalse(self.sent)

    def test_factory_reset_is_reported_without_motion_or_latching_unknown(self):
        self.session.reset_supported = False
        self.session.command('reset', 1)
        self.assertFalse(self.sent)
        self.assertEqual(self.session.state, 'idle')
        self.assertIn('原厂复位接口尚未接入', self.speech[-1])
        self.session.last_input = 2
        self.session.command('start', 2)
        self.assertIsNotNone(self.session.start_at)

    def test_idle_reset_sends_no_recorder_command(self):
        self.session.command('reset', 1)
        self.assertEqual([c for c, _ in self.sent], ['reset'])
        self.assertEqual(self.session.state, 'resetting')
        self.sent[0][1].set_result({'success': True})
        self.session.tick(2)
        self.assertEqual(self.session.state, 'idle')

    def test_reset_preserves_open_invalid_and_unknown_episode(self):
        for state in ('recording', 'invalid', 'unknown'):
            self.session.state = state
            self.session.command('reset', 1)
            self.assertFalse(self.sent)
            self.assertEqual(self.session.state, state)

    def test_pending_save_blocks_x_and_no_reset_is_queued(self):
        self.session.command('save', 1)
        self.session.command('reset', 2)
        self.sent[0][1].set_result(self.receipt('save'))
        self.session.tick(3)
        self.assertEqual([c for c, _ in self.sent], ['save'])

    def test_reset_during_countdown_cancels_then_resets(self):
        self.session.last_input = 1
        self.session.command('start', 1)
        self.session.command('reset', 2)
        self.assertIsNone(self.session.start_at)
        self.assertEqual([c for c, _ in self.sent], ['reset'])

    def test_distinct_feedback_after_ack_only(self):
        from vr_feedback import PATTERNS
        events = []
        self.session.feedback = events.append
        self.session.command('save', 1)
        self.assertEqual(events, ['saving'])
        self.sent[0][1].set_result(self.receipt('save'))
        self.session.tick(2)
        self.assertEqual(events, ['saving', 'save'])
        self.assertEqual(len(PATTERNS), len({tuple(p) for p in PATTERNS.values()}))

    def test_delayed_mark_receipt_cannot_undo_newer_invalidation(self):
        self.session.accept_event(dict(event='episode_invalidated', wall_time=102, reason='gap'))
        self.session.accept_status(self.receipt('mark_subtask', stamp=101))
        self.assertEqual(self.session.state, 'invalid')
        self.assertFalse(any(text == '下一步' for text in self.speech))

    def test_old_future_receipt_cannot_undo_newer_keyboard_save(self):
        self.session.accept_status(self.receipt('save', stamp=102))
        self.session.accept_status(self.receipt('start', stamp=101))
        self.assertEqual(self.session.state, 'idle')


class LauncherHelpTest(unittest.TestCase):
    launcher = Path(__file__).resolve().parents[1] / 'start_lerobot_official_collect.sh'

    def test_vr_parameter_help_works_without_ros(self):
        for name in ('VR_CONTROL', 'VR_SPEECH', 'VR_START_DELAY_SEC', 'SUBTASKS_JSON', 'VR_A_LONG_PRESS_SEC', 'VR_INPUT_TOPIC', 'VR_RESET_PREFIX'):
            result = subprocess.run(['bash', str(self.launcher), name, '--help'], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn(name, result.stdout)


class SubtaskVrTest(unittest.TestCase):
    tap = SessionTest.tap
    receipt = SessionTest.receipt

    def setUp(self):
        self.sent, self.speech, self.logs = [], [], []

        def submit(command):
            future = Future()
            self.sent.append((command, future))
            return future

        self.session = VrSession(submit, self.speech.append, start_delay=0,
                                 subtask_mode=True, a_long_press_sec=1, log=self.logs.append)

    def hold_b(self, start=1):
        self.session.on_payload(payload(), start)
        for i in range(7):
            self.session.on_payload(payload(('r', 5)), start + .1 + i * .2)
        self.assertFalse(self.sent, 'B must not fire until release')
        self.session.on_payload(payload(), start + 1.4)

    def test_only_a_starts_in_subtask_mode(self):
        self.hold_b()
        self.assertFalse(self.sent)
        self.tap(('r', 5), 4)
        self.assertFalse(self.sent)
        self.tap(('r', 4), 5)
        self.assertEqual([c for c, _ in self.sent], ['start'])

    def test_recording_short_b_marks_and_waits_for_real_confirmation(self):
        self.session.state = 'recording'
        self.tap(('r', 5), 1)
        self.assertEqual([c for c, _ in self.sent], ['mark_subtask'])
        self.assertFalse(self.speech)
        self.sent[0][1].set_result(self.receipt('mark_subtask', subtasks={
            'enabled': True, 'confirmed': 1, 'total': 3, 'next_subtask': 'handover'}))
        self.session.tick(1.3)
        self.assertEqual(self.session.state, 'recording')
        self.assertEqual(self.speech, ['下一步'])

    def test_recording_long_b_only_saves_no_short_mark(self):
        self.session.state = 'recording'
        self.hold_b()
        self.assertEqual([c for c, _ in self.sent], ['save'])
        self.session.on_payload(payload(), 2.5)
        self.assertEqual(len(self.sent), 1)

    def test_final_mark_announces_saving_but_never_success_before_receipt(self):
        self.session.state = 'recording'
        self.session.subtask_progress = {'confirmed': 2, 'total': 3}
        self.tap(('r', 5), 1)
        self.assertEqual([c for c, _ in self.sent], ['mark_subtask'])
        self.assertEqual(self.speech, ['保存中'])
        self.assertEqual(self.session.state, 'recording')

    def test_last_mark_save_and_early_save_have_distinct_brief_speech(self):
        for index, (complete, phrase) in enumerate([(True, '已保存'), (False, '已保存，标注未完成')]):
            self.session.accept_status(self.receipt('save', stamp=100 + index, subtasks={
                'enabled': True, 'confirmed': 3 if complete else 1, 'total': 3, 'complete': complete}))
            self.assertEqual(self.session.state, 'idle')
            self.assertEqual(self.speech[-1], phrase)

    def test_press_started_during_pending_save_does_not_start_new_episode(self):
        self.session.state = 'recording'
        self.session.command('save', 1)
        self.session.on_payload(payload(), 1.1)
        self.session.on_payload(payload(('r', 5)), 1.2)
        self.sent[0][1].set_result(self.receipt('save'))
        self.session.tick(1.3)
        self.session.on_payload(payload(), 1.4)
        self.assertEqual([c for c, _ in self.sent], ['save'])

    def test_no_heartbeat_or_grip_loss_cancels_long_press(self):
        for kind in ('gap', 'grip', 'malformed'):
            self.session.state = 'recording'
            self.session.on_payload(payload(), 1)
            self.session.on_payload(payload(('r', 5)), 1.1)
            if kind == 'grip':
                self.session.on_payload(payload(('r', 5), grips=False), 1.2)
            elif kind == 'malformed':
                self.session.on_payload('{}', 1.2)
            self.session.on_payload(payload(), 2.4 if kind == 'gap' else 1.3)
            self.assertFalse(self.sent)

    def test_threshold_boundary_and_held_repeats(self):
        for duration, expected in ((.99, 'a_short'), (1.0, 'a_long'), (2., 'a_long')):
            gesture = ButtonGestures(a_long_press_sec=1)
            gesture.update(True, set(), 0)
            gesture.update(True, {('r', 5)}, 1)
            gesture.update(True, {('r', 5)}, 1 + duration / 2)
            self.assertEqual(gesture.update(True, set(), 1 + duration), expected)

    def test_bad_long_press_threshold_is_rejected_without_ros(self):
        script = Path(__file__).resolve().parents[1] / 'vr_collector_control.py'
        for value in ('nan', 'inf', '0', '-1', '11'):
            result = subprocess.run(['python3', str(script), '--base-dir', '/tmp/test',
                                     '--topic', '/test/input', '--tts-topic', '/test/tts',
                                     '--a-long-press-sec', value, '--check-config'], capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('--a-long-press-sec must', result.stderr)


if __name__ == '__main__':
    unittest.main()
