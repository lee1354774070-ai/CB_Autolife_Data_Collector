"""VR gesture and acknowledgement tests without ROS, hardware, or sleeping."""

import json
import subprocess
import unittest
from concurrent.futures import Future
from pathlib import Path

from vr_collector_control import ButtonGestures, VrSession, decode_controls


def payload(*faces, grips=True):
    data = {h: {"b": [{"p": False} for _ in range(6)]} for h in ("l", "r")}
    for hand in ("l", "r"):
        data[hand]["b"][1]["p"] = grips
    for hand, index in faces:
        data[hand]["b"][index]["p"] = True
    return json.dumps(data)


class GestureTest(unittest.TestCase):
    def test_all_bindings_fire_only_on_release(self):
        for button, expected in [(('r', 4), 'start'), (('r', 5), 'save'), (('l', 4), 'discard'), (('l', 5), 'arm_quit')]:
            with self.subTest(button=button):
                gestures = ButtonGestures()
                self.assertIsNone(gestures.update(True, set(), 1))
                for _ in range(10):
                    self.assertIsNone(gestures.update(True, {button}, 2))
                self.assertEqual(gestures.update(True, set(), 3), expected)
                self.assertIsNone(gestures.update(True, set(), 4))

    def test_held_button_at_startup_cannot_fire(self):
        gestures = ButtonGestures()
        gestures.update(True, {('r', 4)}, 1)
        self.assertIsNone(gestures.update(True, set(), 2))

    def test_grip_loss_and_multiple_faces_cancel(self):
        for grips, faces in [(False, set()), (True, {('r', 4), ('l', 5)})]:
            gestures = ButtonGestures()
            gestures.update(True, set(), 1)
            gestures.update(True, {('r', 4)}, 2)
            gestures.update(grips, faces, 3)
            self.assertIsNone(gestures.update(True, set(), 4))

    def test_switching_face_without_neutral_cancels(self):
        gestures = ButtonGestures()
        gestures.update(True, set(), 1)
        gestures.update(True, {('r', 4)}, 2)
        gestures.update(True, {('r', 5)}, 3)
        self.assertIsNone(gestures.update(True, set(), 4))

    def test_double_y_requires_time_window_and_continuous_grips(self):
        for release_grips, second_time, expected in [(False, 4, 'quit'), (False, 10, 'arm_quit'), (True, 4, 'arm_quit')]:
            gestures = ButtonGestures()
            gestures.update(True, set(), 1)
            gestures.update(True, {('l', 5)}, 2)
            self.assertEqual(gestures.update(True, set(), 2.1), 'arm_quit')
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
        self.assertEqual(decode_controls(payload(('r', 4))), (True, {('r', 4)}))

    def test_y_packet_bounce_does_not_quit(self):
        gestures = ButtonGestures()
        gestures.update(True, set(), 1)
        gestures.update(True, {('l', 5)}, 1.1)
        self.assertEqual(gestures.update(True, set(), 1.2), 'arm_quit')
        gestures.update(True, {('l', 5)}, 1.21)
        self.assertIsNone(gestures.update(True, set(), 1.22))


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
        for now in [2, 3, 4, 4.3]:
            self.session.on_payload(payload(), now)
            self.session.tick(now)
        self.assertEqual([c for c, _ in self.sent], ['start'])
        self.assertEqual(self.session.state, 'idle')
        self.sent[0][1].set_result(self.receipt('start'))
        self.session.tick(4.4)
        self.assertEqual(self.session.state, 'recording')
        self.assertIn('开始录制', self.speech)

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
        self.session.on_payload(payload(('r', 4)), 1.1)
        self.session.on_payload('{}', 1.2)
        self.session.on_payload(payload(), 1.3)
        self.assertIsNone(self.session.start_at)

    def test_b_or_x_cancels_countdown_without_sending(self):
        for command in ('save', 'discard'):
            self.session.last_input = 1
            self.session.command('start', 1)
            self.session.command(command, 2)
            self.assertIsNone(self.session.start_at)
            self.assertFalse(self.sent)

    def test_save_wait_keeps_buttons_responsive_and_never_queues_start(self):
        self.session.state = 'recording'
        self.session.command('save', 1)
        self.tap(('r', 4), 2)
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


class LauncherHelpTest(unittest.TestCase):
    launcher = Path(__file__).resolve().parents[1] / 'start_lerobot_official_collect.sh'

    def test_vr_parameter_help_works_without_ros(self):
        for name in ('VR_CONTROL', 'VR_SPEECH', 'VR_START_DELAY_SEC'):
            result = subprocess.run(['bash', str(self.launcher), name, '--help'], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn(name, result.stdout)


if __name__ == '__main__':
    unittest.main()
