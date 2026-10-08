#!/usr/bin/env python3
"""Opt-in ROS transport smoke test with a fake recorder and private topic names.

Run with a ROS-enabled Python. No camera is opened, no joint/action command is
published, no existing dataset is touched, and TTS goes to a test-only topic.
This validates ROS -> gestures -> FIFO -> receipt -> TTS, not physical buttons
or LeRobot persistence. It is deliberately outside automatic unittest discovery.
"""

import json
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--subtasks', action='store_true', help='Test short-B marks, final autosave and long-B early save.')
    parser.add_argument('--v4-input', action='store_true', help='Use actual V4 numeric face-button packets.')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('collector_vr_transport_test')
    prefix = f'/collector_test_{os.getpid()}'
    publisher = node.create_publisher(String, prefix + '/input', 10)
    speech = []
    node.create_subscription(String, prefix + '/tts', lambda msg: speech.append(json.loads(msg.data)), 10)
    received = []
    feedback = []
    reset_calls = []
    reset_started = None
    controller_enabled = False
    controller_pub = node.create_publisher(String, prefix + '/controller/status', 10)
    node.create_subscription(String, prefix + '/feedback',
                             lambda msg: feedback.append(json.loads(msg.data)['event']), 10)

    def enable(request, response):
        nonlocal controller_enabled, reset_started
        reset_calls.append(('enable', request.data))
        controller_enabled = request.data
        if not request.data:
            reset_started = None
        response.success = True
        return response

    def reset(request, response):
        nonlocal reset_started, controller_enabled
        reset_calls.append(('reset', None))
        controller_enabled = True
        reset_started = time.monotonic()
        response.success = True
        return response

    node.create_service(SetBool, prefix + '/controller/set_hardware_enabled', enable)
    node.create_service(Trigger, prefix + '/controller/full_body_reset', reset)
    helper = None
    fd = None
    try:
        with tempfile.TemporaryDirectory(prefix='collector_vr_smoke_') as directory:
            root = Path(directory)
            (root / '.official_recording_pids').write_text(f'recorder {os.getpid()} fake\n')
            fifo = root / '.official_recording_control'
            os.mkfifo(fifo)
            fd = os.open(fifo, os.O_RDWR | os.O_NONBLOCK)
            script = Path(__file__).resolve().parents[1] / 'vr_collector_control.py'
            helper = subprocess.Popen([sys.executable, '-u', str(script), '--base-dir', str(root),
                                       '--topic', prefix + '/input', '--tts-topic', prefix + '/tts',
                                       '--start-delay', '0.2', '--command-timeout', '5',
                                       '--motion-lock-file', str(root / 'motion.lock'),
                                       '--reset-prefix', prefix + '/controller', '--feedback-topic', prefix + '/feedback']
                                      + (['--subtask-mode', '--a-long-press-sec', '1'] if args.subtasks else []))
            confirmed = 0

            def packet(faces=(), grips=True):
                if args.v4_input:
                    data = {'leftController': {'gripActive': grips, 'xButton': 0, 'yButton': 0},
                            'rightController': {'gripActive': grips, 'aButton': 0, 'bButton': 0}}
                    for hand, index in faces:
                        name = 'leftController' if hand == 'l' else 'rightController'
                        field = ('xButton', 'yButton') if hand == 'l' else ('aButton', 'bButton')
                        data[name][field[index - 4]] = 1
                    return String(data=json.dumps(data))
                data = {h: {'b': [{'p': False} for _ in range(6)]} for h in ('l', 'r')}
                for hand in data:
                    data[hand]['b'][1]['p'] = grips
                for hand, index in faces:
                    data[hand]['b'][index]['p'] = True
                return String(data=json.dumps(data))

            def pump(seconds, faces=(), grips=True):
                nonlocal confirmed
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    if helper.poll() is not None:
                        raise RuntimeError(f'VR helper exited unexpectedly: {helper.returncode}')
                    publisher.publish(packet(faces, grips))
                    pending = reset_started is not None and time.monotonic() - reset_started < .3
                    controller_pub.publish(String(data=json.dumps({
                        'hardware_enabled': controller_enabled,
                        'hardware_enable_pending': pending,
                        'hardware_ready': controller_enabled and not pending,
                        'state': 'ARMED' if controller_enabled else 'DISARMED'})))
                    rclpy.spin_once(node, timeout_sec=.02)
                    try:
                        data = os.read(fd, 4096).decode()
                    except BlockingIOError:
                        data = ''
                    for line in data.splitlines():
                        parts = line.split()
                        command = parts[0]
                        received.append(command)
                        if command == 'quit':
                            continue
                        event = command
                        if command == 'start':
                            confirmed = 0
                        elif command == 'mark_subtask':
                            confirmed += 1
                            if confirmed == 3:
                                event = 'save'
                        status = dict(event=event, success=True, request_id=parts[1],
                                      wall_time=time.time(), frames=60, episode_index=0,
                                      total_saved_episodes=int(event == 'save'), session_saved_episodes=int(event == 'save'),
                                      subtasks=dict(enabled=args.subtasks, confirmed=confirmed, total=3,
                                                    complete=confirmed == 3, next_subtask='next' if confirmed < 3 else None))
                        temporary = root / 'status.tmp'
                        temporary.write_text(json.dumps(status))
                        temporary.replace(root / '.official_recording_status.json')
                    time.sleep(.02)

            def tap(hand, index):
                pump(.15)
                pump(.15, [(hand, index)])
                pump(.55)

            deadline = time.monotonic() + 10
            while node.count_subscribers(prefix + '/input') == 0 and time.monotonic() < deadline:
                pump(.1)
            assert node.count_subscribers(prefix + '/input'), 'VR input subscription not discovered'
            pump(1)
            tap('r', 4)  # start
            if args.subtasks:
                tap('r', 5)
                tap('r', 5)
                tap('r', 5)  # final mark -> save receipt
                tap('r', 4)  # start again
                pump(.1)
                pump(1.2, [('r', 5)])
                pump(.6)  # long-B early save
            else:
                tap('r', 5)  # save
            tap('r', 4)  # start
            tap('l', 5)  # discard
            expected = (['start', 'mark_subtask', 'mark_subtask', 'mark_subtask', 'start', 'save', 'start', 'discard']
                        if args.subtasks else ['start', 'save', 'start', 'discard'])
            assert received == expected, received
            assert not reset_calls, 'B/Y unexpectedly requested motion'
            tap('l', 4)  # X only resets; waits for Grip release without any recorder command.
            assert received == expected, received
            assert not reset_calls, 'reset requested before Grip release'
            pump(1.5, grips=False)
            assert reset_calls == [('enable', False), ('reset', None), ('enable', False)], reset_calls
            assert 'resetting' in feedback and 'reset' in feedback, feedback
            assert all(name in feedback for name in ('start', 'saving', 'save', 'discard')), feedback
            if args.subtasks:
                assert 'mark' in feedback, feedback
            spoken = [item.get('text', '') for item in speech]
            assert any(('已保存' if args.subtasks else '保存成功') in item for item in spoken), spoken
            if args.subtasks:
                assert spoken.count('下一步') == 2, spoken
                assert '已保存，标注未完成' in spoken, spoken
            assert any('已丢弃' in item for item in spoken), spoken
            assert '复位完成' in spoken, spoken
            print('ROS_VR_SMOKE_PASS: A/B/Y/X; fake reset disable->reset->confirm->disable; '
                  f'feedback={feedback}; hardware_calls=0', flush=True)
    finally:
        if helper is not None and helper.poll() is None:
            helper.send_signal(signal.SIGINT)
            try:
                helper.wait(timeout=5)
            except subprocess.TimeoutExpired:
                helper.kill()
                helper.wait()
                raise RuntimeError('VR helper did not exit promptly')
        if fd is not None:
            os.close(fd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
