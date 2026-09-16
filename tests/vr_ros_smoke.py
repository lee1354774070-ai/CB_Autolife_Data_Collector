#!/usr/bin/env python3
"""Opt-in ROS transport smoke test with a fake recorder and private topic names.

Run with a ROS-enabled Python. No camera is opened, no joint/action command is
published, no existing dataset is touched, and TTS goes to a test-only topic.
This validates ROS -> gestures -> FIFO -> receipt -> TTS, not physical buttons
or LeRobot persistence. It is deliberately outside automatic unittest discovery.
"""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.create_node('collector_vr_transport_test')
    prefix = f'/collector_test_{os.getpid()}'
    publisher = node.create_publisher(String, prefix + '/input', 10)
    speech = []
    node.create_subscription(String, prefix + '/tts', lambda msg: speech.append(json.loads(msg.data)), 10)
    received = []
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
                                       '--start-delay', '0.2', '--command-timeout', '5'])

            def packet(faces=()):
                data = {h: {'b': [{'p': False} for _ in range(6)]} for h in ('l', 'r')}
                for hand in data:
                    data[hand]['b'][1]['p'] = True
                for hand, index in faces:
                    data[hand]['b'][index]['p'] = True
                return String(data=json.dumps(data))

            def pump(seconds, faces=()):
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    if helper.poll() is not None:
                        raise RuntimeError(f'VR helper exited unexpectedly: {helper.returncode}')
                    publisher.publish(packet(faces))
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
                        status = dict(event=command, success=True, request_id=parts[1],
                                      wall_time=time.time(), frames=60, episode_index=0,
                                      total_saved_episodes=int(command == 'save'), session_saved_episodes=int(command == 'save'))
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
            tap('r', 5)  # save
            tap('r', 4)  # start
            tap('l', 4)  # discard
            tap('l', 5)  # arm quit, must not quit yet
            assert received == ['start', 'save', 'start', 'discard'], received
            tap('l', 5)
            assert received == ['start', 'save', 'start', 'discard', 'quit'], received
            spoken = [item.get('text', '') for item in speech]
            assert any('保存成功' in item for item in spoken), spoken
            assert any('已丢弃' in item for item in spoken), spoken
            print('ROS_VR_SMOKE_PASS: start/save/start/discard/double-Y quit; test-topic speech received', flush=True)
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
