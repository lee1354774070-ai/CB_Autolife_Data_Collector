#!/usr/bin/env python3
"""Attach a terminal to a compatible running host. Never start/stop that host."""

import argparse
import json
import os
from pathlib import Path
import select
import sys
import termios
import time
import tty
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dagger.compat import SERVICE, check_host


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('task_name')
    parser.add_argument('task_text', nargs='?')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    import rclpy
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParametersAtomically
    from std_msgs.msg import String
    if os.environ.get('ROBOT_ID', '300') != '300':
        parser.error('Only robot 300 is supported')
    publish = os.environ.get('DAGGER_PUBLISH', '0')
    if publish not in ('0', '1'):
        parser.error('DAGGER_PUBLISH must be 0 or 1')
    depth = os.environ.get('WITH_DEPTH')
    if depth not in (None, '0', '1'):
        parser.error('WITH_DEPTH must be 0 or 1')
    if Path(args.task_name).name != args.task_name or args.task_name in ('', '.', '..'):
        parser.error('task_name must be a single directory name')
    fifo = Path(os.environ.get('OUTPUT_BASE_DIR', '/home/ubuntu/nas14')) / args.task_name / '.official_recording_control'
    expected_text = os.environ.get('TASK_TEXT', args.task_text)
    rclpy.init()
    node = rclpy.create_node('collector_dagger_attachment_' + str(os.getpid()))
    latest, received = {}, 0.

    def update(message):
        nonlocal latest, received
        try:
            value = json.loads(message.data)
            if isinstance(value, dict):
                latest, received = value, time.monotonic()
        except (ValueError, TypeError):
            pass

    node.create_subscription(String, '/hg_dagger/control_state', update, 1)
    client = node.create_client(SetParametersAtomically, SERVICE)
    terminal = None
    graph_at, publisher_count = 0., 0
    try:
        deadline = time.monotonic() + 6
        while rclpy.ok() and time.monotonic() < deadline and not latest:
            rclpy.spin_once(node, timeout_sec=.1)
        def host():
            nonlocal graph_at, publisher_count
            if not latest or time.monotonic() - received > 1.:
                raise RuntimeError('No fresh host state; no command sent and no replacement stack started')
            if time.monotonic() - graph_at > .5:
                publisher_count = node.count_publishers('/hg_dagger/control_state')
                graph_at = time.monotonic()
            if publisher_count != 1:
                raise RuntimeError('Expected exactly one authority publisher; attachment refused')
            return check_host(latest, fifo=fifo, task_text=expected_text,
                              publish=publish == '1', depth=None if depth is None else depth == '1',
                              domain=os.environ.get('ROS_DOMAIN_ID', '0'))
        identity = host()['instance_id']
        if not client.wait_for_service(timeout_sec=2):
            raise RuntimeError('Compatible command endpoint unavailable')
        print(f"Attached preflight passed: task={args.task_name}, host={identity}, no processes replaced.", flush=True)
        if args.check:
            return
        print('C=start A=save X/D=discard R=reset Q=detach. Host and VR stay running after detach.', flush=True)
        print('Commands enabled.' if publish == '1' else 'Read-only attachment: DAGGER_PUBLISH=0.', flush=True)
        if sys.stdin.isatty():
            terminal = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        pending = None
        sent = 0.
        uncertain = False
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.02)
            if host()['instance_id'] != identity:
                raise RuntimeError('Host restarted; reconnect explicitly. No command replayed.')
            if pending is not None:
                if pending.done():
                    result = pending.result().result
                    print(('Accepted: ' if result.successful else 'Rejected: ') + result.reason, flush=True)
                    pending = None
                elif time.monotonic() - sent > 5:
                    print('Command outcome unknown. Further commands disabled; inspect host before reconnecting.', flush=True)
                    pending, uncertain = None, True
            if not select.select([sys.stdin], [], [], 0)[0]:
                continue
            key = sys.stdin.read(1).lower()
            if key in ('', 'q'):
                break
            action = {'c': 'start', 'a': 'finish', 'x': 'discard', 'd': 'discard', 'r': 'reset'}.get(key)
            if action and publish == '1' and pending is None and not uncertain:
                packet = json.dumps(dict(instance_id=identity, request_id=uuid.uuid4().hex,
                                         issued_ns=time.time_ns(), command=action))
                request = SetParametersAtomically.Request(parameters=[Parameter(
                    name='collector_command', value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=packet))])
                pending = client.call_async(request)
                sent = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        if terminal is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
