#!/usr/bin/env python3
"""Real attachment client/RPC test in isolated DDS domain, with a fake host.

No policy, recorder, controller, vendor topics or hardware services are loaded.
The fake host records requested actions in memory only.
"""
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import tempfile
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from dagger.compat import CommandGate, FEATURES, SERVICE


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '211':
        raise SystemExit('Requires isolated ROS_DOMAIN_ID=211; never run on the robot control domain')
    import rclpy
    from rcl_interfaces.msg import SetParametersResult
    from rcl_interfaces.srv import SetParametersAtomically
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('collector_attachment_test_host')
    gate, calls = CommandGate(), []
    state = {}
    pub = node.create_publisher(String, '/hg_dagger/control_state', 10)
    node.create_timer(.1, lambda: pub.publish(String(data=json.dumps(state))))

    def command(request, response):
        def execute(action):
            calls.append(action)
            return True, 'fake host queued action; no hardware'
        ok, message = gate.dispatch(request.parameters[0].value.string_value, execute)
        response.result = SetParametersResult(successful=ok, reason=message)
        return response

    node.create_service(SetParametersAtomically, SERVICE, command)
    stop = threading.Event()
    def spin():
        while not stop.is_set():
            rclpy.spin_once(node, timeout_sec=.02)
    thread = threading.Thread(target=spin)
    thread.start()
    child = None
    try:
        with tempfile.TemporaryDirectory(prefix='attach-smoke-') as directory:
            state.update(collector_fifo=str(Path(directory) / 'task/.official_recording_control'),
                         mode='DISARMED', authority_epoch=0, collector_compat=dict(
                             version=1, instance_id=gate.instance_id, features=FEATURES,
                             robot_id='300', domain_id='211', command_service=SERVICE,
                             hardware_publish=True, task_text='test', with_depth=False))
            env = {**os.environ, 'OUTPUT_BASE_DIR': directory, 'TASK_TEXT': 'test',
                   'DAGGER_PUBLISH': '1', 'WITH_DEPTH': '0', 'ROBOT_ID': '300'}
            command_line = [sys.executable, '-u', str(ROOT / 'dagger/attach.py'), 'task']
            checked = subprocess.run([*command_line, '--check'], env=env, capture_output=True, text=True, timeout=12)
            assert checked.returncode == 0, checked.stderr
            assert calls == []
            # Use OS reads: TextIO buffering can hide already-buffered lines
            # from select(), making a healthy child look stalled.
            child = subprocess.Popen(command_line, env=env, stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            output = b''
            def wait_for(fragment):
                nonlocal output
                deadline = time.monotonic() + 12
                while fragment not in output and time.monotonic() < deadline:
                    if select.select([child.stdout], [], [], .1)[0]:
                        chunk = os.read(child.stdout.fileno(), 65536)
                        if not chunk:
                            break
                        output += chunk
                assert fragment in output, output.decode(errors='replace')
            wait_for(b'Commands enabled.')
            child.stdin.write(b'c')
            child.stdin.flush()
            wait_for(b'Accepted:')
            assert calls == ['start'], calls
            child.stdin.write(b'q')
            child.stdin.flush()
            assert child.wait(timeout=5) == 0
            assert calls == ['start'], 'Detach must not stop/reset the host'
            # A legacy publisher must never cause automatic owned-stack fallback.
            del state['collector_compat']
            checked = subprocess.run([*command_line, '--check'], env=env, capture_output=True, text=True, timeout=12)
            assert checked.returncode != 0
            assert 'no compatible attachment contract' in checked.stderr
            assert calls == ['start']
            print('ATTACH_ROS_SMOKE_PASS preflight=true command_once=true detach_preserves_host=true legacy_rejected=true hardware_calls=0')
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
            child.wait(timeout=5)
        stop.set()
        thread.join(timeout=3)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
