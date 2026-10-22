"""Isolated ROS reset receipts; fake supervisor, no controller or model."""
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time

assert os.environ.get('ROS_DOMAIN_ID') == '211', 'requires isolated ROS_DOMAIN_ID=211'
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from dagger.runtime import reset_once

rclpy.init()
server = rclpy.create_node('reset_receipt_test')
client = rclpy.create_node('reset_once_test')
publisher = server.create_publisher(String, '/hg_dagger/control_state', 10)
started, calls, fail = None, 0, False

def request(_request, response):
    global started, calls
    started, calls = time.monotonic(), calls + 1
    response.success, response.message = True, 'accepted'
    return response

def status():
    busy = started is not None and time.monotonic() - started < .8
    packet = {'collector_fifo': os.environ['DAGGER_BASE_DIR'] + '/.official_recording_control',
              'operation_busy': busy, 'quick_reset': {'pending': busy},
              'notice': {'level':'error' if fail else 'success',
                         'text':'target not reached' if fail else 'all joints reached'}}
    publisher.publish(String(data=json.dumps(packet)))

server.create_service(Trigger, '/hg_dagger/launcher/reset', request)
server.create_timer(.05, status)
stop = threading.Event()
def spin():
    while not stop.is_set():rclpy.spin_once(server, timeout_sec=.05)
thread = threading.Thread(target=spin)
with tempfile.TemporaryDirectory() as directory:
    os.environ['DAGGER_BASE_DIR'] = directory
    thread.start()
    try:
        reset_once(client)
        assert calls == 1
        started, fail = None, True
        try:reset_once(client)
        except RuntimeError as exc:assert 'target not reached' in str(exc)
        else:raise AssertionError('failed reset reported success')
        assert calls == 2, 'no automatic retries'
        print('RESET_ONLY_ROS_PASS success_and_failure_receipts=2 hardware_calls=0')
    finally:
        stop.set();thread.join()
        client.destroy_node();server.destroy_node();rclpy.shutdown()
