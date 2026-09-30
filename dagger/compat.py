"""Versioned attachment contract for ONE already-running DAgger host.

This is operator coordination on the robot, not network authentication. An
instance nonce prevents a delayed command from controlling a restarted host.
No attachment creates a controller, recorder, policy client or motor publisher.
"""

from collections import OrderedDict
import json
from pathlib import Path
import time
import uuid

VERSION = 1
SERVICE = '/collector_dagger/operator_command'
COMMANDS = {'start', 'finish', 'discard', 'reset'}
FEATURES = ['immediate_takeover', 'epoch_fencing', 'single_controller']


def check_host(state, *, fifo, task_text=None, publish=False, depth=None, domain='0'):
    host = state.get('collector_compat', {})
    if (not isinstance(host, dict) or host.get('version') != VERSION
            or host.get('command_service') != SERVICE
            or not isinstance(host.get('instance_id'), str)
            or len(host['instance_id']) != 32
            or host.get('robot_id') != '300'
            or host.get('domain_id') != domain
            or not set(FEATURES).issubset(host.get('features', []))):
        raise ValueError('Existing host has no compatible attachment contract; install the compatible COPY once. No second stack started.')
    if state.get('collector_fifo') != str(Path(fifo).resolve()):
        raise ValueError('Existing host uses a different dataset/task; attach to its current task, do not start another recorder')
    if task_text is not None and host.get('task_text') != task_text:
        raise ValueError('TASK_TEXT differs from the running host')
    if depth is not None and host.get('with_depth') != depth:
        raise ValueError('WITH_DEPTH differs from the running host')
    if publish and host.get('hardware_publish') is not True:
        raise ValueError('Host was launched without hardware publishing; attachment cannot enable it')
    return host


class CommandGate:
    """Bounded idempotency cache. Duplicate/ambiguous RPCs never replay actions."""
    def __init__(self, instance_id=None):
        self.instance_id = instance_id or uuid.uuid4().hex
        self.results = OrderedDict()

    def dispatch(self, raw, execute):
        try:
            if len(raw) > 2048:
                raise ValueError('Command too large')
            packet = json.loads(raw)
            if packet.get('instance_id') != self.instance_id:
                raise ValueError('Host instance changed; reconnect before sending commands')
            request = packet.get('request_id')
            if not isinstance(request, str) or uuid.UUID(hex=request).hex != request:
                raise ValueError('Invalid request_id')
            action = packet.get('command')
            if action not in COMMANDS:
                raise ValueError('Unsupported command; Q only detaches the terminal')
            issued = packet.get('issued_ns')
            now = time.time_ns()
            if type(issued) is not int or not 0 <= now - issued <= 30_000_000_000:
                raise ValueError('Command timestamp expired or invalid; no replay')
            for key, (_, stamp, _) in list(self.results.items()):
                if now - stamp > 30_000_000_000:
                    del self.results[key]
            if request in self.results:
                previous, stamp, result = self.results[request]
                return result if (previous, stamp) == (action, issued) else (False, 'request_id reused with a different command')
            if len(self.results) >= 256:
                raise ValueError('Command rate exceeded; no cached command evicted')
        except (AttributeError, ValueError, TypeError) as exc:
            return False, str(exc)
        # Reserve before executing: even an exception after queueing is not
        # permission to repeat a physical action with this request ID.
        result = (False, 'Command outcome uncertain; inspect host state, do not replay')
        self.results[request] = (action, issued, result)
        try:
            result = execute(action)
        except Exception:
            pass  # The reserved uncertain result prevents repeating side effects.
        finally:
            self.results[request] = (action, issued, result)
        return result


class CompatibilityPublisher:
    def __init__(self, publisher, contract):
        self.publisher, self.contract = publisher, contract

    def publish(self, message):
        state = json.loads(message.data)
        state['collector_compat'] = self.contract
        self.publisher.publish(type(message)(data=json.dumps(state, separators=(',', ':'))))
