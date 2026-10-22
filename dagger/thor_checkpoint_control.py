#!/usr/bin/env python3
"""Thor forced-SSH command: list or switch validated local baseline checkpoints.

Install as a dedicated authorized_keys forced command, not a general shell key.
Only manages groot-n1.7:8777; never publishes robot actions.
"""
import fcntl
import json
import os
from pathlib import Path
import re
import struct
import subprocess
import sys
import time
import urllib.request
import uuid

ROOT = Path('/home/wayne/groot_n1_7_runtime')
MODELS = ROOT / 'models'
CONTAINER = 'groot-n1.7'


def request(endpoint, payload=None):
    token = (ROOT / 'secrets/server.token').read_text().strip()
    req = urllib.request.Request('http://127.0.0.1:8777' + endpoint,
        data=None if payload is None else json.dumps(payload).encode(),
        headers={'Authorization': 'Bearer ' + token, 'Content-Type': 'application/json'})
    with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(req, timeout=15) as response:
        return json.load(response)


def model_path(name):
    if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,127}', name):
        raise ValueError('Invalid checkpoint name')
    path = MODELS / name
    if path.is_symlink() or not path.is_dir() or path.resolve().parent != MODELS.resolve():
        raise ValueError('Select a checkpoint directory directly under Thor/models')
    contract = json.loads((path / 'groot_deployment_contract.json').read_text())
    reference = json.loads((MODELS / 'baseline/groot_deployment_contract.json').read_text())
    for key in ('policy_type', 'model_version', 'state_feature_names', 'action_feature_names',
                'image_features', 'chunk_size', 'n_action_steps'):
        if key not in reference or contract.get(key) != reference[key]:
            raise ValueError('Checkpoint contract mismatch: ' + key)
    if len(contract['action_feature_names']) != 21:
        raise ValueError('Only the current 21-D baseline contract is supported')
    for filename in ('config.json', 'policy_preprocessor.json', 'policy_postprocessor.json'):
        config = json.loads((path / filename).read_text())
        if not isinstance(config, dict):
            raise ValueError('Invalid processor/config JSON')
        if filename.startswith('policy_'):
            for step in config.get('steps', []):
                if 'state_file' in step:
                    state_file = step['state_file']
                    if not isinstance(state_file, str) or Path(state_file).name != state_file:
                        raise ValueError('Invalid processor state_file')
                    state = path / state_file
                    if state.resolve().parent != path.resolve():
                        raise ValueError('Processor state must be inside the checkpoint')
                    check_tensor_file(state)
    check_tensor_file(path / 'model.safetensors')
    return path


def check_tensor_file(weights):
    # Check tensor offsets without reading gigabytes of weights into host RAM.
    with weights.open('rb') as stream:
        raw = stream.read(8)
        if len(raw) != 8:
            raise ValueError('Incomplete safetensors file')
        size = struct.unpack('<Q', raw)[0]
        if not 0 < size <= 100_000_000:
            raise ValueError('Invalid safetensors header')
        header = json.loads(stream.read(size))
    offsets = [value['data_offsets'] for key, value in header.items() if key != '__metadata__']
    if not offsets or max(end for _, end in offsets) + size + 8 != weights.stat().st_size:
        raise ValueError('Incomplete checkpoint tensor data')


def installed():
    models = []
    for path in sorted(MODELS.iterdir()):
        if path.is_symlink() or not path.is_dir() or path.name.startswith('.'):
            continue
        try:
            model_path(path.name)
        except (ValueError, OSError, KeyError, TypeError):
            continue
        models.append(path.name)
    return models


def snapshot():
    health = request('/health')
    if health.get('mode') != 'policy_only_baseline' or health.get('robot_io') is not False:
        raise ValueError('The service is not the expected read-only baseline inference server')
    current = health.get('checkpoint')
    if not isinstance(current, str) or not current:
        raise ValueError('Server update required: loaded checkpoint identity is unavailable')
    return {'checkpoint': current, 'health': health, 'models': installed()}


def start_model(path, log):
    command = ['sudo', '-n', 'env', 'GROOT_BASELINE_MODEL_DIR=' + str(path),
        'bash', str(ROOT / 'repo/deploy/groot_n1_7/thor/manage.sh'), 'switch', 'baseline',
        '--gripper-phase-aware', '--max-action-retries', '0', '--max-gripper-step', '20']
    with log.open('ab') as output:
        subprocess.run(command, stdout=output, stderr=subprocess.STDOUT, check=True, timeout=7240)
    result = snapshot()
    if (result['checkpoint'] != path.name or result['health'].get('ready') is not True
            or result['health'].get('gripper_mode') != 'phase_aware'):
        raise RuntimeError('Loaded model/phase readiness verification failed')
    return result


def switch(name):
    path = model_path(name)
    with (ROOT / '.collector_checkpoint.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('Another checkpoint switch is in progress') from None
        old = snapshot()
        if old['health'].get('active_session') is not False or old['health'].get('ready') is not True:
            raise RuntimeError('Stop the active inference client before switching checkpoint')
        if name == old['checkpoint']:
            return old
        previous = model_path(old['checkpoint'])
        if not (MODELS / 'baseline').is_symlink():
            raise RuntimeError('baseline is not a symlink; refusing to overwrite model data')
        # /start shares this lock on the server: no new session can race shutdown.
        lease = request('/prepare_reload', {})['lease']
        logs = ROOT / 'logs'
        logs.mkdir(exist_ok=True)
        log = logs / ('checkpoint_' + time.strftime('%Y%m%d_%H%M%S') + '.log')
        try:
            result = start_model(path, log)
        except Exception as error:
            # Never interrupt a session on a replacement service, even on rollback.
            try:
                health = request('/health')
            except OSError:
                health = None
            if health is None:
                running = subprocess.run(['sudo', '-n', 'docker', 'inspect', '-f', '{{.State.Running}}', CONTAINER],
                    text=True, capture_output=True, check=True, timeout=10).stdout.strip()
                if running != 'false':
                    raise RuntimeError(f'Load status unknown; refusing to interrupt a running container. Log: {log}') from error
            if health is not None and health.get('active_session') is not False:
                raise RuntimeError(f'Load failed; rollback blocked by active/unknown session. Log: {log}') from error
            if health is not None:
                if health.get('ready') is True:
                    request('/prepare_reload', {})
                else:
                    # This may still be the original leased process.
                    try:
                        request('/cancel_reload', {'lease': lease})
                        request('/prepare_reload', {})
                    except OSError:
                        raise RuntimeError(f'Rollback could not reserve service. Log: {log}') from error
            try:
                start_model(previous, log)
            except Exception as rollback:
                raise RuntimeError(f'Load and rollback failed; do not start inference. Log: {log}') from rollback
            raise RuntimeError(f'Load failed; restored {previous.name}. Log: {log}') from error
        temporary = MODELS / ('.baseline-' + uuid.uuid4().hex)
        try:
            temporary.symlink_to(path.name)
            temporary.replace(MODELS / 'baseline')
        finally:
            temporary.unlink(missing_ok=True)
        return result


def main():
    try:
        payload = json.loads(sys.stdin.read(4097))
        if not isinstance(payload, dict) or set(payload) - {'action', 'checkpoint'}:
            raise ValueError('Invalid checkpoint request')
        action = payload.get('action')
        if action == 'list':
            result = snapshot()
        elif action == 'switch':
            result = switch(payload.get('checkpoint'))
        else:
            raise ValueError('Only list/switch operations are allowed')
        print(json.dumps(result))
    except Exception as error:
        print(json.dumps({'error': str(error)}))
        raise SystemExit(1)


if __name__ == '__main__':
    main()
