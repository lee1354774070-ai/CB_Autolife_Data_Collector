#!/usr/bin/env python3
"""Forced-SSH checkpoint management for the native S3 :8783 service only."""
import fcntl
import json
import re
from pathlib import Path
import subprocess
import sys
import time
import urllib.request

from dagger.thor_checkpoint_control import model_path

ROOT = Path('/home/wayne/groot_n1_7_runtime')
MODELS = ROOT / 'models'
REFERENCE = 'hotel_trash_clean233_020000'
TOKEN = ROOT / 's3_robot340/preview.token'
CONTAINER = 'groot-s3-340-trash020000-test'
IMAGE = 'autolife/groot-n1.7-thor:0.6.1'
DEPLOY = Path(__file__).resolve().parents[1]


def validate_model(path):
    path = Path(path)
    checked = model_path(path.name, models=path.parent, reference=path.parent / REFERENCE)
    config = json.loads((checked / 'config.json').read_text())
    contract = json.loads((checked / 'groot_deployment_contract.json').read_text())
    if (config.get('type') != 'groot' or config.get('chunk_size') != 40
            or config.get('input_features', {}).get('observation.state', {}).get('shape') != [21]
            or config.get('output_features', {}).get('action', {}).get('shape') != [21]):
        raise ValueError('S3 requires a GR00T 21-D chunk=40 checkpoint')
    for key, shape in contract['image_features'].items():
        if config['input_features'].get(key, {}).get('shape') != shape:
            raise ValueError('Checkpoint image contract mismatch: ' + key)
    return checked


def request(endpoint, payload=None):
    req = urllib.request.Request('http://192.168.8.224:8783' + endpoint,
        data=None if payload is None else json.dumps(payload).encode(),
        headers={'Authorization': 'Bearer ' + TOKEN.read_text().strip(), 'Content-Type': 'application/json'})
    with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(req, timeout=15) as response:
        return json.load(response)


def snapshot():
    health = request('/health')
    if (health.get('mode') != 'policy_only_baseline' or health.get('robot_io') is not False
            or health.get('robot_profile') != 'autolife_s3_v3_1' or health.get('robot_id') != 340
            or health.get('gripper_mode') != 'binary330' or not health.get('checkpoint')):
        raise ValueError('8783 is not the expected S3 binary330 service')
    models = []
    for path in sorted(MODELS.iterdir()):
        try:
            validate_model(path)
        except (ValueError, OSError, KeyError, TypeError):
            continue
        models.append(path.name)
    return {'checkpoint': health['checkpoint'], 'health': health, 'models': models}


def docker(*args):
    return subprocess.run(['sudo', '-n', 'docker', *args], text=True,
                          capture_output=True, check=True, timeout=30).stdout.strip()


def current_container():
    item = json.loads(docker('inspect', CONTAINER))[0]
    mounts = {v['Destination']: v['Source'] for v in item['Mounts']}
    command = item['Config']['Cmd']
    if (item['Config']['Image'] != IMAGE or '--port' not in command
            or command[command.index('--port') + 1] != '8783'
            or mounts.get('/run/s3.token') != str(TOKEN)):
        raise RuntimeError('Container identity differs; refusing to stop another service')
    return item


def start_model(path, log):
    base = '/data/models/models--nvidia--GR00T-N1.7-3B/snapshots/2fc962b973bccdd5d8ce4f67cc63b264d6886495'
    # The reviewed native server keeps physical decoding, receipt and ACK logic.
    command = ['sudo', '-n', 'docker', 'run', '-d', '--rm', '--name', CONTAINER,
        '--runtime=nvidia', '--network=host', '--ipc=host', '--memory=32g', '--memory-swap=34g',
        '-e', 'OMP_NUM_THREADS=4', '-e', 'OPENBLAS_NUM_THREADS=4', '-e', 'MKL_NUM_THREADS=4',
        '--ulimit', 'memlock=-1', '--ulimit', 'stack=67108864',
        '-e', 'PYTHONPATH=/workspace:/collector', '-e', 'HF_HOME=/cache/huggingface',
        '-e', 'HF_HUB_OFFLINE=1', '-e', 'TRANSFORMERS_OFFLINE=1',
        '-v', f'{ROOT}/repo:/workspace:ro', '-v', f'{DEPLOY}:/collector:ro',
        '-v', f'{DEPLOY}/s3:/s3:ro',
        '-v', f'{MODELS}:/models:ro',
        '-v', f'{ROOT}/base/GR00T-N1.7-3B:{base}:ro', '-v', f'{ROOT}/cache:/cache',
        '-v', f'{TOKEN}:/run/s3.token:ro', IMAGE,
        'python', '-u', '/s3/execution_server.py', '--model-dir', '/models/' + path.name,
        '--token-file', '/run/s3.token', '--host', '192.168.8.224', '--port', '8783',
        '--gripper-mode', 'binary330', '--head-pitch-max', '34.5']
    with log.open('ab') as output:
        subprocess.run(command, stdout=output, stderr=subprocess.STDOUT, check=True, timeout=30)
    deadline = time.monotonic() + 600
    while time.monotonic() < deadline:
        try:
            result = snapshot()
        except (OSError, ValueError):
            time.sleep(2)
            continue
        if result['checkpoint'] != path.name:
            raise RuntimeError('Actual checkpoint differs from selection')
        if result['health'].get('ready') is True:
            return result
        time.sleep(2)
    raise RuntimeError('Model readiness timed out; inspect container logs')


def switch(name):
    if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,127}', name):
        raise ValueError('Invalid checkpoint name')
    path = validate_model(MODELS / name)
    # validate_model rejects nested, hidden and symlink paths; additionally bind
    # the request to this exact repository, never user supplied filesystem roots.
    if path.parent != MODELS or path.name != name:
        raise ValueError('Select a checkpoint directly under Thor/models')
    with (ROOT / '.collector_s3_checkpoint.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('Another S3 checkpoint switch is in progress') from None
        old = snapshot()
        if old['health'].get('active_session') is not False or old['health'].get('ready') is not True:
            raise RuntimeError('Stop the active S3 inference session before switching')
        if name == old['checkpoint']:
            return old
        previous = validate_model(MODELS / old['checkpoint'])
        item = current_container()
        lease = request('/prepare_reload', {})['lease']
        try:
            docker('stop', '--time', '10', item['Id'])
        except Exception:
            # This only releases our lease; it cannot start/stop any session.
            try:
                request('/cancel_reload', {'lease': lease})
            except OSError:
                pass
            raise
        log = ROOT / 'logs' / ('s3_checkpoint_' + time.strftime('%Y%m%d_%H%M%S') + '.log')
        log.parent.mkdir(exist_ok=True)
        try:
            return start_model(path, log)
        except Exception as error:
            # Roll back only a definitively absent/exited replacement. An alive
            # but unreachable service may own a session and must be preserved.
            result = subprocess.run(['sudo', '-n', 'docker', 'ps', '-aq', '--filter', 'name=^/' + CONTAINER + '$'],
                text=True, capture_output=True, check=True, timeout=15)
            if result.stdout.strip():
                running = current_container()
                if running['State']['Running']:
                    raise RuntimeError(f'Load unconfirmed; running service preserved. Log: {log}') from error
                docker('rm', running['Id'])
            try:
                start_model(previous, log)
            except Exception as rollback:
                raise RuntimeError(f'Load and rollback failed. Log: {log}') from rollback
            raise RuntimeError(f'Load failed; restored {previous.name}. Log: {log}') from error


def main():
    try:
        payload = json.loads(sys.stdin.read(4097))
        if not isinstance(payload, dict) or set(payload) - {'action', 'checkpoint'}:
            raise ValueError('Invalid request')
        if payload.get('action') == 'list':
            result = snapshot()
        elif payload.get('action') == 'switch':
            result = switch(payload.get('checkpoint'))
        else:
            raise ValueError('Only list/switch operations are allowed')
        print(json.dumps(result))
    except Exception as error:
        print(json.dumps({'error': str(error)}))
        raise SystemExit(1)


if __name__ == '__main__':
    main()
