#!/usr/bin/env python3
"""Create and verify an isolated Collector environment, without hardware control.

Requires an existing Conda installation and ROS 2 Jazzy on Linux x86_64. The
installer never changes drivers, system packages, services or an existing
unmanaged environment. --check only imports dependencies, probes JPEG decode
and round-trips a tiny synthetic dataset in a temporary directory.
"""
from __future__ import annotations

import argparse
import fcntl
import json
import os
from pathlib import Path
import platform
import re
import shlex
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = (
    'lerobot==0.6.0', 'datasets==4.8.5', 'pyarrow==25.0.0', 'av==15.1.0',
    'numpy==2.2.6', 'opencv-python-headless==4.13.0.92',
)
PROTECTED = {'base', 'lerobot', 'robot_env', 'cuda_env'}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--name', default='collector', help='New Conda environment name (default: collector).')
    parser.add_argument('--backend', choices=('auto', 'cpu', 'nvjpeg'), default='auto',
                        help='auto detects NVIDIA hardware; nvjpeg requires it; cpu installs CPU torch.')
    parser.add_argument('--clone', help='Optional existing Conda environment name or absolute prefix to clone; never modified.')
    parser.add_argument('--conda', type=Path, help='Conda executable; otherwise use CONDA_EXE, PATH or common home installations.')
    parser.add_argument('--ros-setup', type=Path, default=Path('/opt/ros/jazzy/setup.bash'),
                        help='Existing Jazzy setup.bash; ROS and drivers are not installed.')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--resume', action='store_true', help='Resume only an incomplete install owned by this script, with the same options.')
    mode.add_argument('--check', action='store_true', help='Check an existing environment without installing or configuring packages.')
    mode.add_argument('--dry-run', action='store_true', help='Print the installation plan without writing files or installing packages.')
    parser.add_argument('--_probe', choices=('cpu', 'nvjpeg'), help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', args.name):
        parser.error('--name must be a simple environment name, not a path')
    if args.name in PROTECTED and not args.check:
        parser.error('Use a new environment name; base/lerobot/robot_env/cuda_env are protected')
    return args


def conda_executable(explicit=None):
    candidates = ([explicit] if explicit else [os.environ.get('CONDA_EXE'), shutil.which('conda'),
                  Path.home() / 'miniconda3/bin/conda', Path.home() / 'anaconda3/bin/conda'])
    for value in candidates:
        if value and Path(value).is_file() and os.access(value, os.X_OK):
            return Path(value).resolve()
    raise RuntimeError('Conda was not found. Install Miniconda first or pass --conda /path/bin/conda.')


def choose_backend(requested):
    if requested == 'cpu':
        return 'cpu'
    try:
        result = subprocess.run(['nvidia-smi', '--query-gpu=name', '--format=csv,noheader'],
                                capture_output=True, text=True, timeout=10, check=False)
        available = result.returncode == 0 and bool(result.stdout.strip())
    except (OSError, subprocess.TimeoutExpired):
        available = False
    if requested == 'nvjpeg' and not available:
        raise RuntimeError('NVIDIA GPU/driver is not available; no packages were changed.')
    return 'nvjpeg' if available else 'cpu'


def run(command, *, env=None):
    print('+ ' + shlex.join(map(str, command)), flush=True)
    return subprocess.run(list(map(str, command)), check=True, env=env)


def atomic_json(path, payload):
    # Keep install ownership even after interruption, without a partial JSON.
    fd, temporary = tempfile.mkstemp(prefix=path.name + '.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(payload, stream, indent=2)
            stream.write('\n')
        os.replace(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)


def install_commands(conda, prefix, backend, source):
    flavor = 'cu126' if backend == 'nvjpeg' else 'cpu'
    torch_pins = [f'torch==2.7.1+{flavor}', f'torchvision==0.22.1+{flavor}']
    create = [conda, 'create', '-y', '-p', prefix]
    create += ['--clone', source, '--offline'] if source else ['python=3.12', 'pip', '--no-default-packages']
    pip = [prefix / 'bin/python', '-m', 'pip', 'install', '--no-cache-dir', '--disable-pip-version-check']
    return [create,
            pip + torch_pins + ['--index-url', f'https://download.pytorch.org/whl/{flavor}'],
            pip + list(PACKAGES) + torch_pins]


def check_environment(prefix, backend, ros_setup):
    env = os.environ.copy()
    for name in ('PYTHONHOME', 'PYTHONPATH', 'CYCLONEDDS_URI'):
        env.pop(name, None)
    env.update(PYTHONDONTWRITEBYTECODE='1', HF_HUB_OFFLINE='1', HF_DATASETS_OFFLINE='1',
               ROS_DOMAIN_ID='211', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    run([prefix / 'bin/python', '-m', 'pip', 'check'], env=env)
    # Source ROS only inside this subprocess; no rclpy.init, subscriptions,
    # publishers or SHM reads are performed by the probe.
    run(['bash', '-c', 'set -e\nset +u\nsource "$1"\n'
         'export LD_LIBRARY_PATH="$2/lib:${LD_LIBRARY_PATH:-}"\n'
         'exec "$2/bin/python" "$3" --_probe "$4"',
         'collector-env-check', ros_setup, prefix, Path(__file__).resolve(), backend], env=env)


def probe(backend):
    from importlib.metadata import version
    import logging
    import av
    import numpy as np
    import pyarrow.parquet as pq
    import torch
    sys.path.insert(0, str(ROOT))
    import record_lerobot_official as recorder  # Imports ROS and the real recorder dependencies; no node created.
    from jpeg_decoding import select_jpeg_decoder
    if sys.version_info[:2] != (3, 12):
        raise RuntimeError('ROS 2 Jazzy requires this installer to use Python 3.12')
    logging.basicConfig(level=logging.INFO)
    decoder = select_jpeg_decoder('nvjpeg' if backend == 'nvjpeg' else 'opencv',
                                  'cuda:0', logging.getLogger('collector-setup'))
    features = {
        'observation.state': {'dtype': 'float32', 'shape': (1,), 'names': ['joint']},
        'action': {'dtype': 'float32', 'shape': (1,), 'names': ['joint']},
        'observation.images.probe': {'dtype': 'video', 'shape': (32, 32, 3),
                                     'names': ['height', 'width', 'channels']},
    }
    with tempfile.TemporaryDirectory(prefix='collector-install-probe-') as temporary:
        root = Path(temporary) / 'dataset'
        dataset = recorder.LeRobotDataset.create('local/install_probe', fps=30, root=root, features=features,
                         rgb_encoder=recorder.RGBEncoderConfig(vcodec='h264'), image_writer_threads=0)
        try:
            for i in range(3):
                dataset.add_frame({'observation.state': np.array([i], dtype=np.float32),
                                   'action': np.array([i], dtype=np.float32),
                                   'observation.images.probe': np.full((32, 32, 3), 60 + i, dtype=np.uint8),
                                   'task': 'Synthetic installation probe, not training data'})
            dataset.save_episode(parallel_encoding=False)
        finally:
            dataset.finalize()
        rows = sum(pq.read_metadata(p).num_rows for p in (root / 'data').rglob('*.parquet'))
        paths = list((root / 'videos').rglob('*.mp4'))
        if rows != 3 or len(paths) != 1:
            raise RuntimeError('LeRobot dataset probe produced incomplete metadata/video')
        with av.open(str(paths[0])) as container:
            if sum(1 for _ in container.decode(video=0)) != 3:
                raise RuntimeError('Encoded probe video did not decode to three frames')
    print(json.dumps({'status': 'ready', 'python': sys.executable, 'jpeg_backend': 'nvjpeg' if decoder else 'opencv',
                      'cuda': torch.version.cuda, 'packages': {name.split('==')[0]: version(name.split('==')[0])
                      for name in (*PACKAGES, 'torch', 'torchvision')},
                      'verified': ['ROS imports', 'JPEG probe', 'LeRobot save', 'Parquet', 'video decode'],
                      'hardware_control': False}, indent=2))


def main(argv=None):
    args = parse_args(argv)
    if args._probe:
        probe(args._probe)
        return
    if platform.system() != 'Linux' or platform.machine() not in ('x86_64', 'AMD64'):
        raise RuntimeError('This installer targets Linux x86_64 robots, not Jetson/Thor or other platforms.')
    if not args.ros_setup.is_file():
        raise RuntimeError(f'ROS setup not found: {args.ros_setup}. Install ROS 2 Jazzy separately.')
    conda = conda_executable(args.conda)
    base = Path(subprocess.check_output([str(conda), 'info', '--base'], text=True).strip()).resolve()
    prefix = base / 'envs' / args.name
    if prefix.is_symlink():
        raise RuntimeError('Refusing a symlink environment prefix')
    backend = choose_backend(args.backend)
    if args.check:
        if not (prefix / 'bin/python').is_file():
            raise RuntimeError(f'Environment not found: {prefix}')
        check_environment(prefix, backend, args.ros_setup)
        return
    source = None
    if args.clone:
        if not Path(args.clone).is_absolute() and not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', args.clone):
            raise RuntimeError('--clone must be an environment name or absolute prefix')
        source = (Path(args.clone) if Path(args.clone).is_absolute() else base / 'envs' / args.clone).resolve()
        if source == prefix or not (source / 'bin/python').is_file():
            raise RuntimeError('Clone source must be an existing, different environment')
        version = subprocess.check_output([str(source / 'bin/python'), '-c',
                   'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")'], text=True).strip()
        if version != '3.12':
            raise RuntimeError('Clone source must use Python 3.12 for ROS 2 Jazzy; use a fresh install instead.')
    commands = install_commands(conda, prefix, backend, source)
    marker = base / 'envs' / f'.collector-setup-{args.name}.json'
    spec = dict(version=1, prefix=str(prefix), backend=backend, clone=str(source) if source else None,
                ros_setup=str(args.ros_setup.resolve()),
                commands=[[str(value) for value in command] for command in commands])
    # Lists round-trip through JSON, so normalize before comparing resumes.
    spec = json.loads(json.dumps(spec))
    if args.dry_run:
        if prefix.exists() or marker.exists():
            raise RuntimeError('Target exists; choose another --name or use --check')
        print(json.dumps(spec, indent=2))
        for command in commands:
            print(shlex.join(map(str, command)))
        print('Then: pip check, isolated synthetic probe, configure LEROBOT_PY for this environment only.')
        return
    required_gb = 16 if backend == 'nvjpeg' else 6
    free = shutil.disk_usage(prefix.parent if prefix.parent.exists() else base).free
    if free < required_gb * 1024 ** 3:
        raise RuntimeError(f'At least {required_gb} GiB free is required before installation; '
                           f'available: {free / 1024 ** 3:.1f} GiB. No packages were changed.')
    prefix.parent.mkdir(parents=True, exist_ok=True)
    lock_path = marker.with_suffix('.lock')
    with os.fdopen(os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600), 'w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(f'Another installer is using {prefix}') from exc
        if marker.is_symlink():
            raise RuntimeError('Refusing a symlink install marker')
        if args.resume:
            record = json.loads(marker.read_text()) if marker.exists() else None
            if not record or record.get('spec') != spec or record.get('status') == 'ready':
                raise RuntimeError('--resume requires an incomplete install owned by this script with identical options')
            if not (prefix / 'bin/python').is_file():
                raise RuntimeError('Environment creation was incomplete; choose a new --name. No files were removed.')
        elif prefix.exists() or marker.exists():
            raise RuntimeError('Target exists; it will not be changed. Choose another --name or use --check.')
        atomic_json(marker, {'spec': spec, 'status': 'installing'})
        try:
            for command in commands[1:] if args.resume else commands:
                run(command)
            check_environment(prefix, backend, args.ros_setup)
            run([conda, 'env', 'config', 'vars', 'set', '-p', prefix, f'LEROBOT_PY={prefix}/bin/python'])
            atomic_json(marker, {'spec': spec, 'status': 'ready'})
        except BaseException:
            atomic_json(marker, {'spec': spec, 'status': 'failed'})
            raise
    print(f'Environment ready: {prefix}\nActivate with: conda activate {args.name}\n'
          f'Or set: LEROBOT_PY={prefix}/bin/python\n'
          'No recorder, robot control or hardware service was started.')


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, ValueError, subprocess.SubprocessError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        sys.exit(1)
