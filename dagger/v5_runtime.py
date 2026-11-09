"""Pinned S3 V5 source, in a private copy using DAgger's legacy wire namespace.

Only the package/node/topic namespace is translated to keep the established
supervisor, recorder and ACK interfaces. Mink and incremental mapping remain
V5. The upstream checkout, ROS installation and V4 backend are never edited.
"""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
PACKAGE = 'openarmx_teleop_vr_306_v5'
LEGACY = 'openarmx_teleop_vr_306_v4'
MANIFEST = HERE / 'v5_source.json'


def selected(env=None):
    value = (os.environ if env is None else env).get('DAGGER_TELEOP_BACKEND', 'v4')
    if value not in ('v4', 'v5'):
        raise ValueError('DAGGER_TELEOP_BACKEND must be v4 or v5')
    return value == 'v5'


def source_root(env=None):
    return Path((os.environ if env is None else env).get(
        'DAGGER_V5_SOURCE_ROOT', '/home/ubuntu/openarmx_teleop_vr_v5')).expanduser().resolve()


def validate(env=None):
    from .robot_profile import resolve_robot
    env = os.environ if env is None else env
    if resolve_robot(env).model != 's3_v3_1':
        raise ValueError('The reviewed V5 snapshot requires DAGGER_ROBOT_PROFILE=s3_v3_1')
    from .dependencies import BUNDLED_RESET_ROOT, RESET_POSE_LOADER_HASH
    loader = BUNDLED_RESET_ROOT / 'reset_pose_loader.py'
    if hashlib.sha256(loader.read_bytes()).hexdigest() != RESET_POSE_LOADER_HASH:
        raise RuntimeError('DAgger reset loader changed; inspect before running')
    root = source_root(env)
    manifest = json.loads(MANIFEST.read_text())
    for directory in (PACKAGE, 'config', 'urdf', 'meshes'):
        actual = {str(p.relative_to(root)) for p in (root / directory).rglob('*')
                  if p.is_file() and '__pycache__' not in p.parts and not p.name.endswith('.pyc')}
        expected = {name for name in manifest['files'] if name.startswith(directory + '/')}
        if actual != expected:
            raise RuntimeError(f'V5 dependency files changed or missing: {root / directory}')
    for name, digest in manifest['files'].items():
        path = root / name
        if not path.is_file() or path.is_symlink() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise RuntimeError(f'V5 dependency changed or missing: {path}; inspect before running')
    return root


def prepare_copy(cache_root, env=None):
    from .runtime_copy import _hashes
    from .dependencies import BUNDLED_RESET_ROOT
    source = validate(env)
    cache_root = Path(cache_root).resolve()
    if source == cache_root or source in cache_root.parents:
        raise ValueError('Runtime cache must be outside the upstream source')
    patch = HERE / 'v5_controller.patch'
    loader = BUNDLED_RESET_ROOT / 'reset_pose_loader.py'
    reset_patch = HERE / 'task_reset_loader.patch'
    identity = hashlib.sha256(b''.join(p.read_bytes() for p in (
        MANIFEST, Path(__file__), patch, loader, reset_patch, HERE / 'v5_reset_guard.py'))).hexdigest()
    target = cache_root / ('v5-' + identity[:20])
    if target.exists():
        manifest = json.loads((target / '.collector_copy.json').read_text())
        if manifest['identity'] != identity or manifest['files'] != _hashes(target):
            raise RuntimeError(f'V5 runtime copy changed; inspect before use: {target}')
        return target
    files = json.loads(MANIFEST.read_text())
    cache_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=cache_root, prefix='.prepare-v5-') as directory:
        stage = Path(directory) / 'runtime'
        stage.mkdir()
        for name in files['files']:
            dest = stage / name.replace(PACKAGE, LEGACY)
            dest.parent.mkdir(parents=True, exist_ok=True)
            content = (source / name).read_bytes()
            if dest.suffix in ('.py', '.yaml', '.urdf', '.srdf'):
                content = content.replace(b'_306_v5', b'_306_v4')
            dest.write_bytes(content)
        shutil.copy2(loader, stage / LEGACY / 'reset_pose_loader.py')
        for item in (patch, reset_patch):
            subprocess.run(['patch', '--batch', '--forward', '--fuzz=0', '-p1', '-i', str(item)],
                           cwd=stage, check=True, capture_output=True, text=True)
        (stage / '.collector_copy.json').write_text(json.dumps({
            'identity': identity, 'backend': 'v5', 'revision': files['revision'],
            'source': str(source), 'wire_namespace': LEGACY, 'files': _hashes(stage)}, indent=2))
        stage.rename(target)
    return target


def controller_parameters(runtime, env=None):
    """V5's matched collision pair, with the same 21 joint axes/limits as S3 SDK."""
    if not selected(env):
        return {}
    # Match the reviewed V5/normal inference per-frame policy. Reset planning
    # still uses its always-on collision model; native dead-zone limits remain.
    return {'urdf_path': str(Path(runtime) / 'urdf/robot_v3_1_simplified.urdf'),
            'srdf_path': str(Path(runtime) / 'urdf/robot_v3_1.srdf')}


def validate_server_head_limit(health, env=None):
    if not selected(env):
        return
    value = health.get('head_pitch_max_degrees')
    if type(value) not in (int, float) or not 0 < value <= 34.5:
        raise ValueError('S3推理服务缺少34.5°头部上限；请先更新8783服务，不在客户端裁剪动作')
