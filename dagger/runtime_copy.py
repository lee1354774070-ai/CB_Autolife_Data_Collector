"""Build a checked V4 runtime COPY; never patch the colleague's source tree."""

import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

from .dependencies import validate


def _hashes(root):
    return {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(root.rglob("*")) if path.is_file()
            and not any(part.startswith(".") or part == "__pycache__" for part in path.relative_to(root).parts)}


def prepare_copy(source_root: Path, cache_root: Path) -> Path:
    """Copy, patch and publish atomically. Existing copies are never overwritten."""
    source_root, cache_root = source_root.resolve(), cache_root.resolve()
    if source_root == cache_root or source_root in cache_root.parents:
        raise ValueError("Runtime cache must be outside the colleague's source tree")
    validate(source_root)
    source = source_root / "openarmx_teleop_vr_306_v4"
    patch = Path(__file__).with_name("v4_controller.patch")
    source_hashes = _hashes(source)
    identity = hashlib.sha256(json.dumps(source_hashes, sort_keys=True).encode() + patch.read_bytes()).hexdigest()
    target = cache_root / identity[:20]
    if target.exists():
        manifest = json.loads((target / ".collector_copy.json").read_text())
        if manifest["identity"] != identity or manifest["files"] != _hashes(target):
            raise RuntimeError(f"Runtime copy changed; inspect before use: {target}")
        return target
    cache_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=cache_root, prefix=".prepare-") as temporary:
        stage = Path(temporary) / "v4"
        shutil.copytree(source, stage, ignore=shutil.ignore_patterns(
            ".*", "__pycache__", "*.pyc", "build", "install", "log", "logs"))
        subprocess.run(["patch", "--batch", "--forward", "--fuzz=0", "-p1", "-i", str(patch)],
                       cwd=stage, check=True, capture_output=True, text=True)
        (stage / ".collector_copy.json").write_text(json.dumps(
            {"identity": identity, "source": str(source), "files": _hashes(stage)}, indent=2))
        stage.rename(target)
    return target
