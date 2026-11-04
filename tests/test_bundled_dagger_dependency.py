"""Validate real bundled sources and private runtime construction; no ROS/motors."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

from dagger import dependencies as deps
from dagger.run import ROOT, environment
from dagger.runtime_copy import prepare_copy, _hashes


class BundledDependencyTest(unittest.TestCase):
    def setUp(self):
        self.source = ROOT / 'teleop_snapshot'
        self.package = self.source / 'openarmx_teleop_vr_306_v4'

    def test_default_entry_uses_bundle_and_preserves_explicit_override(self):
        values = dict(ROBOT_ID='340', DAGGER_ROBOT_PROFILE='s3_v3_1',
                      DAGGER_SERVER_URL='http://unused.invalid')
        self.assertEqual(environment('test', None, values)['DAGGER_DEPENDENCY_ROOT'], str(self.source))
        values['DAGGER_DEPENDENCY_ROOT'] = '/explicit/reviewed'
        self.assertEqual(environment('test', None, values)['DAGGER_DEPENDENCY_ROOT'], '/explicit/reviewed')

    def test_bundle_and_saved_reset_validate_without_editing_source(self):
        before = _hashes(self.package)
        self.assertEqual(before['openarmx_teleop_vr_306_v4/controller_node.py'], deps.BUNDLED_CONTROLLER)
        deps.validate(self.source)
        deps.validate_saved_reset(self.source)
        with tempfile.TemporaryDirectory() as directory:
            cache = Path(directory) / 'cache'
            runtime = prepare_copy(self.source, cache)
            self.assertEqual(prepare_copy(self.source, cache), runtime)
            controller = (runtime / 'openarmx_teleop_vr_306_v4/controller_node.py').read_text()
            self.assertEqual(controller.count('def _collector_target_allowed('), 1)
            self.assertEqual(controller.count('from .reset_pose_loader import load_quick_reset_targets'), 1)
            self.assertIn("saved_targets = load_quick_reset_targets(robot_id)", controller)
            self.assertIn("groups['leg_waist'] = leg_waist", controller)
            self.assertIn('collector_startup_guard_error', controller)
            manifest = json.loads((runtime / '.collector_copy.json').read_text())
            self.assertEqual(manifest['files'], _hashes(runtime))
        self.assertEqual(before, _hashes(self.package))

    def test_runtime_loader_requires_task_snapshot_and_correct_robot(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = prepare_copy(self.source, Path(directory) / 'cache')
            path = runtime / 'openarmx_teleop_vr_306_v4/reset_pose_loader.py'
            spec = importlib.util.spec_from_file_location('bundled_reset_test', path)
            loader = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(loader)
            with patch.dict(os.environ, {}, clear=True):
                with self.assertRaisesRegex(ValueError, 'task reset snapshot is missing'):
                    loader.load_quick_reset_targets('340')
            snapshot = Path(directory) / 'pose.json'
            snapshot.write_text(json.dumps(dict(version=1, robot_id='340', mode='custom',
                custom=dict(joints_deg={name: float(i) for i, name in enumerate(loader.CUSTOM_POSE_JOINTS)}))))
            with patch.dict(os.environ, DAGGER_RESET_STATE_PATH=str(snapshot)):
                targets = loader.load_quick_reset_targets('340')
                self.assertEqual(targets['leg_waist'], [0., 1., 2., 3.])
                self.assertEqual(sum(map(len, targets.values())), 21)
                with self.assertRaisesRegex(ValueError, 'different robot'):
                    loader.load_quick_reset_targets('300')

    def test_unknown_controller_cannot_create_runtime(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / 'source'
            for name in deps.HASHES:
                target = root / name
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(self.source / name, target)
            controller = root / 'openarmx_teleop_vr_306_v4/openarmx_teleop_vr_306_v4/controller_node.py'
            controller.write_text(controller.read_text() + '\n# unreviewed\n')
            cache = Path(directory) / 'cache'
            with self.assertRaisesRegex(RuntimeError, 'controller_node.py'):
                prepare_copy(root, cache)
            self.assertFalse(cache.exists())

    def test_reset_adapter_missing_or_modified_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / 'reset'
            shutil.copytree(deps.BUNDLED_RESET_ROOT, root)
            for name in deps.BUNDLED_RESET_HASHES:
                path = root / name
                original = path.read_bytes()
                for modified in (None, original + b'\n# unreviewed\n'):
                    if modified is None:
                        path.unlink()
                    else:
                        path.write_bytes(modified)
                    with patch.object(deps, 'BUNDLED_RESET_ROOT', root):
                        with self.assertRaisesRegex(RuntimeError, name):
                            deps.validate_saved_reset(self.source)
                    path.write_bytes(original)

    def test_modified_cached_runtime_stays_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            cache = Path(directory) / 'cache'
            runtime = prepare_copy(self.source, cache)
            path = runtime / 'openarmx_teleop_vr_306_v4/controller_node.py'
            path.write_text(path.read_text() + '\n# unexpected runtime edit\n')
            with self.assertRaisesRegex(RuntimeError, 'Runtime copy changed'):
                prepare_copy(self.source, cache)


if __name__ == '__main__':
    unittest.main()
