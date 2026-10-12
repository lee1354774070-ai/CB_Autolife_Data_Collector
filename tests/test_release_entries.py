"""Release names, legacy entry compatibility and bundled asset integrity; no hardware."""
from contextlib import redirect_stdout
import hashlib
import io
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch
from dagger.dependencies import ASSETS, validate
from tools.patch_thor_receipts import replace_once, run_patch

ROOT = Path(__file__).resolve().parents[1]

class ReleaseEntryTest(unittest.TestCase):
    def test_thor_patch_entries_offer_help_without_loading_ros(self):
        for name in ('patch_thor_receipts.py', 'patch_cb_thor_gripper.py'):
            result = subprocess.run(['python3', str(ROOT / 'tools' / name), '--help'],
                                    capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('--apply', result.stdout)

    def test_dagger_entry_and_legacy_alias_both_offer_help_without_starting(self):
        for name in ('start_dagger.sh', 'start_mzj300_dagger.sh'):
            self.assertTrue(os.access(ROOT / name, os.X_OK))
            result = subprocess.run(['bash', str(ROOT / name), '--help'],
                                    capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('COLLECTOR_MODE', result.stdout)
            self.assertIn('DAGGER_PUBLISH', result.stdout)

    def test_bundled_assets_validate_without_legacy_hg_and_tampering_fails(self):
        with tempfile.TemporaryDirectory() as directory, patch('dagger.dependencies.HASHES', {}):
            root = Path(directory)
            validate(root)  # No external HG source tree exists here.
            copy = root / 'assets'
            shutil.copytree(ASSETS, copy)
            (copy / 'web/vr_app.js').write_text('unreviewed')
            with patch('dagger.dependencies.ASSETS', copy):
                with self.assertRaisesRegex(RuntimeError, 'changed or missing'):
                    validate(root)

    def test_thor_phase_option_preserves_legacy_alias_and_exact_arguments(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = Path(directory) / 'repo/deploy/groot_n1_7/thor/manage.sh'
            manager.parent.mkdir(parents=True)
            manager.write_text('printf "%s\\n" "$@"\n')
            for filename, options, aware in (
                ('start_thor_baseline.sh', {}, True),
                ('start_thor_baseline.sh', {'GROOT_GRIPPER_PHASE_AWARE': '0'}, False),
                ('start_mzj_thor_baseline.sh', {'MZJ_GRIPPER_PHASE_AWARE': '0'}, False),
                ('start_thor_baseline.sh', {'GROOT_GRIPPER_PHASE_AWARE': '1', 'MZJ_GRIPPER_PHASE_AWARE': '0'}, True),
            ):
                env = {k:v for k,v in os.environ.items() if k not in ('GROOT_GRIPPER_PHASE_AWARE','MZJ_GRIPPER_PHASE_AWARE')}
                env.update(GROOT_RUNTIME_ROOT=directory, **options)
                result = subprocess.run(['bash', str(ROOT / 'tools' / filename), 'argument with spaces'],
                                        env=env, text=True, capture_output=True, timeout=5)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stdout.splitlines(), ['start', 'baseline'] +
                                 (['--gripper-phase-aware'] if aware else []) + ['argument with spaces'])

class ThorPatchTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.repo = Path(temporary.name)
        directory = self.repo / 'deploy/groot_n1_7'
        directory.mkdir(parents=True)
        self.paths = [directory / name for name in ('server.py', 'policy_only_session.py')]
        self.original = b'value = 1\n'
        for path in self.paths:
            path.write_bytes(self.original)
        self.hashes = {path.name: hashlib.sha256(self.original).hexdigest() for path in self.paths}

    def run_patch(self, apply=False, transform=lambda name, text: replace_once(text, '1', '2')):
        args = ['patch.py', str(self.repo)] + (['--apply'] if apply else [])
        with patch('sys.argv', args), redirect_stdout(io.StringIO()):
            run_patch(self.hashes, transform, description='test', backup_prefix='test_',
                      completion_message='Done')

    def test_preview_does_not_write_sources_or_backups(self):
        self.run_patch()
        self.assertTrue(all(path.read_bytes() == self.original for path in self.paths))
        self.assertFalse((self.repo / '.codex_backups').exists())

    def test_apply_backs_up_all_originals(self):
        self.run_patch(apply=True)
        backup, = (self.repo / '.codex_backups').iterdir()
        for path in self.paths:
            self.assertEqual((backup / path.name).read_bytes(), self.original)
            self.assertEqual(path.read_text(), 'value = 2\n')

    def test_later_invalid_source_never_partially_patches(self):
        # Refusal includes a later hash mismatch, invalid Python or an ambiguous
        # replacement. Earlier valid sources must remain byte-for-byte intact.
        for failure in ('hash', 'syntax', 'structure'):
            with self.subTest(failure=failure):
                self.paths[1].write_bytes(b'value = 3\n' if failure == 'hash' else self.original)
                before = [path.read_bytes() for path in self.paths]
                def transform(name, text):
                    if name == self.paths[1].name:
                        if failure == 'syntax':
                            return 'def broken('
                        if failure == 'structure':
                            return replace_once(text, 'missing', '2')
                    return replace_once(text, '1', '2')
                error = {'hash': SystemExit, 'syntax': SyntaxError, 'structure': ValueError}[failure]
                with self.assertRaises(error):
                    self.run_patch(apply=True, transform=transform)
                self.assertEqual([path.read_bytes() for path in self.paths], before)
                self.assertFalse((self.repo / '.codex_backups').exists())


if __name__ == '__main__':
    unittest.main()
