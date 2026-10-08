"""Release names, legacy entry compatibility and bundled asset integrity; no hardware."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch
from dagger.dependencies import ASSETS, validate

ROOT = Path(__file__).resolve().parents[1]

class ReleaseEntryTest(unittest.TestCase):
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

if __name__ == '__main__':
    unittest.main()
