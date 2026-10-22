import json
from pathlib import Path
import tempfile
import struct
import unittest
from unittest.mock import patch

from dagger.inference_options import TASKS, selected_phase, validate_selection, checkpoint_request
from dagger import thor_checkpoint_control as control


class InferenceOptionsTest(unittest.TestCase):
    def test_truncated_weight_and_missing_processor_state_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / 'new_model'
            path.mkdir()
            (root / 'baseline').symlink_to('new_model')
            contract = dict(policy_type='groot', model_version='n1.7', state_feature_names=['state']*21,
                action_feature_names=['action']*21, image_features={'rgb':[3,480,640]}, chunk_size=40, n_action_steps=40)
            (path / 'groot_deployment_contract.json').write_text(json.dumps(contract))
            for filename in ('config.json','policy_preprocessor.json','policy_postprocessor.json'):
                (path / filename).write_text('{}')
            header = json.dumps({'weight':{'dtype':'F32','shape':[1],'data_offsets':[0,4]}}).encode()
            weights = struct.pack('<Q',len(header)) + header + bytes(4)
            (path / 'model.safetensors').write_bytes(weights)
            with patch.object(control, 'MODELS', root):
                self.assertEqual(control.model_path('new_model'), path)
                (path / 'policy_preprocessor.json').write_text(json.dumps({'steps':[{'state_file':'stats.safetensors'}]}))
                with self.assertRaises(OSError):control.model_path('new_model')
                (path / 'stats.safetensors').write_bytes(weights)
                self.assertEqual(control.model_path('new_model'), path)
                (path / 'model.safetensors').write_bytes(weights[:-1])
                with self.assertRaisesRegex(ValueError, 'Incomplete'):control.model_path('new_model')

    def test_exact_presets_custom_phase_and_checkpoint_identity(self):
        health = {'ready': True, 'gripper_phases': ['pick', 'place', 'continuous'], 'checkpoint': 'model_b'}
        for _, prompt, phase in TASKS:
            self.assertEqual(validate_selection(health, prompt), phase)
        with self.assertRaises(ValueError):
            selected_phase('Move a different object.')
        self.assertEqual(validate_selection(health, 'custom', 'continuous', 'model_b'), 'continuous')
        with self.assertRaises(ValueError):
            validate_selection(health, 'custom', 'pick', 'model_a')
        with self.assertRaises(ValueError):
            validate_selection({**health, 'ready': False}, 'custom', 'pick')

    def test_old_server_only_retains_legacy_prompts(self):
        health = {'ready': True, 'gripper_mode': 'phase_aware'}
        self.assertEqual(validate_selection(health, 'Pick the laundry bag.'), 'pick')
        with self.assertRaises(ValueError):
            validate_selection(health, TASKS[0][1])

    def test_management_never_targets_another_service(self):
        with patch('subprocess.run') as run:
            with self.assertRaises(ValueError):
                checkpoint_request(server_url='http://192.168.8.224:8778')
            run.assert_not_called()

    def test_model_paths_and_busy_switch_fail_before_restart(self):
        for name in ('../model', '/model', 'model; reboot', '', None):
            with self.assertRaises(ValueError):
                control.model_path(name)
        with tempfile.TemporaryDirectory() as directory, patch.object(control, 'ROOT', Path(directory)), \
                patch.object(control, 'model_path', return_value=Path(directory) / 'new'), \
                patch.object(control, 'snapshot', return_value={'checkpoint': 'old', 'health': {'ready': True, 'active_session': True}}), \
                patch.object(control, 'start_model') as start:
            with self.assertRaisesRegex(RuntimeError, 'active'):
                control.switch('new')
            start.assert_not_called()

    def test_same_checkpoint_is_noop_and_no_reload_lease(self):
        state = {'checkpoint': 'same', 'health': {'ready': True, 'active_session': False}}
        with tempfile.TemporaryDirectory() as directory, patch.object(control, 'ROOT', Path(directory)), \
                patch.object(control, 'model_path', return_value=Path(directory) / 'same'), \
                patch.object(control, 'snapshot', return_value=state), patch.object(control, 'request') as request:
            self.assertEqual(control.switch('same'), state)
            request.assert_not_called()

    def test_switch_verifies_new_before_persisting_and_rolls_back_failed_load(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            models = root / 'models'
            models.mkdir()
            (models / 'baseline').symlink_to('old')
            state = {'checkpoint': 'old', 'health': {'ready': True, 'active_session': False}}
            with patch.object(control, 'ROOT', root), patch.object(control, 'MODELS', models), \
                    patch.object(control, 'model_path', side_effect=lambda name: models / name), \
                    patch.object(control, 'snapshot', return_value=state), \
                    patch.object(control, 'request', return_value={'lease': 'lease', 'ready': True, 'active_session': False}), \
                    patch.object(control, 'start_model', side_effect=[RuntimeError('bad weights'), state]) as start:
                with self.assertRaisesRegex(RuntimeError, 'restored old'):
                    control.switch('new')
                self.assertEqual((models / 'baseline').readlink(), Path('old'))
                self.assertEqual([c.args[0].name for c in start.call_args_list], ['new', 'old'])


if __name__ == '__main__':
    unittest.main()
