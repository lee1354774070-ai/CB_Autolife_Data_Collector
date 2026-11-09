import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest

from dagger import v5_runtime
from dagger.inference_options import validate_selection
from dagger.run import environment


class V5SelectionTest(unittest.TestCase):
    def test_v4_remains_default_and_s2_cannot_select_s3_snapshot(self):
        self.assertFalse(v5_runtime.selected({}))
        self.assertEqual(v5_runtime.controller_parameters(Path('/runtime'),{}),{})
        with self.assertRaises(ValueError):v5_runtime.selected({'DAGGER_TELEOP_BACKEND':'unknown'})
        source=dict(ROBOT_ID='300',DAGGER_SERVER_URL='http://example:8777',DAGGER_TELEOP_BACKEND='v5')
        with self.assertRaisesRegex(ValueError,'V5 requires'):environment('test',None,source)
        result=environment('test',None,{**source,'ROBOT_ID':'340','DAGGER_ROBOT_PROFILE':'s3_v3_1'})
        self.assertIn('openarmx_v5_mink',result['DAGGER_VR_PY'])
        self.assertEqual(result['ACTION_MODE'],'joint')
        self.assertEqual(result['DAGGER_PUBLISH'],'0')

    def test_v5_requires_server_head_bound_but_s2_health_is_unchanged(self):
        env={'DAGGER_TELEOP_BACKEND':'v5'}
        v5_runtime.validate_server_head_limit({}, {})
        v5_runtime.validate_server_head_limit({'head_pitch_max_degrees':34.5}, env)
        for value in (None, True, 0, 42.5, float('nan')):
            with self.assertRaises(ValueError):
                v5_runtime.validate_server_head_limit({'head_pitch_max_degrees':value}, env)

    def test_binary330_requires_explicit_selection_and_server_support(self):
        h=dict(ready=True,gripper_mode='binary330',gripper_phases=['binary330'],checkpoint='020000')
        self.assertEqual(validate_selection(h,'Clear the trash from the table.','binary330','020000'),'binary330')
        with self.assertRaises(ValueError):validate_selection(h,'Clear the trash from the table.')
        with self.assertRaises(ValueError):validate_selection(h,'custom','continuous')
        with self.assertRaises(ValueError):validate_selection({**h,'gripper_phases':[]},'custom','binary330')


@unittest.skipUnless(os.environ.get('DAGGER_V5_SOURCE_ROOT'), 'set DAGGER_V5_SOURCE_ROOT for pinned checkout tests')
class V5CopyTest(unittest.TestCase):
    def test_pinned_private_copy_keeps_mapper_and_mink_and_detects_tampering(self):
        env=dict(ROBOT_ID='340',DAGGER_ROBOT_PROFILE='s3_v3_1',
                 DAGGER_V5_SOURCE_ROOT=os.environ['DAGGER_V5_SOURCE_ROOT'])
        source=v5_runtime.validate(env)
        manifest=json.loads(v5_runtime.MANIFEST.read_text())
        with tempfile.TemporaryDirectory() as directory:
            runtime=v5_runtime.prepare_copy(Path(directory),env)
            self.assertEqual(runtime,v5_runtime.prepare_copy(Path(directory),env))
            for name in ('mink_kinematics.py','quest_incremental.py','vr_mapper_node.py','waist_follow_state.py','arm_reference_frame.py'):
                expected=(source/v5_runtime.PACKAGE/name).read_bytes().replace(b'_306_v5',b'_306_v4')
                self.assertEqual((runtime/v5_runtime.LEGACY/name).read_bytes(),expected)
            controller=(runtime/v5_runtime.LEGACY/'controller_node.py').read_text()
            self.assertIn('MinkKinematics(',controller)
            self.assertIn('task reset pose is unavailable',controller)
            # Future unreviewed modification must fail, never regenerate over it.
            (runtime/v5_runtime.LEGACY/'controller_node.py').write_text(controller+'\n# changed\n')
            with self.assertRaisesRegex(RuntimeError,'Runtime|runtime copy changed'):
                v5_runtime.prepare_copy(Path(directory),env)
        for name,digest in manifest['files'].items():
            self.assertEqual(hashlib.sha256((source/name).read_bytes()).hexdigest(),digest)
