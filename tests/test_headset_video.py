"""Preview selection must never change model/control/recording configuration."""
import importlib.util
import os
from pathlib import Path
import types
import unittest
from unittest.mock import patch
from dagger.run import environment

ROOT = Path(__file__).resolve().parents[1]

class HeadsetVideoTests(unittest.TestCase):
    def test_preview_flag_does_not_change_recorder_schema(self):
        env = {'ROBOT_ID':'340', 'DAGGER_ROBOT_PROFILE':'s3_v3_1',
               'DAGGER_SERVER_URL':'http://thor:8783', 'WITH_DEPTH':'1'}
        on = environment('test', 'Clear the trash from the table.', env)
        off = environment('test', 'Clear the trash from the table.', {**env, 'DAGGER_HEADSET_VIDEO':'0'})
        self.assertEqual(on.pop('DAGGER_HEADSET_VIDEO'), '1')
        self.assertEqual(off.pop('DAGGER_HEADSET_VIDEO'), '0')
        self.assertEqual(on, off)
        self.assertEqual(off['MIN_CAMERAS'], '4')
        with self.assertRaisesRegex(ValueError, 'DAGGER_HEADSET_VIDEO'):
            environment('test', None, {**env, 'DAGGER_HEADSET_VIDEO':'false'})

    def test_only_web_camera_parameter_changes_on_s2_and_s3(self):
        class Process:
            def __init__(self, *args, **kw): self.__dict__.update(kw)
        modules = {}
        for name, attrs in {
            'launch':dict(LaunchDescription=list),
            'launch.actions':dict(ExecuteProcess=Process,RegisterEventHandler=Process,EmitEvent=Process),
            'launch.event_handlers':dict(OnProcessExit=Process),
            'launch.events':dict(Shutdown=Process),
        }.items():
            module=types.ModuleType(name);module.__dict__.update(attrs);modules[name]=module
        for robot, model, backend in [('300','s2_v2_2','v4'), ('340','s3_v3_1','v5')]:
            env=dict(ROBOT_ID=robot,DAGGER_ROBOT_PROFILE=model,DAGGER_TELEOP_BACKEND=backend,
                DAGGER_V4_RUNTIME_ROOT='/runtime',DAGGER_BASE_DIR='/data/test',ROS_DOMAIN_ID='211',
                DAGGER_PUBLISH='0',DAGGER_TOKEN_FILE='/token',TASK_TEXT='test')
            results=[]
            for value in ('1','0'):
                with patch.dict(os.environ,{**env,'DAGGER_HEADSET_VIDEO':value},clear=True),patch.dict('sys.modules',modules):
                    spec=importlib.util.spec_from_file_location('video_stack',ROOT/'dagger/stack.launch.py')
                    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
                    results.append([n.cmd for n in module.generate_launch_description() if hasattr(n,'cmd')])
            on,off=results
            self.assertEqual(on[:4],off[:4])
            self.assertIn('rgbd_camera_enabled:=false',off[4])
            self.assertEqual([x.replace('rgbd_camera_enabled:=true','rgbd_camera_enabled:=false') for x in on[4]],off[4])

if __name__ == '__main__': unittest.main()
