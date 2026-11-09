"""S3 picker isolates the service and refuses busy/unknown rollback states."""
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock, patch

from dagger import s3_checkpoint_control as control
from dagger.inference_options import checkpoint_request


class S3CheckpointTests(unittest.TestCase):
    def test_separate_key_command_and_strict_url(self):
        with patch('subprocess.run', return_value=SimpleNamespace(returncode=0, stdout='{}')) as run:
            checkpoint_request(server_url='http://192.168.8.224:8783')
            args=run.call_args.args[0]
            self.assertIn(str(Path.home()/'.ssh/id_ed25519_collector_s3_checkpoint'),args)
            self.assertEqual(args[-1], 's3-checkpoint-control')
            checkpoint_request(server_url='http://192.168.8.224:8777')
            self.assertEqual(run.call_args.args[0][-1], 'checkpoint-control')
        for url in ('http://192.168.8.224:8778','http://evil:8783','http://user@192.168.8.224:8783',
                    'http://192.168.8.224:8783/wrong','http://192.168.8.224:8783?x=1'):
            with self.assertRaises(ValueError), patch('subprocess.run') as run:
                checkpoint_request(server_url=url)
            run.assert_not_called()

    def test_invalid_names_never_touch_service(self):
        for name in ('../x','a/b','/tmp/model',None,'','a;reboot'):
            with self.assertRaises(ValueError),patch.object(control,'snapshot') as snap:
                control.switch(name)
            snap.assert_not_called()

    def switch_context(self):
        root=Path(self.tmp.name)
        return [patch.object(control,'ROOT',root),patch.object(control,'MODELS',root/'models'),
                patch.object(control,'validate_model',side_effect=lambda path:path)]

    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        for p in self.switch_context():p.start();self.addCleanup(p.stop)
        self.old={'checkpoint':'old','health':{'ready':True,'active_session':False}}

    def test_busy_and_same_model_do_not_stop_or_reload(self):
        for health in ({'ready':True,'active_session':True},{'ready':False,'active_session':False}, {'ready':True}):
            with patch.object(control,'snapshot',return_value={**self.old,'health':health}),patch.object(control,'docker') as docker:
                with self.assertRaisesRegex(RuntimeError,'active'):control.switch('new')
                docker.assert_not_called()
        with patch.object(control,'snapshot',return_value=self.old),patch.object(control,'request') as request:
            self.assertEqual(control.switch('old'),self.old)
            request.assert_not_called()

    def test_success_leases_before_stopping_only_verified_id(self):
        trace=[]
        with patch.object(control,'snapshot',return_value=self.old), \
             patch.object(control,'current_container',return_value={'Id':'exact-s3-container'}), \
             patch.object(control,'request',side_effect=lambda *a: trace.append(a) or {'lease':'lease'}), \
             patch.object(control,'docker',side_effect=lambda *a:trace.append(a)), \
             patch.object(control,'start_model',return_value={'checkpoint':'new'}) as start:
            self.assertEqual(control.switch('new'),{'checkpoint':'new'})
        self.assertEqual(trace,[('/prepare_reload',{}),('stop','--time','10','exact-s3-container')])
        self.assertEqual(start.call_args.args[0].name,'new')

    def test_absent_replacement_rolls_back_but_alive_unknown_is_preserved(self):
        for alive in (False,True):
            with patch.object(control,'snapshot',return_value=self.old), \
                 patch.object(control,'current_container',return_value={'Id':'s3','State':{'Running':alive}}), \
                 patch.object(control,'request',return_value={'lease':'lease'}),patch.object(control,'docker') as docker, \
                 patch('subprocess.run',return_value=SimpleNamespace(stdout='s3' if alive else '')), \
                 patch.object(control,'start_model',side_effect=[RuntimeError('load failed'),self.old]) as start:
                with self.assertRaisesRegex(RuntimeError,'preserved' if alive else 'restored old'):
                    control.switch('new')
                self.assertEqual(start.call_count,1 if alive else 2)
                docker.assert_called_once_with('stop','--time','10','s3')

    def test_container_identity_mismatch_never_passes(self):
        item={'Mounts':[{'Destination':'/run/s3.token','Source':str(control.TOKEN)}],
              'Config':{'Image':control.IMAGE,'Cmd':['python','server','--port','8777']}}
        with patch.object(control,'docker',return_value=json.dumps([item])):
            with self.assertRaisesRegex(RuntimeError,'identity'):control.current_container()

    def test_new_container_keeps_native_protocol_and_only_8783(self):
        state={'checkpoint':'new','health':{'ready':True}}
        with patch.object(control,'snapshot',return_value=state),patch('subprocess.run') as run:
            self.assertEqual(control.start_model(control.MODELS/'new',Path(self.tmp.name)/'log'),state)
        command=run.call_args.args[0]
        self.assertEqual(command[command.index('--port')+1],'8783')
        self.assertEqual(command[-4:],['--gripper-mode','binary330','--head-pitch-max','34.5'])
        self.assertIn('/models/new',command)
        self.assertFalse(any('baseline:' in word for word in command))


if __name__=='__main__':unittest.main()
