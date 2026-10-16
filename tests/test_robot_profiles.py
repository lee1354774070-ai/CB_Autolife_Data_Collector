"""Identity routing and model boundaries without hardware or network access."""
import importlib.util
import os
from pathlib import Path
import types
import unittest
from unittest.mock import patch

from dagger.compat import check_host, FEATURES, SERVICE
from dagger.dependencies import command_publishers, command_conflicts
from dagger.robot_profile import resolve_robot, recorder_cpus
from dagger.run import environment, default_ros_setup

ROOT = Path(__file__).resolve().parents[1]


class RobotProfileTest(unittest.TestCase):
    def test_legacy_300_defaults_and_host_detection(self):
        robot = resolve_robot({}, hostname='autolife-robot-300')
        self.assertEqual((robot.suffix, robot.model), ('0_300', 's2_v2_2'))
        with self.assertRaisesRegex(ValueError, 'does not identify'):
            resolve_robot({}, hostname='autolife-robot-347')
        robot = resolve_robot({'ROBOT_TYPE': 'autolife_s3'}, hostname='autolife-robot-347')
        self.assertEqual((robot.robot_id, robot.model), ('347', 's3_v3_1'))

    def test_robot_ids_and_domains_are_not_a_300_allowlist(self):
        locks = set()
        for robot_id, model in (('283','s2_v2_2'), ('306','s2_v2_2'), ('328','s2_v2_2'),
                                 ('345','s3_v3_1'), ('346','s3_v3_1'), ('347','s3_v3_1')):
            env = environment('test', None, dict(ROBOT_ID=robot_id, ROS_DOMAIN_ID='211',
                DAGGER_ROBOT_PROFILE=model, DAGGER_SERVER_URL='http://thor:8777',
                HG_DAGGER_RECORDER_CPUS='0'))
            robot = resolve_robot(env)
            self.assertEqual(robot.suffix, f'211_{robot_id}')
            self.assertEqual(env['ROBOT_ID'], robot_id)
            self.assertEqual(env['ROS_DOMAIN_ID'], '211')
            self.assertTrue(robot.lease_service.endswith(robot.suffix))
            self.assertTrue(robot.joint_topic.endswith(robot.suffix))
            locks.add(robot.lock_file)
            self.assertEqual(env['DAGGER_PUBLISH'], '0')
        self.assertEqual(len(locks), 6)

    def test_invalid_identity_or_mismatched_model_rejected(self):
        for value in ({'ROBOT_ID':'../300'}, {'ROBOT_ID':'0300'}, {'ROS_DOMAIN_ID':'233'},
                      {'ROS_DOMAIN_ID':'-1'}, {'DAGGER_ROBOT_PROFILE':'guessed'},
                      {'ROBOT_TYPE':'autolife_s3','DAGGER_ROBOT_PROFILE':'s2_v2_2'}):
            with self.subTest(value=value), self.assertRaises(ValueError):
                resolve_robot({'ROBOT_ID':'300', **value})

    def test_ros_workspace_is_optional_only_when_not_explicitly_requested(self):
        with patch.object(Path, 'is_file', return_value=False):
            self.assertEqual(default_ros_setup(), '/opt/ros/jazzy/setup.bash')
        env=environment('test', None, {'ROBOT_ID':'300', 'DAGGER_SERVER_URL':'http://thor',
                                     'DAGGER_ROS_SETUP':'/explicit/missing.bash',
                                     'HG_DAGGER_RECORDER_CPUS':'0'})
        self.assertEqual(env['DAGGER_ROS_SETUP'], '/explicit/missing.bash')

    def test_command_allowlist_stays_robot_and_domain_specific(self):
        topics = command_publishers('211_347')
        self.assertTrue(all(t.endswith('_211_347') for t in topics))
        topic = '/topic_arm_whole_body_target_joints_position_211_347'
        vendor = '/node_arm_vr_control_service_211_347'
        self.assertFalse(any(command_conflicts({topic:[vendor]}, {}, suffix='211_347').values()))
        self.assertTrue(command_conflicts({topic:[vendor.replace('347','300')]}, {},
                                         suffix='211_347')['unknown_publishers'])
        self.assertTrue(command_conflicts({topic:[vendor]}, {topic:1},
                                         suffix='211_347')['active_commands'])

    def test_server_model_binding_keeps_legacy_s2_and_rejects_preview(self):
        health = dict(ready=True, policy_type='groot', mode='policy_only_baseline',
                      controller_submission_receipts=True, outcome_history_offsets=[])
        s2=resolve_robot({'ROBOT_ID':'300'})
        s3=resolve_robot({'ROBOT_ID':'347','DAGGER_ROBOT_PROFILE':'s3_v3_1'})
        s2.validate_health(health)
        with self.assertRaisesRegex(ValueError, 'profile mismatch'):
            s3.validate_health(health)
        mapped={**health,'robot_profile':s3.wire_profile}
        s3.validate_health(mapped)
        with self.assertRaisesRegex(ValueError, 'profile mismatch'):
            s2.validate_health(mapped)
        for changes in ({'preview_only':True}, {'hardware_execution_allowed':False},
                        {'mode':'policy_only_baseline_s3_preview'}, {'controller_submission_receipts':False},
                        {'outcome_visual_history_required':True}, {'ready':False}):
            with self.assertRaises(ValueError): s3.validate_health({**mapped,**changes})

    def test_attachment_checks_id_model_and_domain(self):
        host=dict(version=1,instance_id='a'*32,robot_id='347',robot_profile='s3_v3_1',
                  domain_id='211',command_service=SERVICE,features=FEATURES)
        state=dict(collector_compat=host,collector_fifo=str(Path('/tmp/test/.official_recording_control').resolve()))
        options=dict(fifo=state['collector_fifo'],robot_id='347',robot_profile='s3_v3_1',domain='211')
        self.assertEqual(check_host(state,**options),host)
        for changes in ({'robot_id':'300'}, {'robot_profile':'s2_v2_2'}, {'domain':'0'}):
            with self.assertRaises(ValueError):check_host(state,**{**options,**changes})

    def test_cpu_mask_keeps_300_default_and_never_selects_absent_cores(self):
        self.assertEqual(recorder_cpus(range(20),'s2_v2_2'), '12-19')
        self.assertEqual(recorder_cpus(range(22),'s3_v3_1'), '18,19,20,21')
        self.assertEqual(recorder_cpus([1,3,5,7],'s3_v3_1'), '5,7')
        with self.assertRaises(ValueError): recorder_cpus([0,1],'s3_v3_1')

    def test_launch_routes_all_consumers_and_keeps_model_reset_pose(self):
        class Process:
            def __init__(self,*args,**kw):self.__dict__.update(kw)
        mods={}
        for name, attrs in {
            'launch':dict(LaunchDescription=list),
            'launch.actions':dict(ExecuteProcess=Process,RegisterEventHandler=Process,EmitEvent=Process),
            'launch.event_handlers':dict(OnProcessExit=Process),
            'launch.events':dict(Shutdown=Process),
        }.items():
            module=types.ModuleType(name);module.__dict__.update(attrs);mods[name]=module
        for robot_id,model in (('300','s2_v2_2'),('347','s3_v3_1')):
            env=dict(ROBOT_ID=robot_id,ROS_DOMAIN_ID='211',DAGGER_ROBOT_PROFILE=model,
                     DAGGER_V4_RUNTIME_ROOT='/runtime',DAGGER_BASE_DIR='/data/test',
                     DAGGER_PUBLISH='0',DAGGER_SERVER_URL='http://thor:8777',
                     DAGGER_TOKEN_FILE='/test-token',TASK_TEXT='Pick the laundry bag.')
            with patch.dict(os.environ,env,clear=True),patch.dict('sys.modules',mods):
                spec=importlib.util.spec_from_file_location('profile_stack',ROOT/'dagger/stack.launch.py')
                module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
                nodes=module.generate_launch_description()
            commands=[node.cmd for node in nodes if hasattr(node,'cmd')]
            supervisor,controller,mapper,bridge,web=commands
            suffix=f'211_{robot_id}'
            self.assertIn(f'joint_state_topic:=/topic_arm_whole_body_and_gripper_current_joints_status_{suffix}',supervisor)
            self.assertIn(f'joint_state_topic:=/topic_arm_whole_body_and_gripper_current_joints_status_{suffix}',bridge)
            self.assertIn(f'topic_suffix:={suffix}',controller)
            self.assertIn(f'topic_suffix:={suffix}',mapper)
            self.assertIn('sync_hold_only_enable:=true',controller)
            self.assertIn('require_sync_session_service:=true',controller)
            self.assertIn('dry_run:=true',controller)
            if model=='s3_v3_1':
                self.assertIn('quick_reset_left_arm_joints:=[20.0, 0.0, 0.0, -110.0, 0.0, 0.0, 0.0]',controller)
                self.assertNotIn('0_300',' '.join(sum(commands,[])))
            else:
                self.assertIn('urdf_path:=/runtime/urdf/robot_v2_2_simplified.urdf',controller)


if __name__=='__main__':unittest.main()
