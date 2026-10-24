"""Idle-owner startup repair: no robot or real service calls."""
from pathlib import Path
import ast
import io
import signal
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from dagger import startup_repair as repair


class StartupRepairTest(unittest.TestCase):
    def setUp(self):
        self.robot = SimpleNamespace(robot_id='300', suffix='0_300')
        self.idle = dict(hardware_enabled=False, hardware_unlock_mode='locked',
                         action_running=False, active_goal_groups=[], hardware_prepare_stage=None)
        self.launch = ('/usr/bin/python3', '/opt/ros/jazzy/bin/ros2', 'launch',
                       'control_center_joint_control', 'whole_body_joint_control.launch.py', 'robot_id:=300')

    def conflicts(self, *owners, active=None):
        return dict(unknown_publishers={'topic': list(owners)}, active_commands=active or {})

    def test_only_the_locked_idle_joint_module_is_stopped(self):
        with patch.object(repair, '_processes', return_value={12: self.launch}), patch.object(repair, '_stop') as stop:
            self.assertEqual(repair.repair_idle_conflicts(self.conflicts('/whole_body_joint_bridge'), self.idle, self.robot), 1)
            stop.assert_called_once_with(12, self.launch)

    def test_unknown_or_active_owner_never_stops_anything(self):
        for conflicts in (self.conflicts('/unknown'), self.conflicts('/whole_body_joint_bridge',active={'topic':1}),
                          self.conflicts('/action_player_0_346'), self.conflicts('/whole_body_joint_bridge','/unknown')):
            with self.subTest(conflicts=conflicts), patch.object(repair, '_stop') as stop:
                with self.assertRaises(RuntimeError): repair.repair_idle_conflicts(conflicts, self.idle, self.robot)
                stop.assert_not_called()

    def test_busy_or_unconfirmed_joint_status_never_stops(self):
        for update in ({}, dict(hardware_enabled=True), dict(action_running=True),
                       dict(active_goal_groups=['left_arm']),dict(hardware_prepare_stage='enabling')):
            state = {} if not update else {**self.idle, **update}
            with self.subTest(state=state), patch.object(repair, '_stop') as stop:
                with self.assertRaises(RuntimeError): repair.repair_idle_conflicts(self.conflicts('/whole_body_joint_bridge'), state, self.robot)
                stop.assert_not_called()

    def test_wrong_robot_or_ambiguous_launcher_is_not_stopped(self):
        for processes in ({}, {12: self.launch,13:self.launch}, {12:self.launch[:-1]+('robot_id:=346',)}):
            with patch.object(repair, '_processes',return_value=processes), patch.object(repair,'_stop') as stop:
                with self.assertRaises(RuntimeError): repair.repair_idle_conflicts(self.conflicts('/whole_body_joint_bridge'),self.idle,self.robot)
                stop.assert_not_called()

    def test_void_busy_failure_prevents_stopping_joint_too(self):
        with patch.object(repair, '_processes', return_value={12:self.launch}), \
             patch.object(repair, '_void_idle', side_effect=RuntimeError('busy')), patch.object(repair, '_stop') as stop:
            with self.assertRaisesRegex(RuntimeError,'busy'):
                repair.repair_idle_conflicts(self.conflicts('/whole_body_joint_bridge','/action_player_0_300'),self.idle,self.robot)
            stop.assert_not_called()

    def test_changed_or_missing_live_state_cancels_repair_before_any_stop(self):
        conflict = self.conflicts('/whole_body_joint_bridge')
        for current, state in [(conflict, {}), (conflict, {**self.idle, 'hardware_enabled': True}),
                               (self.conflicts('/whole_body_joint_bridge', active={'topic':1}), self.idle),
                               (self.conflicts('/whole_body_joint_bridge','/unknown'), self.idle)]:
            with patch.object(repair, '_processes',return_value={12:self.launch}), patch.object(repair,'_stop') as stop:
                with self.assertRaises(RuntimeError):
                    repair.repair_idle_conflicts(conflict, self.idle, self.robot,
                                                recheck=lambda: (current,state))
                stop.assert_not_called()

    def test_void_preserves_frontend_and_stops_backend_before_watchdog(self):
        with tempfile.TemporaryDirectory() as directory:
            home=Path(directory); (home/'code/void-cog/logs').mkdir(parents=True)
            watchdog=('bash',str(home/'code/void-cog/scripts/service/watch-backend.sh'),'--real')
            backend=('python','-m','link.client','--real','--port','8080')
            with patch.object(Path,'home',return_value=home), patch.object(Path,'read_text',return_value='0::/system.slice/void-cog-h5.service'), \
                 patch.object(repair,'_void_idle'), patch.object(repair,'_processes',return_value={21:watchdog,22:backend}), \
                 patch.object(repair,'_stop') as stop:
                self.assertEqual(repair.repair_idle_conflicts(self.conflicts('/action_player_0_300'),{},self.robot),2)
                self.assertEqual([c.args[0] for c in stop.call_args_list],[22,21])
                self.assertTrue((home/'code/void-cog/logs/backend.stop').exists())

    def test_process_identity_change_and_timeout_never_force_kill(self):
        with patch.object(repair.os,'pidfd_open',return_value=88,create=True), patch.object(repair.os,'close'), \
             patch.object(repair.select,'select',return_value=([],[],[])), \
             patch.object(repair.signal,'pidfd_send_signal',create=True) as send:
            with patch.object(repair,'_processes',return_value={12:('other',)}):
                with self.assertRaisesRegex(RuntimeError,'变化'): repair._stop(12,self.launch)
            send.assert_not_called()
            with patch.object(repair,'_processes',return_value={12:self.launch}), patch.object(repair.select,'select',return_value=([],[],[])):
                with self.assertRaisesRegex(RuntimeError,'未强杀'): repair._stop(12,self.launch)
            send.assert_called_once_with(88,signal.SIGINT)

    def test_natural_exit_during_identity_check_needs_no_signal(self):
        with patch.object(repair.os,'pidfd_open',return_value=88,create=True), patch.object(repair.os,'close'), \
             patch.object(repair.select,'select',side_effect=[([],[],[]),([88],[],[])]), \
             patch.object(repair,'_processes',return_value={}), \
             patch.object(repair.signal,'pidfd_send_signal',create=True) as send:
            repair._stop(12,self.launch)
            send.assert_not_called()


class VoidIdleTest(unittest.TestCase):
    def state(self, path, token=None, data=None):
        return {'connect': {'token':'test'}, 'disconnect': {}, 'jobs': [], 'laundry/tasks': [],
                'motion/busy':dict(running=False,action=False,speaking=False),
                'motion/host_loop':dict(running=False), 'tracking/watch':dict(running=False),
                'motion/actions/record/status':dict(state='idle',frames=0), 'collect/sessions':dict(sessions=[])}[path]

    def test_idle_audit_revokes_its_session_without_motion_requests(self):
        with patch.object(repair,'_request',side_effect=self.state) as request:
            repair._void_idle()
            self.assertEqual(request.call_args.args,('disconnect','test'))
            self.assertFalse(any(c.kwargs.get('data') is not None and c.args[0] not in ('connect','disconnect') for c in request.call_args_list))

    def test_jobs_loops_recordings_and_malformed_status_block_repair(self):
        for path,value in [('jobs',[dict(status='running')]),('jobs',[dict(status='queued')]),('jobs',[None]),
                           ('laundry/tasks',[{}]),('motion/busy',dict(running=False,action=True,speaking=False)),
                           ('motion/host_loop',dict(running=True)),('tracking/watch',{}),
                           ('motion/actions/record/status',dict(state='idle',frames=2)),('collect/sessions',dict(sessions=[{}]))]:
            def state(key,*args,**kwargs):return value if key==path else self.state(key,*args,**kwargs)
            with self.subTest(path=path,value=value), patch.object(repair,'_request',side_effect=state) as request:
                with self.assertRaises(RuntimeError): repair._void_idle()
                self.assertEqual(request.call_args.args,('disconnect','test'))


class LauncherStatusTest(unittest.TestCase):
    def test_failure_reason_and_repair_progress_reach_the_visible_launcher(self):
        source=Path(__file__).resolve().parents[1]/'gui/dagger_launcher.py'
        method=next(n for c in ast.parse(source.read_text()).body if isinstance(c,ast.ClassDef)
                    for n in c.body if isinstance(n,ast.FunctionDef) and n.name=='poll')
        scope=dict(rclpy=Mock(), time=SimpleNamespace(monotonic=lambda:100.),MUTED='muted',BLUE='blue')
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),scope)
        for line,code in [('FAIL  启动自检修复中止：控制中心正在运动',1),
                          ('run.py: error: Missing Thor token file',1),
                          ('INFO  自检修复：正常退出空闲控制模块 PID 12',None)]:
            node=SimpleNamespace(node=Mock(),process=Mock(),state_at=0,control_state={},stopping=False,
                count_future=None,count_next=0,task='',
                request=None,estop_request=None,control_buttons={},reader=io.StringIO(line+'\n'),
                pending='',failure='',status=Mock(),detail=Mock(),synced=Mock(),root=Mock(),poll=Mock(),
                start_button=Mock(),stop_button=Mock(),instruction=Mock(),reset_pose=Mock(),phase=Mock(),remark=Mock(),option_widgets=[])
            node.process.poll.return_value=code
            scope['poll'](node)
            self.assertEqual(node.detail.configure.call_args.kwargs['text'],line)
            if code is not None:self.assertEqual(node.failure,line)


if __name__=='__main__':unittest.main()
