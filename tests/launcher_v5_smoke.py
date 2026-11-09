"""Real Tk/ROS GUI in a private domain, all hardware and child starts mocked."""
import json
import os
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

assert os.environ.get('ROS_DOMAIN_ID')=='212', 'Use isolated ROS_DOMAIN_ID=212'
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import rclpy
import tkinter as tk
from std_msgs.msg import String
import gui.dagger_launcher as gui


def main():
    rclpy.init();root=tk.Tk();root.withdraw()
    app=gui.Launcher(root)
    try:
        assert not hasattr(app, 'publish')
        assert app.instruction.get()=='Clear the trash from the table.'
        assert app.phase.get()=='S3二值夹爪（服务端）'
        assert app.reset_pose.get()==(os.environ.get('DAGGER_RESET_POSE') or '按任务配置')
        assert app.checkpoint=='hotel_trash_clean233_020000'
        assert set(app.clients)=={'start','finish','fail','discard','reset','seal','quit'}
        assert not any(app.node.count_publishers(topic) for topic in (
            '/topic_arm_whole_body_target_joints_position_0_340','/topic_arm_gripper_target_joints_position_0_340'))
        with tempfile.TemporaryDirectory() as tmp,patch.object(gui,'DATA_ROOT',Path(tmp)),patch.object(gui,'LOGS',Path(tmp)/'logs'), \
             patch.object(gui.subprocess,'Popen') as popen:
            popen.return_value.poll.return_value=None
            app.start()
            command=popen.call_args.args[0];env=popen.call_args.kwargs['env']
            assert command[1]==str(gui.BASE/'dagger/run.py') and command[-1]=='--serve'
            assert env['DAGGER_PUBLISH']=='1'
            assert env['DAGGER_TELEOP_BACKEND']=='v5' and env['ROBOT_ID']=='340'
            assert env['DAGGER_SERVER_URL'].endswith(':8783') and env['TASK_TEXT']==app.instruction.get()
            assert env['WITH_DEPTH']=='1' and gui.task_pose(env)['pose_name']=='table_trash_020000'
            expected=str(Path(tmp)/app.task/'.official_recording_control')
            app.on_state(String(data=json.dumps({'collector_fifo':'other','mode':'EXPERT_ACTIVE'})))
            assert not app.control_state
            state={'collector_fifo':expected,'mode':'DISARMED','trial':{'recording':False}}
            app.on_state(String(data=json.dumps(state)));assert app.state_at>0
            # Invoke every actual GUI command path without needing a VR client.
            with patch.object(gui.messagebox,'askokcancel',return_value=True):
                for action in ('start','finish','fail','discard','reset','seal'):
                    client=Mock();client.service_is_ready.return_value=True;app.clients[action]=client
                    app.request=None;app.command(action);client.call_async.assert_called_once()
                app.estop_client=Mock();app.estop_client.service_is_ready.return_value=True
                app.emergency_stop();app.estop_client.call_async.assert_called_once()
                with patch.object(gui.os,'killpg') as kill:
                    app.command('quit');kill.assert_called_once()
            app.request=None;app.process=None;app.stopping=False
            # Independent reset does not start a recorder/model/web worker.
            app.start(reset_only=True)
            assert popen.call_args.kwargs['env']['DAGGER_PUBLISH']=='1'
            assert popen.call_args.args[0][-1]=='--reset-only'
            assert gui.task_pose(popen.call_args.kwargs['env'])['pose_name']=='table_trash_020000'
            app.process=None
        for mode in ('POLICY_ACTIVE','EXPERT_ACTIVE','DISARMED','ESTOP'):
            text,color=gui.describe_state({'mode':mode,'trial':{},'background_saves':{'pending':2}})
            assert '后台保存中：2条' in text
        print('LAUNCHER_V5_SMOKE_PASS all_controls=true standalone_reset=true task_model_pose=true status_isolated=true hardware_calls=0 child_processes=0')
    finally:
        if app.reader:app.reader.close()
        app.process=None;app.close();rclpy.shutdown()


if __name__=='__main__':main()
