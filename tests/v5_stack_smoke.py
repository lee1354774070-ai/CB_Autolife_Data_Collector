"""Start all five real components in an isolated domain, remain DISARMED, exit."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import ssl
import urllib.request


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='211' or os.environ.get('DAGGER_PUBLISH')!='0':
        raise SystemExit('Isolated ROS_DOMAIN_ID=211 DAGGER_PUBLISH=0 required')
    root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root))
    from dagger.runtime_copy import prepare_copy
    runtime=prepare_copy(root/'teleop_snapshot',root/'.runtime')
    import rclpy
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init();monitor=rclpy.create_node('v5_stack_no_motion_monitor')
    states={};counts={}
    def status(msg):states.update(json.loads(msg.data))
    monitor.create_subscription(String,'/hg_dagger/control_state',status,10)
    for name in ('whole_body_target_joints_position','gripper_target_joints_position','joints_set_enable_state','joints_clear_error'):
        topic=f'/topic_arm_{name}_211_340';counts[topic]=0
        def cb(msg,key=topic):counts[key]+=1
        monitor.create_subscription(String,topic,cb,qos_profile_sensor_data)
    with tempfile.TemporaryDirectory(prefix='v5_stack_smoke_') as tmp:
        env=dict(os.environ,DAGGER_BASE_DIR=tmp,DAGGER_V4_RUNTIME_ROOT=str(runtime),DAGGER_WEB_PORT='18447',
            DAGGER_VR_PY='/home/ubuntu/ros2_ws/venvs/openarmx_v5_mink/bin/python',
            DAGGER_WEB_PY='/home/ubuntu/miniconda3/envs/robot_env/bin/python')
        log=Path(root.parent/'stack_components.log').open('w')
        proc=subprocess.Popen(['ros2','launch',str(root/'dagger/stack.launch.py')],env=env,stdout=log,stderr=subprocess.STDOUT)
        try:
            start=time.monotonic();web=False
            while time.monotonic()-start<15:
                if proc.poll() is not None:raise RuntimeError('Stack exited early; see stack_components.log')
                rclpy.spin_once(monitor,timeout_sec=.1)
                if not web and time.monotonic()-start>3:
                    try:
                        with urllib.request.urlopen('https://127.0.0.1:18447/',context=ssl._create_unverified_context(),timeout=.3) as response:
                            web=response.status==200
                    except OSError:pass
                if web and states and time.monotonic()-start>6:break
            assert web,'Web did not become ready'
            assert states.get('mode')=='DISARMED',states
            assert not any(counts.values()),counts
            print('V5_STACK_START_PASS web=200 mode=DISARMED hardware_messages=0')
        finally:
            proc.send_signal(signal.SIGINT) if proc.poll() is None else None
            try:proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                proc.terminate();proc.wait(timeout=10)
            log.close();monitor.destroy_node();rclpy.shutdown()
        text=(root.parent/'stack_components.log').read_text()
        assert 'process has died' not in text,text[-2500:]
        print('V5_STACK_EXIT_PASS all_launch_children_stopped=true')

if __name__=='__main__':main()
