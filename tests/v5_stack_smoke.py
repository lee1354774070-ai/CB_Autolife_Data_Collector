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
    states={};counts={};input_count=[0]
    def vr_input(msg):input_count[0]+=1
    monitor.create_subscription(String,'/openarmx_teleop_vr_306_v4/vr_input',vr_input,qos_profile_sensor_data)
    def status(msg):states.update(json.loads(msg.data))
    monitor.create_subscription(String,'/hg_dagger/control_state',status,10)
    for name in ('whole_body_target_joints_position','gripper_target_joints_position','joints_set_enable_state','joints_clear_error'):
        topic=f'/topic_arm_{name}_211_340';counts[topic]=0
        def cb(msg,key=topic):counts[key]+=1
        monitor.create_subscription(String,topic,cb,qos_profile_sensor_data)
    with tempfile.TemporaryDirectory(prefix='v5_stack_smoke_') as tmp:
        env=dict(os.environ,DAGGER_BASE_DIR=tmp,DAGGER_DEPENDENCY_ROOT=str(root/'teleop_snapshot'),
            DAGGER_V4_RUNTIME_ROOT=str(runtime),DAGGER_WEB_PORT='18447',
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
            # Exercise the real HTTPS -> WebSocket -> ROS input path as well.
            probe = r"""
import asyncio,aiohttp,json,time
async def check():
 async with aiohttp.ClientSession(connector=aiohttp.TCPConnector(ssl=False)) as client:
  async with client.ws_connect('https://127.0.0.1:18447/ws?client_id=isolated-stack-test') as ws:
   for sequence in range(10):
    await ws.send_json({'packetType':'pose','sequence':sequence,'timestamp':int(time.time()*1000),
      'leftController':{'active':True,'gripActive':False,'position':{'x':-.2,'y':1.,'z':-.3}},
      'rightController':{'active':True,'gripActive':False,'position':{'x':.2,'y':1.,'z':-.3}}})
    await asyncio.sleep(.04)
   async with client.get('https://127.0.0.1:18447/api/config') as response:
    config=await response.json()
    assert config['vr_images']['default_mode']=='rgbd',config
    assert config['vr_images']['cameras'][0]['fps']==15,config
   async with client.get('https://127.0.0.1:18447/api/camera/frame.jpg?camera=rgbd_head_color') as response:
    frame=await response.read()
    assert response.status==200 and frame[:2]==b'\xff\xd8' and len(frame)>1000,(response.status,frame[:100])
    print('HEAD_RGB_PASS jpeg_bytes='+str(len(frame)))
   import rclpy
   from std_msgs.msg import String
   rclpy.init();node=rclpy.create_node('isolated_feedback_probe')
   try:
    publisher=node.create_publisher(String,'/collector/feedback',10)
    async with client.get('https://127.0.0.1:18447/collector_events') as response:
     await response.content.readline();await response.content.readline()
     until=time.monotonic()+3
     while publisher.get_subscription_count()==0 and time.monotonic()<until:
      rclpy.spin_once(node,timeout_sec=.05);await asyncio.sleep(.05)
     event={'id':'isolated-takeover','event':'takeover','pulses_ms':[300,70],'text':'人工接管已生效'}
     publisher.publish(String(data=json.dumps(event)))
     line=await asyncio.wait_for(response.content.readline(),3)
     assert line.startswith(b'data: ') and json.loads(line[6:])['id']==event['id'],line
     print('ROS_TO_SSE_PASS takeover_received=true')
   finally:node.destroy_node();rclpy.shutdown()
   async with client.get('https://127.0.0.1:18447/api/status') as response:
    value=await response.json()
    assert value['vrConnected'],value
    assert isinstance(value['teleop']['hg_dagger']['collector_compat']['feedback_events'],list),value['teleop']
    print('WEB_FEEDBACK_HISTORY_PASS')
    print('WEBSOCKET_CONNECTED',json.dumps(value.get('realtimePacketCounts',{})))
asyncio.run(check())
"""
            subprocess.run([env['DAGGER_WEB_PY'],'-B','-c',probe],check=True,timeout=15)
            deadline=time.monotonic()+1
            while time.monotonic()<deadline:rclpy.spin_once(monitor,timeout_sec=.05)
            assert input_count[0] >= 1, 'WebSocket packets never reached the ROS input topic'
            print('WEBSOCKET_TO_ROS_PASS packets='+str(input_count[0]))
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
