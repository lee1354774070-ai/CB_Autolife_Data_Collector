import json,time,rclpy,urllib.request,ssl
from pathlib import Path
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from collections import Counter
from dagger.dependencies import COMMAND_PUBLISHERS
rclpy.init();node=rclpy.create_node("mzj_dryrun_acceptance")
states={}; counts=Counter()
for name,topic in (("control","/hg_dagger/control_state"),("controller","/openarmx_teleop_vr_306_v4/status"),("policy","/hg_dagger/policy_status")):
 node.create_subscription(String,topic,lambda msg,key=name:states.update({key:json.loads(msg.data)}),10)
for topic in COMMAND_PUBLISHERS:
 node.create_subscription(String,topic,lambda msg,key=topic:counts.update([key]),qos_profile_sensor_data)
deadline=time.monotonic()+18
try:
 while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
 print("STATE_SUMMARY",json.dumps({k:{a:v.get(a) for a in ("mode","state","phase","dry_run","hardware_enabled","session_id")} for k,v in states.items()}),flush=True)
 assert states["control"]["mode"]=="DISARMED"
 assert states["controller"]["dry_run"] is True
 assert states["controller"].get("hardware_enabled") is not True
 assert states["policy"]["phase"]=="idle"
 assert not counts,dict(counts)
 names=set()
 for topic in COMMAND_PUBLISHERS:
  names.update(i.node_name for i in node.get_publishers_info_by_topic(topic))
 assert "independent_arm_controller_306_v4" not in names,names
 context=ssl._create_unverified_context()
 for path in ("/","/vr_app.js"):
  with urllib.request.urlopen("https://127.0.0.1:8447"+path,context=context,timeout=5) as response:
   body=response.read().decode()
   assert response.status==200
   if path=="/vr_app.js": assert "进入 VR 后保持待命；按 A 开始。" in body
 print("FULL_STACK_DRYRUN_PASS idle=true hardware_publishers=0 vendor_commands=0 web=OK",flush=True)
finally:node.destroy_node();rclpy.shutdown()
