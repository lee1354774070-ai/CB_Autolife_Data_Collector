"""Verify original servo code + preserved receipts, with no motor publishers."""
from pathlib import Path
import ast,json,os,sys,time
from unittest.mock import Mock
import numpy as np

assert os.environ.get('ROS_DOMAIN_ID')=='211' and os.environ.get('DAGGER_PUBLISH')=='0'
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root))
from dagger.runtime_copy import prepare_copy
from dagger.v5_runtime import controller_parameters
from dagger.robot_profile import resolve_robot
from dagger.local_policy_execution import binding,matches
runtime=prepare_copy(root/'teleop_snapshot',root/'.runtime')
baseline=Path('/home/ubuntu/collector_validation/dagger_v5_20261106/collector/.runtime/v5-2c34ab6101ff01910ede/openarmx_teleop_vr_306_v4/controller_node.py')
controller=runtime/'openarmx_teleop_vr_306_v4/controller_node.py'
def control_tick(path):
 tree=ast.parse(path.read_text())
 cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='IndependentArmController')
 return next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_control_tick')
assert ast.dump(control_tick(baseline),include_attributes=False)==ast.dump(control_tick(controller),include_attributes=False),'Servo loop differs from original'
assert '_bounded_local_policy_command' not in ast.unparse(control_tick(controller))
sys.path.insert(0,str(runtime))
import rclpy
from std_msgs.msg import String
from openarmx_teleop_vr_306_v4.controller_node import IndependentArmController
from openarmx_teleop_vr_306_v4.schema import JointFeedback
robot=resolve_robot(os.environ)
params={**robot.controller_params(runtime,os.environ),**controller_parameters(runtime),
 'dry_run':True,'topic_suffix':'211_340','allow_waist_in_ik':True,
 'output_control_mode':'position','follow_authority_topic':'/hg_dagger/control_state',
 'authority_heartbeat_topic':'/hg_dagger/authority_heartbeat'}
args=['--ros-args','-r','__node:=independent_arm_controller_306_v4','--params-file',str(runtime/'config/controller.yaml')]
for k,v in params.items():args+=['-p',f'{k}:={json.dumps(v)}']
rclpy.init(args=args);node=IndependentArmController()
try:
 assert node._dry_run and node._joint_pub is None and node._gripper_pub is None
 node._hardware_output_guard_error=Mock(return_value='')
 node._publish_owned_command=Mock();node._set_hardware_fault=Mock()
 node._bounded_local_policy_command=Mock(side_effect=AssertionError('Retired smoothing called'))
 traces=Path('/home/ubuntu/nas/dagger/hotel_scene_dataset_261109_test_182242_812750_dagger/dagger_trace')
 path=next(traces.rglob('policy_trace.jsonl'))
 with path.open() as f:row=json.loads(next(f))['payload']
 actual=np.array(row['measured_policy_state'],dtype=float);legs=row['measured_leg_waist'][:2]
 def groups():
  return dict(leg_waist=np.r_[legs,actual[19:21]],left_arm=actual[:7].copy(),right_arm=actual[7:14].copy(),neck=actual[16:19].copy(),left_gripper=actual[14:15].copy(),right_gripper=actual[15:16].copy())
 node._feedback=JointFeedback(**groups());node._feedback_time=time.monotonic()
 node._begin_fresh_teleop_session_locked(groups())
 node._on_follow_authority(String(data=json.dumps(dict(mode='POLICY_ACTIVE',authority_epoch=101,session_id='restore-test'))))
 node._hardware_enabled=True
 receipts=[];node._publish_local_policy_receipt=lambda p:receipts.append(json.loads(json.dumps(p)))
 action=actual.tolist();action[0]+=1.;action[8]=9.8
 packet=dict(collector_session_id='restore-test',authority_epoch=101,proposal_id='native:1',chunk_step=0,bridge_generation=1,action=action,units='degrees',timestamp_ns=time.time_ns())
 node._on_local_policy_submission(String(data=json.dumps(packet)))
 assert receipts[-1]['accepted'] and matches(receipts[-1],binding(packet)),receipts[-1]
 assert receipts[-1]['requested_action']==action and abs(receipts[-1]['bounded_target'][8]-9.)<1e-6
 for _ in range(10):
  now=time.monotonic();node._feedback=JointFeedback(**groups())
  node._feedback_time=node._follow_authority_time=node._authority_heartbeat_time=now
  node._last_tick=now-.01;node._control_tick()
  node._set_hardware_fault.assert_not_called()
  q=node._last_position_command;actual[:14]=q[4:18];actual[16:19]=q[18:21];actual[19:21]=q[2:4]
 node._bounded_local_policy_command.assert_not_called();node._publish_owned_command.assert_not_called()
 before=node._target_time
 node._on_local_policy_submission(String(data=json.dumps(dict(packet,query_only=True))))
 assert receipts[-1]['accepted'] and node._target_time==before
 bad=dict(packet,chunk_step=1,action=list(action));bad['action'][8]=90.;bad['timestamp_ns']=time.time_ns()
 node._on_local_policy_submission(String(data=json.dumps(bad)))
 assert not receipts[-1]['accepted'] and 'Shoulder_Outer' in receipts[-1]['reason']
 report=dict(status='passed',original_servo_ast_equal=True,added_smoothing_calls=0,control_ticks=10,hardware_publishers=0,soft_limit_projection_retained=True,large_limit_violation_rejected=True,controller_receipts_retained=True,query_does_not_repeat_submission=True,runtime=str(runtime))
 (root.parent/'native_execution_restore_validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
finally:
 node._hardware_enabled=False;node.destroy_node();rclpy.shutdown()
