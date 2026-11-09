"""Replay recorded policy targets through the real V5 receiver; isolated, no I/O."""
import json
import os
from pathlib import Path
import sys
import time
from dataclasses import dataclass
from types import SimpleNamespace
import numpy as np


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '211' or os.environ.get('DAGGER_PUBLISH') != '0':
        raise SystemExit('Requires isolated ROS_DOMAIN_ID=211 DAGGER_PUBLISH=0')
    root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root))
    from dagger.runtime_copy import prepare_copy
    from dagger.robot_profile import resolve_robot
    from dagger.v5_runtime import controller_parameters
    from dagger.s3_head_limit import HeadPitchLimitedGenerator
    runtime=prepare_copy(root/'teleop_snapshot',root/'.runtime');sys.path.insert(0,str(runtime))
    import rclpy
    from std_msgs.msg import String
    from openarmx_teleop_vr_306_v4.controller_node import IndependentArmController
    from openarmx_teleop_vr_306_v4.schema import JointFeedback
    robot=resolve_robot(os.environ)
    params={**robot.controller_params(runtime,os.environ),**controller_parameters(runtime),
            'dry_run':True,'topic_suffix':'211_340','follow_authority_topic':'/hg_dagger/control_state',
            'authority_heartbeat_topic':'/hg_dagger/authority_heartbeat'}
    args=['--ros-args','--params-file',str(runtime/'config/controller.yaml')]
    for key,value in params.items():args+=['-p',f'{key}:={json.dumps(value)}']
    rclpy.init(args=args);node=IndependentArmController();results=[]
    try:
        assert node._dry_run and node._joint_pub is None and node._gripper_pub is None
        assert not node._collision_required and node._kinematics.collision_available
        for name in sys.argv[1:]:
            rows=[json.loads(line)['payload'] for line in Path(name).open()]
            raw=np.asarray([r['action'] for r in rows],np.float32)
            @dataclass(frozen=True)
            class Batch:
                model_actions:np.ndarray
                physical_actions:np.ndarray
            filtered=HeadPitchLimitedGenerator(SimpleNamespace(generate_candidates=lambda:Batch(raw[None],raw[None])),34.5).generate_candidates().physical_actions[0]
            unchanged=[i for i in range(21) if i!=17]
            np.testing.assert_array_equal(filtered[:,unchanged],raw[:,unchanged])
            before={};rejected=[]
            for index,(row,action) in enumerate(zip(rows,filtered)):
                measured=row['measured_policy_state']; lower=row['measured_leg_waist'][:2]
                groups=dict(leg_waist=np.asarray([*lower,*measured[19:21]]),
                    left_arm=np.asarray(measured[:7]),right_arm=np.asarray(measured[7:14]),
                    neck=np.asarray(measured[16:19]),left_gripper=np.asarray(measured[14:15]),right_gripper=np.asarray(measured[15:16]))
                node._feedback=JointFeedback(**groups);node._feedback_time=time.monotonic()
                node._on_follow_authority(String(data=json.dumps(dict(mode='POLICY_ACTIVE',authority_epoch=1,session_id='replay'))))
                old=dict(groups,left_arm=raw[index,:7],right_arm=raw[index,7:14],neck=raw[index,16:19],leg_waist=np.asarray([*lower,*raw[index,19:21]]))
                node._collision_required=True
                reason,_=node._configuration_guard_error(old,'replay')
                before[reason.split(';')[0].split(':')[0] if reason else 'accepted']=before.get(reason.split(';')[0].split(':')[0] if reason else 'accepted',0)+1
                node._collision_required=False
                payload=dict(leg_waist_target_joints_position=[*lower,*action[19:21].tolist()],
                    left_arm_target_joints_position=action[:7].tolist(),right_arm_target_joints_position=action[7:14].tolist(),
                    neck_target_joints_position=action[16:19].tolist(),authority_epoch=1,collector_session_id='replay')
                node._target_source='replay_awaiting';node._on_joint_target(String(data=json.dumps(payload)))
                if node._target_source!='joint_space':rejected.append(dict(index=index,reason=node._reason))
                else:
                    for group,values in [('left_arm',action[:7]),('right_arm',action[7:14]),('neck',action[16:19]),('leg_waist',payload['leg_waist_target_joints_position'])]:
                        np.testing.assert_allclose(node._target_groups[group],values,atol=1e-6,rtol=0)
            result=dict(trace=str(Path(name).parent.name),frames=len(rows),old_accepted=before.get('accepted',0),
                head_bounded=int(np.count_nonzero(raw[:,17]>34.5)),new_rejected=rejected,hardware_publishers=0)
            print(json.dumps(result));results.append(result)
            assert not rejected,rejected[:3]
        assert node._joint_pub is None and node._gripper_pub is None
        print('V5_POLICY_REPLAY_PASS')
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
