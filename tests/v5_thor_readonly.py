"""Read live observations, request ONE proposal, verify digest, discard and close.

No controller construction, SYNC acquisition, hardware enable, or action ACK.
All DAgger authority/output topics are remapped away from the real stack.
"""
import json
import os
from pathlib import Path
import sys
import time
from unittest.mock import Mock
import numpy as np


def main():
    if os.environ.get('ROBOT_ID') != '340' or os.environ.get('DAGGER_PUBLISH') != '0':
        raise SystemExit('Explicit ROBOT_ID=340 DAGGER_PUBLISH=0 required')
    root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root))
    import rclpy
    from dagger.thor_bridge import CollectorThorBridge
    from dagger.control.groot_bridge_core import array_digest_float32
    params=dict(server_url=os.environ['DAGGER_SERVER_URL'],token_file=os.environ['DAGGER_TOKEN_FILE'],
        task=os.environ['TASK_TEXT'],collector_root=str(root),
        joint_state_topic='/topic_arm_whole_body_and_gripper_current_joints_status_0_340')
    args=['--ros-args','-r','__node:=collector_v5_readonly_check']
    for name in ('control_state','policy_action','policy_status','policy_forward_ack'):
        args+=['-r',f'/hg_dagger/{name}:=/collector_v5_readonly/{name}']
    for k,v in params.items():args+=['-p',f'{k}:={v}']
    rclpy.init(args=args);node=None
    try:
        node=CollectorThorBridge()
        node._policy_pub=Mock()
        node._policy_pub.publish.side_effect=AssertionError('Action publication forbidden')
        node._connect()
        assert node._health.get('active_session') is False,'service busy; refusing model request'
        deadline=time.monotonic()+8
        while node._q23 is None and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        observation,_,_=node._observation()
        response=node._next_response(observation)
        proposal=response['proposal']
        actions=np.asarray(proposal['executable_actions'],dtype=np.float32)
        assert actions.shape==(40,21) and np.isfinite(actions).all()
        assert array_digest_float32(actions.tolist())==proposal['executable_chunk_digest']
        node._resolve_proposal(cancel=True)
        node._close_session();assert not node._session_id
        node._policy_pub.publish.assert_not_called()
        print('DAGGER_V5_READONLY_INFERENCE_PASS '+json.dumps(dict(checkpoint=node._health['checkpoint'],
            shape=list(actions.shape),round_trip_ms=response['bridge_round_trip_ms'],discarded=True,
            gripper_mode=node._health['gripper_mode'],gripper_range=[float(actions[:,14:16].min()),float(actions[:,14:16].max())],
            hardware_calls=0,actions_published=0,controller_acks=0)))
    finally:
        if node is not None:
            try:
                if node._proposal is not None:node._resolve_proposal(cancel=True)
                if node._session_id:node._close_session()
            finally:node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':main()
