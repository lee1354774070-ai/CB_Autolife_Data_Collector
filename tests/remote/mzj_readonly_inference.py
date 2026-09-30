"""Read-only baseline inference: live SHM/state in, discard proposals, no action publishers."""
import os, time
import numpy as np
import cv2
import rclpy
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from deploy.common.robot_io import DirectShmCameraSet, image_to_policy_chw
from deploy.groot_n1_7.robot_client import GrootRemoteClient
from deploy.groot_n1_7.auth import read_token
from deploy.groot_n1_7.robot_mapping import policy_state_from_q23
from deploy.groot_n1_7.protocol import array_digest
from robot_schema import parse_whole_body_state
import json

def main():
    rclpy.init(); node=rclpy.create_node("mzj_readonly_inference_probe")
    latest=[None,0.]
    def joints(msg):
        try:
            q=parse_whole_body_state(json.loads(msg.data))
            if q is not None and np.isfinite(q).all(): latest[:]=[q,time.monotonic()]
        except (ValueError,TypeError,KeyError): pass
    node.create_subscription(String,"/topic_arm_whole_body_and_gripper_current_joints_status_0_300",joints,qos_profile_sensor_data)
    client=GrootRemoteClient(os.environ["DAGGER_SERVER_URL"],
        read_token(None,os.environ["DAGGER_TOKEN_FILE"]),timeout_sec=15)
    cv2.setNumThreads(1)
    try:
        health,contract=client.health()
        assert health["mode"]=="policy_only_baseline"
        assert health["controller_submission_receipts"] is True
        assert health["gripper_mode"]=="phase_aware"
        assert contract.action_dim==21
        print("HEALTH_PASS baseline,21D,phase_aware,controller_receipts",flush=True)
        cameras=DirectShmCameraSet(tuple(key.rsplit(".",1)[-1] for key in contract.image_shapes),set())
        for seq in range(1,3):
            deadline=time.monotonic()+8; frames=None
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.01); cameras.refresh()
                frames=cameras.synchronized_latest("hand_left",.04,.25)
                if frames is not None and latest[0] is not None and time.monotonic()-latest[1]<.35: break
            else: raise RuntimeError("Fresh synchronized cameras/state not available")
            images={key:image_to_policy_chw(frames[key.rsplit(".",1)[-1]].image_hwc,shape)
                    for key,shape in contract.image_shapes.items()}
            state=policy_state_from_q23(latest[0])
            payload=client.observation_payload(seq,state,images,contract)
            start=time.monotonic()
            response=client.start("Pick the laundry bag.",payload)
            sid=response["session_id"]; proposal=response.get("proposal")
            # Even invalid outputs must resolve this known unexecuted proposal.
            try:
                assert response["decision"]=="execute" and isinstance(proposal,dict)
                actions=np.asarray(proposal["executable_actions"],np.float32)
                assert actions.shape[1]==21 and np.isfinite(actions).all()
                assert array_digest(actions)==proposal["executable_chunk_digest"]
                print("INFERENCE_PASS",seq,"shape",actions.shape,"round_trip_ms",round((time.monotonic()-start)*1000,1),flush=True)
            finally:
                if isinstance(proposal,dict):
                    client.discard_unexecuted(sid,proposal)
                    client._request("/close",{"session_id":sid})
            print("DISCARD_CLOSE_PASS",seq,flush=True)
        print("NO_MOTION_PASS action_publishers=0 hardware_calls=0",flush=True)
    finally:
        client.close_connections();node.destroy_node();rclpy.shutdown()
if __name__=="__main__": main()
