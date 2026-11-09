"""Compare ordinary ACK and DAgger submission receipts in the real Thor session."""
import unittest
import numpy as np
try:
    from deploy.groot_n1_7.demo import demo_contract
    from deploy.groot_n1_7.inference_session import CandidateBatch, CurrentObservation
    from deploy.groot_n1_7.policy_only_session import PolicyOnlyInferenceSession
    from deploy.groot_n1_7.protocol import ExecutionAck, array_digest
    AVAILABLE=hasattr(PolicyOnlyInferenceSession,'finish_controller_submission')
except ImportError:
    AVAILABLE=False

@unittest.skipUnless(AVAILABLE,'requires deployed Thor receipt protocol')
class GripperReceiptParity(unittest.TestCase):
    def test_partial_and_full_receipts_preserve_exact_same_gripper_state(self):
        contract=demo_contract()
        raw=np.zeros((40,21),dtype=np.float32);raw[:,14:16]=10
        class Generator:
            def generate_candidates(self,*args):return CandidateBatch(raw[None].copy(),raw[None].copy())
        sessions=[PolicyOnlyInferenceSession(contract=contract,candidate_generator=Generator(),
            task='Clear the trash from the table.',gripper_guard=True,gripper_phase='binary330') for _ in range(2)]
        state=np.zeros(21,dtype=np.float32);state[14:16]=10
        for sequence,(left,right,steps) in enumerate([(300,10,30),(180,300,40),(10,180,12),(180,10,40)],1):
            raw[:,14:16]=[left,right]
            observation=CurrentObservation(sequence,state,{k:np.zeros(v,dtype=np.uint8) for k,v in contract.image_shapes.items()})
            proposals=[s.infer(observation).proposal for s in sessions]
            outputs=[np.asarray(p.executable_actions,dtype=np.float32) for p in proposals]
            np.testing.assert_array_equal(*outputs)
            p=proposals[0];prefix=outputs[0][:steps]
            ack=ExecutionAck(sequence,p.proposal_id,p.context_digest,p.executable_chunk_digest,steps,array_digest(prefix))
            sessions[0].acknowledge_execution(ack,prefix,state)
            p=proposals[1]
            sessions[1].finish_controller_submission(dict(receipt_scope='controller_submission',
                proposal_id=p.proposal_id,context_digest=p.context_digest,
                executable_chunk_digest=p.executable_chunk_digest,submitted_prefix=prefix.tolist(),
                submitted_prefix_digest=array_digest(prefix)))
            for a,b in zip(sessions[0]._binary_state,sessions[1]._binary_state):np.testing.assert_array_equal(a,b)
            state[14:16]=prefix[-1,14:16]
        # Both paths must preserve an intentional release, not latch forever.
        self.assertLess(outputs[0][-1,15],330)

if __name__=='__main__':unittest.main()
