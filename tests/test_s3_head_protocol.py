"""Runs against the installed Thor protocol, with synthetic observations only."""
import unittest
import numpy as np
from dagger.s3_head_limit import HeadPitchLimitedGenerator
try:
    from deploy.groot_n1_7.demo import demo_contract
    from deploy.groot_n1_7.inference_session import CandidateBatch, CurrentObservation
    from deploy.groot_n1_7.policy_only_session import PolicyOnlyInferenceSession
    from deploy.groot_n1_7.protocol import ExecutionAck, ProtocolError, array_digest
    AVAILABLE=True
except ImportError:
    AVAILABLE=False

@unittest.skipUnless(AVAILABLE,'requires the reviewed Thor protocol checkout')
class HeadProtocolTest(unittest.TestCase):
    def setUp(self):
        self.contract=demo_contract()
        self.actions=np.random.default_rng(340).uniform(-15,15,(40,21)).astype(np.float32)
        self.actions[:,17]=np.linspace(25,39,40)
        self.actions[:,14:16]=[300,10]
        class Generator:
            def generate_candidates(inner,*args):
                return CandidateBatch(self.actions[None].copy(),self.actions[None].copy())
        self.session=PolicyOnlyInferenceSession(contract=self.contract,
            candidate_generator=HeadPitchLimitedGenerator(Generator(),34.5),
            task='Clear the trash from the table.',gripper_guard=True,gripper_phase='binary330')
    def infer(self,seq=1):
        state=np.zeros(21,dtype=np.float32);state[14:16]=[260,10]
        obs=CurrentObservation(seq,state,{k:np.zeros(v,dtype=np.uint8) for k,v in self.contract.image_shapes.items()})
        proposal=self.session.infer(obs).proposal
        return proposal,np.asarray(proposal.executable_actions,dtype=np.float32)
    def ack(self,p,a,steps=40):
        prefix=a[:steps];state=np.zeros(21,dtype=np.float32);state[14:16]=[260,10]
        ack=ExecutionAck(1,p.proposal_id,p.context_digest,p.executable_chunk_digest,steps,array_digest(prefix))
        self.session.acknowledge_execution(ack,prefix,state)
    def test_signed_chunk_and_ack_use_bounded_head_and_original_gripper_rules(self):
        p,a=self.infer()
        physical=self.actions.copy();physical[:,17]=np.minimum(physical[:,17],34.5)
        self.assertEqual(p.scored_chunk_digest,array_digest(physical))
        self.assertEqual(p.executable_chunk_digest,array_digest(a))
        np.testing.assert_array_equal(a[:,17],physical[:,17])
        np.testing.assert_array_equal(a[:,:14],self.actions[:,:14])
        np.testing.assert_array_equal(a[:,18:],self.actions[:,18:])
        np.testing.assert_array_equal(a[2:,14],330)
        wrong=a.copy();wrong[:,17]=self.actions[:,17]
        with self.assertRaises(ProtocolError):self.ack(p,wrong)
        self.ack(p,a);self.assertFalse(self.session.has_live_proposal)
    def test_partial_ack_and_discard_do_not_commit_future_actions(self):
        self.actions[20:,14]=10
        p,a=self.infer();self.ack(p,a,steps=20)
        self.actions[:,14]=180
        p,a=self.infer(2);np.testing.assert_array_equal(a[:,14],330)
        self.session.discard_unexecuted(p.proposal_id)
        self.assertFalse(self.session.has_live_proposal)

if __name__=='__main__':unittest.main()
