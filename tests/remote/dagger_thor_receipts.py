"""Opt-in DAgger Thor regression tests. Run with the Thor repo on PYTHONPATH."""
from dataclasses import replace
from types import SimpleNamespace
import unittest
import numpy as np
from deploy.groot_n1_7.demo import demo_contract, demo_observation
from deploy.groot_n1_7.policy_only_session import PolicyOnlyInferenceSession
from deploy.groot_n1_7.server import GrootRemoteEngine
from deploy.groot_n1_7.protocol import ExecutionAck, ProtocolError, array_digest

PICK = "Pick the laundry bag."
PLACE = "Place the laundry bag in the upper compartment of the delivery robot."

class ReceiptsTest(unittest.TestCase):
    def session(self, task=PICK, phase=True):
        self.actions = np.zeros((1,40,21), dtype=np.float32)
        self.actions[:,:,14:16] = 180.
        generator = SimpleNamespace(generate_candidates=lambda *a: SimpleNamespace(
            model_actions=self.actions.copy(), physical_actions=self.actions.copy()))
        return PolicyOnlyInferenceSession(contract=demo_contract(), candidate_generator=generator,
            task=task, gripper_phase_aware=phase)

    def infer(self, session, seq=1, grips=(11.,9.)):
        obs = demo_observation(seq)
        obs.state[14:16] = grips
        return session.infer(obs).proposal

    def receipt(self, session, proposal, steps=5):
        prefix=np.asarray(proposal.executable_actions,dtype=np.float32)[:steps]
        return dict(session_id=session.session_id, receipt_scope="controller_submission",
            proposal_id=proposal.proposal_id, context_digest=proposal.context_digest,
            executable_chunk_digest=proposal.executable_chunk_digest,
            submitted_prefix=prefix.tolist(), submitted_prefix_digest=array_digest(prefix))

    def test_pick_continuous_then_measured_side_lock(self):
        s=self.session()
        self.actions[0,:5,14:16]=10
        p=self.infer(s)
        a=np.asarray(p.executable_actions)
        np.testing.assert_array_equal(a[:5,14:16],10)
        np.testing.assert_array_equal(a[5:,14:16],180)
        s.finish_controller_submission(self.receipt(s,p))
        p=self.infer(s,2,(120,9))
        np.testing.assert_array_equal(np.asarray(p.executable_actions)[:,14],360)
        self.assertEqual(p.executable_actions[0][15],10)

    def test_place_discard_does_not_latch_unsubmitted_release(self):
        s=self.session(PLACE)
        self.actions[0,10:,14]=50
        p=self.infer(s,grips=(360,360))
        a=np.asarray(p.executable_actions)
        np.testing.assert_array_equal(a[:10,14],360)
        np.testing.assert_array_equal(a[10:,14],10)
        s.finish_controller_submission(self.receipt(s,p,5))
        self.assertFalse(s._gripper_released.any())
        self.actions[:,:,14:16]=180
        p=self.infer(s,2,(360,360))
        np.testing.assert_array_equal(np.asarray(p.executable_actions)[:,14:16],360)

    def test_place_submitted_release_latches_only_that_side(self):
        s=self.session(PLACE)
        self.actions[0,2:,14]=50
        p=self.infer(s,grips=(360,360))
        s.finish_controller_submission(self.receipt(s,p,5))
        np.testing.assert_array_equal(s._gripper_released,[True,False])
        self.actions[:,:,14:16]=180
        p=self.infer(s,2,(360,360))
        np.testing.assert_array_equal(np.asarray(p.executable_actions)[:,14],10)
        np.testing.assert_array_equal(np.asarray(p.executable_actions)[:,15],360)

    def test_invalid_receipt_retains_proposal(self):
        for field,value in (("receipt_scope","executed"),("proposal_id","wrong"),
            ("context_digest","wrong"),("executable_chunk_digest","wrong"),
            ("submitted_prefix_digest","wrong"),("submitted_prefix",[]),
            ("submitted_prefix",[[float("nan")]*21])):
            with self.subTest(field=field):
                s=self.session(); p=self.infer(s); payload=self.receipt(s,p)
                payload[field]=value
                with self.assertRaises(ProtocolError): s.finish_controller_submission(payload)
                self.assertTrue(s.has_live_proposal)
        s=self.session(); p=self.infer(s); payload=self.receipt(s,p)
        payload["submitted_prefix"][0][0]+=1
        payload["submitted_prefix_digest"]=array_digest(np.array(payload["submitted_prefix"],np.float32))
        with self.assertRaises(ProtocolError): s.finish_controller_submission(payload)
        self.assertTrue(s.has_live_proposal)

    def test_receipt_replay_rejected_and_close_allowed(self):
        s=self.session(); p=self.infer(s)
        engine=GrootRemoteEngine(demo_contract(),lambda _:s,mode="policy_only_baseline")
        engine.session=s
        payload=self.receipt(s,p)
        response=engine.controller_ack(payload)
        self.assertFalse(response["physical_execution_confirmed"])
        self.assertEqual(response["submitted_steps"],5)
        with self.assertRaises(ProtocolError): engine.controller_ack(payload)
        engine.close({"session_id":s.session_id})
        self.assertIsNone(engine.session)

    def test_wrong_session_or_outcome_cannot_use_submission(self):
        s=self.session(); p=self.infer(s)
        engine=GrootRemoteEngine(demo_contract(),lambda _:s,mode="policy_only_baseline")
        engine.session=s; payload=self.receipt(s,p)
        with self.assertRaises(ProtocolError):
            engine.controller_ack({**payload,"session_id":"wrong"})
        engine.history_offsets=(-1,0)
        self.assertFalse(engine.health()["controller_submission_receipts"])
        with self.assertRaises(ProtocolError): engine.controller_ack(payload)
        self.assertTrue(s.has_live_proposal)

    def test_continuous_and_legacy_ack_unchanged(self):
        s=self.session(phase=False)
        p=self.infer(s); payload=self.receipt(s,p)
        prefix=np.asarray(payload["submitted_prefix"],dtype=np.float32)
        s.acknowledge_execution(ExecutionAck(ack_sequence_id=1,
            proposal_id=p.proposal_id,context_digest=p.context_digest,
            executable_chunk_digest=p.executable_chunk_digest,
            executed_steps=len(prefix),executed_prefix_digest=array_digest(prefix)),
            prefix, np.zeros(21,np.float32))
        self.assertFalse(s.has_live_proposal)
        np.testing.assert_array_equal(np.asarray(p.executable_actions)[:,14:16],180)

    def test_digest_binds_transformed_actions_and_discard_is_clean(self):
        s=self.session(); p=self.infer(s,grips=(120,9))
        self.assertEqual(p.scored_chunk_digest,array_digest(self.actions[0]))
        self.assertEqual(p.executable_chunk_digest,array_digest(np.array(p.executable_actions,np.float32)))
        self.assertNotEqual(p.scored_chunk_digest,p.executable_chunk_digest)
        s.discard_unexecuted(p.proposal_id)
        self.assertFalse(s.has_live_proposal)

    def test_unsupported_task_is_rejected(self):
        with self.assertRaises(ValueError): self.session("unknown task")

if __name__=="__main__":
    unittest.main()
