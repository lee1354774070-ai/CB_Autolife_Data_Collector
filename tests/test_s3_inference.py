"""S3 coordinate conversion; optional real Thor protocol tests need its runtime on PYTHONPATH."""
from dataclasses import dataclass, replace
import hashlib
from inspect import signature
from pathlib import Path
from types import ModuleType, SimpleNamespace
import unittest

import numpy as np
from s3_thor_preview import NAMES, SIGNS, LIMITS, MODE, model_state, S3CandidateGenerator, preview_engine, require_phase_runtime


@dataclass(frozen=True)
class Observation:
    state: np.ndarray
    causal_history_window: object = None


@dataclass(frozen=True)
class Batch:
    model_actions: np.ndarray
    physical_actions: np.ndarray


class Generator:
    def __init__(self):
        s3 = np.zeros((1, 40, 21), dtype=np.float32)
        s3[:, :, 3] = -110
        s3[:, :, 10] = -100
        s3[:, :, 14:16] = 10
        self.physical = s3 * SIGNS

    def generate_candidates(self, observation, task, k):
        self.observed = observation.state.copy()
        return Batch(self.physical.copy(), self.physical.copy())


def state():
    result = np.zeros(21, dtype=np.float32)
    result[3] = result[10] = -110
    result[14:16] = [11, 9]
    return result


class S3MappingTest(unittest.TestCase):
    def setUp(self):
        self.contract = SimpleNamespace(state_feature_names=NAMES, action_feature_names=NAMES, chunk_size=40)
        self.generator = Generator()
        self.adapter = S3CandidateGenerator(self.generator, self.contract)

    def test_requires_explicit_phase_runtime_before_model_loading(self):
        def unpatched(pretrained_dir, *, device, num_candidates=1):
            self.fail('must not load a model during the dependency check')
        def patched(pretrained_dir, *, device, num_candidates=1, gripper_phase_aware=False):
            self.fail('must not load a model during the dependency check')
        with self.assertRaisesRegex(RuntimeError, 'patch_cb_thor_gripper.py'):
            require_phase_runtime(unpatched)
        require_phase_runtime(patched)

    def test_state_signs_and_gripper_units(self):
        original = state()
        actual = model_state(original)
        self.assertEqual(actual[3], 110)
        self.assertEqual(actual[10], -110)
        np.testing.assert_array_equal(actual[14:16], [11, 9])
        np.testing.assert_array_equal(actual * SIGNS, original)
        np.testing.assert_array_equal(np.flatnonzero(SIGNS < 0), [1,2,3,4,5,7,8,9,11,12,16,17])

    def test_maps_observation_and_postprocessed_actions_without_mutation(self):
        original = self.generator.physical.copy()
        observation = Observation(state())
        result = self.adapter.generate_candidates(observation, "task", 1)
        np.testing.assert_array_equal(self.generator.observed, model_state(observation.state))
        np.testing.assert_array_equal(result.physical_actions, original * SIGNS)
        np.testing.assert_array_equal(result.model_actions, original)
        np.testing.assert_array_equal(self.generator.physical, original)
        np.testing.assert_array_equal(observation.state, state())

    def test_rejects_wrong_order_history_and_candidates(self):
        with self.assertRaisesRegex(ValueError, "joint order"):
            S3CandidateGenerator(self.generator, SimpleNamespace(state_feature_names=NAMES[::-1], action_feature_names=NAMES))
        for observation, k in ((Observation(state(), object()), 1), (Observation(state()), 2)):
            with self.assertRaises(ValueError):
                self.adapter.generate_candidates(observation, "task", k)

    def test_rejects_bad_state_shape_nonfinite_and_range(self):
        bad = state(); bad[3] = 10
        for value in (np.zeros(23), np.full(21, np.nan), bad):
            with self.assertRaises(ValueError): model_state(value)

    def test_rejects_bad_action_without_clipping(self):
        for value in (np.nan, -1, 160):  # S2 left elbow maps to NaN, +1, -160 in S3.
            self.generator.physical[0, 20, 3] = value
            with self.assertRaises(ValueError):
                self.adapter.generate_candidates(Observation(state()), "task", 1)
        self.generator.physical = np.zeros((40, 21), dtype=np.float32)
        with self.assertRaises(ValueError):
            self.adapter.generate_candidates(Observation(state()), "task", 1)


try:
    from deploy.groot_n1_7.demo import demo_contract, demo_observation
    from deploy.groot_n1_7 import policy_only_session as runtime
    from deploy.groot_n1_7.protocol import ProtocolError, array_digest
except ImportError:
    runtime = None


@unittest.skipIf(runtime is None, "requires existing Thor runtime on PYTHONPATH")
class S3ProtocolTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if 'gripper_phase_aware' in signature(runtime.PolicyOnlyInferenceSession).parameters:
            cls.session_class = runtime.PolicyOnlyInferenceSession
            return
        # Exercise the shipped, hash-pinned migration in memory. Never patch
        # the user's live deployment merely to run Collector regression tests.
        from tools.patch_cb_thor_gripper import HASHES, transform
        raw = Path(runtime.__file__).read_bytes()
        if hashlib.sha256(raw).hexdigest() != HASHES['policy_only_session.py']:
            raise RuntimeError('Unreviewed Thor source; inspect before testing the phase patch')
        module = ModuleType('deploy.groot_n1_7._s3_phase_test')
        module.__package__ = 'deploy.groot_n1_7'
        source = transform('policy_only_session.py', raw.decode())
        exec(compile(source, '<isolated-s3-phase-test>', 'exec'), module.__dict__)
        require_phase_runtime(module.build_policy_only_session_factory)
        cls.session_class = module.PolicyOnlyInferenceSession

    def setUp(self):
        self.g = Generator()
        self.contract = demo_contract()
        def factory(task):
            return self.session_class(contract=self.contract, candidate_generator=self.g,
                                      task=task, gripper_phase_aware=True)
        self.engine = preview_engine(self.contract, factory)

    def session(self, task="Pick the laundry bag."):
        return self.engine.session_factory(task)

    def observe(self, seq, grips):
        values = state(); values[14:16] = grips
        return replace(demo_observation(seq), state=values)

    def test_preview_health_and_execution_receipts_are_refused(self):
        health = self.engine.health()
        self.assertEqual(health['mode'], MODE)
        self.assertTrue(health['preview_only'])
        self.assertFalse(health['controller_submission_receipts'])
        for name in ('acknowledge', 'controller_ack', 'cancel'):
            with self.assertRaisesRegex(ProtocolError, 'inspect/discard'):
                getattr(self.engine, name)({})

    def test_pick_latches_only_observation_and_hashes_s3_actions(self):
        session = self.session()
        self.g.physical[0,20:,14:16] = 300
        p = session.infer(self.observe(1, [11, 9])).proposal
        action = np.asarray(p.executable_actions, dtype=np.float32)
        np.testing.assert_array_equal(action[:20,14:16], 10)
        np.testing.assert_array_equal(action[:,3], -110)
        self.assertEqual(p.executable_chunk_digest, array_digest(action))
        self.assertEqual(p.scored_chunk_digest, array_digest(self.g.physical[0] * SIGNS))
        session.discard_unexecuted(p.proposal_id)
        p = session.infer(self.observe(2, [120, 119])).proposal
        action = np.asarray(p.executable_actions, dtype=np.float32)
        np.testing.assert_array_equal(action[:,14], 360)
        np.testing.assert_array_equal(action[:20,15], 10)
        self.assertEqual(p.executable_chunk_digest, array_digest(action))
        session.discard_unexecuted(p.proposal_id)

    def test_place_release_and_discard_preserve_hold(self):
        session = self.session("Place the laundry bag in the upper compartment of the delivery robot.")
        self.g.physical[:,:,14:16] = 200
        self.g.physical[0,7,14] = 60
        p = session.infer(self.observe(1, [200, 300])).proposal
        action = np.asarray(p.executable_actions, dtype=np.float32)
        np.testing.assert_array_equal(action[:7,14], 360)
        np.testing.assert_array_equal(action[7:,14], 10)
        np.testing.assert_array_equal(action[:,15], 360)
        session.discard_unexecuted(p.proposal_id)
        self.g.physical[:,:,14:16] = 200
        p = session.infer(self.observe(2, [200, 300])).proposal
        np.testing.assert_array_equal(np.asarray(p.executable_actions)[:,14:16], 360)
        session.discard_unexecuted(p.proposal_id)

    def test_bad_prediction_creates_no_live_proposal(self):
        session = self.session()
        self.g.physical[0,20,3] = -10
        with self.assertRaisesRegex(ValueError, 'left_elbow'):
            session.infer(self.observe(1, [11, 9]))
        self.assertFalse(session.has_live_proposal)
        with self.assertRaises(ValueError): self.session("unsupported task")


if __name__ == '__main__':
    unittest.main()
