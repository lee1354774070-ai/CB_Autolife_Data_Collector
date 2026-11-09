from dataclasses import dataclass
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import numpy as np
from dagger.s3_head_limit import HeadPitchLimitedGenerator

@dataclass(frozen=True)
class Batch:
    model_actions: np.ndarray
    physical_actions: np.ndarray

class HeadLimitTest(unittest.TestCase):
    def test_only_physical_pitch_changes_before_digest(self):
        actions = np.arange(2*40*21,dtype=np.float32).reshape(2,40,21)/100
        actions[...,17] = np.linspace(-25,42.5,80).reshape(2,40)
        raw=actions.copy(); model=actions.copy()/10
        batch=Batch(model,raw)
        generate=Mock(return_value=batch)
        wrapper=HeadPitchLimitedGenerator(SimpleNamespace(generate_candidates=generate,gripper_indices=()),34.5)
        result=wrapper.generate_candidates('observation',task='task')
        generate.assert_called_once_with('observation',task='task')
        unchanged=[i for i in range(21) if i!=17]
        np.testing.assert_array_equal(result.physical_actions[...,unchanged],raw[...,unchanged])
        np.testing.assert_array_equal(result.physical_actions[...,17],np.minimum(raw[...,17],34.5))
        np.testing.assert_array_equal(batch.physical_actions,actions)
        self.assertIs(result.model_actions,model)
        self.assertEqual(wrapper.gripper_indices,())
        self.assertEqual(result.physical_actions.dtype,np.float32)

    def test_invalid_limits_and_chunks_fail_before_publication(self):
        for limit in (0,-1,35,float('nan'),float('inf')):
            with self.assertRaises(ValueError):HeadPitchLimitedGenerator(None,limit)
        for value in (np.zeros((40,21),np.float32),np.zeros((1,40,22),np.float32),
                      np.zeros((1,40,21),np.float64),np.full((1,40,21),np.nan,np.float32)):
            wrapper=HeadPitchLimitedGenerator(SimpleNamespace(generate_candidates=lambda:Batch(value,value)),34.5)
            with self.assertRaises(ValueError):wrapper.generate_candidates()

if __name__=='__main__':unittest.main()
