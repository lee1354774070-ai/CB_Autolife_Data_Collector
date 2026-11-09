"""Reset phase boundaries and refusal cases, without ROS or motor I/O."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch
import numpy as np
from dagger import v5_reset_guard as guard

p=Path(__file__).resolve().parents[1]/'teleop_snapshot/openarmx_teleop_vr_306_v4/openarmx_teleop_vr_306_v4/reset_path.py'
spec=importlib.util.spec_from_file_location('reset_path_fixture',p);path=importlib.util.module_from_spec(spec);spec.loader.exec_module(path)

class HeadEscapeTest(unittest.TestCase):
    def setUp(self):
        context=patch.dict(sys.modules,{'openarmx_teleop_vr_306_v4.reset_path':path});context.start();self.addCleanup(context.stop)
        self.start=np.zeros(21);self.start[19]=31.6
        self.goal=np.ones(21);self.goal[19]=25
        self.waypoint=self.start.copy();self.waypoint[19]=25

    def test_only_head_pitch_moves_until_feedback_clears_waypoint(self):
        split=.4
        for progress in np.linspace(0,split,30):
            sample=guard.head_first_sample(self.start,self.waypoint,self.goal,progress,split)
            np.testing.assert_array_equal(np.delete(sample,19),np.delete(self.start,19))
        progress=guard.head_first_progress(self.start,self.waypoint,self.goal,self.start,0,.2,split,2)
        target=guard.head_first_sample(self.start,self.waypoint,self.goal,progress,split)
        self.assertLessEqual(abs(target[19]-self.start[19]),1.+1e-6)
        self.assertEqual(guard.head_first_progress(self.start,self.waypoint,self.goal,self.start,split,.5,split,2),split)
        self.assertGreater(guard.head_first_progress(self.start,self.waypoint,self.goal,self.waypoint,split,.5,split,2),split)
        np.testing.assert_array_equal(guard.head_first_sample(self.start,self.waypoint,self.goal,1,split),self.goal)

    def controller(self):
        request=SimpleNamespace(enable_contact=False,num_max_contacts=1)
        k=SimpleNamespace(collision_available=True,geom_data=SimpleNamespace(collisionRequests=[request]),
            joint_q={str(i):i for i in range(21)},lower=np.full(21,-90.),upper=np.full(21,90.),
            q_from_feedback=lambda g,clip=False:np.asarray(g))
        direct=lambda a,b:('',dict(start=a,goal=b,duration_sec=2.))
        c=SimpleNamespace(_kinematics=k,_group_vector=lambda g:np.asarray(g),
            _kinematics_groups_from_vector=lambda v:v,_prepare_direct_quick_reset_path=direct,
            get_parameter=lambda key:SimpleNamespace(value=10. if 'velocity' in key else 1.))
        return c,request

    def test_existing_head_overlap_can_only_shrink_then_reach_a_clear_full_path(self):
        c,request=self.controller()
        depths=lambda k,q: {guard.HEAD_PAIR:(q[19]-31)*.01} if q[19]>31 else {}
        with patch.object(guard,'collision_depths',side_effect=depths):
            error,result=guard.prepare_reset(c,self.start,self.goal)
        self.assertFalse(error,error);np.testing.assert_array_equal(result['head_waypoint'],self.waypoint)
        self.assertFalse(request.enable_contact);self.assertEqual(request.num_max_contacts,1)

    def test_other_collision_or_deeper_overlap_or_bad_remainder_is_rejected(self):
        for contacts in (lambda k,q:{frozenset(('arm','torso')):.01},
                         lambda k,q:{guard.HEAD_PAIR:.01+(31.6-q[19])*.01}):
            c,request=self.controller()
            with patch.object(guard,'collision_depths',side_effect=contacts):
                error,result=guard.prepare_reset(c,self.start,self.goal)
            self.assertTrue(error);self.assertIsNone(result);self.assertFalse(request.enable_contact)
        c,_=self.controller();c._prepare_direct_quick_reset_path=lambda a,b:('arm collision',None)
        with patch.object(guard,'collision_depths',side_effect=lambda k,q:{guard.HEAD_PAIR:.01} if q[19]>31 else {}):
            error,result=guard.prepare_reset(c,self.start,self.goal)
        self.assertEqual(error,'arm collision');self.assertIsNone(result)

if __name__=='__main__':unittest.main()
