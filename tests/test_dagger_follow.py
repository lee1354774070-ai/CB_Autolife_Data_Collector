"""Execute the V4 authority gate without ROS or motor publishers."""
import ast
from pathlib import Path
import time
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]


class FollowTest(unittest.TestCase):
    def test_waist_only_while_fresh_expert_active(self):
        path = ROOT / 'teleop_snapshot/openarmx_teleop_vr_306_v4/openarmx_teleop_vr_306_v4/controller_node.py'
        tree = ast.parse(path.read_text())
        names = {'_follow_authority_allowed', '_waist_follow_active'}
        methods = [n for cls in tree.body if isinstance(cls, ast.ClassDef)
                   for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
        namespace = {'time': time}
        exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), namespace)
        node = SimpleNamespace(_follow_authority_required=True, _waist_follow_enabled=True,
                               _follow_authority_time=time.monotonic())
        node._follow_authority_allowed = lambda: namespace['_follow_authority_allowed'](node)
        for mode in ('DISARMED', 'POLICY_WARMUP', 'POLICY_ACTIVE', 'EXPERT_READY',
                     'EXPERT_ACTIVE', 'EXPERT_RELEASE_REQUIRED', 'ESTOP', 'FAILURE_HOLD', 'LAUNCHER'):
            node._follow_authority_mode = mode
            self.assertEqual(namespace['_waist_follow_active'](node), mode == 'EXPERT_ACTIVE', mode)
        node._follow_authority_mode = 'EXPERT_ACTIVE'
        node._follow_authority_time -= 1
        self.assertFalse(namespace['_waist_follow_active'](node))

    def test_launch_keeps_accepted_arm_mapping_and_gates_waist(self):
        tree = ast.parse((ROOT / 'dagger/stack.launch.py').read_text())
        calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Name) and n.func.id == 'process']
        params = []
        for call in calls:
            for kw in call.keywords:
                if kw.arg == 'params':
                    params.append({k.value: ast.literal_eval(v) for k, v in zip(kw.value.keys, kw.value.values)
                                   if isinstance(k, ast.Constant) and isinstance(v, ast.Constant)})
        controller = next(p for p in params if 'allow_waist_in_ik' in p)
        mapper = next(p for p in params if 'gripper_filter_alpha' in p)
        self.assertEqual(controller['allow_waist_in_ik'], 'true')
        self.assertEqual(controller['follow_authority_topic'], '/hg_dagger/control_state')
        self.assertEqual(controller['body_height_control_enabled'], 'false')
        self.assertEqual(controller['waist_forward_assist_command_lead_deg'], '5.0')
        self.assertNotIn('forward_position_scale', mapper)
        self.assertNotIn('wrist_reach_reserve_enabled', mapper)
        self.assertEqual(mapper['quick_reset_enabled'], 'false')
