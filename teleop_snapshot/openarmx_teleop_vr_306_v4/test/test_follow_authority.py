"""Run without ROS/hardware: pytest test/test_follow_authority.py."""
import ast
import json
import threading
from pathlib import Path
from types import SimpleNamespace, MethodType

SOURCE=(Path(__file__).resolve().parents[1]/'openarmx_teleop_vr_306_v4/controller_node.py').read_text()
TREE=ast.parse(SOURCE)

def test_authority_gate_and_reanchor():
    clock=SimpleNamespace(monotonic=lambda:10.0)
    scope={'time':clock,'json':json}
    names=('_follow_authority_allowed','_head_follow_active','_waist_follow_active','_on_follow_authority')
    for name in names:
        fn=next(n for n in ast.walk(TREE) if isinstance(n,ast.FunctionDef) and n.name==name)
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'<controller>','exec'),scope)
    node=SimpleNamespace(_lock=threading.RLock(),_follow_authority_required=True,
        _follow_authority_mode='',_follow_authority_time=0.,_head_follow_enabled=True,
        _waist_follow_enabled=True,_feedback=SimpleNamespace(leg_waist=[0,0,14,5]))
    for name in names:setattr(node,name,MethodType(scope[name],node))
    def state(mode):node._on_follow_authority(SimpleNamespace(data=json.dumps({'mode':mode})))
    for mode in ('DISARMED','POLICY_WARMUP','POLICY_ACTIVE','EXPERT_READY','EXPERT_RELEASE_REQUIRED','ESTOP'):
        state(mode)
        assert not node._head_follow_active() and not node._waist_follow_active()
        assert node._follow_authority_allowed(for_arm=True) == (mode == 'EXPERT_READY')
    state('EXPERT_READY')
    assert node._follow_authority_allowed(for_arm=True)
    assert not node._head_follow_active() and not node._waist_follow_active()
    state('EXPERT_ACTIVE')
    assert node._head_follow_active() and node._waist_follow_active()
    assert node._head_target is None and node._waist_follow_neutral_pitch_deg==14
    assert node._waist_follow_locked_yaw_deg==5
    clock.monotonic=lambda:10.6
    assert not node._head_follow_active() and not node._waist_follow_active()
    assert not node._follow_authority_allowed(for_arm=True)
    state('EXPERT_ACTIVE');assert node._head_follow_active()
    node._head_follow_enabled=False;assert not node._head_follow_active()
    node._follow_authority_required=False;state('POLICY_ACTIVE')
    assert node._waist_follow_active()  # Ordinary non-DAgger teleop is unchanged.


def test_output_gates_cover_head_and_waist():
    functions={n.name:ast.get_source_segment(SOURCE,n) for n in ast.walk(TREE) if isinstance(n,ast.FunctionDef)}
    tick=functions['_control_tick']
    assert '_head_follow_active()' in tick and '_waist_follow_active()' in tick
    assert "self._target_source == 'joint_space'" in tick
    assert '_head_follow_active()' in functions['_on_head_target']
    worker=functions['_process_latest_target']
    assert worker.index('if not self._follow_authority_allowed(for_arm=True):') < worker.index('self._last_ik = result')


def test_supervisor_heartbeat_replaces_only_vr_requirement():
    fn=next(n for n in ast.walk(TREE) if isinstance(n,ast.FunctionDef) and n.name=='_heartbeat_guard_error')
    scope={};exec(compile(ast.Module(body=[fn],type_ignores=[]),'<guard>','exec'),scope)
    params={'authority_heartbeat_timeout_sec':1.0,'require_teleop_heartbeat':True,'teleop_heartbeat_timeout_sec':5.5}
    node=SimpleNamespace(_authority_heartbeat_required=True,_authority_heartbeat_time=10.,
        _teleop_heartbeat_time=0.,_teleop_heartbeat_rejection='VR missing',
        get_parameter=lambda name:SimpleNamespace(value=params[name]))
    guard=MethodType(scope['_heartbeat_guard_error'],node)
    assert guard(10.2,require_fresh=True)==''  # no VR, but live supervisor
    assert 'expired' in guard(11.1)
    node._authority_heartbeat_time=0
    assert 'missing' in guard(0.1)
    node._authority_heartbeat_required=False
    assert guard(10.2,require_fresh=True)=='VR missing'  # shared ordinary teleop unchanged
    enable=next(n for n in ast.walk(TREE) if isinstance(n,ast.FunctionDef) and n.name=='_on_hardware_enabled')
    source=ast.get_source_segment(SOURCE,enable)
    assert 'self._estop_latched' in source and 'self._dry_run' in source
