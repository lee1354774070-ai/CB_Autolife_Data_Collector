import ast
from pathlib import Path


CONTROLLER = (
    Path(__file__).resolve().parents[1]
    / 'openarmx_teleop_vr_306_v4'
    / 'controller_node.py'
)
SOURCE = CONTROLLER.read_text(encoding='utf-8')
TREE = ast.parse(SOURCE)


def _function(name):
    for node in ast.walk(TREE):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f'{name} was not found')


def _source(name):
    return ast.get_source_segment(SOURCE, _function(name))


def _call_line(function_name, attribute):
    calls = [
        node.lineno
        for node in ast.walk(_function(function_name))
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == attribute
        )
    ]
    assert calls, f'{attribute} was not called by {function_name}'
    return min(calls)


def test_quick_reset_validates_all_rejectable_inputs_before_session_commit():
    function = _function('_start_quick_reset_locked')
    commit_lines = [
        node.lineno
        for node in ast.walk(function)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Attribute)
            and target.attr in {
                '_grip_release_held', '_clutch_session', '_clutch_sequence',
                '_target_groups', '_reset_active',
            }
            for target in node.targets
        )
    ]
    first_commit = min(commit_lines)

    # Every return before the commit belongs to validation.  There must be no
    # rejectable return after session state starts changing.
    late_returns = [
        node.lineno
        for node in ast.walk(function)
        if isinstance(node, ast.Return)
        and node.lineno >= first_commit
        and node.value is not None
        and not (
            isinstance(node.value, ast.Constant) and node.value.value == ''
        )
    ]
    assert late_returns == []
    assert _call_line('_start_quick_reset_locked', '_configuration_guard_error') < first_commit
    assert _call_line('_start_quick_reset_locked', '_build_motion_controller_state') < first_commit


def test_quick_reset_invalidates_mailbox_only_after_motion_state_builds():
    assert (
        _call_line('_start_quick_reset_locked', '_build_motion_controller_state')
        < _call_line('_start_quick_reset_locked', '_prepare_quick_reset_path')
        < _call_line('_start_quick_reset_locked', 'reset')
        < _call_line('_start_quick_reset_locked', '_commit_motion_controller_state')
    )


def test_v4_reset_uses_one_continuous_direct_path():
    start_source = _source('_start_quick_reset_locked')
    tick_source = _source('_control_tick')
    prepare_source = _source('_prepare_quick_reset_path')
    assert '_prepare_quick_reset_path' in start_source
    assert 'continuous_direct_reset' in tick_source
    assert 'continuous_outward_reset' not in tick_source
    assert 'outward_clearance' not in prepare_source
    assert '_reset_phase' not in start_source
    assert '_reset_waypoint' not in start_source


def test_xa_quick_reset_always_uses_configured_neck_default():
    source = _source('_start_quick_reset_locked')
    assert "get_parameter('quick_reset_neck_joints')" in source
    assert "groups['neck'] = neck" in source
    assert 'reset_neck_to_default' not in source


def test_quick_reset_completion_waits_for_physical_neck():
    source = _source('_reset_required_vector')
    assert 'IndependentArmController._reset_vector(groups)' in source
    assert 'IndependentArmController._enable_vector(groups)' not in source


def test_quick_reset_and_gripper_hold_reset_keep_body_height():
    start_source = _source('_start_quick_reset_locked')
    vector_source = _source('_reset_vector')
    assert "groups['leg_waist'][2:4]" not in start_source
    assert "get_parameter('quick_reset_waist_joints')" not in start_source
    assert "groups['leg_waist']" not in vector_source
    assert "groups['leg_waist'][:3] = 0.0" not in start_source
    assert 'measured_reverse_squat(' in start_source
    assert "groups['leg_waist']" in _source('_reset_required_vector')
    assert '_deactivate_body_height_locked()' in start_source
    assert "clamped['leg_waist'][2]" in start_source


def test_fresh_session_builds_before_invalidating_mailbox():
    assert (
        _call_line('_begin_fresh_teleop_session_locked', '_build_motion_controller_state')
        < _call_line('_begin_fresh_teleop_session_locked', 'reset')
        < _call_line('_begin_fresh_teleop_session_locked', '_commit_motion_controller_state')
    )


def test_every_enable_attempt_requests_fresh_vendor_sync_confirmation():
    hold_stage = _source('_run_hardware_enable').split(
        "if self._enable_stage == 'hold_before_enable':", 1
    )[1].split("if self._enable_stage == 'waiting_sync_session':", 1)[0]
    assert 'self._sync_session_client.call_async' in hold_stage
    assert 'if not self._sync_session_acquired' not in hold_stage

    enable_service = _source('_on_hardware_enabled')
    readiness_guard = enable_service.split(
        "bool(self.get_parameter('require_sync_session_service').value)", 1
    )[1].split("response.message = (", 1)[0]
    assert '_sync_session_acquired' not in readiness_guard


def test_full_body_reset_reuses_enable_guards_and_does_not_change_normal_start():
    from types import SimpleNamespace
    from unittest.mock import Mock
    import threading
    scope={'SetBool':SimpleNamespace(Request=lambda **kw:SimpleNamespace(**kw),Response=lambda:SimpleNamespace())}
    fn=_function('_on_full_body_reset')
    exec(compile(ast.Module(body=[fn],type_ignores=[]),'<full-reset>','exec'),scope)
    node=SimpleNamespace(_lock=threading.RLock(),_hardware_enabled=False,_enable_pending=False,
        _reset_active=False,_on_hardware_enabled=Mock(return_value=SimpleNamespace(success=True,message='accepted')))
    response=scope['_on_full_body_reset'](node,None,SimpleNamespace())
    assert response.success
    assert node._on_hardware_enabled.call_args.kwargs=={'force_full_reset':True}
    node._hardware_enabled=True
    assert not scope['_on_full_body_reset'](node,None,SimpleNamespace()).success
    assert node._on_hardware_enabled.call_count==1
    enable=_source('_on_hardware_enabled')
    assert 'force_full_reset=False' in enable
    assert "self._estop_latched" in enable and 'self._heartbeat_guard_error' in enable
    assert 'self._enable_full_reset' in _source('_on_command_observed')
    assert 'self._enable_full_reset' in _source('_run_hardware_enable')
