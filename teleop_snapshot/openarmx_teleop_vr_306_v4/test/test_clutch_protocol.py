import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAPPER = ROOT / 'openarmx_teleop_vr_306_v4' / 'vr_mapper_node.py'
CONTROLLER = ROOT / 'openarmx_teleop_vr_306_v4' / 'controller_node.py'


def _function(path, name):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == name:
            return node
    raise AssertionError(f'{name} was not found in {path}')


def test_mapper_payload_contains_complete_sequenced_clutch_state():
    module = ast.parse(MAPPER.read_text(encoding='utf-8'))
    function = next(
        node for node in module.body
        if isinstance(node, ast.FunctionDef) and node.name == 'eef_target_payload'
    )
    namespace = {}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(MAPPER), 'exec'), namespace)
    payload = namespace['eef_target_payload'](
        {}, clutch_session='session-a', clutch_sequence=7,
        active_arms=['left'],
    )
    assert payload['clutch_session'] == 'session-a'
    assert payload['clutch_sequence'] == 7
    assert payload['clutch_state'] == {'left': True, 'right': False}


def test_release_is_applied_before_latest_ik_mailbox_put():
    function = _function(CONTROLLER, '_on_target')
    calls = [
        (node.lineno, node.func.attr)
        for node in ast.walk(function)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    ]
    call_lines = {name: line for line, name in calls}
    assert call_lines['_apply_clutch_envelope_locked'] < call_lines['put']


def test_reanchor_hold_is_released_only_after_successful_ik():
    process = ast.get_source_segment(
        CONTROLLER.read_text(encoding='utf-8'),
        _function(CONTROLLER, '_process_latest_target'),
    )
    assert 'if result.success:' in process
    success = process.split('if result.success:', 1)[1]
    assert 'self._grip_release_held.discard(side)' in success
    prefix = process.split('if result.success:', 1)[0]
    assert 'self._grip_release_held.discard(side)' not in prefix


def test_arm_enable_excludes_neck_but_full_reset_waits_for_neck():
    controller = CONTROLLER.read_text(encoding='utf-8')
    assert 'ENABLE_TRACKED_JOINT_NAMES = ARM_JOINT_NAMES + WAIST_JOINT_NAMES' in controller
    required = ast.get_source_segment(
        controller, _function(CONTROLLER, '_reset_required_vector')
    )
    assert '_reset_vector(groups)' in required
    enable_vector = ast.get_source_segment(
        controller, _function(CONTROLLER, '_enable_vector')
    )
    assert "groups['neck']" not in enable_vector


def test_new_mapper_session_forces_both_arms_through_measured_hold():
    controller = CONTROLLER.read_text(encoding='utf-8')
    function = ast.get_source_segment(
        controller, _function(CONTROLLER, '_apply_clutch_envelope_locked')
    )
    assert 'self._grip_release_held.clear()' in function
    assert 'if new_session or (' in function
    assert 'self._hold_sides_locked(released, now)' in function
