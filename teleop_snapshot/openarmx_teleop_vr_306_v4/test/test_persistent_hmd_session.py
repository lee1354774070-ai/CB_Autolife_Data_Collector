import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONTROLLER = ROOT / 'openarmx_teleop_vr_306_v4' / 'controller_node.py'
MAPPER = ROOT / 'openarmx_teleop_vr_306_v4' / 'vr_mapper_node.py'
BRIDGE = ROOT / 'openarmx_teleop_vr_306_v4' / 'vr_web_bridge.py'
CONFIG = ROOT / 'config' / 'teleop.yaml'
WEB = ROOT / 'web' / 'vr_app.js'


def test_sources_parse_and_disconnect_does_not_implicitly_disable():
    for path in (CONTROLLER, MAPPER, BRIDGE):
        ast.parse(path.read_text(encoding='utf-8'))
    assert "disable_hardware_on_last_disconnect', False" in BRIDGE.read_text(
        encoding='utf-8'
    )
    assert 'disable_on_vr_timeout: false' in CONFIG.read_text(encoding='utf-8')
    assert "'disable_on_vr_timeout': False" in MAPPER.read_text(encoding='utf-8')
    assert '后台正在自动关闭真机遥操' not in WEB.read_text(encoding='utf-8')


def test_hold_heartbeat_keeps_only_an_existing_session_alive():
    controller = CONTROLLER.read_text(encoding='utf-8')
    mapper = MAPPER.read_text(encoding='utf-8')
    assert "payload.get('hold_only') is True" in controller
    assert "self._teleop_heartbeat_mode = mode" in controller
    assert 'require_fresh and self._teleop_heartbeat_mode' in controller
    assert controller.count('require_fresh=True') >= 2
    assert "'hold_only': True" in mapper
    assert "elif self._hardware_enabled:" in mapper
    assert "self._release_all(require_release=True" in mapper


def test_heartbeat_classifier_separates_tracking_from_hold_authority():
    source = CONTROLLER.read_text(encoding='utf-8')
    tree = ast.parse(source)
    function = next(
        node for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == 'classify_teleop_heartbeat'
    )
    namespace = {
        'vr_input_is_fresh': lambda age, maximum: (
            age is not None and 0.0 <= float(age) <= float(maximum)
        )
    }
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(CONTROLLER), 'exec'), namespace)
    classify = namespace['classify_teleop_heartbeat']

    assert classify({
        'vr_input_fresh': True,
        'vr_age': 0.02,
        'tracked_hands': ['left'],
        'mapper_alive': True,
        'hold_only': False,
    }, 0.8) == ('tracking', 0.02, ['left'])
    assert classify({
        'vr_input_fresh': False,
        'vr_age': 30.0,
        'tracked_hands': [],
        'mapper_alive': True,
        'hold_only': True,
    }, 0.8) == ('hold', None, [])

    for invalid in (
        {'vr_input_fresh': True, 'vr_age': 30.0, 'tracked_hands': ['left']},
        {'vr_input_fresh': False, 'tracked_hands': [], 'hold_only': True},
        {'vr_input_fresh': False, 'tracked_hands': ['left'], 'mapper_alive': True,
         'hold_only': True},
    ):
        try:
            classify(invalid, 0.8)
        except ValueError:
            pass
        else:
            raise AssertionError(f'invalid heartbeat was accepted: {invalid}')
