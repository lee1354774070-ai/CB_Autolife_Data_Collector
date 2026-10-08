import ast

from _subject import source_path


def controller_method(name):
    tree = ast.parse(source_path('controller_node').read_text(encoding='utf-8'))
    controller = next(
        item for item in tree.body
        if isinstance(item, ast.ClassDef)
        and item.name == 'IndependentArmController'
    )
    return next(
        item for item in controller.body
        if isinstance(item, ast.FunctionDef) and item.name == name
    )


def test_live_waist_disable_uses_real_monotonic_clock():
    method = controller_method('_on_waist_follow_enabled')
    calls = [
        node for node in ast.walk(method)
        if isinstance(node, ast.Call)
    ]
    assert any(
        isinstance(call.func, ast.Attribute)
        and isinstance(call.func.value, ast.Name)
        and call.func.value.id == 'time'
        and call.func.attr == 'monotonic'
        for call in calls
    )
    assert not any(
        isinstance(call.func, ast.Attribute)
        and isinstance(call.func.value, ast.Name)
        and call.func.value.id == 'self'
        and call.func.attr == '_now'
        for call in calls
    )
