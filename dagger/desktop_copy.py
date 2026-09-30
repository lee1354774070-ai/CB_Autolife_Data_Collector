#!/usr/bin/env python3
"""Build a compatible COPY of the reviewed colleague desktop launcher.

The original file and running processes are never changed. The copy keeps the
same UI but launches one integrated host; additional terminals attach to it.
Only reviewed source identities are accepted, so changed UI/cleanup code must
be inspected before this adapter is used with a new colleague revision.
"""

import argparse
import ast
import hashlib
from pathlib import Path

REVIEWED = {
    '57eabfaf38daf63d9e37a5e7bf18ab65a1a0fac168fae7efbb35b63598e0014f',
    'cf9bbba84cf66edeb25409aa460d7e9ac5a417e4ad847d4e5a859eb24036c653',
}

LABELS = {
    '云蝶DAgger启动器': 'Collector compatible DAgger host',
    '丢弃 / 全身复位': 'Discard (no reset)',
    '保存并退出': 'Stop host',
    '丢弃本条并机械复位？': 'Discard this episode without resetting?',
    'A 开始   ·   B 保存后复位   ·   X 丢弃后复位   ·   双 Y 退出': 'Y: start | Grip: immediate takeover | A: save | X: discard | B: reset',
    '本窗口 Shift+A 开始 / B 保存 / X 丢弃 / Y 五秒内按两次退出': 'Shift+Y: start | Shift+A: save | Shift+X: discard | Shift+B: reset',
    '接管：手柄 Grip   /   键盘：控制页面 Shift + A / B / X / Y': 'Grip: takeover | Y: start | A: save | X: discard | B: reset',
    '未接管，结束时丢弃': 'No takeover; A still saves the episode',
    '数据位置  /home/ubuntu/nas/dagger': 'Data location: OUTPUT_BASE_DIR (default /home/ubuntu/nas14)',
}


def render(source, tools_root):
    if hashlib.sha256(source).hexdigest() not in REVIEWED:
        raise ValueError('Unreviewed desktop launcher; inspect its code before adapting it')
    tree = ast.parse(source.decode())
    run = str(Path(tools_root).resolve() / 'lerobot_data_collector/dagger/run.py')

    class Adapter(ast.NodeTransformer):
        def visit_Constant(self, node):
            return ast.copy_location(ast.Constant(LABELS.get(node.value, node.value)), node) if isinstance(node.value, str) else node

        def visit_Assign(self, node):
            if any(isinstance(t, ast.Name) and t.id == 'DATA_ROOT' for t in node.targets):
                node.value = ast.parse("Path(os.environ.get('OUTPUT_BASE_DIR', '/home/ubuntu/nas14'))", mode='eval').body
            if any(isinstance(t, ast.Name) and t.id == 'LOGS' for t in node.targets):
                node.value = ast.parse("Path.home() / '.local/state/collector-compatible-dagger'", mode='eval').body
            if any(isinstance(t, ast.Name) and t.id == 'expected' for t in node.targets):
                node.value = ast.parse("DATA_ROOT / self.task / '.official_recording_control'", mode='eval').body
            return self.generic_visit(node)

        def visit_Call(self, node):
            if isinstance(node.func, ast.Attribute) and node.func.attr == 'Popen' and node.args and isinstance(node.args[0], ast.List):
                if 'quickstart_dagger.sh' in ast.unparse(node.args[0]):
                    node.args[0] = ast.parse(f"[os.environ.get('DAGGER_ROS_PY', '/usr/bin/python3'), {run!r}, task, instruction, '--serve']", mode='eval').body
                    for keyword in node.keywords:
                        if keyword.arg == 'env':
                            keyword.value = ast.parse("{**os.environ, 'DAGGER_BACKEND':'owned', 'OUTPUT_BASE_DIR':str(DATA_ROOT), 'TASK_TEXT':instruction, **{key:str(int(value.get())) for key,value in self.options.items()}}", mode='eval').body
            return self.generic_visit(node)

        def visit_FunctionDef(self, node):
            if node.name == 'command':
                node.body.insert(0, ast.parse("if action == 'quit':\n    return self.stop()").body[0])
            if node.name == 'shortcut_dispatch':
                node.body = ast.parse("""
if key not in self.shortcut_held:
    return
self.shortcut_held.pop(key)
action = {'y':'start', 'a':'finish', 'x':'discard', 'b':'reset'}[key]
button = self.control_buttons[action]
if str(button['state']) != 'disabled':
    button.invoke()
""").body
            return self.generic_visit(node)

    tree = ast.fix_missing_locations(Adapter().visit(tree))
    compile(tree, '<compatible-desktop>', 'exec')
    return '# Generated compatible COPY; colleague source unchanged.\n' + ast.unparse(tree) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--tools-root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source, output = args.source.resolve(), args.output.resolve()
    source_root = source.parent.parent if source.parent.name == 'scripts' else source.parent
    if source == output or source_root in output.parents:
        parser.error('Output must be outside the colleague source tree')
    content = render(source.read_bytes(), args.tools_root)
    if output.exists():
        if output.read_text() != content:
            parser.error('Output already exists with different contents; choose a new copy path')
    else:
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('x') as stream:
            stream.write(content)
    print(output)


if __name__ == '__main__':
    main()
