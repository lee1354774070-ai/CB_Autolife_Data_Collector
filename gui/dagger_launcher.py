# Generated compatible COPY; colleague source unchanged.
"""Quiet desktop launcher; the existing collector owns all robot operations."""
import datetime
import os
import json
import re
import time
import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from pathlib import Path
import signal
import subprocess
import tkinter as tk
from tkinter import messagebox
BASE = Path('/home/ubuntu/CB_Autolife_Data_Collector')
LOGS = Path.home() / '.local/state/collector-compatible-dagger'
DATA_ROOT = Path(os.environ.get('OUTPUT_BASE_DIR', '/home/ubuntu/nas14'))

def reserve_task(root, remark, now=None):
    remark = remark.strip().lstrip('_')
    if remark and (not re.fullmatch('[A-Za-z0-9][A-Za-z0-9_-]{0,95}', remark)):
        raise ValueError('备注可留空；填写时请用1–96位英文字母、数字、下划线或连字符')
    now = now or datetime.datetime.now()
    name = 'hotel_scene_dataset_' + now.strftime('%y%m%d') + '_' + (remark or now.strftime('%H%M%S_%f'))
    root.mkdir(parents=True, exist_ok=True)
    try:
        (root / name).mkdir()
    except FileExistsError:
        name += now.strftime('_%H%M%S_%f')
        (root / name).mkdir()
    return name
(BG, CARD, INK, MUTED, BLUE) = ('#101827', '#1b273b', '#eef4ff', '#a4b2ca', '#6fa8ff')

def describe_state(state):
    """Render the supervisor snapshot; command acceptance is not save success."""
    mode = state.get('mode', '')
    modes = {'DISARMED': '待开始', 'POLICY_ACTIVE': '模型控制', 'POLICY_WARMUP': '模型准备中', 'EXPERT_READY': '等待手柄接管', 'EXPERT_ACTIVE': '人工接管', 'EXPERT_RELEASE_REQUIRED': '等待松开 Grip', 'FAILURE_HOLD': '故障保持', 'ESTOP': '安全停止'}
    trial = state.get('trial', {})
    reset = state.get('quick_reset', {})
    title = '全身复位中' if reset.get('pending') or reset.get('active') else modes.get(mode, mode or '状态未知')
    if state.get('collection_exited'):
        title = '采集已退出'
    recording = '录制中' if trial.get('recording') else '未录制'
    if trial.get('save_pending'):
        recording = '保存待确认（请勿关闭）'
    if trial.get('invalid_reason'):
        recording = '录制失效'
    lines = [f'{title}  ·  {recording}', f"本条 {trial.get('recorded_frames', 0)} 帧  /  人工 {trial.get('expert_frames', 0)} 帧  ·  " + ('已接管' if trial.get('human_takeover') else '无接管也可按 B 保存整条')]
    notice = state.get('notice', {})
    error = trial.get('invalid_reason') or state.get('gripper_tracking_warning') or state.get('failure_reason')
    if error:
        lines.append('异常：' + str(error))
    if notice.get('text'):
        lines.append(str(notice['text']))
    if mode == 'EXPERT_ACTIVE':
        sides = '、'.join((label for (side, label) in [('left', '左'), ('right', '右')] if state.get('gripper_pickup_pending', {}).get(side)))
        if sides:
            lines.append(f'{sides}夹爪保持中：Trigger 对齐后可控；此提示不阻止录制')
    color = '#ffb09a' if error or mode == 'ESTOP' or notice.get('level') == 'error' else '#8bd5ae'
    return ('\n'.join(lines), color)

class Launcher:

    def __init__(self, root):
        (self.root, self.process, self.log_path, self.reader) = (root, None, None, None)
        self.stopping = False
        self.task = ''
        self.variant = 'rgbd'
        (self.control_state, self.state_at, self.request) = ({}, 0.0, None)
        self.node = rclpy.create_node('dagger_desktop_' + str(os.getpid()))
        self.node.create_subscription(String, '/hg_dagger/control_state', self.on_state, 1)
        self.clients = {name: self.node.create_client(Trigger, '/hg_dagger/launcher/' + name) for name in ('start', 'finish', 'discard', 'reset', 'quit')}
        self.estop_client = self.node.create_client(Trigger, '/openarmx_teleop_vr_306_v4/emergency_stop')
        self.estop_request = None
        root.title('云蝶DAgger启动器')
        root.geometry('820x900')
        root.resizable(False, False)
        root.configure(bg=BG)
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.shortcut_held = {}
        self.shortcut_y_at = None
        root.bind('<KeyPress>', self.shortcut_press, add='+')
        root.bind('<KeyRelease>', self.shortcut_release, add='+')
        root.bind('<FocusOut>', self.shortcut_cancel, add='+')
        body = tk.Frame(root, bg=BG, padx=32, pady=26)
        body.pack(fill='both', expand=True)
        self.label(body, '云蝶  /  ROBOT 300', 11, BLUE).pack(anchor='w')
        self.label(body, 'DAgger 启动器', 25, INK, True).pack(anchor='w', pady=(8, 4))
        self.label(body, '启动服务 → 在此开始 / 结束本条；人工接管时再连接头显', 11).pack(anchor='w')
        panel = tk.Frame(body, bg=CARD, padx=20, pady=16)
        panel.pack(fill='x', pady=20)
        self.label(panel, '任务指令', 11, MUTED).pack(anchor='w')
        self.instruction = tk.Entry(panel, font=('Sans', 12), bg=BG, fg=INK, insertbackground=INK, relief='flat')
        self.instruction.insert(0, 'Pick the laundry bag.')
        self.instruction.pack(fill='x', ipady=9, pady=(8, 12))
        self.label(panel, '备注（选填；留空自动按时间命名）', 11).pack(anchor='w')
        self.remark = tk.Entry(panel, font=('Sans', 12), bg=BG, fg=INK, insertbackground=INK, relief='flat')
        self.remark.pack(fill='x', ipady=9, pady=(8, 12))
        options = tk.Frame(panel, bg=CARD)
        options.pack(fill='x')
        (self.options, self.option_widgets) = ({}, [])
        for (key, label) in [('WITH_HEAD', '录制头部'), ('WITH_UPPER_WAIST', '录制上腰'), ('WITH_DEPTH', '录制深度')]:
            value = tk.BooleanVar(value=True)
            self.options[key] = value
            check = tk.Checkbutton(options, text=label, variable=value, bg=CARD, fg=INK, selectcolor=BG, activebackground=CARD, activeforeground=INK, font=('Sans', 11))
            check.pack(side='left', padx=(0, 14))
            self.option_widgets.append(check)
        self.schema_hint = self.label(panel, '', 10)
        self.schema_hint.pack(anchor='w', pady=(6, 0))
        for value in self.options.values():
            value.trace_add('write', lambda *_: self.update_schema_hint())
        self.update_schema_hint()
        self.status = self.label(body, '●  未启动', 15, BLUE, True)
        self.status.pack(anchor='w')
        self.detail = self.label(body, '准备好后点击启动。详细输出仅写入日志。', 11)
        self.detail.configure(wraplength=600, justify='left')
        self.detail.pack(anchor='w', pady=(8, 18))
        actions = tk.Frame(body, bg=BG)
        actions.pack(fill='x')
        self.start_button = self.button(actions, '启动采集服务', self.start, BLUE, BG)
        self.start_button.pack(side='left')
        self.stop_button = self.button(actions, '停止服务', self.stop)
        self.stop_button.pack(side='left', padx=12)
        self.stop_button.configure(state='disabled')
        self.log_button = self.button(actions, '查看日志', self.logs)
        self.log_button.pack(side='right')
        self.log_button.configure(state='disabled')
        controls = tk.Frame(body, bg=BG)
        controls.pack(fill='x', pady=(16, 0))
        self.control_buttons = {}
        for (index, (name, title)) in enumerate([('start', '开始推理 + 录制'), ('finish', '结束本条 / 保存'), ('discard', '丢弃本条（不复位）'), ('reset', '全身复位'), ('quit', '退出采集')]):
            button = self.button(controls, title, lambda name=name: self.command(name))
            button.configure(state='disabled', padx=10)
            button.grid(row=index // 3, column=index % 3, sticky='ew', padx=(0, 8), pady=(0, 8))
            self.control_buttons[name] = button
        self.estop_button = self.button(controls, '急停（锁定）', self.emergency_stop, '#c73838', '#ffffff')
        self.estop_button.grid(row=1, column=2, sticky='ew', padx=(0, 8), pady=(0, 8))
        self.live = self.label(body, '等待后台状态', 11)
        self.live.configure(wraplength=690, justify='left')
        self.live.pack(anchor='w', pady=(8, 0))
        self.synced = self.label(body, '等待后台状态同步', 11)
        self.synced.configure(wraplength=690, justify='left')
        self.synced.pack(anchor='w', pady=(8, 0))
        self.label(body, 'A 开始 · 握持接管 · B 保存 · Y 丢弃 · X 仅复位', 11).pack(anchor='w', pady=(12, 8))
        self.label(body, 'Shift+A 开始 / Shift+B 保存 / Shift+Y 丢弃 / Shift+X 仅复位', 10).pack(anchor='w')
        self.label(body, '数据位置由 OUTPUT_BASE_DIR 指定', 10).pack(anchor='w', pady=(12, 0))
        root.after(400, self.poll)

    def shortcut_press(self, event):
        key = event.keysym.lower()
        if key not in 'abxy' or len(key) != 1:
            return
        if key in self.shortcut_held:
            pending = self.shortcut_held[key]
            if pending is not None:
                self.root.after_cancel(pending)
                self.shortcut_held[key] = None
            return 'break'
        if not event.state & 1 or event.state & (4 | 8 | 64 | 128):
            return
        if isinstance(event.widget, (tk.Entry, tk.Text, tk.Spinbox)) and str(event.widget['state']) != 'disabled':
            return
        self.shortcut_held[key] = None
        return 'break'

    def shortcut_release(self, event):
        key = event.keysym.lower()
        if key in self.shortcut_held:
            self.shortcut_held[key] = self.root.after_idle(lambda : self.shortcut_dispatch(key))
            return 'break'

    def shortcut_cancel(self, _event=None):
        for pending in self.shortcut_held.values():
            if pending is not None:
                self.root.after_cancel(pending)
        self.shortcut_held.clear()
        self.shortcut_y_at = None

    def shortcut_dispatch(self, key):
        if key not in self.shortcut_held:
            return
        self.shortcut_held.pop(key)
        action = {'a': 'start', 'b': 'finish', 'y': 'discard', 'x': 'reset'}[key]
        button = self.control_buttons[action]
        if str(button['state']) != 'disabled':
            button.invoke()

    def label(self, parent, text, size, color=MUTED, bold=False):
        return tk.Label(parent, text=text, bg=parent['bg'], fg=color, font=('Sans', size, 'bold' if bold else 'normal'))

    def button(self, parent, text, command, bg=CARD, fg=INK):
        return tk.Button(parent, text=text, command=command, bg=bg, fg=fg, activebackground=BLUE, relief='flat', cursor='hand2', font=('Sans', 11, 'bold'), padx=16, pady=10)

    def update_schema_hint(self):
        size = 16 + 3 * self.options['WITH_HEAD'].get() + 2 * self.options['WITH_UPPER_WAIST'].get()
        self.schema_hint.configure(text=f'数据 state/action：{size}维；当前模型要求21维，请保持头部和上腰开启')

    def start(self):
        if self.process and self.process.poll() is None:
            return
        instruction = self.instruction.get().strip()
        if not instruction:
            self.detail.configure(text='请填写任务指令。')
            return
        try:
            task = reserve_task(DATA_ROOT, self.remark.get())
        except (ValueError, OSError) as exc:
            self.detail.configure(text=str(exc))
            return
        self.task = task
        self.variant = 'rgbd' if self.options['WITH_DEPTH'].get() else 'rgb'
        (self.control_state, self.state_at) = ({}, 0.0)
        LOGS.mkdir(parents=True, exist_ok=True)
        self.log_path = LOGS / (task + '.log')
        try:
            with self.log_path.open('w') as output:
                self.process = subprocess.Popen([os.environ.get('DAGGER_ROS_PY', '/usr/bin/python3'), '/home/ubuntu/CB_Autolife_Data_Collector/dagger/run.py', task, instruction, '--serve'], stdout=output, stderr=subprocess.STDOUT, start_new_session=True, env={**os.environ, 'DAGGER_BACKEND': 'owned', 'OUTPUT_BASE_DIR': str(DATA_ROOT), 'TASK_TEXT': instruction, **{key: str(int(value.get())) for (key, value) in self.options.items()}})
        except OSError as exc:
            self.status.configure(text='●  启动失败', fg='#ffb09a')
            self.detail.configure(text=str(exc))
            return
        if self.reader:
            self.reader.close()
        self.reader = self.log_path.open(errors='replace')
        self.pending = ''
        self.failure = ''
        self.stopping = False
        self.start_button.configure(state='disabled')
        self.instruction.configure(state='disabled')
        self.remark.configure(state='disabled')
        for widget in self.option_widgets:
            widget.configure(state='disabled')
        self.stop_button.configure(state='normal')
        self.log_button.configure(state='normal')
        self.status.configure(text='●  正在检查与启动', fg=BLUE)
        self.detail.configure(text='本次目录：' + task)

    def on_state(self, message):
        try:
            state = json.loads(message.data)
        except (ValueError, TypeError):
            return
        expected = DATA_ROOT / self.task / '.official_recording_control'
        if not self.task or state.get('collector_fifo') != str(expected):
            return
        (self.control_state, self.state_at) = (state, time.monotonic())

    def emergency_stop(self):
        if self.estop_request:
            return
        if not self.estop_client.service_is_ready():
            self.live.configure(text='急停接口不可达，请使用实体急停')
            return
        self.estop_request = self.estop_client.call_async(Trigger.Request())
        self.estop_requested_at = time.monotonic()
        self.live.configure(text='急停请求已发送，等待控制器确认')

    def command(self, action):
        if action == 'quit':
            return self.stop()
        if not self.process or self.process.poll() is not None or self.request or (time.monotonic() - self.state_at > 3):
            return
        if action == 'discard' and (not messagebox.askokcancel('丢弃本条', '丢弃当前条？本操作不复位。', parent=self.root)):
            return
        client = self.clients[action]
        if not client.service_is_ready():
            self.live.configure(text='后台接口未就绪，请稍后重试')
            return
        self.request = client.call_async(Trigger.Request())
        self.request_at = time.monotonic()
        self.live.configure(text='请求已发送，等待后台确认…')

    def poll(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        rclpy.spin_once(self.node, timeout_sec=0)
        now = time.monotonic()
        fresh = self.process is not None and self.process.poll() is None and (now - self.state_at <= 3)
        state = self.control_state
        available = fresh and (not self.stopping) and (not state.get('launcher_pending')) and (not state.get('collection_exited'))
        if self.request:
            if self.request.done():
                try:
                    result = self.request.result()
                    self.live.configure(text=result.message)
                except Exception as exc:
                    self.live.configure(text='控制请求失败：' + str(exc))
                self.request = None
            elif now - self.request_at > 5:
                self.request.cancel()
                self.request = None
                self.live.configure(text='请求结果未知，请查看后台状态；不会自动重试')
        if fresh:
            (text, color) = describe_state(state)
            self.synced.configure(text=text, fg=color)
        else:
            self.synced.configure(text='后台状态未连接或已过期；录制/保存结果尚未确认', fg=MUTED)
        for (action, button) in self.control_buttons.items():
            allowed = available and (not self.request)
            if action == 'start':
                allowed = allowed and (not state.get('operation_busy')) and (state.get('mode') == 'DISARMED')
            button.configure(state='normal' if allowed else 'disabled')
        if self.process:
            chunk = self.reader.read(32768) if self.reader else ''
            lines = (self.pending + chunk).split('\n')
            self.pending = lines.pop()[-8192:]
            for line in lines:
                if 'FAIL ' in line or 'PREFLIGHT FAILED' in line or '快捷启动已在运行' in line:
                    if not self.failure:
                        self.failure = line.strip()[:220]
                    self.status.configure(text='●  启动检查未通过', fg='#ffb09a')
                    self.detail.configure(text=self.failure)
                elif 'VR网页与WebSocket已共用' in line and (not self.failure):
                    self.status.configure(text='●  服务已启动', fg='#8bd5ae')
                    self.detail.configure(text='服务已就绪，可直接开始本条；头显仅在人工接管时需要。')
            code = self.process.poll()
            if code is not None:
                failed = bool(self.failure) or (code != 0 and (not self.stopping))
                self.status.configure(text='●  启动或运行异常' if failed else '●  服务已结束', fg='#ffb09a' if failed else MUTED)
                self.detail.configure(text=self.failure or ('请查看日志了解原因。' if failed else '可以重新启动下一次采集。'))
                self.process = None
                self.reader.close()
                self.reader = None
                self.start_button.configure(state='normal')
                self.instruction.configure(state='normal')
                self.remark.configure(state='normal')
                for widget in self.option_widgets:
                    widget.configure(state='normal')
                self.stop_button.configure(state='disabled')
        if self.estop_request:
            if self.estop_request.done():
                try:
                    result = self.estop_request.result()
                    self.live.configure(text='急停已锁定' if result.success else '急停失败：' + result.message)
                except Exception as exc:
                    self.live.configure(text='急停确认失败：' + str(exc))
                self.estop_request = None
            elif now - self.estop_requested_at > 2:
                self.live.configure(text='急停尚未确认，请使用实体急停')
        self.root.after(400, self.poll)

    def stop(self):
        if not self.process or self.process.poll() is not None:
            return
        if not messagebox.askokcancel('停止采集服务', '请先保存或丢弃本条并等待结果。\n未确认的数据退出时会丢弃。\n\n现在停止服务？', parent=self.root):
            return
        self.stopping = True
        self.status.configure(text='●  正在退出', fg=MUTED)
        self.detail.configure(text='等待现有采集脚本清理子进程。')
        self.stop_button.configure(state='disabled')
        try:
            os.killpg(self.process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass

    def logs(self):
        if self.log_path:
            subprocess.Popen(['xdg-open', str(self.log_path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def close(self):
        if self.process and self.process.poll() is None:
            messagebox.showinfo('服务仍在运行', '请先保存或丢弃本条并等待回执，停止服务后再关闭窗口。', parent=self.root)
            return
        self.node.destroy_node()
        self.root.destroy()
if __name__ == '__main__':
    rclpy.init()
    root = tk.Tk()
    Launcher(root)
    root.mainloop()
    rclpy.shutdown()
