"""One checkpoint picker used by both desktop launchers."""
import queue
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from dagger.inference_options import checkpoint_request


class CheckpointDialog:
    def __init__(self, parent, can_switch, on_applied, server_url):
        self.can_switch, self.on_applied, self.server_url = can_switch, on_applied, server_url
        self.events = queue.Queue()
        self.busy = False
        self.window = tk.Toplevel(parent)
        self.window.title("Thor · 切换 checkpoint")
        self.window.geometry("720x290")
        self.window.transient(parent)
        self.window.grab_set()
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        body = ttk.Frame(self.window, padding=20)
        body.pack(fill="both", expand=True)
        self.actual = ttk.Label(body, text="正在读取 Thor 实际状态…", wraplength=670)
        self.actual.pack(anchor="w", pady=(0, 12))
        self.choice = ttk.Combobox(body, state="readonly", width=65)
        self.choice.pack(fill="x")
        buttons = ttk.Frame(body)
        buttons.pack(fill="x", pady=14)
        self.refresh = ttk.Button(buttons, text="刷新模型列表", command=lambda: self.run("list"))
        self.refresh.pack(side="left")
        self.apply = ttk.Button(buttons, text="应用所选模型", command=lambda: self.run("switch"))
        self.apply.pack(side="left", padx=10)
        self.status = ttk.Label(body, text="模型来自 Thor/models；任务文本不会因切换模型自动改写。", wraplength=670)
        self.status.pack(anchor="w")
        self.window.after(100, self.poll)
        self.run("list")

    def run(self, action):
        if self.busy:
            return
        checkpoint = self.choice.get()
        if action == "switch" and (not checkpoint or not self.can_switch()):
            self.status.configure(text="请先停止采集服务/结束推理并等待退出，再选择模型。")
            return
        self.busy = True
        self.refresh.configure(state="disabled")
        self.apply.configure(state="disabled")
        self.choice.configure(state="disabled")
        self.status.configure(text="正在加载模型，可能需要数分钟；就绪前不会启动推理。" if action == "switch" else "读取中…")
        def work():
            try:
                self.events.put((action, checkpoint_request(action, checkpoint, self.server_url), None))
            except Exception as error:
                self.events.put((action, None, str(error)))
        threading.Thread(target=work, daemon=True).start()

    def poll(self):
        try:
            action, result, error = self.events.get_nowait()
        except queue.Empty:
            pass
        else:
            self.busy = False
            self.refresh.configure(state="normal")
            self.apply.configure(state="normal")
            self.choice.configure(state="readonly")
            if error:
                self.actual.configure(text="实际模型状态未确认，请刷新；不能按已选名称判断加载成功。")
                self.status.configure(text=error)
            else:
                current = result["checkpoint"]
                health = result["health"]
                self.choice.configure(values=result["models"])
                self.choice.set(current)
                ready = health.get("ready") is True
                self.actual.configure(text=f"实际加载：{current}  ·  {'就绪' if ready else '未就绪'}" + (" · 会话使用中" if health.get("active_session") else ""))
                self.status.configure(text="模型已核实；关闭此窗口后选择任务，再开始。" if ready else "等待模型就绪后重新刷新。")
                if ready:
                    try:
                        self.on_applied(current)
                    except Exception as exc:
                        self.status.configure(text=f"模型已加载，但本地配置未保存：{exc}")
        self.window.after(100, self.poll)

    def close(self):
        if self.busy:
            messagebox.showinfo("模型操作中", "请等待本次操作结束；关闭窗口不会取消 Thor 加载。", parent=self.window)
            return
        self.window.destroy()
