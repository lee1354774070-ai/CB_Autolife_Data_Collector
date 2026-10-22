"""Shared launcher choices; no robot IO and no client-side action changes."""
import json
from pathlib import Path
import subprocess
from urllib.parse import urlsplit

TASKS = (
    ("桌面取袋", "Pick the laundry bag from the table.", "pick"),
    ("配送机器人取袋", "Pick the laundry bag from the delivery robot.", "pick"),
    ("洗衣机取袋", "Pick the laundry bag from the washing machine.", "pick"),
    ("放入配送机器人", "Place the laundry bag in the upper compartment of the delivery robot.", "place"),
    ("放入洗衣机", "Put the laundry bag into the washing machine.", "place"),
    ("放入烘干机", "Put the laundry bag into the dryer.", "place"),
)
PHASES = {"自动（仅已知任务）": "auto", "抓取：实测闭合后锁持": "pick",
          "放置：预测张开后释放": "place", "连续：保留原限速保护": "continuous"}


def selected_phase(task, phase="auto"):
    if phase not in ("auto", "pick", "place", "continuous"):
        raise ValueError("夹爪阶段无效")
    if phase != "auto":
        return phase
    known = {prompt: value for _, prompt, value in TASKS}
    known["Pick the laundry bag."] = "pick"
    if task.strip() not in known:
        raise ValueError("自定义任务请明确选择夹爪阶段：抓取、放置或连续")
    return known[task.strip()]


def validate_selection(health, task, phase="auto", checkpoint=""):
    selected = selected_phase(task, phase)
    if health.get("ready") is not True:
        raise ValueError("Thor 模型尚未就绪")
    # Old servers accept only their exact legacy prompts, without a new field.
    legacy = {"Pick the laundry bag.": "pick",
              TASKS[3][1]: "place"}
    if selected not in health.get("gripper_phases", ()):
        if not (phase == "auto" and legacy.get(task.strip()) == selected
                and health.get("gripper_mode") == "phase_aware"):
            raise ValueError("Thor 版本不支持该夹爪阶段，请先更新推理服务")
    if checkpoint and health.get("checkpoint") != checkpoint:
        raise ValueError("Thor 实际 checkpoint 与选择不一致；请重新刷新或应用模型")
    return selected


def checkpoint_request(action="list", checkpoint="", server_url="http://192.168.8.224:8777"):
    # This dedicated SSH key can run only the fixed checkpoint helper on Thor.
    url = urlsplit(server_url)
    if (url.scheme, url.hostname, url.port or 80) != ("http", "192.168.8.224", 8777):
        raise ValueError("模型切换接口仅配置了 Thor 192.168.8.224:8777；不会修改其他服务")
    if action not in ("list", "switch"):
        raise ValueError("Invalid checkpoint operation")
    result = subprocess.run([
        "ssh", "-T", "-o", "BatchMode=yes", "-o", "IdentitiesOnly=yes",
        "-o", "StrictHostKeyChecking=yes", "-o", "ConnectTimeout=8",
        "-o", "ServerAliveInterval=15", "-o", "ServerAliveCountMax=4",
        "-i", str(Path.home() / ".ssh/id_ed25519_collector_checkpoint"),
        "wayne@192.168.8.224", "checkpoint-control"],
        input=json.dumps({"action": action, "checkpoint": checkpoint}),
        text=True, capture_output=True, timeout=7350 if action == "switch" else 20)
    try:
        response = json.loads(result.stdout)
    except ValueError:
        raise RuntimeError("Thor 模型管理连接失败：" + result.stderr.strip()[-500:]) from None
    if result.returncode or response.get("error"):
        raise RuntimeError(response.get("error", "模型管理失败；请刷新实际状态"))
    return response
