"""Resolve named task poses; never infer a posture from free-form text."""
import json
import math
from pathlib import Path

from .inference_options import TASKS

PLAN = Path('/home/ubuntu/MZJ_laundry_launcher/laundry_cycle_plan.json')
TASK_IDS = dict(zip((task[1] for task in TASKS), ('4', '0', '1', '3', '2', '5')))


def task_pose(env):
    import yaml
    plan_path = Path(env.get('DAGGER_RESET_PLAN', str(PLAN))).expanduser()
    plan = json.loads(plan_path.read_text())
    # This legacy plan has no robot identity: it is calibrated only on 300.
    robot_id = str(env.get('ROBOT_ID', ''))
    if str(plan.get('robot_id', '300')) != robot_id:
        raise ValueError('任务复位配置不属于当前机器人，请提供该机器人的标定计划')
    name = env.get('DAGGER_RESET_POSE', '').strip()
    if not name:
        task_id = TASK_IDS.get(env.get('TASK_TEXT', '').strip())
        name = plan.get('task_poses', {}).get(task_id, '')
    if not isinstance(name, str) or not name.strip():
        raise ValueError('当前任务没有复位姿态，请在启动器明确选择，或设置 DAGGER_RESET_POSE')
    library = Path(plan['poses_file']).expanduser()
    if not library.is_absolute():
        library = plan_path.parent / library
    try:
        data = yaml.safe_load(library.read_text())
    except yaml.YAMLError as exc:
        raise ValueError('复位姿态库 YAML 格式错误') from exc
    if not isinstance(data, dict) or not isinstance(data.get('poses'), dict):
        raise ValueError('复位姿态库缺少 poses')
    joints = data.get('poses', {}).get(name, {}).get('joints_deg')
    if not isinstance(joints, dict):
        raise ValueError('姿态不存在或格式错误：' + name)
    from teleop_snapshot.control_center_reset_pose.reset_pose_loader import CUSTOM_POSE_JOINTS
    selected = {}
    for key in CUSTOM_POSE_JOINTS:
        value = joints.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('任务姿态缺少有效关节角度：' + key)
        selected[key] = value
    return {'version': 1, 'robot_id': robot_id, 'mode': 'custom',
            'custom': {'joints_deg': selected}, 'task_text': env.get('TASK_TEXT', ''),
            'pose_name': name, 'source': str(library)}


def pose_choices(env):
    import yaml
    path = Path(env.get('DAGGER_RESET_PLAN', str(PLAN))).expanduser()
    plan = json.loads(path.read_text())
    library = Path(plan['poses_file']).expanduser()
    if not library.is_absolute():
        library = path.parent / library
    try:
        data = yaml.safe_load(library.read_text())
    except yaml.YAMLError as exc:
        raise ValueError('复位姿态库 YAML 格式错误') from exc
    if not isinstance(data, dict) or not isinstance(data.get('poses'), dict):
        raise ValueError('复位姿态库缺少 poses')
    return list(data['poses'])
