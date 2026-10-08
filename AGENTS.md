# AutoLife Collector 协作约定

## 入口与目录

- 统一入口 `start_lerobot_official_collect.sh`，`COLLECTOR_MODE=keyboard|vr|subtask|dagger`。
- `record_lerobot_official.py` 管理同步采集、FIFO回执和LeRobot数据保存。
- `vr_collector_control.py`、`subtask_annotations.py` 管理单人按键、语音及子任务标记。
- `dagger/control/` 为已验证控制基础；外层 `dagger/` 负责权限、部署与集合版适配。
- `dagger/assets/` 为随版本发布的配置与网页；哈希在 `dependencies.py` 和 `SOURCE.json`。
- `teleop_snapshot/` 为已打补丁的审查快照，不是待打补丁的运行依赖。
- 300正式部署在 `/home/ubuntu/CB_Autolife_Data_Collector`；部署路径不等于修改硬件的授权。

## 环境和验证

无构建步骤。机器人依赖ROS2 Jazzy、既有robot_env/lerobot环境、相机SHM。
DAgger额外依赖审核过的V4、Placo/Web环境和Thor baseline/frame服务。
新终端先 `source scripts/source_ros_env.sh`，再 `set +u`。

```bash
/home/ubuntu/miniconda3/envs/lerobot/bin/python -B -m unittest discover -s tests -p 'test_*.py'
node tests/test_feedback.js
bash -n start_lerobot_official_collect.sh
ROS_DOMAIN_ID=211 /usr/bin/python3 tests/vr_ros_smoke.py --subtasks
ROS_DOMAIN_ID=211 DAGGER_DEPENDENCY_ROOT=/home/ubuntu/ros2_ws/src /usr/bin/python3 tests/dagger_ros_smoke.py
```

完整流程和证据见 `ROBOT300_TESTING_zh.md`。不在真实控制域注入合成动作。
测试数据放独立目录并标记不可训练；已保存用户数据不因整理代码而改名、覆盖或删除。

## 保持不变的契约

- 普通/单人/子任务使用原厂输入；DAgger使用独立增量控制器，不能同时运行两套。
- 单人面键需GL+GR；DAgger面键无需双Grip，Grip用于人工接管。
- A开始、B保存/阶段标记、X仅复位、Y仅丢弃；终端键位按README保留。
- 原厂X复位未接入，不调用另一套控制器补偿；错误和未确认结果不能显示成功。
- 21维顺序、相机字段、FIFO回执、epoch/session门控与训练mask不得随意修改。
- 模型动作只在Thor端digest前修改；客户端不能裁剪或重写proposal/ACK前缀。
- 保留低CPU写入器、队列上限和同步阈值；不能通过丢帧或放宽阈值掩盖异常。
- 不清错、不停止或重启ARM服务来规避冲突。真机运动、服务切换需当前任务明确授权。
- 版本校验失败先检查差异，不增加跳过校验开关。更新打包资源时同时核对来源、哈希和回归。
- 正式版不新增个人后缀；旧名称仅保留在兼容入口、竞争进程检测和历史来源中。
