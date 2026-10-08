# V4 迁移要点（AI 阅读）

本包固定用于 306。V4 不覆盖 V3；迁移时必须保持两者为不同 ROS 包，禁止真机模式同时运行。

## 必须准备

1. ROS 2 Jazzy 工作空间：`/home/ubuntu/ros2_ws`。
2. 厂商环境：`/home/ubuntu/miniconda3/envs/robot_env`，需能导入 Pinocchio、aiohttp、aiortc。
3. 安装独立 Placo 环境：

   ```bash
   cd /home/ubuntu/ros2_ws/src/openarmx_teleop_vr_306_v4
   bash scripts/setup_v4_placo_env.sh
   ```

   该脚本在 `/home/ubuntu/ros2_ws/venvs/openarmx_v4_placo` 安装 `placo==0.9.17`，不得直接改动 `robot_env`。

4. 检查 `config/controller.yaml` 与 `config/teleop.yaml`：机器人后缀为 `0_306`，厂商反馈/位置话题必须与本机一致。
5. 默认网页端口为 `8446`；若改端口，需要同步启动参数和防火墙设置。

## 编译与验证

```bash
cd /home/ubuntu/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select openarmx_teleop_vr_306_v4 --symlink-install
source install/setup.bash
ros2 launch openarmx_teleop_vr_306_v4 full_vr_teleop.launch.py dry_run:=true
```

先验证网页、最新帧、双臂增量映射和 Placo IK，再单独安排有人监护的真机测试。真机测试前退出 V3、导航遥操和厂商官方 VR 控制入口，保留厂商机械臂服务，由 V4 申请 SYNC 会话。

## V4 边界

- Quest3 部分：相邻帧增量、持续末端目标、实测关节反馈刷新、Placo 一步 QP、可操作度与动能正则。
- V3 保留部分：网页按键、WebRTC/WebSocket、最新帧邮箱、旧 IK 结果丢弃、掉线保持、硬限位、可选 SRDF 碰撞、SYNC 会话、厂商位置指令、CAN 与 DM 电机位置闭环。
- Placo 每次只开放正在接管的机械臂关节。默认腰部使用 `forward_pitch_only`：不进入冗余 IK，只在手臂接近伸直后增加正向腰俯仰并锁住腰偏航；`ik_pitch_yaw` 仅保留为诊断选项。脚踝、膝、头颈及未接管手臂不得参与该次 IK。
- 手肘接近伸直时，Placo 可操作度权重按配置从 `0.05` 平滑降到 `0.008`，避免固定奇异点权重抵抗正常前伸；不要将最低权重直接设为零。
