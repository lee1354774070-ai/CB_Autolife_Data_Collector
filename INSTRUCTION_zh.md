# Collector 实现说明

本文档说明内部实现约束；日常命令请看 [README_zh.md](README_zh.md)。

## 架构

| 模块 | 职责 |
| --- | --- |
| `start_lerobot_official_collect.sh` | 解析 session 配置、管理进程生命周期、把按键转换为 IPC 命令。 |
| `record_lerobot_official.py` | 缓存 ROS/SHM 信号、同步 dataset 行并管理 `LeRobotDataset`。 |
| `robot_schema.py` | 维护关节名称、canonical policy 顺序、物理 q23 转换和命令解析。 |
| `camera_config.py` | 维护相机名称、SHM 路径、ROS topic 和共享默认值。 |
| `shm_camera.py` | 读取稳定的 metadata/图像对，并解码 BGR 或 uint16 depth 帧。 |
| `time_sync.py` | 提供时间戳归一化、FIFO 选择和插值函数。 |
| `collector_control.py` | 实现 launcher 与 recorder 的 IPC、状态报告和数据集摘要。 |

默认 direct-SHM 运行结构：

```text
launcher
├── recorder          LeRobot + ROS2 Python
└── hand producer     需要手部相机时使用 robot_env 启动
```

`IMAGE_SOURCE=ros` 会增加 `shm_camera_topic_bridge.py`。direct-SHM 是默认方式，
因为它省去了图像序列化和 DDS 队列。

## 数据契约

policy schema 固定以 16 维开始：

```text
左臂 (7)、右臂 (7)、左夹爪 (1)、右夹爪 (1)
```

`WITH_HEAD=1` 追加颈部 roll/pitch/yaw；`WITH_UPPER_WAIST=1` 只追加 waist
pitch/yaw；`WITH_WAIST=1` 追加 ankle、knee、waist pitch、waist yaw。两个腰部
模式互斥。关节 state 和关节 action 必须使用完全相同的顺序，因为 LeRobot
relative action 按数组索引计算。

ROS 控制器的物理 q23 顺序不同：

```text
腰部 (4)、左臂 (7)、右臂 (7)、夹爪 (2)、头部 (3)
```

只有 `robot_schema.py` 可以在两种顺序间转换。policy 没有启用头部或腰部时，
部署端会保留这些物理关节的最新实测值。

## 图像与同步

每路相机有两个 SHM 文件：

```text
/dev/shm/camera_metadata_struct_<name>
/dev/shm/camera_image_buffer_<name>
```

metadata 格式为 `"<qiiiii"`：时间戳、宽、高、通道数、像素格式和字节数。
只有复制图像前后 metadata 一致时，当前帧才会被接受。SHM 边界的 RGB 是 BGR
uint8，写入 LeRobot 前转换为 RGB HWC；depth 是小端 uint16 毫米值。

recorder 批量读取 metadata，只复制新图像，并为每路相机维护有界 FIFO。每行数据：

1. 取参考相机中已经等待匹配窗口的最早帧；
2. 在 `MAX_SYNC_DELTA_SEC` 内为其他相机选择最近帧；
3. 在参考时间戳处插值 state；
4. 选择不晚于该时间戳的因果 action；
5. 将完整行加入 LeRobot 的 pending episode。

缺图像、state 插值区间或因果 action，或者出现过期数据时，当前 episode 会作废。
录制期间 FIFO 溢出同样会作废 episode。“参考帧仍在等待其他帧”只会增加
`waiting ticks`，不代表源图像跳帧，也不会单独作废 episode。

## Session 生命周期

launcher 先启动 recorder，再启动相机 producer，保证图像锚点到来时已有 state 历史。
`Enter` 清空预热图像 FIFO 并开始新的 episode；`S` 保存；`D` 丢弃；`Q` 只保存有效
pending episode 后 finalize 数据集。

IPC 位于 task 目录下：命令 FIFO、原子 status JSON、ready JSON、episode-event JSON
和 PID 文件。这些均为临时文件；数据集和日志会保留。

续采要求 FPS、相机 feature、depth、关节 schema 和 action mode 完全一致。LeRobot
生成多个视频文件属于正常行为，metadata 会索引它们，不能手动拼接。

## 扩展规则

- 新增相机：修改 `camera_config.py`，并补充配置与 SHM 测试。
- 新增关节：修改 `robot_schema.py`，不要在 recorder 或 deployer 中重复维护顺序。
- 新增信号：建立带时间戳的 buffer，并定义明确的因果或插值规则。
- 不要把 FIFO 匹配替换为 latest-frame 复用，这会改变数据契约。

## 验证

```bash
PYTHONPATH=. python -m pytest -q tests
python -m py_compile *.py
bash -n start_lerobot_official_collect.sh
```

机器人现场还应检查 ROS 发现、SHM 文件、相机源 FPS、编码器、CPU、磁盘吞吐和
`sync_log.jsonl`，确认生产采集前没有异常 drop 或 episode invalidation。
