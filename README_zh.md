# LeRobot 数据采集工具

本工具将 Autolife 机器人的同步 state、action、RGB 和可选 depth 写入官方
`LeRobotDataset`。正常采集只需要一个终端。

## 启动

```bash
cd /home/ubuntu/lerobot_data_collector
TASK_TEXT="pick up the water bottle" \
bash start_lerobot_official_collect.sh pick_up_water_bottle
```

默认输出目录为 `/home/ubuntu/nas14/<task_name>/dataset`。使用
`OUTPUT_BASE_DIR` 可以修改保存位置：

```bash
OUTPUT_BASE_DIR=/mnt/data \
TASK_TEXT="put the water bottle in the box" \
bash start_lerobot_official_collect.sh put_water_bottle_in_box
```

## 录制控制

| 按键 | 作用 |
| --- | --- |
| `Enter` | 开始一条新的 episode。 |
| `S` | 保存当前有效 episode 并暂停。 |
| `D` | 丢弃当前 episode 并暂停。 |
| `Q` | 保存有效的 pending episode，finalize 后退出。 |
| `Ctrl+C` | 停止相关进程，已保存数据保留。 |

终端出现 `EPISODE INVALID` 时必须按 `D`。无效 episode 不应保存。

## 常用采集方式

```bash
# 16 维基础关节和三路 RGB。
bash start_lerobot_official_collect.sh rgb_task

# 30 FPS、23 维关节、三路 RGB 和头部 depth。
COLLECT_FPS=30 WITH_HEAD=1 WITH_WAIST=1 WITH_DEPTH=1 \
bash start_lerobot_official_collect.sh hotel_service

# 18 维：手臂、夹爪、腰部 pitch/yaw，不包含腿部升降关节。
WITH_UPPER_WAIST=1 bash start_lerobot_official_collect.sh upper_waist_task

# 仅诊断相机链路：头部彩色相机，state 回退为 action。
CAMERA_ONLY=1 bash start_lerobot_official_collect.sh camera_test
```

## 主要配置

| 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `TASK_TEXT` | task name | 写入每帧的自然语言任务。 |
| `COLLECT_FPS` | `30` | 数据集帧率。 |
| `WITH_HEAD` | `0` | 追加 3 个头部关节。 |
| `WITH_UPPER_WAIST` | `0` | 只追加 `waist_pitch`、`waist_yaw`（2 维）。 |
| `WITH_WAIST` | `0` | 追加 ankle、knee、waist pitch/yaw（4 维）。 |
| `WITH_DEPTH` | `0` | 增加 `rgbd_head_depth` 的 uint16 depth 视频。 |
| `ACTION_MODE` | `status_target` | `status_target`、`joint` 或 `eef`。 |
| `IMAGE_SOURCE` | `shm` | 直读共享内存或 `ros` topic。 |
| `SYNC_REFERENCE_CAMERA` | `hand_left` | 图像时间戳锚点。 |
| `MAX_SYNC_DELTA_SEC` | `0.03` | 相机允许的最大时间差。 |
| `SYNC_IMAGE_BUFFER_SIZE` | `16` | 每路图像 FIFO 容量。 |
| `MIN_CAMERAS` | 全部已选相机 | 任一请求相机不可用时拒绝启动。 |

所有参数都可通过内置帮助查看：

```bash
bash start_lerobot_official_collect.sh --help
bash start_lerobot_official_collect.sh MAX_SYNC_DELTA_SEC --help
python record_lerobot_official.py --with-depth --help
```

## 续采和输出

相同 `task_name` 会尝试续采已有数据集，但相机 feature、FPS、关节 schema、depth
开关和 action mode 必须完全一致。改变其中任何一项时，请使用新的 task name 或输出目录。

`WITH_UPPER_WAIST=1` 与 `WITH_WAIST=1` 不能同时使用。

```text
<task_name>/
├── dataset/    # parquet、metadata、videos、sync_log.jsonl
└── logs/       # 每次启动的一组日志
```

LeRobot 生成多个视频文件属于正常行为，训练前不要手动拼接。

## 环境要求

机器人需要 ROS2 Jazzy、`robot_env`、`lerobot`（depth 需要 LeRobot 0.6+）、
正常运行的关节 state/action 服务和相机服务。同步逻辑与实现细节请看
[INSTRUCTION_zh.md](INSTRUCTION_zh.md)。
