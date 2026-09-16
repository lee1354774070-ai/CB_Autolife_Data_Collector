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

## VR 单人数采

仍使用同一个启动脚本，只需增加 `VR_CONTROL=1`：

```bash
VR_CONTROL=1 TASK_TEXT="pick up the water bottle" WITH_DEPTH=1 \
bash start_lerobot_official_collect.sh pick_up_water_bottle
```

先按住 **GL+GR 两个握持键**，再点按并松开对应按钮，松开按钮时仍需保持两个握持键。
启动时已按住的面板按钮需先松开，避免误触。

| VR 按键 | 作用 |
| --- | --- |
| `A` | 倒计时 3 秒后开始新 episode。 |
| `B` | 保存并暂停，recorder 真正确认成功后才播报保存成功。 |
| `X` | 丢弃并暂停。 |
| 5 秒内双击 `Y` | 与键盘 `Q` 相同，保存有效数据并退出；两次点击之间保持 GL+GR。 |

倒计时期间按 `B` 或 `X` 可取消；VR 消息中断也会取消倒计时。
等待保存/丢弃结果后再开始下一条。无效 episode 即使按 `B` 也只会丢弃，不会播报保存成功。
命令超时后请检查终端和日志，不要反复按键重试。

`VR_CONTROL=0` 为默认键盘模式；开启 VR 后键盘仍可使用。
可选 `VR_START_DELAY_SEC=3` 调整倒计时（0 至 30 秒），`VR_SPEECH=0` 关闭中文语音。
语音使用机器人已有 TTS 服务及当前系统音量，不会自动调节音量。
需事先启动原厂 VR 输入服务；collector 本身不会启动遥操作或控制关节运动。

VR 控制结果和错误记录在 `logs/<序号>.vr_control.log`。

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
| `VR_CONTROL` | `0` | 开启带组合键保护和语音反馈的 VR 单人数采。 |
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
