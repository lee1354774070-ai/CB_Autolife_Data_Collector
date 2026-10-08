# LeRobot 数据采集工具

机器人300的 MZJ 融合版本、先退旧再启动流程及验收步骤见 [ROBOT300_TESTING_zh.md](ROBOT300_TESTING_zh.md)。

本工具将 Autolife 机器人的同步 state、action、RGB 和可选 depth 写入官方
`LeRobotDataset`。正常采集只需要一个终端。

双手相机需要视觉服务发布 `hand_left_jpeg` / `hand_right_jpeg` SHM。
默认只读 JPEG 并解码，无需新增参数；JPEG 缺失时不再回退到双手 BGR。
头部彩色和可选深度的使用方式不变。

## 选择模式

使用 `COLLECTOR_MODE` 选择工作方式：

| 值 | 当前状态和用途 |
| --- | --- |
| `keyboard` | 原版数采，通过终端 Enter/S/D/Q 控制。 |
| `vr` | 单人数采，使用既有 GL+GR 组合键和语音反馈。 |
| `subtask` | 单人 VR 子任务标注，必须同时提供 `SUBTASKS_JSON`。 |
| `dagger` | 300 机器人新版 V4 增量控制 + Thor GR00T 纠错采集适配，实验功能，已通过本地检查，尚未实机验收。 |

不设置时兼容原来的 `VR_CONTROL`、`SUBTASKS_JSON` 命令；均未设置则为 `keyboard`。
显式设置模式后，无需再设置 `VR_CONTROL`，冲突的旧变量会报错，不会静默切换模式。

## DAgger 纠错采集

首次启动宿主前先停止原 HG-DAgger/V4 控制任务，不能同时开启两套控制器。
兼容宿主运行后，其他终端可以直接接入，不需要重启它。
本适配依赖机器人上的 `openarmx_teleop_vr_306_v4` 和 `autolife_hg_dagger_MZJ_300`，
会核对关键源码与配置版本，不修改这两个原目录。当前仅接入 GR00T 21 维
`baseline/frame` 模型，三路 RGB 输入；不是 PI0.5 或 EDVA/SOMA verifier 的通用入口。
控制与推理以已跑通真机的 MZJ 快照 `dagger/mzj_base/` 为基础；集合版增加统一入口、
数据来源标记、状态同步和新按键。快照来源与小改动记录在 `SOURCE.json`。
需同时更新我们的 Thor server，使 `/health` 包含 `controller_submission_receipts: true`。
独立 Collector 目录部署时，用 `DAGGER_TOOLS_ROOT` 指向完整的 `Autolife_VLA_Tools`。
启动时自动生成并校验 `.runtime/` 下的遥操作副本，只对副本应用补丁。

```bash
COLLECTOR_MODE=dagger DAGGER_SERVER_URL=http://THOR_IP:8777 \
DAGGER_PUBLISH=0 WITH_HEAD=1 WITH_UPPER_WAIST=1 WITH_DEPTH=1 \
TASK_TEXT="put the towel in the basket" \
bash start_lerobot_official_collect.sh towel_dagger
```

将 `THOR_IP` 换成实际地址。初次务必使用新任务目录；省略 `WITH_DEPTH=1` 则不录深度。
默认 `DAGGER_PUBLISH=0` 不向硬件发布动作，不能视为真实录制验收；现场确认安全后，
改为 `DAGGER_PUBLISH=1` 才允许按键触发运动。启动本身不会自动开始推理。

VR 页面：`https://机器人IP:8447`。A 开始推理和录制；任意 GL/GR 接管，
继续握持即可增量遥操，不等待模型返回。B 保存、Y 丢弃，均不复位；
X 仅全身复位（包括夹爪打开），有待处理数据时会拒绝，先 B/Y 并等待回执。
接管后由人完成，不自动交还模型。只有接管振动，其余状态/报错显示在头显下方。
进入或重连 VR 保持待命；B/Y/X 完成后仍需 A 才开始下一条。
终端保留原快捷键：C 开始、A 保存、X/D 丢弃、R 复位、Q 或 Ctrl+C 退出。
保存结果未知时保留原请求，B/Y 只核对原结果，不重复保存或提前复位。
完整 300 操作与验收见 [ROBOT300_TESTING_zh.md](ROBOT300_TESTING_zh.md)。

只读预检：在上述命令末尾加 `--check`。参数帮助：`bash start_lerobot_official_collect.sh DAGGER_PUBLISH --help`。
数据字段、训练 mask、验证边界见 [INSTRUCTION_zh.md](INSTRUCTION_zh.md#dagger-接入)。

### 接入已经运行的兼容宿主

同事未改造的旧进程不能直接热接入。先生成兼容 GUI 副本，原文件不变：

```bash
python3 dagger/desktop_copy.py --source /path/to/original/scripts/dagger_launcher.py \
  --tools-root /path/to/Autolife_VLA_Tools --output /path/to/copy/dagger_launcher.py
```

首次切换需停止旧任务，再运行副本。启动副本时沿用宿主的
`DAGGER_DEPENDENCY_ROOT`、`DAGGER_SERVER_URL`、`DAGGER_TOKEN_FILE` 和数据目录配置；
现场确认安全后才设置 `DAGGER_PUBLISH=1`。副本启动的宿主不会因终端没有输入而退出。
之后保持 GUI 运行，另一个终端使用**正在运行的同一任务目录**接入：

```bash
COLLECTOR_MODE=dagger DAGGER_BACKEND=attach DAGGER_PUBLISH=1 \
OUTPUT_BASE_DIR=/home/ubuntu/nas14 \
bash start_lerobot_official_collect.sh EXISTING_TASK --check
```

去掉 `--check` 即可操作：C 开始、A 保存、X/D 丢弃、R 复位。
此模式下 Q 或输入结束**仅断开终端，不会暂停推理、保存、复位或关闭宿主**。
要结束当前条目请先按 A/X。VR/GUI 保持有效。
旧版宿主、任务不一致、状态过期或多个宿主都会拒绝接入，不会回退启动第二套控制器。
命令结果不确定时不会自动重试。参数说明见 `DAGGER_BACKEND --help`。

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

仍使用同一个启动脚本，设置 `COLLECTOR_MODE=vr`：

```bash
COLLECTOR_MODE=vr TASK_TEXT="pick up the water bottle" WITH_DEPTH=1 \
bash start_lerobot_official_collect.sh pick_up_water_bottle
```

先按住 **GL+GR 两个握持键**，再点按并松开对应按钮，松开按钮时仍需保持两个握持键。
启动时已按住的面板按钮需先松开，避免误触。

| VR 按键 | 作用 |
| --- | --- |
| `A` | 倒计时 3 秒后开始新 episode。 |
| `B` | 保存并暂停，等待 recorder 的成功回执。 |
| `X` | 仅全身复位；录制中或结果未确认时拒绝，数据保留。 |
| `Y` | 仅丢弃并暂停，不复位、不退出。 |

倒计时期间 B/Y 取消；X 取消倒计时后复位（没有正在录制的数据）。
VR 消息中断也会取消倒计时。无效 episode 即使按 B 也只会丢弃，不会提示保存成功。
等待保存/丢弃结果后再开始下一条；结果未知时先检查日志。

`COLLECTOR_MODE=keyboard` 为原版键盘模式；开启 VR 后键盘仍可使用。
未指定模式时，旧的 `VR_CONTROL=0/1` 仍有效。
可选 `VR_START_DELAY_SEC=3` 调整倒计时（0 至 30 秒），`VR_SPEECH=0` 关闭中文语音。
语音使用机器人已有 TTS 服务及当前系统音量，不会自动调节音量。
需事先启动遥操作。默认监听原厂输入；增量 V4 输入可设置
`VR_INPUT_TOPIC=/openarmx_teleop_vr_306_v4/vr_input`。
**X 会导致真实运动**：目前对接 V4 的受保护复位服务，默认
`VR_RESET_PREFIX=/openarmx_teleop_vr_306_v4`。必须关闭遥操作端旧的 B/组合键复位映射，
由 collector 唯一处理 X。接口不可用时不复位，不直接发布原厂 reset topic。
保存/丢弃忙碌时 X 被忽略；结果未知时禁止复位。已保存数据不删除。
复位成功后保持控制器停用，需在遥操作页面重新使能。退出使用终端 Q。
语音、振动接入方法见 [VR 反馈接入](VR_FEEDBACK.md)。原厂网页不会自动获得新振动功能。

VR 控制结果和错误记录在 `logs/<序号>.vr_control.log`。

## 连续采集并标注子任务

用 `TASK_TEXT` 定义总任务，`SUBTASKS_JSON` 定义有序子任务。首次开启请使用新的数据目录：

```bash
COLLECTOR_MODE=subtask VR_A_LONG_PRESS_SEC=1.0 \
TASK_TEXT="把毛巾放到篮子里" \
SUBTASKS_JSON='["右手拿起毛巾","双手交接毛巾","左手放下毛巾"]' \
bash start_lerobot_official_collect.sh towel_to_basket_annotated
```

仍需保持 **GL+GR**；短按和长按都在 B **松开时**执行：

| 操作 | 作用 |
| --- | --- |
| 空闲时按 `A` | 倒计时后开始整条 episode。 |
| 录制时短按 `B` | 确认当前子任务片段，切换到下一子任务，录制不中断。 |
| 最后一个子任务短按 `B` | 标注最后片段并保存完整 episode。 |
| 录制时按住 `B` 至少 1 秒再松开 | 提前保存，不要求所有子任务已完成或已标注。 |
| 键盘 `N` / `S` | 分别标注当前子任务 / 提前保存。 |

`Y` 丢弃、`X` 仅复位（先保存/丢弃）；终端 Q 退出。空片段不会被标注；丢弃后下一条从第一个子任务开始。
中间标注不编码视频、不暂停录制，后台写入回执。语音仅简短提示“开始”“下一步”“已保存”；
提前保存提示“已保存，标注未完成”。提示来自 recorder 确认，不需要停下动作等语音播放。
最后一步或长按提交保存时先提示“保存中”，该提示不代表保存已成功。
完整子任务文本和进度显示在终端；`VR_SPEECH=0` 可关闭声音。带反馈扩展的 VR 网页仅在接管时振动，其他操作显示文字。

每条数据仍是一个总任务、一个 episode。Parquet 增加 `subtask_index`：`0/1/2` 对应本条
episode 的三个子任务，未确认尾段为 `-1`。文本和片段边界保存在
`dataset/annotations/subtasks/episode_000000.json`，提前保存会标记 `needs_review=true`。
边界以 recorder 处理标注时已经写入的帧为准，不是手柄硬件时间戳。
漏按后工具不能猜测正确边界，请长按保存并后续人工修正。同步失败的 `EPISODE INVALID`
仍整条丢弃，不能通过长按保存绕过。

使用 `keyboard`/`vr` 时不设置 `SUBTASKS_JSON`（默认 `[]`），保持原有行为。标注开关不能在已有数据根目录中改变。
普通 LeRobot 训练仍使用总任务文本；训练时若要按子任务文本取样，需要额外的数据处理。

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
| `COLLECTOR_MODE` | 按旧参数推导，通常为 `keyboard` | `keyboard`、`vr`、`subtask`、`dagger`。 |
| `TASK_TEXT` | task name | 写入每帧的自然语言任务。 |
| `SUBTASKS_JSON` | `[]` | 有序子任务文本数组；空数组关闭子任务标注。 |
| `VR_A_LONG_PRESS_SEC` | `1.0` | 子任务模式下 B 长按保存阈值（兼容旧参数名），单位秒，范围 0.2 至 10。 |
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
开关、action mode 和子任务标注开关必须完全一致。改变其中任何一项时，请使用新的 task name 或输出目录。

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
