# 云蝶 AutoLife 数据采集器

统一采集入口，支持普通数采、单人 VR、子任务标注和 DAgger 纠错采集。
机器人状态、动作、RGB 和可选深度写入官方 `LeRobotDataset`。

**正式入口：`start_lerobot_official_collect.sh`，通过 `COLLECTOR_MODE` 选择模式。**
300 部署目录：`/home/ubuntu/CB_Autolife_Data_Collector`。首次使用先阅读下面的环境和按键说明。

| 模式 | 适用场景 | 遥操入口 |
|---|---|---|
| `keyboard` | 在终端控制录制 | 原厂遥操 |
| `vr` | 一个人戴头显完成录制和保存 | 原厂遥操，双 Grip + 面键 |
| `subtask` | 一条连续数据中标注多个阶段 | 原厂遥操，双 Grip + 面键 |
| `dagger` | 模型执行，人工接管并完成纠错 | 独立增量遥操，HTTPS 8447 |

普通数采、单人和子任务模式需要原厂遥操保持运行，**不需要另开8446网页**。
DAgger 自带独占控制器，启动前先正常退出其他控制任务。
本次整理保留已验证的控制、夹爪、防失步、录制和 ACK 逻辑。

## 1. 环境与安装

已配置好的300可直接进入正式目录。新机器先安装已有机器人环境，本仓库不会自动安装或重启硬件服务：

- ROS 2 Jazzy、机器人关节反馈及相机 SHM 服务。
- `robot_env` Python：ROS、OpenCV、音频/网页依赖；`lerobot` Python：官方 LeRobot、PyArrow、视频编解码依赖。
- 默认三路 RGB：`hand_left`、`hand_right`、`rgbd_head_color`；可选 `rgbd_head_depth`。
  双手相机使用视觉服务的 JPEG SHM，缺失时不会回退到另一路旧 BGR 数据。
- DAgger 额外需要已审核的 V4 源码、Placo/Web 环境，以及兼容 Thor GR00T 服务。
  `DAGGER_DEPENDENCY_ROOT` 指向包含 `openarmx_teleop_vr_306_v4` 的目录。
  原 V4 源码按哈希校验，启动时只给 `.runtime/` 中的副本打补丁。
- DAgger 的控制模块、配置和网页已随本仓库发布，不再从个人实验包读取。
  GR00T 预检客户端通过 `DAGGER_TOOLS_ROOT` 使用已有 `Autolife_VLA_Tools`。

新安装位置：

```bash
cd /home/ubuntu
git clone https://github.com/lee1354774070-ai/CB_Autolife_Data_Collector.git
cd /home/ubuntu/CB_Autolife_Data_Collector
```

每个新终端执行：

```bash
cd /home/ubuntu/CB_Autolife_Data_Collector
source scripts/source_ros_env.sh
set +u
unset VR_CONTROL SUBTASKS_JSON
export OUTPUT_BASE_DIR=/home/ubuntu/nas
export WITH_HEAD=1 WITH_UPPER_WAIST=1 WITH_WAIST=0 WITH_DEPTH=1
export START_HAND_PRODUCER=0 ACTION_MODE=status_target
export VR_INPUT_TOPIC=/control_topic_0_300
export VR_SPEECH=1 VR_START_DELAY_SEC=3
```

这里的 `nas` 是普通文件夹。上面显式打开头部、上腰和深度；可按任务修改。
`START_HAND_PRODUCER=0` 用于复用已有相机 SHM；没有相机生产服务的新机器应先配置相机。
启动首行会打印 **`AutoLife Collector | mode=... | 实际代码目录`**，可据此排查旧入口误启动。
未指定 `OUTPUT_BASE_DIR` 时通用脚本仍兼容原默认 `/home/ubuntu/nas14`。

## 2. 普通数采

```bash
COLLECTOR_MODE=keyboard TASK_TEXT='Pick the laundry bag.' \
bash start_lerobot_official_collect.sh keyboard_01
```

| 终端按键 | 功能 |
|---|---|
| Enter | 开始新一条 |
| S | 保存并暂停，等待成功回执 |
| D | 丢弃并暂停 |
| Q | 保存有效待处理数据并正常退出 |
| Ctrl+C | 中断本次程序；已保存数据保留，不保证当前条目保存 |

遇到 `EPISODE INVALID` 时应丢弃当前条目，不把无效数据计为保存成功。

## 3. VR 单人数采

```bash
COLLECTOR_MODE=vr TASK_TEXT='Pick the laundry bag.' \
bash start_lerobot_official_collect.sh vr_01
```

戴好头显，进入**原厂沉浸式遥操**。停在菜单或手柄未激活时，话题可能有消息但按键始终为松开。
保持 **GL+GR 两个握持键按住**，点按并松开面键；面键松开时双 Grip 仍需按住。

| 面键 | 功能 |
|---|---|
| A | 语音“三、二、一”，收到录制回执后提示开始 |
| B | 保存；先提示保存中，成功后播报条数和帧数 |
| X | 仅复位；**原厂模式尚未接入此接口，请使用原厂复位功能** |
| Y | 仅丢弃，不复位、不退出 |

B/Y 可以取消倒计时。保存忙碌、没有待处理数据、输入断开、录制异常和操作失败均有提示。
等待操作回执后再开始下一条。终端键盘仍可操作；退出用终端 Q。
原厂页面不会自动加载集合版的头显提示扩展，原厂模式以机器人语音和终端回执为准。
显式接入独立 V4 时，X 走受保护复位服务；它会产生真实运动，必须先结束当前录制并释放双 Grip。

## 4. 子任务标注

`TASK_TEXT` 是整条任务，`SUBTASKS_JSON` 是有序阶段。开启标注时使用**新的任务目录**：

```bash
COLLECTOR_MODE=subtask VR_A_LONG_PRESS_SEC=1.0 \
TASK_TEXT='Pick up the bag, move it, and put it down.' \
SUBTASKS_JSON='["右手抓起袋子","移动袋子","放下袋子"]' \
bash start_lerobot_official_collect.sh subtask_01
```

启动后必须看到 `VR annotation` 和 `[VR] Listening: /control_topic_0_300`。
仍需 GL+GR；短按、长按都在 **B 松开时**执行：

| 操作 | 功能 |
|---|---|
| A | 三秒倒计时，开始整条连续录制 |
| 短 B | 确认当前阶段，继续录制下一阶段 |
| 最后阶段短 B | 标注最后阶段并保存整条 |
| 长 B ≥1秒 | 提前保存，未完成部分标记待审核 |
| Y | 丢弃；下一条从第一个阶段重新开始 |
| 键盘 N / S | 标记当前阶段 / 提前保存 |

`VR_A_LONG_PRESS_SEC` 是兼容保留的旧参数名，实际控制 **B** 的长按阈值。
阶段边界以录制器处理标记时已经写入的帧为准，并非手柄硬件时间戳。
Parquet 中 `subtask_index=0/1/2` 对应三个阶段；未确认尾段为 `-1`。
文本、边界和 `needs_review` 写入 `dataset/annotations/subtasks/episode_*.json`。

## 5. DAgger 纠错采集

当前适配 **机器人300 / ROS domain0 / 21维 / GR00T baseline或frame协议**。
模型使用三路 RGB，深度可录制但不送入当前模型；不支持 SOMA/Outcome 历史模式。

先结束原厂遥操或其他关节控制任务。控制中心主界面可保留，但其关节控制模块可能占用发布端点。
预检报冲突时按进程和任务逐一核对，正常退出占用者；**不要清错或重启 `arm-control-service` 来处理冲突**。

```bash
unset SUBTASKS_JSON VR_CONTROL ACTION_MODE
export COLLECTOR_MODE=dagger
export DAGGER_SERVER_URL=http://192.168.8.224:8777
export DAGGER_TOOLS_ROOT=/home/ubuntu/Autolife_VLA_Tools
export DAGGER_TOKEN_FILE=/home/ubuntu/.config/autolife_hg_dagger/groot_server.token
export OUTPUT_BASE_DIR=/home/ubuntu/nas/dagger
export TASK_TEXT='Pick the laundry bag.'
export WITH_HEAD=1 WITH_UPPER_WAIST=1 WITH_WAIST=0 WITH_DEPTH=1

# 只读预检，不录制、不发布运动动作。
DAGGER_PUBLISH=0 bash start_lerobot_official_collect.sh dagger_check --check

# 无动作待命检查；检查完用Q退出。
DAGGER_PUBLISH=0 bash start_lerobot_official_collect.sh dagger_dryrun

# 上一步退出、现场人员确认后，再启动真实控制。
DAGGER_PUBLISH=1 bash start_lerobot_official_collect.sh dagger_live_01
```

头显页面：`https://192.168.8.122:8447`。进入或重连保持待命，必须主动按 A 开始。
DAgger 面键**不需要双 Grip 组合**；Grip 专门用于接管。

| VR按键 | 功能 |
|---|---|
| A | 确认录制就绪后开始模型推理，不倒计时 |
| 任一 GL/GR | 立即选择人工权限；握持时增量遥操，模型请求在后台退出 |
| B | 保存当前有效条目，等待结果；不自动复位 |
| Y | 丢弃当前条目；不自动复位 |
| X | 仅全身复位并张开夹爪；有未处理数据时拒绝 |

接管后由人完成本条，不自动交还模型。只有接管振动，其余状态和报错显示在头显下方。
下一条仍需主动按 A。终端保留已有键位：**C开始、A保存、X/D丢弃、R复位、Q退出**。
DAgger退出会丢弃未确认条目，想保留时应先B保存并确认；保存结果未知时先检查状态，不重复提交。

也可使用 `start_dagger.sh` 作为300的便捷入口，默认不发布硬件动作。
它预设服务器、21维、深度和数据目录；统一入口与这个脚本使用同一套实现。

### 桌面启动器

300桌面的“云蝶DAgger启动器”使用同一正式后端，保留原有界面、状态同步和按钮。
快捷方式预设允许按钮触发硬件控制；打开窗口本身不开始推理。
也可从终端启动：

```bash
DAGGER_PUBLISH=1 bash scripts/open_dagger_launcher.sh
```

省略 `DAGGER_PUBLISH=1` 时默认无动作模式。桌面Shift+A开始、Shift+B保存、
Shift+X仅复位、Shift+Y仅丢弃；退出用退出按钮，先处理当前数据并确认回执。
这个桌面窗口用于DAgger，其他模式仍使用本文统一脚本。

### 夹爪与 Thor

Thor 必须提供 `controller_submission_receipts=true`，并保持已验证的阶段夹爪策略。
抓取实测达到120°后锁持；放置明确预测≤60°时释放。左右分别判断，动作修改在服务端 digest 前完成。
当前 `phase_aware` 仅接受以下完整任务文本：

- `Pick the laundry bag.`
- `Place the laundry bag in the upper compartment of the delivery robot.`

机器人端不改写模型动作或 ACK 前缀。`/controller_ack` 证明已提交目标，不证明电机物理到位。
Thor 运维脚本为 `tools/start_thor_baseline.sh`；`GROOT_GRIPPER_PHASE_AWARE=0` 可回到连续模式。
该脚本会启动服务，只应在明确安排的服务维护时使用；机器人采集无需每次重启 Thor。

## 6. 数据、资源与兼容性

输出为 `<OUTPUT_BASE_DIR>/<task_name>/dataset`，同任务日志在旁边的 `logs/`。
同模式、同schema可正常续录；改变头部/腰部/深度、动作模式或子任务标注设置时换目录。

DAgger 记录整条模型与人工过程，增加 `dagger.control_source`、`dagger.train_mask`、会话和接管标记。
训练纠错优先筛选 `dagger.train_mask=1`；模型和保持段不冒充专家标签。
是否与历史专家数据混合、采样比例及训练配置需单独确定，本工具不自动训练或上传数据。

DAgger 已保留低CPU录制：录制进程CPU绑定、线程池限制、有限写入队列、无损临时图片。
这会增加临时磁盘占用；其他采集模式的长时间高负载表现需要分别验证。
不会通过放宽时间同步阈值或静默漏帧掩盖录制异常。

| 常用参数 | 作用 |
|---|---|
| `OUTPUT_BASE_DIR` / `TASK_TEXT` | 保存位置 / 整体任务文本 |
| `WITH_HEAD` / `WITH_UPPER_WAIST` / `WITH_DEPTH` | 头、上腰、深度开关；通用入口默认0 |
| `WITH_WAIST` | 完整腰腿字段；不能与上腰开关同时打开 |
| `VR_SPEECH` / `VR_START_DELAY_SEC` | 单人语音 / 开始倒计时，默认1 / 3秒 |
| `VR_A_LONG_PRESS_SEC` | 子任务B长按阈值，默认1秒 |
| `DAGGER_PUBLISH` | 硬件发布许可，默认0；不会自动开始 |
| `DAGGER_BACKEND` | `owned`启动宿主；`attach`接入已有兼容宿主 |
| `HG_DAGGER_RECORDER_CPUS` | DAgger录制CPU范围，300默认12-19 |

完整参数帮助：`bash start_lerobot_official_collect.sh --help`；单项：`bash start_lerobot_official_collect.sh WITH_DEPTH --help`。
旧脚本名保留兼容转发；历史路径和来源哈希留在溯源记录中，不改名已保存的数据。

## 7. 验证与源码导航

- [300测试与验收流程](ROBOT300_TESTING_zh.md)：四种模式、通过标准、停止和回滚。
- [实现说明](INSTRUCTION_zh.md) / [Technical details](INSTRUCTION.md)：同步、数据字段、生命周期和协议边界。
- [VR反馈接入](VR_FEEDBACK.md)：网页扩展、状态事件与接管振动。
- `collector_modes.py` / `start_lerobot_official_collect.sh`：统一模式和录制生命周期。
- `vr_collector_control.py` / `subtask_annotations.py`：组合键、语音、阶段标注。
- `dagger/control/`：已验证的控制、录制协议和GR00T桥接基础；`dagger/assets/`：随版本发布的配置、网页。
- `dagger/supervisor.py` / `controller_handoff.py`：集合版权限、保存回执和接管适配。
- `teleop_snapshot/`：已打补丁的V4审查快照和许可证；不要作为待打补丁的原始依赖。

```bash
/home/ubuntu/miniconda3/envs/lerobot/bin/python -B -m unittest discover -s tests -p 'test_*.py'
node tests/test_feedback.js
```

2026-10-08用户已完成DAgger、单人和子任务现场功能测试。
正式版的改名、资源打包和部署另做无动作回归；长时间稳定性、极端负载和硬件断线仍需现场持续验收。
