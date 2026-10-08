# Robot 300：MZJ 融合集合版测试

更新：2026-10-08。分支：`mzj/robot300-thor-compat`。

## 版本与入口

使用 MZJ 已跑通的 supervisor、状态机、GR00T bridge 和 FIFO 作为基础，保留集合版
的来源标记、保存回执、附加终端、统一 ABXY 和头显反馈。
已合入同事主线 `4055b75` 的 V4 遥操快照；其73个非文档文件与原运行副本一致，
控制器补齐 MZJ 复位张爪的权限检查后，74个非文档文件全部一致。
增量映射、平滑、IK和位姿配置未重复重写。

300 代码目录：
`/home/ubuntu/collector_validation/20260930_github_dryrun/collector`

两个入口不要混用：

| 场景 | 入口与说明 |
|---|---|
| 本文的融合 DAgger | `start_mzj300_dagger.sh`；8447；A/B/X/Y；从启动推理开始录制 |
| 独立 V4 遥操 | 包内 `full_vr_teleop.launch.py`；通常8446；保留原包手势，包括 X+A；不等于 DAgger |

GitHub 的 `teleop_snapshot/README_zh.md` 明确快照用于发布和审查；集合版仍由
`COLLECTOR_MODE=dagger` 入口校验依赖、生成运行副本。不要把
`DAGGER_DEPENDENCY_ROOT` 指向已打补丁的快照，也不要同时启动原包 launch 和 DAgger。
原 MZJ 和共享 V4 源码未覆盖。

## 本次验证结果

全部为无真机动作测试；实体夹爪、全身复位、头显实际振动和持续运行负载仍待现场验收。

- 185项 Python 测试；另有 Node 头显反馈检查，只有接管触发振动。
- 隔离 domain211：快速扳机可到360°、360°反馈有效、361°指令拒绝；接管后旧动作被拒绝；
  保存阻塞不阻塞控制回调；保存结果不明时保留请求并禁止复位；trace启动失败可正确结束并重试。
- 单人采集、分段采集、附加终端验证通过；附加终端退出不终止宿主。
- 真正 RGB-D SHM/FIFO/视频写盘：保存100帧（62帧人工训练标记）及35帧纯模型段，
  丢弃第三条35帧；每条首/中/末帧可由 LeRobot 加载，纯模型训练标记为0，无录制失效。
  **关节和动作是合成测试信号，不能用于训练。** 文件在
  `/home/ubuntu/collector_validation/20260930_github_dryrun/data/mzj_merged_rgbd_synthetic_20261008_02/dataset`。
- 完整 `DAGGER_PUBLISH=0`、深度开启：DISARMED / DRY_RUN / idle，
  控制器硬件发布端点0、观测电机指令0、网页及说明可访问，随后正常退出。
- Thor baseline、phase_aware、40×21动作、digest/discard/close验证通过，无policy动作发布。
  当天无线连接曾导致2.2～6.5秒超时；重连后又出现过4秒峰值。
  重连并临时关闭两端Wi-Fi省电后，连续6轮约320～1203毫秒通过。
  2秒动作时效限制未放宽；这不代表无线网络已长期稳定，重连/重启后应再次只读检查。
- ARM服务未清错、未停止、未重启。测试期间暂退的dashboard/action_player已恢复。
  控制中心关节模块在确认未使能、无任务、5秒无指令后退出，主界面保留。

日志在300的 `/tmp/mzj_20261008_*.log`，重启可能清除。

## 2026-10-08：恢复 MZJ 低 CPU 录制

集合版此前遗漏了 MZJ 的录制资源限制和临时图片写入器，已补回，仅作用于 DAgger：

- 录制器及其线程、编码子进程绑定 CPU 12-19、nice=5；控制进程不随之绑定。
  `HG_DAGGER_RECORDER_CPUS` 保留原覆盖接口，无效 CPU 列表拒绝启动。
- 2 个图片写入线程、最多64项待写入；RGB临时PNG不压缩、深度保留原始uint16。
  保存时仍使用原视频编码、21维字段、接管标记和原同步阈值。
- Torch、Arrow、BLAS各线程池限制为1。队列满或磁盘失败可见，不静默漏帧。
- 接管/松开Grip时，实测夹爪超调不再直接成为越界保持指令；按配置范围生成保持目标。
  原始观测、Thor phase_aware输出和proposal/ACK均不改写。

用同一组真实四路640×480图像、同一CPU绑定进行10次写入比较：默认写入约72.68毫秒CPU时间/组，
MZJ写入约13.94毫秒/组，下降80.8%；RGB与深度读回像素逐个相同。
这是**图片写入开销**，不是整机CPU降幅。临时图片约由1.47MB/组增至3.39MB/组；
保存后的格式不变，长录制仍需保证磁盘空间和吞吐。

189项单元测试与隔离ROS接管回归通过；实际四路SHM、2线程、CPU12-19录制
保存100帧（61专家帧）和35帧（纯模型mask=0），第三条丢弃，首/中/末帧LeRobot加载通过。
以上关节信号为合成数据，位于验证目录 `data/mzj_lowcpu_rgbd_synthetic_20261008_01`，不可训练。
完整深度dry-run通过：所有录制线程CPU12-19/nice=5，控制器硬件发布端点0、观测电机指令0，
正常退出后8447释放。ARM服务PID2051始终未变。
CPU压力是否仍导致实际电机异动，必须现场复测，不能由无动作测试判定已消除。

复测沿用本文启动命令，换任务名 `mzj_lowcpu_live_02`。先X复位并确认张爪，再A；
依次检查模型闭合、Grip接管、快/慢扳机、松开保持、B保存、X复位和第二次A。
日志应显示 `Recorder isolation: CPUs=12-19, nice=5`、`Recorder resources`、
`MZJ capture: ... 2 writer threads`。异常测试轨迹先保留排查，不进入训练集。

## 先退旧，再启动新程序

1. 保存或丢弃旧任务并确认回执，正常退出旧8444/8446/8447遥操或采集程序。
2. 在控制中心停止“关节控制”模块，主界面可以保留。
3. 若原厂dashboard/action_player仍占用控制端点，确认没有使用者或播放任务后：

```bash
systemctl --user stop dashboard-backend.service
sudo systemctl stop void-cog-h5.service
```

**不要停止、重启或清错 `arm-control-service.service`。**
遇到冲突先核对报出的进程；不批量kill Python/ROS，不修改发布者白名单。

## 无动作检查

在300新终端中：

```bash
cd /home/ubuntu/collector_validation/20260930_github_dryrun/collector
source /home/ubuntu/ros2_ws/src/autolife_hg_dagger_MZJ_300/scripts/source_hg_ros_env.sh
set +u
DAGGER_PUBLISH=0 bash start_mzj300_dagger.sh mzj_fusion_dryrun --check
```

先验证相机、关节和Thor只读推理（只discard，不执行）：

```bash
PYTHONPATH="$PWD:${PYTHONPATH:-}" /usr/bin/python3 tests/remote/mzj_pinned_readonly.py
```

应出现两轮 `PINNED_MZJ_DISCARD_CLOSE_PASS` 和 `PINNED_MZJ_NO_MOTION_PASS`。
若出现超时、相机不同步或服务错误，先解决再进入真机模式，不提高时效上限掩盖问题。

然后启动完整待命界面：

```bash
DAGGER_PUBLISH=0 bash start_mzj300_dagger.sh mzj_fusion_dryrun
```

头显打开 `https://192.168.8.122:8447`，刷新到新说明。进入/重连应保持待命。
本模式用于启动和页面检查，不能验收真实动作或产生可训练的动作数据。
终端 Q 或 Ctrl+C 正常退出，再启动下面的真机模式。

## 有人在场的真机验收

确认运动空间和物理急停可用，使用独立任务名，不与旧schema混写：

```bash
DAGGER_PUBLISH=1 \
TASK_TEXT='Pick the laundry bag.' \
WITH_DEPTH=1 \
bash start_mzj300_dagger.sh mzj_fusion_live_01
```

输出：`/home/ubuntu/nas/dagger/mzj_fusion_live_01/dataset`。
`nas` 是普通目录名。当前baseline固定头部+上腰，对应21维；深度可用 `WITH_DEPTH=0` 关闭，
但改变schema时要换任务目录。phase_aware仅接受以下完整任务文本：

- `Pick the laundry bag.`
- `Place the laundry bag in the upper compartment of the delivery robot.`

| 操作 | 期望结果 |
|---|---|
| 进入/重连头显 | 待命，不自动推理、不自动录制 |
| 空闲时 X | 全身复位并打开夹爪；不开始下一条 |
| A | 先确认录制就绪，再允许模型；无倒计时 |
| 任意 GL/GR | 立即选择人工权限；仅此时振动，不等待HTTP结束 |
| 松开再握 | 松开保持，再握从实测位姿继续增量控制；不自动交还模型 |
| 快速/慢速扳机 | 都能完整开合；双手分别测，检查有无截断或异动 |
| 头/上腰跟随 | 人工接管后按开关生效；模型控制时不被人覆盖 |
| 录制中 X | 拒绝并提示先B/Y；不丢数据、不机械复位 |
| B | 保存整条轨迹，等待明确成功；不自动复位、不自动开始 |
| Y | 丢弃本条，等待回执；不复位、不退出 |
| 保存/丢弃确认后 X | 全身复位且夹爪张开，完成后等待A |
| 浏览器聚焦后 Shift+A/B/X/Y | 与面键同义；无需握持键组合 |
| 急停/输入失联/录制失效 | 停止输出并在头显显示原因；不得默默继续录坏数据 |
| 终端 Q / Ctrl+C | 正常退出自己的子进程、结束文件写入；Y不是退出 |

至少完成“模型→接管→B→X→A”“模型→Y→X→A”各一轮，并累计重复3轮。
保存不明时B/Y只核对原请求结果，不重复执行；不要强行复位或直接kill。
B可保存纯模型数据，其 `dagger.train_mask` 全为0；无接管且不想保留时用Y。

数据验收：核对保存回执、episode数、三路RGB和深度、每条首/中/末帧可加载，
检查21维action/state、单调时间戳、接管标记和 `dagger.train_mask`。
模型及保持段mask应为0，人工段只有来源与时间戳满足条件才为1。
现场持续录制再观察CPU、夹爪和录制失效提示；本次短时软件测试不能替代负载验收。

## 单人/分段与终端按键

单人VR保留 GL+GR 组合条件，再点按并松开面键：A开始，B保存，X仅复位，Y仅丢弃。
分段模式B短按标记、最后标记自动保存，B长按提前保存；
`VR_A_LONG_PRESS_SEC` 为兼容旧参数名，实际控制B长按阈值。

DAgger终端保留原键位：C开始、A保存、X/D丢弃、R复位、Q退出。
`DAGGER_BACKEND=attach` 中Q只断开终端，不关闭宿主。

## 普通数采、单人VR和子任务现场验收

普通键盘、单人VR和子任务采集默认使用**原厂遥操**，不需要打开8446。
原厂遥操保持运行；DAgger须先保存/丢弃并退出，避免控制器同时运行。
使用已经部署的集合版绝对路径；`/home/ubuntu/lerobot_data_collector` 是旧版，
不识别 `COLLECTOR_MODE`。新终端不要依赖此前导出的变量：

```bash
source /home/ubuntu/ros2_ws/src/autolife_hg_dagger_MZJ_300/scripts/source_hg_ros_env.sh
set +u
cd /home/ubuntu/collector_validation/20260930_github_dryrun/collector
unset VR_CONTROL SUBTASKS_JSON
export OUTPUT_BASE_DIR=/home/ubuntu/nas/collector_test
export TASK_TEXT='Pick the laundry bag.'
export WITH_HEAD=1 WITH_UPPER_WAIST=1 WITH_WAIST=0 WITH_DEPTH=1
export START_HAND_PRODUCER=0 ACTION_MODE=status_target
export VR_INPUT_TOPIC=/control_topic_0_300
export VR_SPEECH=1 VR_START_DELAY_SEC=3
```

原厂模式启动后必须显示 `[VR] Listening: /control_topic_0_300`。
使用按键前戴上头显、进入原厂沉浸式遥操；停留在菜单或手柄未激活时，
话题可能仍有消息，但按键全部为松开，采集端无法识别组合键。
只有明确使用独立V4遥操时才改用它的输入话题；不要把DAgger的网页/输入配置用于原厂数采。
下面一次只运行一个采集模式，每种结束用终端Q退出，原厂遥操保持运行。

| 模式 | 采集启动命令 | 现场流程和通过标准 |
|---|---|---|
| 普通键盘数采 | `COLLECTOR_MODE=keyboard bash start_lerobot_official_collect.sh keyboard_01` | Enter开始，遥操5～10秒后S保存；再Enter录制后D丢弃；再保存一条，Q退出。应有2条保存，丢弃不增加条目。 |
| 单人VR | `COLLECTOR_MODE=vr bash start_lerobot_official_collect.sh vr_01` | 全程按住GL+GR，再短按并松开面键。A倒计时3秒开始，B保存，Y丢弃，X仅复位。录制中X应拒绝；原厂模式X会明确提示复位未接入，使用原厂复位功能。倒计时中B/Y应取消。 |
| 子任务 | 见下方 | A开始，第一/二次短B标记并继续录制，第三次短B自动保存。第二条长B至少1秒再松开，应提前保存并标记待审核。Y丢弃后下一条从第一个子任务开始。 |

```bash
COLLECTOR_MODE=subtask VR_A_LONG_PRESS_SEC=1.0 \
SUBTASKS_JSON='["右手抓起袋子","移动袋子","放下袋子"]' \
bash start_lerobot_official_collect.sh subtask_01
```

单人和子任务模式的A/B/X/Y均需GL+GR组合，面键松开时双Grip仍需按住。
网页实际发送面键0/1，2026-10-08已修复采集端只接受bool的兼容遗漏；非法值仍拒绝。
本次反馈补丁通过196项单元测试，以及原厂格式的单人/分段隔离ROS链路；
此前V4数值按键格式也已通过两种隔离链路。隔离测试不发布硬件动作，不替代现场验收。
用户在原厂沉浸式遥操中确认组合键可用；现场日志`unified_vr_02/logs/5.vr_control.log`
记录完整3/2/1、A开始、B保存146帧，数据目录累计3条640帧。
这条现场记录来自部署反馈补丁前的版本，用于确认原厂按键与保存链路已恢复。
原厂网页不自动加载集合版头显提示扩展，以采集终端和机器人语音回执验收。
每次A按键接受后播报“三、二、一”，完整倒计时后才发送录制请求；收到回执再播报“开始录制”。
B播报保存中，收到回执后播报已保存条数和本条帧数；Y播报丢弃结果。
没有待处理数据、保存忙碌、输入失联或失败都有明确提示。日志中的`[VR input]`、
`Button accepted`、`Countdown`、`[VR speech]`可区分按键未到达、组合未匹配和命令等待。
音量故障排查应检查原厂TTS接收日志及实际输出声卡；采集程序本身不改变系统音量。

同模式、同参数、同任务名正常退出后重新启动，保存一条，应接着编号，旧数据仍在。
深度开关、关节维度、action模式或子任务标注开关改变时必须换任务目录。
分段数据额外核对`subtask_index`及`dataset/annotations/subtasks/episode_000000.json`；
完整条目0/1/2段连续覆盖，提前保存的未确认尾段为-1，`needs_review=true`。
这几种普通示教不使用DAgger的`dagger.train_mask`。

本轮低CPU写入器/CPU绑定仅作用于DAgger，其他模式的长时录制负载尚需单独验收。
先做每条5～10秒的功能测试，再做连续多条；记录失败或夹爪异常即停止本轮并保留日志。

## 结束与回滚

先B保存或Y丢弃并确认，再Q退出。若需要重新使用原厂控制入口：

```bash
systemctl --user start dashboard-backend.service
sudo systemctl start void-cog-h5.service
```

在控制中心重新启动“关节控制”模块。核对服务健康和端口释放，不以start命令返回代替确认。
回滚时先退出融合版，再使用原MZJ入口；原源码未覆盖，禁止同时启动两套。
