# S3 collector 测试与使用

分支：`s3-collector`。2026-10-16 基于 GitHub main `04621f13c62a1f79228af3248336d381a930bb38`。
当前部署目标 S3-347：`ubuntu@192.168.8.28`。S3-346（`192.168.8.27`）已完成一次静态采集验证；S3-345（`192.168.8.176`）未验证。每台机器的验收结论以各自 `validation.json` 为准。

## 适配范围

- 普通、VR 单人、子任务共用现有原厂 ROS JSON 和相机 SHM 接口。
- 环境脚本按主机名识别机器人编号；S3 没有 `ros2_ws` 时仅加载系统 ROS。
- 启动器根据 `ROBOT_PY` / `LEROBOT_PY` 的实际基础解释器选择动态库，支持独立 venv。
- 采集适配保留原厂控制和相机参数。本次为恢复语音调整了原厂TTS配置，并在空闲时重载vision服务；ARM服务未重启。
- DAgger已拆分机器人编号与型号配置，使用`ROBOT_ID=347 DAGGER_ROBOT_PROFILE=s3_v3_1`。347的增量遥操和独立控制权租约接口尚未部署，真实接管、复位、运动/ACK尚未验收；不能把编号伪装成300或跳过接口检查。

S3 原厂反馈按同名字段解析为双臂14 + 夹爪2 + 头部3 + 上腰2，共21维。
`ACTION_MODE=status_target` 的夹爪标签来自实测位置，不是独立夹爪命令；沿用现有契约。
该 schema 可被读取不代表旧机器人策略或归一化参数可直接用于 S3。

## S3-347 本次部署

```bash
cd /home/ubuntu/collector_validation/20261016_s3_347/collector
source scripts/source_ros_env.sh
set +u
unset VR_CONTROL SUBTASKS_JSON ACTION_MODE HG_DAGGER_ROBOT_ID
export ROBOT_ID=347 ROBOT_TYPE=autolife_s3
export LEROBOT_PY=/home/ubuntu/collector_validation/20261016_s3_347/venv/bin/python
export OUTPUT_BASE_DIR=/home/ubuntu/nas
export WITH_HEAD=1 WITH_UPPER_WAIST=1 WITH_WAIST=0 WITH_DEPTH=1
export START_HAND_PRODUCER=0 ACTION_MODE=status_target
export TASK_TEXT='Pick the laundry bag.'
```

隔离环境为 LeRobot 0.6.0 + CPU PyTorch。完整锁定列表见上级 `requirements-tested.txt`。
此目录为验证部署，不会自动随 GitHub 更新。测试数据位于上级 `data/`，标记为不可训练。

按模式选择其中一条：

```bash
COLLECTOR_MODE=keyboard bash start_lerobot_official_collect.sh s3_keyboard_01
COLLECTOR_MODE=vr bash start_lerobot_official_collect.sh s3_vr_01
COLLECTOR_MODE=subtask SUBTASKS_JSON='["抓起袋子","移动袋子","放下袋子"]' \
  bash start_lerobot_official_collect.sh s3_subtask_01
```

- 键盘：Enter 开始；单按 S 保存、D 丢弃、Q 正常退出。S/D/Q 后不需要回车；回车本身会开始下一条。
- 单人：戴好头显并进入原厂沉浸式遥操，按住 GL+GR，再点按并松开 A/B/Y。A 倒计时321后录制，B 保存，Y 丢弃。
- 子任务：同样 GL+GR + A 开始；短按 B 标记阶段，最后一次自动保存；长按 B（默认1秒）提前保存，未确认部分为 `subtask_index=-1`；Y 丢弃。
- X 原厂复位尚未集成，需使用原厂的复位入口。不要把隔离测试中的模拟复位成功当作真机复位能力。
- 退出在终端按 Q。三种模式按键已由操作员测试；语音修复后短句试听完整清晰，完整按键播报流程建议再验一次。

## 硬件同步问题

现场已明确有同步帧硬件问题，本次不修改同步阈值、不改时间戳、不丢帧绕过保护。
若出现 `EPISODE INVALID` 或相机/同步报错，记录错误后丢弃当前条目；硬件返修后重验。
短时间通过仅说明该时间段采集正常，不代表硬件问题修复。

## S3-347 静态验收结果（2026-10-16）

操作员重启后，347 恢复原厂关节反馈，4秒内收到401条完整反馈。
真实静态录制保存2条，共179帧（89 + 90）；另有59帧通过丢弃操作移除，未计入已保存数据。
两条均为21维状态/动作、三路RGB和一路深度。8个视频逐帧解码通过，LeRobot逐条首/中/尾加载通过。
单条约3秒录制的保存回执耗时约6.36/6.46秒；这是短条测量，不代表长条耗时。
正常退出后重新打开同一数据集，识别已有2条并保持暂停；10个已保存数据/视频文件哈希不变。
本次短录未触发无效条目保护，相机对齐最大偏差约15.6毫秒；已知硬件同步问题仍未处理。
测试数据：上级 `data/s3_live_rgbd_01/dataset`；证据见上级 `live-validation.json`、`live-control-receipts.json` 和 `validation.json`。
上述为早先静态验证；随后现场按键与语音修复结果见下节。运动采集、长时负载和S3 DAgger仍未验收，静态测试数据不能用于训练。

## 软件验证

在本次隔离环境执行，合成输入只使用 ROS 域211：

```bash
source scripts/source_ros_env.sh
export ROS_DOMAIN_ID=211 HF_HUB_OFFLINE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export LD_LIBRARY_PATH="/home/ubuntu/miniconda3/envs/robot_env/lib:${LD_LIBRARY_PATH:-}"
../venv/bin/python -B -m unittest discover -s tests -p 'test_*.py'
../venv/bin/python -B tests/vr_ros_smoke.py
../venv/bin/python -B tests/vr_ros_smoke.py --subtasks
```

测试后重新开终端或重新 source 环境脚本回到域0，再运行实际采集。
机器上级目录保留 `unit.log`、`vr-smoke.log`、`subtask-smoke.log` 和实际数据验证报告。

## 2026-10-16 现场按键验收与修复

- 普通保存1条880帧、丢弃1条131帧；单人保存1条174帧、丢弃1条227帧。
- 子任务保存512帧完整标注（238/152/122）和355帧提前保存（86帧已标注，269帧=-1）。
- 共4条1921帧；16个视频全量解码、12个LeRobot首中尾加载点通过；原数据内容哈希在只读审查前后不变。
- 操作员确认本次仅按键测试，没有实际运动，不能据此验收运动采集。
- S3数据的新建元数据请明确使用 ROBOT_TYPE=autolife_s3。旧数据续录保留原元数据，不会自动改型号。
- 347原厂配置 tts_enabled=false 导致收到播报后报 AIChatbotManager 无 tts。已启用TTS并使用已有离线Piper，重载vision后正常初始化；操作员确认测试短句完整清晰，ARM进程未重启。
- 346旧数据及347源JPEG均出现左手视角比右手窄；collector无裁切、缩放或去畸变处理。镜头/相机内部处理和安装差异待硬件确认。

- 已备份并更正三套用户数据的robot_type为autolife_s3，复查只有该字段变化；四路相机与LeRobot加载复核通过。
- 本轮87项针对性测试通过。详见上级user-audit-20261016/RESULTS_zh.md及post-repair-validation.json。
- 右腕marker移除及原厂复位对照尚未完成；保存耗时、运动采集和长时负载仍需单独验收。

## DAgger 与旧模型的无动作验证

2026-10-16 在347完成以下检查；不代表S3真机运动验收：

- ROS域211中运行真实采集器及监督器，使用真实四路相机、合成关节/控制信号。
- 保存2条共135帧：第一条38帧policy和62帧expert，第二条35帧policy且训练mask全0；第三条35帧丢弃。
- 8个视频全部解码，LeRobot逐条首/中/尾加载通过；纠错标记、训练mask和命令时间戳因果关系检查通过。
- 测试目录有 `SYNTHETIC_SIGNALS_DO_NOT_TRAIN.txt`，仅供检查流程，不能用于训练。
- Thor `192.168.8.224:8777` 的GR00T baseline通信正常。用真实S3静态观测回放3次，每次返回40×21动作，digest检查通过；全部discard后关闭本次session，没有执行ACK或下发真机动作。端到端耗时约0.56、1.58、0.74秒。
- **旧S2模型不能直接向S3下发动作**：S3实测左肘约−110°，旧模型首帧却输出+52.5～+70.0°；左肘及部分头部动作超出S3的URDF范围。需要验证状态/动作坐标映射及S3控制器，而不是客户端裁剪动作。
- ARM和vision进程在上述验证中均未重启。68项DAgger针对性测试通过。

上面的历史验证发生在编号限制修复前。当前入口已接受显式编号/型号，但仍校验控制依赖、模型描述和服务端型号；347缺失控制接口及仅供预览的8779仍会阻止真机执行。
只读回放使用的VR数据已增加到2条364帧；其中新增190帧未包含在前述1921帧的完整审计中。

## 其他S3恢复离线语音

347的原厂vision配置位于：
`~/miniconda3/envs/robot_env/lib/python3.12/site-packages/autolife_robot_vision/settings.toml`。
在已安装Piper资源的机器上，备份配置后仅核对 `tts_enabled = true` 和 `TTS_PROVIDER = "piper"`。
服务空闲时重新加载vision，再做一次现场试听；不需要重启ARM。
不要上传完整配置，该文件可能含服务密钥；其他机器人需单独确认资源和出声效果。

分支提交前回归：347隔离域211中227项Python测试全部通过；网页反馈测试通过，确认仅接管振动；两个启动环境脚本语法检查通过。

## 分支与推理隔离

- `main` 继续用于现有S2；`s3-collector` 用于本次S3适配。当前不合并main，不改变S2部署。
- S3采集适配和推理预览是分开的提交。新增 `s3_thor_preview.py` 是独立入口，普通、VR和子任务的启动方式不变。
- S3预览已完成状态/动作方向转换、夹爪阶段逻辑和真实观测回放；**S3 DAgger真机接管、复位、排他控制与运动验收仍未完成**。
- 预览使用Thor独立容器和8779端口；原8777/8778服务、源码和checkpoint没有替换。预览容器验证后已停止，不持续占用GPU。
- 详细结果、复现方式和限制见 [S3推理验证记录](docs/validation/20261016_s3_inference.md)。

## 多机器人入口与训练掩码修复

只新增这两个逻辑修复，详见[修复验证记录](docs/validation/20261016_dagger_profiles_masks.md)。
新增21维`dagger.action_train_mask`保留身体纠错，沿用模型的夹爪维度继续屏蔽；旧整帧掩码不变。升级后请用新DAgger任务目录，旧数据不自动修改。
