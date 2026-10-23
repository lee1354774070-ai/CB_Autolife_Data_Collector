# Robot 300 正式版测试与验收

代码目录：`/home/ubuntu/CB_Autolife_Data_Collector`。发布分支：`main`。
所有模式统一使用 `start_lerobot_official_collect.sh` 和 `COLLECTOR_MODE`。
完整安装和命令见 [README](README.md)。历史验证记录保存在
[集成阶段记录](docs/validation/20261008_integration_history.md)，其中的旧路径不是新入口。

## 发布检查与已有证据

本轮整理：入口和内部模块去除个人前缀，DAgger配置和网页随版本打包，
保留旧命令跳转；不改变控制参数、动作维度、阶段夹爪策略、ACK或数据字段。
原始来源、哈希保存在 `dagger/control/SOURCE.json`、`dagger/assets/SOURCE.json`。
原V4控制器仍按哈希检查并在独立副本运行；原个人实验目录和已保存数据保留。

2026-10-08用户已完成基本现场功能测试，日志证据：

- 单人 `unified_vr_02`：3条、640帧；最新条146帧，完整A倒计时和B保存回执。
- 子任务 `subtask_vr_02`：1条、616帧；3/3阶段确认、`complete=true`、`needs_review=false`。
- 用户反馈DAgger工作正常；录制负载和长时间硬件稳定性仍需持续验收。

这些是整理前已跑通版本的现场证据。正式版的重命名和资源打包以本轮无动作回归为准，
不能用隔离测试代替真实夹爪、复位、网络波动或硬件断线测试。

本轮正式版无动作检查已通过：199项Python测试、接管振动JS测试、四条隔离ROS链路，
V4运行副本74个源码文件一致；完整待命节点和网页在domain211/18447验证通过，
`DISARMED`、`dry_run=true`、policy idle，硬件发布端点和观测运动指令均为0，正常退出。
使用真实观测的两轮Thor baseline只读推理分别约767/652毫秒，40×21动作，
proposal全部discard并close，没有发布policy动作。正式工具目录的健康预检也通过。
桌面入口经独立虚拟显示检查：窗口创建成功、业务按钮初始禁用、打开不启动子进程，
没有硬件调用；桌面文件格式校验通过。
原始本轮日志在300的 `/tmp/collector-release.8N91UO/`；临时目录可能在重启后清除。

## 无动作回归

```bash
cd /home/ubuntu/CB_Autolife_Data_Collector
source scripts/source_ros_env.sh
set +u
/home/ubuntu/miniconda3/envs/lerobot/bin/python -B -m unittest discover -s tests -p 'test_*.py'
node tests/test_feedback.js
export DAGGER_DEPENDENCY_ROOT=/home/ubuntu/ros2_ws/src
ROS_DOMAIN_ID=211 /usr/bin/python3 tests/dagger_ros_smoke.py
ROS_DOMAIN_ID=211 /usr/bin/python3 tests/dagger_attach_ros_smoke.py
ROS_DOMAIN_ID=211 /usr/bin/python3 tests/vr_ros_smoke.py
ROS_DOMAIN_ID=211 /usr/bin/python3 tests/vr_ros_smoke.py --subtasks
```

隔离测试使用合成信号、模拟服务或独立FIFO，不操作真机。
关注通过标志及退出码；测试需要机器人上对应ROS环境，不能把缺依赖跳过计为通过。

真实相机和关节观测的只读推理检查：先确认没有其他模型会话，再执行：

```bash
PYTHONPATH="$PWD:${PYTHONPATH:-}" /usr/bin/python3 tests/remote/dagger_pinned_readonly.py
```

只生成并丢弃proposal，不执行动作；预期两轮 `PINNED_DAGGER_DISCARD_CLOSE_PASS` 和
`PINNED_DAGGER_NO_MOTION_PASS`。网络、相机或时效失败先排查，不提高阈值掩盖问题。

## 四种模式的现场流程

使用README中的完整环境，每种模式用新任务名，每条先做5～10秒。
普通/单人/子任务保持原厂遥操；DAgger需要先退出其他控制任务。

| 模式 | 测试顺序 | 通过标准 |
|---|---|---|
| keyboard | Enter→S；Enter→D；Enter→S；Q | 保存2条；丢弃不增加条数；退出完成 |
| vr | 戴头显进入原厂遥操，GL+GR+A；B保存；第二条Y丢弃；第三条B保存；终端Q | 每条倒计时、开始和结果有语音；保存2条 |
| subtask | A后做3阶段，各短B一次；第二条长B提前保存；第三条Y丢弃 | 完整条标注0/1/2；提前保存尾段-1且待审核；下一条进度归零 |
| dagger | dry-run检查→退出→现场模式；A模型；Grip接管；快/慢扳机；B保存；X复位；再A；Y丢弃；Q | 进入不自启；接管振动；人工权限优先；夹爪完整行程；保存回执；全身复位张爪；下一条可启动 |

VR/subtask面键需保持双Grip，DAgger面键不需要双Grip。
普通原厂模式X复位尚未接入；它会提示使用原厂复位，不调用另一套控制器。
DAgger B/Y不自动复位，X不保存也不丢弃。有未处理条目或结果未知时拒绝复位。

DAgger启动检查：

```bash
unset SUBTASKS_JSON VR_CONTROL ACTION_MODE
DAGGER_PUBLISH=0 bash start_dagger.sh dagger_release_check --check
```

预检成功后按README运行完整 `DAGGER_PUBLISH=0` 待命检查，退出后再由现场人员启动发布模式。
`--check`发现冲突会只读报告。正式启动（包括DAGGER_PUBLISH=0待命启动）会自动确认并正常退出锁定空闲的控制中心关节模块、无任务/循环/录制的Void-Cog后台，再重复观察4秒确认控制接口释放。控制中心界面、导航与Void-Cog前端保留；未知或忙碌组件明确拒绝修复。退出超时不强杀；不要停止/重启或清错ARM服务。启动器应显示具体自检阶段和中止原因。
当前原厂数采仍在使用的服务不能为了DAgger测试直接关闭。

## 保存和数据检查

1. 只有 `[SAVED]` 和成功回执才计为保存成功，`保存中`不代表完成。
2. 核对 `dataset/meta/info.json`、episode条数和实际视频/Parquet存在。
3. 子任务核对 `subtask_index` 和 `annotations/subtasks/episode_*.json`，边界连续且不越界。
4. DAgger核对 `dagger.control_source`、`dagger.is_intervention`、`dagger.train_mask`、21维 `dagger.action_train_mask`。仅身体纠错而夹爪保持旧指令时，整帧掩码为0、身体19维为1、夹爪2维为0；模型/保持段全0。升级后使用新任务目录，旧schema不直接续录。
   训练部分纠错时用逐维掩码；旧整帧掩码保留保守规则，模型条目不能直接当作专家监督。
5. DAgger结果标签验收：第一条A开始、不接管、成功后B；第二条Grip接管成功后B；第三条按住Y满1秒（也可用启动器失败按钮/Shift+F/采集终端F）；第四条Y短按松开丢弃。前三条annotations/dagger/episode_XXXXXX.json的outcome依次为autonomous_success/human_corrected_success/failure；失败条task_success=false，human_takeover如实记录，第四条不新增episode或标注。Y按下不能提前丢弃，长按保存后松开不能再丢弃；右摇杆不再保存失败。断线、忙碌、换条目后需要松开再按。不建分类文件夹；确认纠错掩码没有因结果标签改变。头显常亮提示、保存中/成功/错误提示与启动器同步；仅Grip接管振动。
6. 上线前检查首/中/末帧能被LeRobot加载，同步日志无EPISODE INVALID，逐步增加连续录制时长。

DAgger低CPU录制暂存图像体积更大；保证磁盘空间和吞吐。
原有深度编解码和时间戳同步限制不变。不同schema不能向同一目录追加。

若出现 `EPISODE INVALID`，先保留该任务的 `logs/` 和 `dataset/sync_log.jsonl`，不要将失效条目用于训练。
失效事件中的 `capture_diagnostics` 记录最近相机轮询间隔、上一帧录制耗时、超过20ms的最近一次Python GC暂停，以及相机生产/读取时间戳。
它用于区分生产端停顿与采集进程延迟，不放宽150ms新鲜度或30ms同步阈值；仅凭 `stale_reference_camera` 不能判定相机硬件故障。
诊断复测用新任务录制30–45秒并接管一次；有效时B保存并等待成功回执，失效时保留日志、记录时间，不反复重试。

2026-10-22 已在正式CPU限制（12–19核、nice=5）下复现同步日志写盘阻塞：第489帧 `_log_sync` 耗时271ms，随后帧失效。同步日志现按顺序进入有上限的后台队列，保存前等待写完并检查错误；队列满/写入失败仍拒绝保存，不丢日志、不放宽阈值。修复后同环境901帧RGBD录制、保存及首/中/末帧加载通过；这是隔离数据验证，不代表现场运动验收。

复位验收先在安全区域由操作员执行：选择任务和“复位姿态”，未启动服务时点击启动器“全身复位”，确认所有身体关节到任务姿态、双夹爪全开，并自动退出临时控制器；此流程不需要模型服务。然后启动DAgger，录制一条并B保存（或Y丢弃）后按X，检查同样的到位效果。有未处理条目时X必须拒绝，碰撞/到位超时/状态丢失不能显示成功。停止独立复位时只报告“到位未确认”。

任务复位沿用 `/home/ubuntu/MZJ_laundry_launcher/laundry_cycle_plan.json` 的 `task_poses` 和 `poses_file`。桌面取袋对应task4，配送取袋task0，洗衣机取袋task1，放配送task3，放洗衣机task2，放烘干机task5。可通过 `DAGGER_RESET_PLAN` 选择其他计划，通过启动器姿态下拉框或 `DAGGER_RESET_POSE` 明确指定姿态。新机器人计划必须声明自己的 `robot_id` 并完成标定；无robot_id的旧计划仅属于300。

自定义指令（包括含糊的 `Pick the laundry bag.`）必须明确选姿态。本轮启动时保存独立 `reset_pose_<时间>.json`，锁定选择；修改任务/姿态需结束当前服务重新启动。不会修改控制中心全局复位配置。夹爪仍按采集流程要求全开，不采用姿态库内的夹爪角度。

2026-10-23无动作检查：`pick up towel`、`bag`、`laundry_up` 从保存的关节反馈起点通过碰撞/限位路径检查；旧 `laundry_down` 的膝关节155°超过当前软限位2°，需重新标定或明确选择已验证替代姿态，不能关闭限位。离线路径检查不等于现场运动验收。

## 正常退出、更新与回滚

- 普通/VR/子任务：先等保存或丢弃回执，再Q；Q也会保存有效待处理条目。
- DAgger：先B成功保存、F失败保存或Y丢弃并确认，再Q；退出不替你保存未确认的条目。
- `DAGGER_BACKEND=attach` 的Q只断开附加终端，宿主和机器人状态由原界面管理。
- 程序运行中不切换目录、不替换源码；退出后再更新正式目录。
- 300旧目录的带模式命令会转发至正式目录；原个人脚本名仅作兼容别名。
- 桌面快捷方式文件为 `autolife-dagger.desktop`，显示名仍为“云蝶DAgger启动器”，使用同一正式后端。
- 回滚先退出正式版，再启动保留的原版本；使用独立数据目录，不同时运行两套控制器。
