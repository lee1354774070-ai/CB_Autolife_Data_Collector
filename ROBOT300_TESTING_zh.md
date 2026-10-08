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
预检发现 dashboard、action_player、控制中心关节模块或原厂遥操冲突时，先确认任务空闲，
正常退出对应组件；不要批量kill，也不要停止/重启或清错ARM服务。
当前原厂数采仍在使用的服务不能为了DAgger测试直接关闭。

## 保存和数据检查

1. 只有 `[SAVED]` 和成功回执才计为保存成功，`保存中`不代表完成。
2. 核对 `dataset/meta/info.json`、episode条数和实际视频/Parquet存在。
3. 子任务核对 `subtask_index` 和 `annotations/subtasks/episode_*.json`，边界连续且不越界。
4. DAgger核对 `dagger.control_source`、`dagger.is_intervention`、`dagger.train_mask`。
   专家纠错mask为1，纯模型/保持段mask为0；模型条目不能直接当作专家监督。
5. 上线前检查首/中/末帧能被LeRobot加载，同步日志无 `EPISODE INVALID`，逐步增加连续录制时长。

DAgger低CPU录制暂存图像体积更大；保证磁盘空间和吞吐。
原有深度编解码和时间戳同步限制不变。不同schema不能向同一目录追加。

## 正常退出、更新与回滚

- 普通/VR/子任务：先等保存或丢弃回执，再Q；Q也会保存有效待处理条目。
- DAgger：先B保存或Y丢弃并确认，再Q；退出不替你保存未确认的条目。
- `DAGGER_BACKEND=attach` 的Q只断开附加终端，宿主和机器人状态由原界面管理。
- 程序运行中不切换目录、不替换源码；退出后再更新正式目录。
- 300旧目录的带模式命令会转发至正式目录；原个人脚本名仅作兼容别名。
- 回滚先退出正式版，再启动保留的原版本；使用独立数据目录，不同时运行两套控制器。
