# S3适配合入main：本地验证

> 以下记录的是当时的合并与测试。后续精简已删除独立S3推理预览、回放脚本及其专属测试；
> 环境适配、型号标签、EEF采集/转换和DAgger保护继续保留。旧测试数量不是当前版本的验收结果。

## 合并范围

- Collector主分支起点：`7be8d0cf68be81e7d87c0ba25621411390561dd9`。
- 合入S3分支：`50d451de7f46b9fe7a7ddcf4027193a177846049`，保留真实合并历史。
- 集合仓库起点：`5072354d6e684942da9c89769fde043c3ac90c53`；仅同步 `lerobot_data_collector/`，其他本地改动不纳入。
- 保留主分支的EEF采集、v3转换和逐维DAgger掩码；加入S3环境、机器人元数据、只读推理预览及历史记录。
- 环境脚本保留显式ROS域/编号，支持缺少独立ROS工作空间；ROS加载失败时也恢复调用者的nounset设置。
- README和中英文实现说明改为S2/S3共用main；既有S2默认数据布局和action模式不变。

## 回归结果

本机ROS2 Jazzy、Conda lerobot，ROS域211、Fast DDS、本机通信、离线模式，CPU线程限为1。
合成帧仅用于测试数据，不连接机器人、相机SHM、Thor服务或真实控制器。

| 范围 | 结果 |
|---|---|
| 集合仓库Collector测试 + `deploy/tests/groot_n1_7/test_robot_io.py` | 271通过，120个子用例通过 |
| 独立Collector仓库，仅自身与ROS的PYTHONPATH | 263通过，4跳过，120个子用例通过 |
| 网页反馈测试 `tests/test_feedback.js` | 1通过 |
| 所有已跟踪 `.sh` 的 `bash -n` | 通过 |
| ROBOT_TYPE、WITH_EEF_ACTION定向帮助及S3预览帮助 | 通过 |
| Git差异空白/冲突检查 | 通过 |

独立仓库跳过的4项是可选Thor协议测试，因为独立Collector不附带推理runtime；
这4项已在集合仓库运行通过，没有通过跳过掩盖失败。

存储集成覆盖两种机器人标签 × EEF state开关 × EEF action开关 × DAgger开关，
共16个组合。使用真实LeRobot写入、保存、加载、续录和转换流程，检查向量尺寸、
机器人标签与掩码保留；输入为合成关节/位姿和小尺寸图像，不代表硬件同步验收。

## Thor依赖边界

初次回归发现基础runtime缺少 `gripper_phase_aware` 参数，3项S3协议测试失败。
该参数来自已存在的 `tools/patch_cb_thor_gripper.py`，不是普通数采的依赖。
现已增加加载模型前的明确依赖检查，文档说明先在独立runtime副本上审核/应用补丁。
本次测试只对SHA256匹配的runtime源码做内存变换，复用原夹爪补丁后测试协议；
没有修改集合仓库的deploy源码或已部署服务，也没有加载真实模型权重。

预览保持独立入口、专用mode、禁止硬件执行和拒绝执行ACK的限制。
2026-10-16的机器人/Thor记录仍为历史证据，本次没有重新验证网络地址、
真实相机、运动、长时负载、S3 EEF报告或DAgger接管。源码合并不意味着机器人已更新。
