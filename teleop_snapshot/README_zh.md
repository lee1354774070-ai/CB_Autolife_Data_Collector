# 修改后的 V4 遥操作副本

此目录是与已发布 Collector 对应的 V4 源码副本，已应用
`dagger/v4_controller.patch`，不再只有补丁文件。同事原源码未被修改。
对应 VLA Tools `2a3afd48`、独立 Collector `17cda47`，发布于 2026-10-07。
具体来源哈希见 [英文说明](README.md)。不包含机器人上后续未核对的实验修改。

保留源码、配置、URDF、测试及许可证声明；排除缓存、带本机路径的运行清单、旧 JS
备份和重复的嵌套网页目录。原包说明书描述原版遥操作，集成后的按键和启动方法以
Collector 的 README / INSTRUCTION 为准。

## 2026-10-08 MZJ 融合

已合入 `4055b75`。逐文件比较确认：除控制器补入 MZJ 复位张爪权限检查外，
其余73个非文档文件与300实际运行副本一致。已有增量遥操、平滑、IK和位姿配置保留。
`tests/dagger_ros_smoke.py` 会校验此快照与生成副本的74个非文档文件一致。
DAgger入口覆盖夹爪范围/响应参数并关闭旧复位手势，按键以 A 开始、B 保存、
X 仅复位、Y 仅丢弃为准。详细验收见 `ROBOT300_TESTING_zh.md`。

## 如何使用

仍通过 `COLLECTOR_MODE=dagger` 启动。启动器会校验原版依赖并生成不可覆盖的运行副本。
这里用于查看、版本管理和审查，不改变当前启动方式，也不会自动替换机器人运行中的程序。
不要将 `DAGGER_DEPENDENCY_ROOT` 指向这里：这里已经打过补丁，且未包含 HG 依赖包。

完整 DAgger 还使用 Collector 的 `dagger/mapper.py`、`supervisor.py`、
`controller_handoff.py`、`web_bridge.py` 等适配代码。直接运行原包 launch 文件
不等于启动我们的 DAgger。仍需安装 ROS 2、Placo、机器人 SDK/服务及已校验的 HG 包。
独立仓库使用 Thor 推理时，`DAGGER_TOOLS_ROOT` 仍需指向完整 VLA Tools 仓库。
不能同时运行两个硬件控制器。

保留上游 `NOTICE`、`THIRD_PARTY_LICENSES.md` 和源码版权信息，未更改原许可证。
代码发布及静态测试不代表真实机器人运动已经验收。

## 测试边界

原版测试作为历史源码保留，不代表全部通过。本机全量测试收集遇到 ROS/测试路径依赖
及过时的 `update_reset_stability` 导入。定向测试为 164 项通过、2 项失败：
旧接管测试未提供外围 `dagger` 模块，夹爪配置测试期望的复位姿态与本版配置不同。
没有为了通过旧测试而修改机器人复位位置。新版接管契约应结合 Collector 的
`tests/test_dagger_handoff.py` 与隔离 ROS 测试验证，不能据此认定实机运动已验收。
