# 云蝶 AutoLife 数据采集器

中文使用说明已统一到 [README.md](README.md)，避免多个入口文档内容不一致。

**常规数采使用 `ACTION_MODE=status_target`，DAgger 使用 `joint`。**
旧 `ACTION_MODE=eef` 已删除；`WITH_EEF_STATE` / `WITH_EEF_ACTION` 及离线EEF转换继续保留。

- [参数设置：关节、深度、帧率、同步、存储与模式专用参数](README.md#parameters)
- [Action 来源：常规数采与 DAgger 的区别](README.md#action-modes)
- [环境准备](README.md#setup)
- [新机器人部署与SSH密钥登录](docs/NEW_ROBOT_DEPLOYMENT_zh.md)
- [可选GPU视频编码及性能验证](docs/GPU_ENCODING_zh.md)
- [录制时GPU JPEG解码、像素差异与性能测试](docs/GPU_JPEG_zh.md)
- [CPU/GPU采集环境一键部署](docs/ENVIRONMENT_SETUP_zh.md)
- [四种模式使用的遥操作版本](README.md#modes)
- [键盘数采](README.md#keyboard) / [原厂单人VR](README.md#vr) / [子任务数采](README.md#subtask)
- [V4增量遥操作与DAgger](README.md#dagger)
- [续录、输出与故障处理](README.md#data)
- [为历史数据补提每条episode的三路RGB首帧](README.md#first-frame-extraction)
- [WITH_EEF_STATE / WITH_EEF_ACTION 与 Joint/EEF 自由组合转换](README.md#conversion)
- [300测试与验收](ROBOT300_TESTING_zh.md)
- [实现与数据格式](INSTRUCTION_zh.md)


四种采集模式（keyboard、vr、subtask、dagger）均支持采集会话只开启必要四路相机、退出恢复原配置，参见 [四路相机模式与测试](CAMERA_SESSION_zh.md)。每台机器人需先核验原厂视觉配置路径和本机配置，不会仅根据机器人编号启用。

## F封口与Q退出（所有采集模式）

键盘、单人VR、子任务和DAgger终端统一：**F仅封口，Q退出**。
先保存或丢弃当前条，按F收到 `[SEALED]` 或头显/启动器“本批已封口”后，
继续在同一程序中开始下一条；无需重启。F不会切换相机或退出控制器。
文件每条保存已落盘，F额外写入 `meta/collection_seal.json` 记录本批封口累计条数。
未处理的录制、保存异常或发布结果未确认时拒绝封口，保留数据。
DAgger失败保存终端改为K、启动器Shift+K，手柄长按Y不变；启动器F封口、Q退出（输入框内不触发）。
整场结束按Q；普通录制保留原有保存待处理条目语义，DAgger退出前须明确成功/失败保存或丢弃，不能由退出推断结果。
只读attach入口的Q仍仅断开当前客户端，不关闭他人拥有的后台服务。
