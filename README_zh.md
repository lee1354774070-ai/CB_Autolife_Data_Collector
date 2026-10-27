# 云蝶 AutoLife 数据采集器

中文使用说明已统一到 [README.md](README.md)，避免多个入口文档内容不一致。

**常规数采使用 `ACTION_MODE=status_target`，DAgger 使用 `joint`。**
旧 `ACTION_MODE=eef` 已删除；`WITH_EEF_STATE` / `WITH_EEF_ACTION` 及离线EEF转换继续保留。

- [参数设置：关节、深度、帧率、同步、存储与模式专用参数](README.md#parameters)
- [Action 来源：常规数采与 DAgger 的区别](README.md#action-modes)
- [环境准备](README.md#setup)
- [新机器人部署与SSH密钥登录](docs/NEW_ROBOT_DEPLOYMENT_zh.md)
- [S3历史采集验收记录](docs/validation/20261016_s3_collection.md)（与S2共用main，使用方法统一见README）
- [四种模式使用的遥操作版本](README.md#modes)
- [键盘数采](README.md#keyboard) / [原厂单人VR](README.md#vr) / [子任务数采](README.md#subtask)
- [V4增量遥操作与DAgger](README.md#dagger)
- [续录、输出与故障处理](README.md#data)
- [WITH_EEF_STATE / WITH_EEF_ACTION 与 Joint/EEF 自由组合转换](README.md#conversion)
- [300测试与验收](ROBOT300_TESTING_zh.md)
- [实现与数据格式](INSTRUCTION_zh.md)


四种采集模式（keyboard、vr、subtask、dagger）均支持采集会话只开启必要四路相机、退出恢复原配置，参见 [四路相机模式与测试](CAMERA_SESSION_zh.md)。300已配置启用，其他机器人需先核验原厂视觉配置路径再启用。
