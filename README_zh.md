# 云蝶 AutoLife 数据采集器

中文使用说明已统一到 [README.md](README.md)，避免多个入口文档内容不一致。

**常规数采使用 `ACTION_MODE=status_target`，DAgger 使用 `joint`。**
旧 `ACTION_MODE=eef` 已删除；`WITH_EEF_STATE` / `WITH_EEF_ACTION` 及离线EEF转换继续保留。

- [参数设置：关节、深度、帧率、同步、存储与模式专用参数](README.md#parameters)
- [Action 来源：常规数采与 DAgger 的区别](README.md#action-modes)
- [环境准备](README.md#setup)
- [S3适配与历史验收记录](S3_TESTING_zh.md)（与S2共用main）
- [四种模式使用的遥操作版本](README.md#modes)
- [键盘数采](README.md#keyboard) / [原厂单人VR](README.md#vr) / [子任务数采](README.md#subtask)
- [V4增量遥操作与DAgger](README.md#dagger)
- [续录、输出与故障处理](README.md#data)
- [WITH_EEF_STATE / WITH_EEF_ACTION 与 Joint/EEF 自由组合转换](README.md#conversion)
- [300测试与验收](ROBOT300_TESTING_zh.md)
- [实现与数据格式](INSTRUCTION_zh.md)
