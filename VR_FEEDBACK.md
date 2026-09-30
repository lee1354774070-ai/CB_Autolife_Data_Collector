# VR 按键与反馈 / Controls and feedback

| 按键 | 单人遥操 / VR | 分段采集 / Subtask | DAgger |
| --- | --- | --- | --- |
| A | 倒计时开始 / Start | 倒计时开始 / Start | 开始推理和录制，无倒计时 |
| B | 保存 / Save | 短按标记，最后片段保存；长按提前保存 | 保存整条 / Save |
| X | 仅全身复位 / Reset only | 仅全身复位 | 仅全身复位并打开夹爪 |
| Y | 仅丢弃 / Discard only | 仅丢弃 | 仅丢弃 |

B/Y 不复位。X 不保存也不丢弃；有录制中、待保存或结果未确认的数据时拒绝 X。
所有模式都需显式 A 开始下一条。Y 不退出；退出使用启动器或终端 Q。

单人/分段：保持 GL+GR 后点按并松开面键。分段 B 的长按阈值沿用配置名
`VR_A_LONG_PRESS_SEC`，名称仅为兼容旧配置。
DAgger 面键不依赖握持键；任意 GL/GR 接管后继续握持增量遥操，松开保持，再握继续。
DAgger 浏览器/启动器 Shift+A/B/X/Y 与面键同义。
DAgger 终端保留 C 开始、A 保存、X/D 丢弃、R 复位、Q 退出；attach 模式 Q 仅断开终端。

## 复位约束 / Reset

X 使用已检查的 V4 全身复位服务。单人数采复位先取得录制运动锁并等待双 Grip 松开；
DAgger 启动器复位不依赖头显在线。服务接受不等于完成，必须等新鲜反馈确认复位和夹爪张开。
失败或超时显示错误，不自动重试、不清电机故障、不重启 ARM 服务。

关闭遥操 mapper 的旧复位快捷键：`quick_reset_enabled=false`。
只有一套控制器和一个复位入口；不存在直接发布原厂 reset topic 的备用路径。

## 反馈 / Feedback

**仅人工接管触发手柄振动**：300 ms、70 ms，两次之间间隔 90 ms。
开始、倒计时、保存、丢弃、复位、错误均显示文字，不振动。
成功文字必须来自 recorder/控制器的确认回执，按下按钮不等于执行成功。
头显上方显示固定按键说明，下方半透明框显示当前状态、保存结果及错误。
实际振动依赖活动中的 WebXR 会话和手柄硬件，软件测试不能代替头显验收。

ROS `/collector/feedback` 经 `/collector_events` SSE 推送；每客户端最多32条、最多4客户端，
过期2秒事件和重连前事件不重放，空闲15秒保活。反馈不阻塞动作传输、ROS 或 IK。
兼容事件仍携带原 pulses_ms 字段；网页只响应 takeover 的振动，其他事件仅文字。

## 网页接入 / Web integration

DAgger 重启集合版启动器并刷新网页即可加载。普通 V4 遥操可仅替换网页组件，设置
`COLLECTOR_WEB_MODE=vr` 或 `subtask`，不额外启动 DAgger/Thor/控制器。
原厂旧网页不会自动获得新说明。V4 输入可设置：

```bash
VR_INPUT_TOPIC=/openarmx_teleop_vr_306_v4/vr_input \
COLLECTOR_MODE=vr TASK_TEXT="pick up the water bottle" \
bash start_lerobot_official_collect.sh pick_up_water_bottle
```

完整300验收见 [ROBOT300_TESTING_zh.md](ROBOT300_TESTING_zh.md)。
