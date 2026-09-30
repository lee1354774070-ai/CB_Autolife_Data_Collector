# VR Controls And Feedback / VR 按键与反馈

## Controls / 按键

| Button | VR | Subtask | DAgger |
| --- | --- | --- | --- |
| Y | Start / 开始 | Start / 开始 | Inference + recording / 推理并录制 |
| A | Save / 保存 | Short: mark; long: save early / 短按标注，长按提前保存 | Save / 保存 |
| X | Discard / 丢弃 | Discard / 丢弃 | Discard / 丢弃 |
| B | Discard then reset / 丢弃后复位 | Discard then reset / 丢弃后复位 | Discard then reset / 丢弃后复位 |

VR/subtask requires GL+GR held and a face-button release. DAgger keeps its
Grip takeover logic; release both Grips before Y. No double-Y exit.
普通模式保持 GL+GR 并松开面板键触发；DAgger 用 Grip 接管，开始前松开 Grip。
Y 不再兼任退出。终端退出方式保留。

## Reset Contract / 复位约束

`vr_reset.py` targets the inspected V4 controller, not arbitrary factory SDKs.
It waits for a recorder discard receipt, obtains the recorder motion lock,
waits for live VR input with both Grips released, disables V4 output, then calls
`<VR_RESET_PREFIX>/full_body_reset`. A service ACK is NOT completion: the helper
must observe fresh `hardware_enable_pending` followed by `hardware_ready`.
It disables the controller again on completion. Re-enable in the VR page.

复位只对接已检查的 V4 接口。必须先丢弃确认、取得运动锁、松开双 Grip，再关闭遥操输出并复位。
等待控制器实际完成后才播报“复位完成”；复位结束后保持停用，需重新使能遥操作。
丢弃不确定时不复位；复位不确定时锁定且不自动重试。不会删除已经保存的 episode。
忙碌时不排队执行 B，请等待当前操作结束。

**Disable the teleoperation mapper's old reset gesture first.** On the inspected
V4 mapper this is `quick_reset_enabled=false`. Do not run an additional UDP
input node with its own B/home reset handler. Otherwise it could reset before
the recorder acknowledges discard. Keep one controller and one reset owner.

**必须关闭遥操作端原有复位快捷键**：检查过的 V4 mapper 参数为 `quick_reset_enabled=false`。
不能同时运行带 B/home 复位的额外 UDP 输入节点。不修改同事原件，在副本配置中设置该参数。
原厂 VR 服务若没有同等受保护接口，则 B 报错，不猜测或直接发布原厂 reset topic。

## Feedback / 反馈

ROS events use `/collector/feedback`; speech uses the robot TTS topic. The web
extension reuses the colleague's `pulseVrControllers()` WebXR actuator path.
One SSE connection pushes events independently of pose transport, with a bounded
32-event queue per client (maximum four clients). Events older than two seconds
are dropped; reconnecting never replays old notices. Idle connections send only
a keepalive every 15 seconds, not ten HTTP requests per second.
反馈改为 SSE 推送，每客户端最多缓存 32 条，最多四个客户端；过期事件和重连前的事件不重放。
空闲时仅每 15 秒保活一次，不再每秒发送十次 HTTP 请求。

| Event / 事件 | Pulse duration (ms), 90 ms gaps / 振动时长，间隔 90 ms |
| --- | --- |
| Countdown / 倒计时 | 35 |
| Started / 开始 | 80, 80 |
| Mark confirmed / 标注确认 | 50, 50, 50 |
| Saving / 保存中 | 45, 140 |
| Saved / 保存成功 | 220 |
| Discarding / 丢弃中 | 140, 45 |
| Discarded / 已丢弃 | 160, 160 |
| Resetting / 复位中 | 90, 90, 250 |
| Reset confirmed / 复位完成 | 250, 90 |
| Takeover request / 接管请求 | 300, 70 |
| Cancelled / 取消 | 70, 180, 70 |
| Error / 异常 | 240, 240, 240 |

Success cues come from receipts, not button presses. Speech is nonblocking TTS;
headset haptics require an active immersive WebXR session and compatible hardware.
成功提示基于回执，不基于按钮按下。语音异步播放；振动必须进入 VR 会话且手柄支持。

## Web Integration / 网页接入

Our DAgger web wrapper includes the extension automatically after restart.
For ordinary V4 teleoperation, use the same wrapper with `COLLECTOR_WEB_MODE=vr`
or `subtask`, WITHOUT the DAgger supervisor/Thor process. Keep the existing single
controller/mapper; stop only its old web component before replacing it. The
wrapper reads the reviewed dependency copy and never modifies colleague files.

DAgger 重启我们的启动器即可加载反馈。普通 V4 模式只替换网页组件，不启动 DAgger
supervisor 或 Thor，不额外启动第二个控制器。旧的原厂网页不会自动具备新反馈。

Example on robot300 with the existing reviewed dependency copy (source ROS first):

```bash
export DAGGER_DEPENDENCY_ROOT=/home/ubuntu/collector_validation/20260928_modes_dryrun_01/dependencies
export COLLECTOR_WEB_MODE=vr  # or subtask
/home/ubuntu/ros2_ws/venvs/hg_dagger_web/bin/python \
  /home/ubuntu/collector_validation/20260928_modes_dryrun_01/tools/lerobot_data_collector/dagger/runtime.py web \
  --ros-args -p https_port:=8447
```

Collector uses the matching V4 input topic:

```bash
VR_INPUT_TOPIC=/openarmx_teleop_vr_306_v4/vr_input \
COLLECTOR_MODE=vr TASK_TEXT="pick up the water bottle" \
bash start_lerobot_official_collect.sh pick_up_water_bottle
```

`VR_RESET_PREFIX` defaults to `/openarmx_teleop_vr_306_v4`. Match ROS domain and
robot ID to the running controller. Do not copy robot300 paths blindly to other robots.
使用前确保副本的原复位快捷键已关闭、ROS domain/robot ID 一致。不要直接将 300 的路径用于其他机器人。
