# Robot 300 集合版测试（2026-09-30）

## 版本与测试范围

目录：/home/ubuntu/collector_validation/20260930_github_dryrun/collector
分支：mzj/robot300-thor-compat

已通过：174 项 Python 单元测试；隔离 ROS domain 211 的单人、分段、
DAgger supervisor/FIFO/接管和附加客户端测试。硬件调用均为 0。
此前实测 Thor 8777 baseline 的两轮只读推理、digest 校验、discard/close 通过。
这些结果不代表真机运动、视频质量或整套运行负载已经验收。

本次完整 DAGGER_PUBLISH=0 启动检查已通过：
control=DISARMED、controller=DRY_RUN/dry_run=true/hardware_enabled=false、
policy=idle、控制器硬件发布端点为 0、观测电机指令为 0、网页与新说明可访问。
Thor 容器内 9 项夹爪阶段/摘要/提交回执测试通过。
检查曾被控制中心的 /whole_body_joint_bridge 拦截；经授权观察5秒无指令后，
正常退出了该关节控制模块，保留控制中心主界面，未放宽发布者检查。
测试结束已恢复 dashboard/action_player，最终三项服务均 active；
期间 dashboard 曾因 DDS participant index 无空位暂时失败，最终已恢复。
ARM 服务全程未停止或重启。以上允许进入有人在场的真机验收，不等于真机已验收。

## 先关闭其他控制入口

关闭旧 MZJ 采集及全身关节控制台，确认没有待保存数据或播放任务。
同一时间只能运行一套控制程序。遇到 unknown_publishers/Command conflict 时，
先检查报出的节点归属，不直接 kill 全部 Python/ROS，也不修改白名单。

原厂 dashboard/action_player 若仍占用控制端点，确认没有使用者和播放任务后：
  systemctl --user stop dashboard-backend.service
  sudo systemctl stop void-cog-h5.service

不要清错、停止或重启 arm-control-service。

## 无动作启动检查

在 300 的新终端执行：

  cd /home/ubuntu/collector_validation/20260930_github_dryrun/collector
  source /home/ubuntu/ros2_ws/src/autolife_hg_dagger_MZJ_300/scripts/source_hg_ros_env.sh
  set +u
  DAGGER_PUBLISH=0 bash start_mzj300_dagger.sh mzj_buttons_dryrun --check

检查通过后：

  DAGGER_PUBLISH=0 bash start_mzj300_dagger.sh mzj_buttons_dryrun

头显打开 https://192.168.8.122:8447 ，允许本机证书，刷新页面。
确认说明为 A 开始、B 保存、X 仅复位、Y 仅丢弃。
进入/重连 VR 应保持待命，不自动开始推理。
本模式不授权机器人动作，不能用它验收抓取、机械复位或真实动作训练数据。
终端按 Q（或 Ctrl+C）退出，然后再启动真机模式。

## 有人在场的真机验收

必须先通过上面的检查，并确认运动空间和物理急停可用。
使用独立测试任务名，不与旧 schema 数据混写：

  DAGGER_PUBLISH=1 TASK_TEXT='Pick the laundry bag.' WITH_DEPTH=1     bash start_mzj300_dagger.sh mzj_buttons_live_01

默认输出 /home/ubuntu/nas/dagger/mzj_buttons_live_01。
nas 是普通目录名，不要求挂载网络存储。
此 baseline 固定 WITH_HEAD=1、WITH_UPPER_WAIST=1、WITH_WAIST=0，对应21维。
夹爪阶段模式只接受经过验证的两个完整任务文本：
- Pick the laundry bag.
- Place the laundry bag in the upper compartment of the delivery robot.
其他 prompt 不应被默默套用夹爪规则。

1. 待命检查：进入头显后机器人不自动运动；松开双握持键，A 开始推理及录制。
2. 接管：任意 GL/GR 接管，模型不得继续夺回控制；分别测试双臂与扳机夹爪。
   松开保持，再握继续增量遥操。
3. X 保护：录制中按 X 应提示先保存/丢弃；不得丢数据，不应开始机械复位。
4. B 保存：等明确成功回执，不以“保存中”或文件夹存在判断成功。
   保存不自动复位、不自动开始下一条。保存不明时不得 X 强行复位。
5. X 全身复位：保存成功后按 X，松开握持键，验证身体复位及夹爪打开。
   等完成，再 A 开始第二条。
6. Y 丢弃：新录一条后 Y，应收到丢弃回执，不机械复位，保存条目数不增加。
7. 再按 X 复位、A 开始，重复至少三轮，检查无旧 chunk、无残留会话错误。
8. 浏览器聚焦后测试 Shift+A/B/X/Y；头显面键功能应一致。
9. 保存一条含模型与人工阶段的数据，核对帧数、视频可解码、时间戳、
   policy/expert/hold 标签及训练 mask，确认无相机同步失效提示。
10. B 保存成功后终端 Q 退出；确认 8447 和本次子进程退出。Y 不是退出键。

单人遥操：保持原来的 GL+GR 组合条件，点按并松开 A/B/X/Y。
分段模式：B 短按标记，B 长按提前保存，最后标记自动保存。
兼容参数 VR_A_LONG_PRESS_SEC 名称保留，实际控制 B 长按阈值。

## 结束后恢复原厂界面（如本次曾暂退）

  systemctl --user start dashboard-backend.service
  sudo systemctl start void-cog-h5.service

再检查 is-active 和日志；start 命令成功不等于界面已健康。
若出现 Failed to find a free participant index，先检查多开的 ROS 程序，
不要通过重启 ARM 解决。

## 回滚

本次改动在独立 Git 分支，未覆盖原 MZJ 源码。需要回滚时先退出集合版，
再用原入口；禁止两套并行。Thor 连续夹爪模式由
MZJ_GRIPPER_PHASE_AWARE=0 bash tools/start_mzj_thor_baseline.sh 显式选择，
该命令会重启自己的 Thor 服务，须在没有活动会话时执行。

## 2026-09-30 深度读取与振动修正

mzj_buttons_live_01 两次已进入录制，随后因 depth 匹配偏差 68.2/73.8 ms
超过30 ms而作废，保存0条。不能将这两条当作可训练数据。

集合版 recorder 现先复制全部 SHM 帧，再做 JPEG 解码；读取竞争时仅重试一次，
两次均验证 metadata 一致性。保留真实源时间戳、30 ms同步限制、深度和逐帧去重。
修正前15秒探测深度读竞争拒绝257次、最大帧间隔924.7 ms。
修正后45秒只读探测深度2645帧、最大间隔34.3 ms；对1345个参考帧做
各图像仅用一次的时间戳配对，各相机均0次超过30 ms，depth最大偏差16.6 ms。
该检查不包含模型运行及实际写盘编码压力，仍需现场录制验收。

176项Python测试通过；node --test tests/test_feedback.js通过。
头显只有takeover事件振动，其余事件继续显示文字，重复事件不重复振动。
重启集合版并刷新8447页面后生效。建议新任务名 mzj_depth_fix_01，
先短录一条，B保存并确认成功，再检查数据；不要仅凭开始提示判断整条有效。
