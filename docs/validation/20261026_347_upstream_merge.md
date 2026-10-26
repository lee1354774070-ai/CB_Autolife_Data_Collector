# 347 修复与主线合并验证

日期：2026-10-26。本文记录代码合并与隔离软件验证，不代表真实遥操作验收。

## 合并来源

- GitHub main：`03084f6b9dd351bb38e6e104ed26bae29b4f169d`。
- 347 部署基线：`0bbaa841e19eaebf9ea9a6f22865aadd68e0def7`，加机器人工作区修改。
- 保留机器人上的 DDS source_timestamp 回退、状态/EEF 回调元数据传递、
  所有模式低 CPU 临时图像写入和原生线程池限制，以及部署文档和验证脚本。
- 保留主线的异步同步日志、采集/保存耗时诊断、DAgger 生命周期与结果标注更新。
- 两边新增的 `tests/test_recorder_timing.py` 不互相覆盖：主线诊断测试保留原名，
  347 回调测试移至 `tests/test_recorder_telemetry.py`。
- 未引入本机集合仓库尚未提交的 GPU 编码、首帧保留功能。

## 验证

在 347 独立目录 `/home/ubuntu/collector-merge-validation-eZLqlj/collector` 中验证：

- Python 全量 unittest：369 项，28.165 秒，全部通过，无跳过。
- 普通 VR 与子任务 ROS 通信冒烟：均通过，`hardware_calls=0`。
- 通信测试显式设置 `ROS_DOMAIN_ID=211`，使用私有测试话题、模拟录制器/复位服务，
  不连接真实控制器，不向真实 TTS 发送消息。
- 本机 Node `tests/test_feedback.js`：2 项通过。
- `bash -n start_lerobot_official_collect.sh`、Git whitespace 检查通过。

完整日志位于机器人上述验证目录的 `unittest.log`、`vr_smoke.log`、`subtask_smoke.log`。
更新前源码和 Git 历史备份为该目录的 `premerge-source-and-git.tar.gz`。

本次未播放真实测试语音、未启动真实采集、未发布运动、未重启 ARM/视觉服务，
不修改已保存数据。JPEG 警告和真实 VR/手臂跟随卡顿仍需要另外验证；
本次通过不能证明这两个问题已经消失。
