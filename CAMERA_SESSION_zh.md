# 采集会话四路相机模式

300 已启用。统一普通、单人、子任务、DAgger 入口和旧 standalone 采集入口共用同一相机锁。

采集程序启动后只初始化左右手相机和头部 RGBD 模块，即 hand_left、hand_right、rgbd_head_color、rgbd_head_depth 四路。其他非相机模块（麦克风、扬声器）保留。数据集是否保存深度仍由 WITH_DEPTH 控制；本功能不修改数据字段、分辨率、帧率或时间戳阈值。

## 其他机器人启用

代码共用，不按机器人编号限制。只拉取代码不会修改其他机器的相机：需先确认原厂视觉服务是用户级 vision-service.service，且其当前实际加载配置含 mod_camera_hand_left、mod_camera_hand_right、mod_camera_rgbd_head 三个模块。不要把300的配置文件复制到其他型号。

确认后，在该机器人 ~/.config/autolife_collector/camera_session.json 写入：

    {
      "vision_config": "/实际路径/autolife_robot_vision/configs/当前机器人版本.json",
      "service": "vision-service.service"
    }

路径必须指向该机实际加载的文件。首次启用应在机器人停稳、无人使用遥操或导航时测试；切换会短暂中断视觉服务。四种模式使用同一份设置，无需分别配置。--check、--reset-only、DAgger attach不主动切换相机。

## 启动和退出

1. 确认没有其他人在使用机器人遥操或导航。
2. 用原来的命令或“云蝶DAgger启动器”启动采集服务。
3. 等待“相机模式：四路持续出帧已确认，可以开始采集”，再连接头显、开始一条数据。
4. 连续录制多条期间保持四路模式；B 保存和 Y 丢弃不会反复重启相机。
5. 完全退出采集服务后，等待“采集前配置已恢复”。关闭录制期间不可用的左右头部/后视画面会恢复到原配置。只关闭启动器窗口而保留后台会话，不算退出采集服务。

切换通过原厂 ENABLED_MODULES 配置和用户级 vision-service.service 重启完成。启动前与结束后画面会短暂断开；不会停止、重启或清错 ARM 服务。实际录制及数据保存尚未完成时，不重启视觉服务。

## 测试

先用测试名称录制 10～20 秒，保留 WITH_DEPTH=1。检查头部画面、双手相机、遥操按键及保存回执；DAgger 再检查 A 开始、Grip 接管、B 保存、X 复位、Y 丢弃及失败保存。退出整个采集服务后确认相机恢复。

另一终端运行以下只读命令，分别查看启动前、服务运行中、退出后的状态：

```bash
python3 /home/ubuntu/CB_Autolife_Data_Collector/camera_session.py status
```

运行时 enabled_camera_modules 应只有 mod_camera_hand_left、mod_camera_hand_right、mod_camera_rgbd_head（三个模块产生四路）。四个必需 stream 的 fresh 均应为 true。退出后模块列表应恢复原值，recovery_pending 应为 false。运行时 recovery_pending=true 代表保存了退出恢复记录，不代表录制故障。

不要仅根据 SHM 文件存在判断相机正在工作；被停用流的旧 SHM 文件可能仍在，时间戳不再前进。

## 故障恢复与停用

正常退出、Ctrl+C、SIGTERM 和启动失败均执行恢复。SIGKILL 或机器断电无法运行退出处理；下次启动会先恢复上次配置。也可以在所有采集/录制进程完全结束后手工恢复：

```bash
python3 /home/ubuntu/CB_Autolife_Data_Collector/camera_session.py recover
```

恢复失败会保留原配置记录并明确报错。若配置已被同事修改，不覆盖其修改；应人工对照 ~/.local/state/autolife_collector/cameras/pending.json。此文件可能含原厂配置中的凭据，勿公开上传。

要停用本功能，先正常退出并确认 recovery_pending=false，再将 ~/.config/autolife_collector/camera_session.json 改名为 camera_session.json.disabled。未配置此文件的其他机器人保持原有启动行为。

## 验证边界

已验证配置按字节恢复、三路闲置头部设备释放及重新打开、四路持续出帧、ARM 用户服务 PID 不变；自动测试覆盖启动失败回滚、重复会话拒绝、其他进程录制时拒绝恢复、外部配置修改保护、遗留状态恢复和未配置机器不变。

这不是 CPU 过载已解决的证明。原厂未使用的三路头部图像可能处于惰性暂停，即使设备已打开，也未必持续采图。真机录制期间的整机 CPU、控制延迟和数据完整性仍需现场验收。

## 300 实测记录（2026-10-27）

稳定15秒后采样10秒，视觉服务进程CPU：原配置102.4%、四路103.1%、恢复后100.1%。100%约为一个逻辑核；这次空闲采样没有显示明显降幅。

已实际验证三路闲置设备/dev/video6、/dev/video10、/dev/video9在四路模式下释放，恢复后重新打开；ARM用户服务PID保持653690。统一keyboard入口启动至4/4相机就绪，Q退出后恢复原配置；未发送开始录制命令，测试数据集0帧、0条，不用于训练。测试日志：/home/ubuntu/collector_validation/camera_session_20261027/startup_exit.log。

启动器会显示相机准备/恢复进度，恢复失败会标记异常，不能误报正常退出。尚未进行头显操作与实际运动录制验收。
