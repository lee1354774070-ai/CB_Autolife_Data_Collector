# Collector 实现说明

本文档说明内部实现约束；日常命令请看 [README_zh.md](README_zh.md)。

## 架构

| 模块 | 职责 |
| --- | --- |
| `start_lerobot_official_collect.sh` | 解析 session 配置、管理进程生命周期、把按键转换为 IPC 命令。 |
| `collector_modes.py` | 在启动进程前解析互斥模式，检查旧配置冲突。 |
| `dagger/` | 新版 V4/HG 适配、独占启动、Thor 会话校验、VR 提示、异步诊断日志。 |
| `dagger_labels.py` | 检查动作的原始时间和控制权，生成逐帧纠错训练 mask。 |
| `record_lerobot_official.py` | 缓存 ROS/SHM 信号、同步 dataset 行并管理 `LeRobotDataset`。 |
| `robot_schema.py` | 维护关节名称、canonical policy 顺序、物理 q23 转换和命令解析。 |
| `camera_config.py` | 维护相机名称、SHM 路径、ROS topic 和共享默认值。 |
| `shm_camera.py` | 读取稳定的 legacy/SHM2 metadata 与图像，解码 JPEG、原始彩色和 uint16 depth。 |
| `time_sync.py` | 提供时间戳归一化、FIFO 选择和插值函数。 |
| `collector_control.py` | 实现 launcher 与 recorder 的 IPC、状态报告和数据集摘要。 |
| `vr_collector_control.py` | 可选的 VR 组合键、非阻塞倒计时、recorder 结果确认和 TTS 语音。 |
| `subtask_annotations.py` | 校验有序子任务、记录片段边界、保存前生成逐帧标签与标注 JSON。 |

默认 direct-SHM 运行结构：

```text
launcher
├── recorder          LeRobot + ROS2 Python
└── hand producer     需要手部相机时使用 robot_env 启动
```

`IMAGE_SOURCE=ros` 会增加 `shm_camera_topic_bridge.py`。direct-SHM 是默认方式，
因为它省去了图像序列化和 DDS 队列。

## 数据契约

policy schema 固定以 16 维开始：

```text
左臂 (7)、右臂 (7)、左夹爪 (1)、右夹爪 (1)
```

`WITH_HEAD=1` 追加颈部 roll/pitch/yaw；`WITH_UPPER_WAIST=1` 只追加 waist
pitch/yaw；`WITH_WAIST=1` 追加 ankle、knee、waist pitch、waist yaw。两个腰部
模式互斥。关节 state 和关节 action 必须使用完全相同的顺序，因为 LeRobot
relative action 按数组索引计算。

ROS 控制器的物理 q23 顺序不同：

```text
腰部 (4)、左臂 (7)、右臂 (7)、夹爪 (2)、头部 (3)
```

只有 `robot_schema.py` 可以在两种顺序间转换。policy 没有启用头部或腰部时，
部署端会保留这些物理关节的最新实测值。

## 图像与同步

每路相机有两个 SHM 文件：

```text
/dev/shm/camera_metadata_struct_<name>
/dev/shm/camera_image_buffer_<name>
```

双手的物理 `<name>` 固定为 `hand_left_jpeg` / `hand_right_jpeg`，只读取 JPEG，
在本进程解码成 BGR。不再探测旧的双手 BGR SHM，JPEG 缺失或停止更新时也不会回退。
数据集键和 ROS topic 中的逻辑名称仍为 `hand_left` / `hand_right`。

旧版 metadata 格式为 `"<qiiiii"`：时间戳、宽、高、通道数、像素格式和字节数；
SHM2 metadata 还指定已发布的缓冲槽位，双缓冲/ring buffer 支持不变。
只有复制图像前后 metadata 一致时才接受当前帧。头部原始彩色数据读取不变，
各路彩色图像写入 LeRobot 前统一转换为 RGB HWC；depth 仍为小端 uint16 毫米值。
可选的手部相机 producer 直接转发 V4L2 原始 JPEG，不逐帧解码或重新编码；
不支持原始 JPEG 的后端会报错退出。它不能与占用同一相机设备的视觉服务同时运行。

recorder 批量读取 metadata，只复制新图像，并为每路相机维护有界 FIFO。每行数据：

1. 取参考相机中已经等待匹配窗口的最早帧；
2. 在 `MAX_SYNC_DELTA_SEC` 内为其他相机选择最近帧；
3. 在参考时间戳处插值 state；
4. 选择不晚于该时间戳的因果 action；
5. 将完整行加入 LeRobot 的 pending episode。

缺图像、state 插值区间或因果 action，或者出现过期数据时，当前 episode 会作废。
录制期间 FIFO 溢出同样会作废 episode。“参考帧仍在等待其他帧”只会增加
`waiting ticks`，不代表源图像跳帧，也不会单独作废 episode。

## Session 生命周期

launcher 先启动 recorder，再启动相机 producer，保证图像锚点到来时已有 state 历史。
`Enter` 清空参考相机 FIFO，其他相机最多保留一帧供起录匹配，仍检查时效和同步误差；
新 episode 只用新的参考帧生成数据行。`S` 保存；`D` 丢弃；`Q` 只保存有效
pending episode 后 finalize 数据集。

IPC 位于 task 目录下：命令 FIFO、原子 status JSON、ready JSON、episode-event JSON
和 PID 文件。这些均为临时文件；数据集和日志会保留。

续采要求 FPS、相机 feature、depth、关节 schema 和 action mode 完全一致。LeRobot
生成多个视频文件属于正常行为，metadata 会索引它们，不能手动拼接。

## VR 控制协议

`COLLECTOR_MODE=vr` 或 `subtask` 自动启用下述 VR 控制；未指定模式时兼容 `VR_CONTROL=1`。
这段协议仍属于原有数采，**不是新版增量遥操作 DAgger 的实现**。

`VR_CONTROL=1` 在 recorder 和相机初始化后才启动 VR 辅助进程，使用 `ROBOT_PY`，
ROS domain 和机器人编号与录制器一致。输入为 `/control_topic_<domain>_<robot>` 的
`std_msgs/String` JSON：`l/r.b[1].p` 为握持键，左侧 4/5 为 X/Y，右侧 4/5 为 A/B。
向 `/topic_tts_<domain>_<robot>` 发布播报请求，向 `/collector/feedback` 发布结构化振动事件。
B 通过 `vr_reset.py` 调用 V4 受保护复位接口，不直接发布关节目标。
不再猜测 ASYNC/HOME/SYNC 状态，也不改变原厂遥操作模式。

组合键要求先无面板按钮、按下单个按钮、保持双握持键并松开按钮。
消息格式错误、多键混按、握持键松开或消息间隔过大均取消当前手势。
开始录制使用可配置倒计时，并要求最近 0.75 秒内收到有效 VR 消息；取消倒计时不会发送 start。

工作线程等待 recorder 确认，ROS 回调继续处理按键释放。等待期间忽略其他操作，
Y 只开始，不再双击退出；退出使用终端。键盘与 VR 共用 `.official_control.lock`，从发送命令到
收到对应 request_id 期间持有 flock，避免单个 status 文件被并发覆盖。忙碌时直接拒绝新命令。
空锁文件保留以避免 inode 竞争，它的存在不代表采集仍在运行。start 成功和拒绝均有明确回执。

后台缓存以 10 Hz 检查回执文件，仅文件替换或内容变化时解析 JSON；ROS 回调不再读取 NAS 文件。
文件缺失、损坏或读取阻塞不会产生成功回执。网页反馈使用单个 SSE 长连接，静态扩展资源在启动时缓存。
每路相机只有一个 SHM 源；时间戳未变化时不读图像、不解码、不写入 FIFO。
不再切换双手的 BGR/JPEG 数据源；FIFO、同步误差和整条作废规则不变。
数据集创建后仅轮询已纳入其 schema 的相机，忽略后来上线的未启用相机，避免无用解码和未消费队列溢出。
启动器在未显式设置时将 OMP/MKL/OpenBLAS 线程设为 1，recorder 的 OpenCV 使用单线程；
LeRobot 图像写入线程和视频编码线程仍按原参数配置，不以降低帧率换取较低占用。

VR 模式由辅助进程读取原子的 episode-event 文件，终端不再抢先删除它。
事件和回执去重，不依赖解析日志文字。对无效数据执行 save 会收到 discard 回执，只播报丢弃。
超时代表结果未知，不能推断命令没执行，也不能自动重发；应检查 recorder 后再重启工具。

丢弃时先等待 LeRobot 图像写入完成，再补充清理当前未保存 episode 的视频临时图片，
兼容 LeRobot 0.6.0 只清理 image 特征目录的情况。不会删除已保存视频或其他 episode。
如果此前保存结果不明确，仍禁止丢弃，不会通过清理删除可能尚未完成保存的数据。

退出时先停本工具自己的 VR 辅助进程，再 finalize recorder；VR 辅助进程异常退出时，
主启动器进入正常清理流程。不重启 SDK 服务、不调整系统音量。
同事额外开发的 HG-DAgger 预录功能不属于普通数采，本次未将其合入，也不依赖它。

## DAgger 接入

### 宿主与附加终端

`DAGGER_BACKEND=owned` 启动完整控制栈；`attach` 只订阅新鲜状态并调用兼容宿主的
操作服务，不创建模型客户端、录制器或电机发布器。协议绑定宿主实例、机器人、DDS 域和
数据 FIFO。命令带请求 ID 和 30 秒有效期；重复请求返回缓存结果，不确定结果不重放。
这是本机协调协议，不是身份认证。附加终端按 Q 不停止宿主及当前任务。
GUI 副本工具保留原文件，把启动入口替换为我们的常驻 `--serve` 宿主；未审查的源码版本
会被拒绝。未改造的旧栈需要首次停止后再启动兼容副本，不能无缝热替换。


`COLLECTOR_MODE=dagger` 使用**新版增量 V4 控制栈**，不是原厂 VR 数采辅助程序。
依赖机器人上的 `/home/ubuntu/ros2_ws/src/openarmx_teleop_vr_306_v4` 和
`/home/ubuntu/ros2_ws/src/autolife_hg_dagger_MZJ_300`。关键源码和配置按 SHA-256 核对；
不同版本必须重新审查测试，没有跳过检查的开关。存在其他控制器、未知命令发布者，
或在启动观察窗口内收到任何控制命令时，拒绝启动；仅允许已审查的原厂节点保留空闲发布端点。
原厂的 `target_robot_eef_pose`、`target_robot_height_z` 是输出报告，不是控制输入；
预检检查实际的 `move_*`、关节和夹爪命令，并要求保持姿态的独占 SYNC 服务及电机订阅者可用。
V4 运行中的命令仲裁、心跳和安全限制继续生效。
基于 Conda 的 IK/Web 虚拟环境单独使用其解释器的动态库路径，不污染系统 ROS 进程。
使能服务的回执不等于硬件就绪。启动工作线程最多等待 20 秒，要求新的 `ARMED` 状态、
`hardware_enabled=true`、`hardware_ready=true` 且使能不再 pending，之后才放行模型。
取消、故障或超时都会停止本次启动，不发布模型目标；等待不阻塞 ROS 按键回调。
不能另外运行普通 Thor robot client，否则会形成双控制器。
`runtime_copy.py` 在 `.runtime/` 创建按内容标识的 V4 副本，补丁只应用于副本；
复用时再次核对文件哈希。同事的 V4/HG 原目录不改动。保留原版 mapper、IK 算法、
限位和碰撞检查，不通过猜测参数或放宽保护来降低延迟。

当前范围：300 机器人、ROS domain 0、21 维手臂+夹爪+头+上腰、三路 RGB，
GR00T 的 `policy_only_baseline` / `policy_only_frame`。depth 可以录制，但不作为
这个 RGB 模型的输入。尚未接入 PI0.5 或 EDVA/SOMA 的因果 Outcome 历史。
`thor_bridge.py` 复用**我们自己的** `GrootRemoteClient`、持久 HTTP 连接、模型 contract、
q23 映射和 SHM 读取；不导入或启动同事的 Thor 客户端。单独部署 Collector 时，
`DAGGER_TOOLS_ROOT` 必须指向完整 VLA 工具目录。

我们的 Thor server 新增 `/controller_ack`，仅供 baseline/frame：核对已提交给控制器的
目标前缀和摘要，再关闭该 chunk；这**不是**硬件已发布或已到位的回执，也不能推进
verifier 历史。Thor server 需一并更新，启动预检会检查能力声明。原 `/ack` 和训练逻辑不变。
HTTP 回包丢失不自动重试；控制器提交回执不明确时锁定失败，不伪造已执行步数。
慢 HTTP 尚未返回时，人工接管仍可独立撤销本地模型输出。

Y 先等待 recorder 的关联 start 回执，再放行模型控制。硬件使能需要时间，recorder
在第一帧之前最多等待 30 秒，以新鲜动作和来源标签建立起点，日志记录 barrier ready。
这是录制起点等待，不是已经录入的 episode 中间跳帧；后续仍严格使用原 FIFO、插值、
新鲜度检查和“同步失败则整条作废”。新按下任意 GL/GR 即发布新的控制权代次并切换到人工，
不等待模型返回、保持确认或松开重握。唯一的 V4 控制出口在锁内清除旧模型目标和在途 IK，
以实测关节作为基准；原有增量 mapper 重新锚定当前手柄/末端姿态，并为人工目标携带控制权代次。
迟到的旧模型/人工目标会被拒绝，不会被重新标记成新目标。故障和复位保护不变。
实际延迟仍包括 DDS 传输、mapper 周期和 IK 求解；持续握持不会重复触发接管。
X 仅丢弃，不触发接管。人工接管后不会自动交还模型。

模型消息必须同时匹配会话 ID 和控制权 epoch，旧一轮迟到的 HTTP 结果在 bridge 和
supervisor 两层被拒绝。只有 V4 controller 发布硬件指令，其原有配置/关节限制、
轨迹限制、watchdog 和碰撞检查继续生效。supervisor 不再额外按 8/10 度的目标与实测
差值中止 episode，因为跟随误差不是相邻 action 的跳变。仍检查 action/state 为
21 维有限数值、控制权有效、夹爪目标在 [10,330] 度内；这不代表取消 V4 的指令超前量
或机械关节限位。软件停止不能替代物理急停。退出时，V4 副本先在有效的
ROS 上下文中发送其拥有的保持/释放指令，再关闭 ROS；未使能的控制器不会因退出发布新目标。

A 先撤销输出，保存整条已接受轨迹，然后停用会话，不自动复位。
X 丢弃且不复位。A/X 按下即触发一次，无长按功能。
B 撤销输出，丢弃
未保存条目并得到回执后，再请求官方全身复位，包含夹爪。若保存超时或结果不明，
保留原 request ID；再按 A/B 只核对原结果，不重复 save，不丢弃部分提交的数据，也不复位。
Q/Ctrl+C 仅关闭本启动器的子进程并丢弃未确认条目。recorder 作废事件会在下一次状态
更新撤销控制；进程退出则关闭整套子进程。默认 `DAGGER_PUBLISH=0` 不授权硬件动作。

以下自定义字段均为 int64 `[1]`，不占 state/action 维度：

| 字段 | 含义 |
| --- | --- |
| `dagger.control_source` | 0=模型，1=人工有效控制，2=保持/交接。 |
| `dagger.is_intervention` | 本条是否已经发生接管，不可直接当 loss mask。 |
| `dagger.intervention_id` | episode 内人工控制段编号。 |
| `dagger.authority_epoch` | 控制权代次，用于审计切换。 |
| `dagger.train_mask` | 仅在人工有效控制，且手臂和夹爪的原始动作都晚于本次控制权边界、epoch 匹配时为 1。 |
| `dagger.anchor_timestamp_ns` | 图像对齐帧的机器人时基时间戳。 |
| `dagger.arm_command_timestamp_ns`、`dagger.gripper_command_timestamp_ns` | 原始控制指令时间；刷新夹爪代理时间不能把旧动作变成人工新标签。 |

必须使用独立数据根目录，不可与普通数采、旧 HG 字段混写。仅模型执行但按 A 保存的
条目，其 mask 全为 0。保存不是任务成功判据。训练需要显式地对 **action chunk 每个时间步**
应用 mask；stock LeRobot 训练不会自动理解这些字段。本次不包含自动微调和 DAgger 迭代训练。

FIFO/编码等待在线程中处理，X 不等待这些操作。诊断 trace 改为容量 2048 的有界后台队列，
丢弃数量/写盘错误写入 trial manifest；逐帧训练标签不依赖此诊断队列。WebXR 振动和浏览器
70 ms 短提示音根据状态/回执变化触发，受头显能力及浏览器音频权限影响，不保证硬实时。
不改变机器人音量或原厂 TTS 设置。

录制状态文件由 20 Hz 后台线程读取，控制回调只读内存；读盘线程超过 1 秒没有完成
更新则撤销输出。trace 的打开、写入、关闭也统一在有界后台队列执行。V4 副本把 DDS
图发现查询移到 5 Hz 后台线程，控制锁内只查缓存；缓存超过 600 ms 时在下一次保护检查拒绝输出。
关节目标在运动学验证之前及提交之前都核对控制权，收到控制权切换即拒绝旧模型目标；
旧的手柄目标和释放指令也会在修改控制器状态之前被拒绝。

V4 副本在成功发布硬件指令后，通过独立 `/collector_dagger/controller_output` 发出
带原始时间、目标来源 epoch 的回执，不修改厂商指令 JSON。训练动作以这些回执为准，
不依赖无来源标签的 DDS 回声。沿用的旧目标不能因控制模式变化被标成人工示教。
这些回执只证明指令发布，不证明实际到位或任务成功；缺失/过期仍按同步规则作废。

模型推理仍为串行 chunk，chunk 交界可能等待 Thor 返回。本次消除的是控制关键路径
中不必要的 I/O 等待，并不消除 GPU 推理耗时，也未实现 RTC 重叠推理，不能承诺零抖动。

验证范围：隔离 ROS domain 下的真实 supervisor/FIFO 软件链路，硬件使用替身；
LeRobot 0.6.0 合成数据的 Parquet/视频保存、续采和读取。以上均不等同于实机接管延迟、
碰撞行为、实际音频/振动或闭环成功的验证。机器人上同事正在运行的程序只读检查，
尚未替换或重启，仍需有人在场完成实机验收。

## 子任务标注与实时性

`SUBTASKS_JSON` 非空时，整条 episode 的 `task` / `task_index` 仍指向总任务。
增加自定义 `subtask_index` feature，dtype 为 `int64`，shape 为 `[1]`。
该编号是 **episode 内**计划序号，不是全局任务编号，也不占用机器人 state/action 的维度。
不同 episode 可有不同计划，必须通过对应 sidecar 查找文本，不能全库直接合并相同数字的标签。

VR 在按下 A 时记下本机 monotonic 时间，松开时比较长按阈值。
没有定时触发长按，也不会同一次按键同时触发标注和保存。断流、多键、握持键松开、
格式错误均取消手势；一次指令未收到结果前不会排队重复指令，收到回执时清除跨状态的半次按键。
`mark_subtask` 经已有 FIFO 到达 recorder，与 `add_frame` 在同一线程串行处理，
采样 `current_episode_frames` 作为 exclusive end。首段从 0 开始，下段从上段 end 开始。
空片段或已完成的计划拒绝再次标注。最后一段确认后直接调用完整 episode 保存。

中间标注的关键路径只追加一个边界、生成小型回执；不遍历图片、不编码视频、不插入 sleep，
不重置相机 FIFO，也不改变 `is_recording`。回执文件写入交给单线程后台 worker；
等待期间只有后续控制指令暂缓，录制 timer 继续运行。逐帧标签回填和标注 JSON 落盘只在保存时执行。
VR 等待确认使用独立 worker，前 1 秒每 10 ms 检查一次回执，之后回退至 100 ms；
完成通知每 20 ms 检查一次，键盘回执及健康检查仍每 100 ms 一次。

日志中的 `Recorder acknowledgement latency` 是本机发送指令到辅助进程处理回执的时间，
**不是**相机同步误差或按钮到扬声器/手柄振动的延迟。Linux、DDS、NAS、相机调度都可能产生抖动，
本实现不是硬实时系统，也不能保证零延迟。边界以已接受的数据帧为准，30 FPS 本身约有 33.3 ms
的帧粒度，还受同步等待、命令处理影响；未入数据集的 FIFO 帧不计入当前片段。

声音通过原有 TTS topic 发送短文本“下一步”等，不等待播放结束，不逐次朗读长子任务文本。
TTS 服务自身仍可能排队，现场需确认扬声器可听及反馈延迟。未找到厂商手柄振动发送协议，
因此没有猜测 topic/消息或向运动通道发送反馈；拿到接口后才能接入振动。

输出 `annotations/subtasks/episode_<六位编号>.json` 包含：

| 字段 | 含义 |
| --- | --- |
| `task`, `episode_index`, `frames`, `fps` | 总任务与本条数据标识。 |
| `subtasks` | 有序文本计划，数组下标对应 `subtask_index`。 |
| `segments` | 已确认片段，含文本、编号、`[start_frame,end_frame)` 及按 FPS 计算的秒数。 |
| `unannotated_ranges` | 尚未确认的尾段；Parquet 中对应 `subtask_index=-1`。 |
| `annotation_complete`, `needs_review` | 操作员是否确认所有片段；**不代表任务物理执行成功**。 |
| `save_reason`, `boundary_basis` | 完整标注自动保存/手动提前保存/退出保存，以及边界时间依据。 |

例如 300 帧的 episode，在第 90、180、300 帧确认：标签依次为 `[0,90)=0`、
`[90,180)=1`、`[180,300)=2`。只确认到 180 就保存，则 `[180,300)=-1`。
即使尚无尾段，但计划未全部确认，也标记需要复核。漏按后再按可能把多个实际动作归到同一个
已确认片段，工具不能识别这种语义错误，清洗时应检查全部边界。

保存前先写 `.pending.json` 恢复标记，再调用 LeRobot 保存，成功后重命名正式 JSON。
保存失败停止 session，不自动重试已可能被修改的 episode buffer。重启遇到 pending 标记会拒绝续采；
须核对 Parquet、视频、meta 和标注的一致性，不能直接删标记。这不是跨文件事务或掉电恢复保证。
普通丢弃不发布正式 sidecar；标注未完成允许保存，但同步无效仍按原规则丢弃。

开启或关闭标注会改变 dataset features，因此要求新的根目录；续采已有标注数据时可调整文本计划，
每条计划单独写入 sidecar。标准 LeRobot 数据读取可保留附加数值列，但不会自动把它当作子任务语言输入。
只训练总任务时保持原流程；按子任务训练需另外构造文本/片段采样，且不要让 action chunk 跨越不期望的边界。

## 扩展规则

- 新增相机：修改 `camera_config.py`，并补充配置与 SHM 测试。
- 新增关节：修改 `robot_schema.py`，不要在 recorder 或 deployer 中重复维护顺序。
- 新增信号：建立带时间戳的 buffer，并定义明确的因果或插值规则。
- 不要把 FIFO 匹配替换为 latest-frame 复用，这会改变数据契约。

## 验证

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python -m pytest -p no:cacheprovider -q tests
bash -n start_lerobot_official_collect.sh
# 可选：ROS 链路测试，独立话题和模拟 recorder，不运动、不实际播音。
python tests/vr_ros_smoke.py
# 子任务手势测试，仍使用独立话题和模拟 recorder。
python tests/vr_ros_smoke.py --subtasks
```

核心回归测试仅供开发验证，启动器不会加载，不占采集 CPU。历史报告和一次性数据集/远程实验脚本
已移除。SSE 测试需要网页环境中的 aiohttp，其余测试不依赖它。软件回归不代表实机验收或最低 CPU 保证。

机器人现场还应检查 ROS 发现、SHM 文件、相机源 FPS、编码器、CPU、磁盘吞吐和
`sync_log.jsonl`，确认生产采集前没有异常 drop 或 episode invalidation。
