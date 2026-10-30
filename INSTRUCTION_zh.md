# Collector 实现说明

使用方法、完整参数和四种模式示例见 [README](README.md)；[English](INSTRUCTION.md)。

## 模块边界

| 模块 | 职责 |
|---|---|
| start_lerobot_official_collect.sh / collector_modes.py | 参数检查、模式分派、会话锁、子进程和键盘 |
| recorder_config.py / launcher_help.py / cli_help.py | 配置及全量、单参数帮助，避免重复定义 |
| record_lerobot_official.py | ROS采样、FIFO同步、录制状态机和保存协调 |
| robot_schema.py / eef_schema.py | 关节映射、末端位姿、单位、插值和schema |
| shm_camera.py / jpeg_decoding.py / time_sync.py | SHM读取、GPU/CPU解码和时间匹配 |
| camera_session.py | 明确配置后的四路相机会话切换、复用与恢复 |
| dagger/capture_image_writer.py | 四种模式共用的有界PNG/TIFF写入器 |
| episode_storage.py / background_saves.py | 磁盘预算、异步日志、持久化保存队列、提交与恢复 |
| first_frames.py / extract_first_frames.py | 原始RGB首帧导出、历史数据离线补提 |
| video_encoding.py | 同步/后台保存共用的硬件编码探针、完整读回与局部适配 |
| vr_collector_control.py / vr_feedback.py / subtask_annotations.py | 手柄、反馈、子任务边界与回执 |
| dagger/ / dagger_labels.py | 独立增量控制、权限、Thor接口、纠错mask和结果标签 |
| convert_dataset.py | 独立state/action投影到新数据集 |
| scripts/setup_collector_env.py | 不覆盖已有环境的CPU/GPU安装及合成检查 |

遥操作快照、补丁、SOURCE.json和依赖哈希是运行/来源契约，不是中间文件。
tests保留故障注入和回归；Python缓存、临时日志不入库。历史过程记录可从Git历史查看。

## 启动与控制边界

普通、单人和子任务模式使用原厂遥操作；DAgger使用审核过的增量控制副本。
启动器先验证参数和锁，再等待完整关节状态，初始化JPEG后端，最后等待相机与数据集就绪。
有camera_session.json时，相机会话管理可能修改已确认的视觉配置并重启视觉服务；
未配置时不主动管理视觉服务。具体恢复行为见 [相机会话](CAMERA_SESSION_zh.md)。
安装器本身不重启服务、不启用电机。独立DAgger的启动自检、权限和复位规则见其专用指南。

会话锁、数据集写锁、机器人控制锁及请求锁职责不同，不能互相替代。
ROBOT_ID与ROS_DOMAIN_ID路由话题；ROBOT_TYPE标注型号，不执行S2/S3模型或动作转换。

## 数据契约

基础顺序：左臂7、右臂7、左右夹爪2，再加头部3和上腰2或完整腰腿4。
WITH_UPPER_WAIST与WITH_WAIST互斥；基础16维、头加上腰21维、全量23维。
常规模式默认status_target：身体取状态包目标，夹爪取实测位置。
DAgger强制joint，依据控制器真实发布的身体/夹爪目标及来源回执，不允许state代替action。

WITH_EEF_STATE/ACTION独立追加左右末端各XYZ+XYZW，共14维，不含夹爪。
使用机器人坐标系、米、单位四元数；缺失拒绝，不填零、不用FK猜测。
实测位置线性插值，四元数最短路径SLERP；action使用不晚于图像时刻的目标。
EEF action的DAgger专家mask固定0，不能从joint来源推断其专家身份。
续录校验FPS、相机、关节/EEF/子任务/DAgger字段和深度，维度相同不代表语义兼容。

## 采样与同步

原厂视觉写SHM，Collector只读。左右手使用JPEG流，无直接BGR手部回退。
ShmCameraReader复用只读文件描述符，每次检查尺寸，检测生产者替换后重新打开；支持旧元数据及SHM2。
使用pread而非mmap，避免生产者截断文件引发映射SIGBUS；不缓存旧图像作为新帧。
读取当前发布槽，复制前后核对元数据；不调用SDK sequential，也不保证读取生产者所有帧。
同次轮询先复制各路图像，再批量解码；时间戳未变化不重复入FIFO。

JPEG_DECODER=auto优先nvJPEG，启动失败才回退OpenCV；nvjpeg严格要求GPU，opencv强制CPU。
GPU输出归一为独立的CPU RGB HWC数组；原始深度不走JPEG。
运行中GPU错误使本条invalid并停止录制器，不静默换后端、不复用旧图像。

图像FIFO默认每路16帧，信号历史默认64条。按参考相机FIFO取帧，其他图像最近时间匹配；
state使用两侧有效样本插值，action只允许过去的因果目标。
等待窗口是max(同步误差, state插值跨度)，不是每帧运算时限。
停流、过旧、同步超限、参考间隔异常、活跃FIFO溢出会使整条invalid，不补帧掩盖。
不可信时间戳回退接收时间，软件时间差不能作为硬件曝光同步证明。

## 写入、后台保存与恢复

写图队列默认64项/256 MiB，深度保留TIFF及原量化路径。磁盘预算后台刷新；
超时快照、空间不足、写图/日志失败拒绝继续本条，不影响已发布数据。
同步日志在单工作线程中序列化写入，有界排队；确认起录/提交前检查屏障。

BACKGROUND_SAVE=1为默认：先停止当前条写图，将数组、图像路径、标签和配置固化到
.collector-queue/<job>/，fsync后返回save_queued。它表示可靠暂存，不表示已发布。
至多两条待处理；一个低优先级编码进程顺序发布，录制器可继续下一条。
队列满拒绝起录，不无限堆积；F封口和Q退出等待队列完成。
保存异常保留raw job，阻止新条，并作废正在采集的未完成条；重启按提交标记恢复，不能手工重发复制数据。

BACKGROUND_SAVE=0使用同步保存；有未完成后台job时拒绝切到同步模式。
两条路径共用：私有工作区、首帧导出、编码器选择、标注、EpisodeStore提交日志。
首帧来自已接受的第0帧PNG，硬链接到first_frame/{head,left,right}_frame；
命名如head_episode_00000.png。首帧与episode共同提交/回滚，不能读取保存时的实时相机替代。

VIDEO_ACCELERATOR=cpu不自动尝试NVENC；auto启动时探测NVENC再VAAPI；
显式硬件不可用则拒绝，运行中不回退。GPU编码只处理RGB，深度不降精度。
启动探针核对颜色/范围；每路硬件输出完整解码核对帧数、尺寸、FPS和时间戳，
回执放annotations/video_encoding。完整读回不等于逐像素无损或模型效果验收。
后台worker恢复每条提交时的配置，不按当前环境重新选择。

EpisodeStore只在新工作区写入，不追加改写历史data/videos。先持久化文件与journal，
再无覆盖发布文件、切换meta、标记committed。prepared回滚；committed只清理，不重复提交。
后台成功以持久化标记和结果回执为准，成功后清理raw job；清理失败不意味着数据未保存。
不要删除正在运行的数据集、队列、锁或恢复文件。

## 操作、标注与DAgger

统一键位详见README：原厂VR/子任务需双Grip，DAgger不需要；Grip立即人工接管。
子任务短B标记exclusive end边界，长B提前保存，未确认尾段为-1并标needs_review。
VR_SUBTASK_B_REARM_SEC要求回执后持续松开B，防止误重复；中途不暂停录制或遍历图片。

DAgger结果与专家mask分开：B成功保存，Y短按丢弃、长按满1秒失败保存；终端K失败保存。
F只封口不退出；Q退出。X仅复位，忙碌/待处理/结果未知时拒绝。
dagger.train_mask保留身体与夹爪AND语义，dagger.action_train_mask逐维区分来源；
held旧目标不能变为新专家动作。模型-only条目mask为0。
保存成功不表示到位或任务成功，任务结果需人工确认。

离线转换独立选择state/action的joint、EEF或混合表示，同步投影mask、统计量、首帧和标注。
首帧补提按LeRobot v3 episode的视频偏移解码，不能对共享MP4统一取第0帧。
所有离线操作必须锁定并避开活动写入；不自动补造历史结果标签。
验证范围和命令见 [验证指南](ROBOT300_TESTING_zh.md)。
