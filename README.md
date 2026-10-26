# 云蝶 AutoLife 数据采集器

> **S2 / S3 共用 `main` 和同一套采集实现**，新S3数据集设置 `ROBOT_TYPE=autolife_s3`。
> 无需S3专用推理预览或模型转换工具；EEF采集与转换继续保留。DAgger需另行核对机器人型号、控制接口并完成现场验收。

统一入口：`start_lerobot_official_collect.sh`。支持键盘、单人 VR、子任务标注和 DAgger，
输出官方 `LeRobotDataset`。本文先解释参数，再给出各模式的完整用法。

- [1. 参数设置与默认值](#parameters)
- [2. 环境准备](#setup)
- [3. 模式与遥操作版本对照](#modes)
- [4. 键盘数采](#keyboard)
- [5. 单人 VR 数采](#vr)
- [6. 子任务数采](#subtask)
- [7. DAgger 纠错数采](#dagger)
- [8. 续录、输出和故障处理](#data)
- [Joint / EEF 数据转换](#conversion)
- [9. 更多说明](#references)

<a id="parameters"></a>
## 1. 参数设置与默认值

参数通过**环境变量**传入，不是在脚本名后追加 `WITH_DEPTH=1`。
下表默认值针对统一入口；DAgger 的差异单独注明。示例中显式设置的值不代表默认值。

```bash
# 只作用于这一次启动；task_name 是保存目录名，TASK_TEXT 是训练任务文本。
WITH_DEPTH=1 COLLECT_FPS=30 TASK_TEXT="Pick up the water bottle." \
bash start_lerobot_official_collect.sh pick_water

# 当前终端后续启动都使用此设置；结束后可以 unset。
export WITH_DEPTH=1
# unset WITH_DEPTH
```

**切换模式时先 `unset VR_CONTROL SUBTASKS_JSON ACTION_MODE`，再填写新模式的参数。**
`export` 的值会一直保留在当前终端；关节、深度开关建议在每次启动命令中显式写出，
避免上一场采集的设置被带入下一场。开关使用 `0` / `1`，不要写 `true` / `false`。

### 1.1 录制内容：深度、头部、腰部、末端位姿

默认录制 **14维手臂 + 2维夹爪 + 三路 RGB**。头部、腰部和深度均为可选扩展。

| 参数 | 通用默认值 | 功能及限制 |
|---|---:|---|
| `WITH_DEPTH` | `0` | `1` 增加 `rgbd_head_depth`，不增加关节维度。保存原生深度视频，不是将深度伪彩色当 RGB。需要相应 SHM 流及支持深度编码的 LeRobot。 |
| `WITH_HEAD` | `0` | `1` 追加 `neck_roll, neck_pitch, neck_yaw`，共3维。 |
| `WITH_UPPER_WAIST` | `0` | `1` 仅追加 `waist_pitch, waist_yaw`，即腰部俯仰、旋转；**不包含腿部升降相关关节**。 |
| `WITH_WAIST` | `0` | `1` 追加 `leg_ankle, leg_knee, waist_pitch, waist_yaw`，共4维，包含腿部关节。 |
| `WITH_EEF_STATE` | `0` | `1` 在 `observation.state` 的完整 joint 向量后追加双臂实测末端位姿，共14维。不会删除原来的关节/夹爪数据。 |
| `WITH_EEF_ACTION` | `0` | `1` 在 `action` 的完整 joint 向量后追加双臂目标末端位姿，共14维。与 `WITH_EEF_STATE` 独立；不会把实测位姿当成目标。 |
| `STATE_EEF_TOPIC` | `/topic_arm_current_robot_eef_pose_<domain>_<robot>` | 实测位姿报告。覆盖时必须保持相同的机器人坐标系、米、XYZW四元数语义，不能传入控制指令话题。 |
| `ACTION_EEF_TOPIC` | `/topic_arm_target_robot_eef_pose_<domain>_<robot>` | 目标位姿报告。该报告不等于 VR 手柄位姿，也不是本工具根据 joint action 做 FK 算出的位姿。 |
| `ACTION_MODE` | 常规 `status_target`；DAgger `joint` | **键盘、单人VR、子任务数采使用 `status_target`**；**DAgger 使用 `joint`**，由启动器设置并校验。仅保留这两个取值，具体来源见下文。 |

`WITH_UPPER_WAIST=1` 与 `WITH_WAIST=1` **不能同时出现**。
开关控制的是保存哪些维度，不是禁止机器人其余关节运动。

<a id="action-modes"></a>
**Action 来源约定：**

- **常规数采 `keyboard / vr / subtask`：`ACTION_MODE=status_target`。** 默认无需填写。手臂/头/腰取状态包中的控制目标；由于状态包没有独立夹爪目标，两维夹爪取实测位置。
- **DAgger：`ACTION_MODE=joint`。** 启动器自动设置，显式设置为 `status_target` 会报错。使用修改后的V4控制器实际发布的目标位置，经 `/hg_dagger/collector/arm_action` 和 `/hg_dagger/collector/gripper_action` 写入，并保留控制来源、时间戳和训练mask；不回退为实测state。

原厂App能让机器人运动、夹爪支持位置控制，都不等于它会发布独立的身体/夹爪目标位置命令。
非DAgger的底层 `joint` 接口仅保留给已核实命令话题的集成或链路调试，不作为常规数采配置。
345原厂链路的实际话题对比见[验证记录](docs/validation/20261019_s3_345.md)，不替代DAgger独立验收。

关节模式的 state/action 顺序固定为：左臂7维 → 右臂7维 → 左夹爪 → 右夹爪 →
可选头部3维 → 可选上腰2维或完整腰腿4维。具体名称见 [robot_schema.py](robot_schema.py)。

| 录制组合 | `WITH_HEAD` | `WITH_UPPER_WAIST` | `WITH_WAIST` | state/action维度 |
|---|---:|---:|---:|---:|
| 手臂+夹爪 | 0 | 0 | 0 | 16 |
| 手臂+夹爪+头 | 1 | 0 | 0 | 19 |
| 手臂+夹爪+上腰 | 0 | 1 | 0 | 18 |
| 手臂+夹爪+完整腰腿 | 0 | 0 | 1 | 20 |
| 手臂+夹爪+头+上腰 | 1 | 1 | 0 | 21 |
| 全量关节，含腿部 | 1 | 0 | 1 | 23 |

每个组合都可另加 `WITH_DEPTH=0/1`。**当前 DAgger 的关节控制只接受21维组合**，不是23维。
EEF 是录制扩展，不改变 DAgger 模型输入、控制命令或接管逻辑。
新建数据集后，改变关节、EEF开关、相机、深度、action格式或标注字段时，应换新目录，不能直接追加到旧 schema。

#### EEF 使用前提

两个开关独立为state/action追加14维：左臂XYZ+XYZW、右臂XYZ+XYZW。
位置为机器人坐标系下的米，姿态为单位四元数；14维不含夹爪，夹爪仍在joint前缀。
16/21/23维joint分别变为30/35/37维；未开启的一侧向量不变。

需要持续更新的实测/目标位姿报告；缺失时拒绝初始化，过旧或无法同步时整条作废。
不会用零、FK或实测state替代缺少的目标。旧15维 `ACTION_MODE=eef` 已移除；
使用追加开关录制后，再通过[离线转换](#conversion)导出纯EEF或混合表示。
新数据集的 `meta/collector_schema.json` 记录来源和索引，不能按向量宽度猜测旧数据含义。

**DAgger追加的14维EEF action训练mask固定为0**：joint命令来源不能证明独立EEF报告的专家身份。
EEF纠错训练需单独核实和标注来源。混合state/action、四元数也不能直接套用逐维相减的relative action。

### 1.2 模式、任务和保存位置

| 参数 | 默认值 | 功能及用法 |
|---|---|---|
| `COLLECTOR_MODE` | 无旧参数时为 `keyboard` | 显式选择 `keyboard / vr / subtask / dagger`。推荐每次写出，不依赖旧开关自动推断。 |
| `task_name` | 普通模式 `mango_pick`；DAgger必填 | 脚本后的第一个位置参数，决定任务目录名和 `local/<task_name>` repo-id。建议只用字母、数字、下划线，不放斜杠。 |
| `TASK_TEXT` | 第二个位置参数，否则为 `task_name` | 写入数据的自然语言总任务。设置该环境变量时优先使用它，含空格的文本加引号。不是保存目录名。 |
| `OUTPUT_BASE_DIR` | `/home/ubuntu/nas14` | 数据父目录，实际输出为 `<OUTPUT_BASE_DIR>/<task_name>/dataset`，日志在同级 `logs/`。目录叫 nas 不代表已经挂载网络存储。 |
| `SUBTASKS_JSON` | `[]` | 有序子任务文本数组，例如 `'["右手拿起毛巾","双手交接","左手放下"]'`。显式 `subtask` 模式必填非空数组，其他显式模式不能携带非空计划。 |
| `VR_CONTROL` | 旧入口默认0 | 旧版兼容开关。指定 `COLLECTOR_MODE` 后不必再设置；冲突时启动器会拒绝运行。 |

### 1.3 帧率、相机与同步

| 参数 | 默认值 | 功能及调整影响 |
|---|---|---|
| `COLLECT_FPS` | `30` | 数据集目标帧率，单位帧/秒；还用于检查参考帧间隔。**不会修改硬件相机帧率**，也不能靠改数值解决源流卡顿。 |
| `IMAGE_SOURCE` | `shm` | `shm` 直接读共享内存；`ros` 启动 SHM→ROS 图像桥后订阅。普通使用优先保留 `shm`，不必增加图像 topic 传输。 |
| `IMAGE_POLL_FPS` | 普通 `120`；DAgger `60` | SHM metadata轮询频率，不是录制帧率。新源时间戳才读入图像 FIFO；提高轮询率不能提高相机实际 FPS，反而会增加CPU开销。 |
| `SYNC_REFERENCE_CAMERA` | 活跃的 `hand_left`，否则首个活跃相机 | 时间基准。可填 `hand_left`、`hand_right`、`rgbd_head_color`，开深度后也可选活跃的 `rgbd_head_depth`。换基准仍须匹配其他相机，不是忽略它们。 |
| `MAX_SYNC_DELTA_SEC` | `0.03` 秒 | 非参考相机与参考帧允许的最大时间差，即默认30ms；也参与等待/动作新鲜度判断。关节插值跨度及action保持另有下列独立限制。 |
| `MAX_IMAGE_AGE_SEC` | `0.15` 秒 | 图像接收后的最大允许年龄，用于判定旧帧和相机停流；不是跨相机同步误差。 |
| `MAX_STATE_AGE_SEC` | `0.15` 秒 | joint 和可选 EEF state 接收后的新鲜度限制；限制插值端点过旧及状态流停滞。 |
| `MAX_STATE_INTERPOLATION_GAP_SEC` | `0.05` 秒 | joint/EEF state 插值的前、后两个样本之间最大跨度。扩大后可跨更长缺口插值，但时间准确性下降。 |
| `MAX_ACTION_HOLD_SEC` | `0.5` 秒 | 参考时刻前最近一条有效目标最多保持多久；不使用未来命令作为当前帧标签，也不是允许无新指令无限续录。 |
| `MAX_FRAME_INTERVAL_ERROR_RATIO` | `0.45` | 参考帧间隔相对 `1/COLLECT_FPS` 的最大误差比例，取值大于0、小于1。30FPS下允许约18.3～48.3ms；不是每帧代码运行超时。 |
| `SYNC_IMAGE_BUFFER_SIZE` | `16` | **每路相机**的图像 FIFO 上限，单位帧，至少2；录制时溢出使当前episode无效。增大占用更多内存，不能修复源流缺帧或长期处理落后。 |
| `SYNC_SIGNAL_BUFFER_SIZE` | `64` | 各状态/action时间缓存容量，单位样本，至少2；用于插值及因果目标匹配，不是图像队列。 |
| `MIN_CAMERAS` | 所选相机全部：RGB为3，带depth为4 | 启动时至少有多少路有效相机。不要为了绕过缺流而调低；可能改变实际录制 schema。DAgger不允许低于所选路数。 |
| `START_HAND_PRODUCER` | 默认1；检测到手相机设备占用时自动0 | `0` 复用已运行视觉服务的手相机SHM；`1` 尝试启动本工具的手相机生产者。已有生产服务时建议显式0，避免争抢设备。不会自动启动原厂视觉服务。 |

双手相机采用 **JPEG SHM→解码为BGR→按数据特征转换**，不回退到旧的直接BGR SHM路径。
源流、匹配或存储失败时不会静默跳帧继续生成“有效”数据，而是作废当前episode。

例如，任务允许40ms容差时可显式设置以下参数；这不是所有机器都必须采用的配置：

```bash
export COLLECT_FPS=30
export IMAGE_POLL_FPS=120
export SYNC_REFERENCE_CAMERA=hand_left
export MAX_SYNC_DELTA_SEC=0.04
export SYNC_IMAGE_BUFFER_SIZE=16
```

`0.04` 只表示接受更大的时间差，并不表示硬件同步变好。
实际使用须结合 `sync_log.jsonl` 判断，不应反复放宽阈值来掩盖故障。
若改为15FPS，仍需验证源参考帧间隔是否匹配，不能假设启动器会自动将30FPS相机降为15FPS。

### 1.4 启动与操作等待时间

| 参数 | 默认值 | 对应阶段 |
|---|---:|---|
| `STATE_READY_TIMEOUT_SEC` | 10秒 | 等待首个完整关节状态包；没有完整包，不启动可选相机生产者。已有SHM可被读取，尚不录制。 |
| `CAMERA_WARMUP_SEC` | 10秒 | 统一启动器传给录制器的相机准备等待时间。直接运行Python的默认值不同，以该入口帮助为准。 |
| `CAMERA_DETECTION_TIMEOUT_SEC` | 15秒 | 外层启动器等待相机/数据集初始化结果。提高相机warmup时，应同时给这个外层等待留足时间。 |
| `CONTROL_ACK_TIMEOUT_SEC` | 300秒 | 等待开始、标记、保存、丢弃等命令的真实回执；不是图像同步阈值。结果未知时先查日志，不盲目重复提交。正常退出会等待编码完成，不按此值强杀录制器。 |

启动慢可增加准备等待时间；录制中 `unsynchronized_camera`、`stale_camera` 等错误
不会因为增加 `CAMERA_WARMUP_SEC` 而得到修复。保存较慢也不等于应放宽同步误差。

### 1.5 暂存、内存与写入上限

以下保护四种模式共用。容量单位分别为 **GiB（1024³字节）/ MiB（1024²字节）**。

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `MIN_FREE_DISK_GB` | 2 | 磁盘剩余空间底线；还会额外预留一份当前条目估算大小供编码使用。 |
| `MAX_EPISODE_TEMP_GB` | 8 | 单条episode原始图像字节及开销的累计预算，不是整个数据集大小上限。 |
| `MAX_EPISODE_SECONDS` | 600 | 单条开始后的最长持续时间，包含等待；**到限作废，不是到时自动保存**。可能先达到容量限制。 |
| `IMAGE_WRITER_MAX_PENDING` | 64 | 所有相机共享的未完成写图任务数上限，不是每路图像FIFO容量。 |
| `IMAGE_WRITER_MAX_MB` | 256 | 未完成写图任务所持原始图像的内存预算，不是进程总内存上限。 |
| `IMAGE_WRITER_THREADS` | 普通4；DAgger2 | 后台写图线程数。0为同步写入，可能阻塞采集；更多线程也可能增加CPU和磁盘争用。 |
| `HG_DAGGER_RECORDER_CPUS` | 按可用CPU选择 | 仅DAgger：300保留`12-19`；其他布局选择可用CPU尾部最多4核并保留至少一半给控制。可显式指定，经`taskset`校验；仍需每台机器做负载验收。 |

前五个限额必须大于0。超限、写图或保存失败会将当前条目标为 `EPISODE INVALID`，
停止接收新帧，不覆盖已经保存的数据。处理方法见[故障与恢复](#recovery)。

### 1.6 RGB/深度编码与文件参数

| 参数 | 默认值 | 功能及注意事项 |
|---|---|---|
| `COLLECT_VCODEC` | `h264` | RGB视频编码器。可用值取决于安装的LeRobot/编解码库；改编码器前先做短条测试。 |
| `VIDEO_CRF` | 未设置 | RGB质量参数，交由编码器默认配置处理；常见CRF编码器中数值越低质量越高、文件越大，范围取决于编码器。 |
| `VIDEO_PRESET` | 未设置 | RGB编码速度/压缩率档位；支持的名称随编码器变化。 |
| `VIDEO_GOP` | 未设置 | RGB关键帧间隔，单位帧；影响随机读取和压缩率。 |
| `DEPTH_VCODEC` | `hevc` | 深度视频编码器；需支持LeRobot使用的高位深格式，不能随意换成普通8位RGB编码配置。 |
| `DEPTH_MIN` | `0.05` 米 | 深度量化的物理距离下限，不是相机硬件近裁剪设置。 |
| `DEPTH_MAX` | `10.0` 米 | 深度量化的物理距离上限，必须大于下限；应覆盖采集对象的实际距离。 |
| `DEPTH_SHIFT` | `3.5` | 对数量化的shift参数，影响距离精度分布；不是将相机或深度值平移3.5米。 |
| `DEPTH_USE_LOG` | `1` | 1使用对数深度量化，0关闭；仅在录制depth时有意义。 |
| `BATCH_ENCODING_SIZE` | `1` | 事务保存要求保持1，不能通过增大它把多条episode批量积累后再保存。 |
| `STREAMING_ENCODING` | `0` | 事务保存要求关闭，设1会被拒绝。 |
| `ENCODER_QUEUE_MAXSIZE` | `30` | 传给LeRobot的编码队列容量；与图像FIFO、写图队列不是同一个队列，不能替代存储限额。 |
| `ENCODER_THREADS` | 普通未设置；DAgger1 | 编码线程数，未设置时由LeRobot决定；不控制相机轮询线程数。 |
| `VIDEO_FILES_SIZE_IN_MB` | 未设置 | LeRobot视频分段大小选项，单位按库的MB接口处理。事务模式每条使用新文件，不能用它合并episode。 |
| `DATA_FILES_SIZE_IN_MB` | 未设置 | LeRobot Parquet分段大小选项；同样不会把新episode追加写入已提交文件。 |

### 1.7 单人和子任务控制参数

这些参数主要作用于 `vr / subtask` 的手柄辅助程序，**不会替换原厂遥操作**。

| 参数 | 默认值 | 功能及注意事项 |
|---|---|---|
| `VR_START_DELAY_SEC` | `3` 秒 | A开始后的倒计时，范围0～30。B/Y可取消；DAgger不使用此倒计时。 |
| `VR_SPEECH` | `1` | 1经机器人现有TTS服务播报，0静音；不修改系统音量，也不保证头显扬声器播放。 |
| `VR_A_LONG_PRESS_SEC` | `1.0` 秒 | 历史参数名，实际是**子任务模式B键**的长按阈值，范围0.2～10秒；在松开时判定。普通VR不使用长按标注。 |
| `VR_INPUT_TOPIC` | `/control_topic_<domain>_<robot>` | 默认读取原厂手柄消息；300/domain0为 `/control_topic_0_300`。不能仅改为V4话题就认为整套控制兼容。 |
| `VR_RESET_PREFIX` | `/openarmx_teleop_vr_306_v4` | 仅显式接入受保护V4复位服务时使用。**原厂模式X复位尚未接入**，设置这个变量不能自动补齐。 |
| `MOTION_LOCK_FILE` | `/tmp/lerobot_robot_<domain>_<robot>.motion.lock` | 录制与受保护复位共用锁；同机器人各组件必须一致，不要为绕过冲突而换锁路径。 |

子任务短B确认后播报“下一步”；保存成功播报“已保存”，提前保存可能播报“已保存，标注未完成”。
**原厂独立App当前没有Collector子任务进度面板，也不会加载8447的提示扩展。**

### 1.8 DAgger 专用参数

除注明外，仅对 `COLLECTOR_MODE=dagger` 生效。按`ROBOT_ID`与`DAGGER_ROBOT_PROFILE`分别选择编号和型号，
保持21维头部+上腰、三路RGB和兼容Thor GR00T baseline/frame协议。
编号可配置不等于该机器已完成控制器部署或真机验收；新机器仍需增量遥操、匹配URDF和独立控制权接口。

| 参数 | 默认值 | 功能及注意事项 |
|---|---|---|
| `DAGGER_ROBOT_PROFILE` | 300默认`s2_v2_2` | 支持`s2_v2_2`、`s3_v3_1`；其他编号须明确型号，`ROBOT_TYPE=autolife_s3`也可选择S3。型号不是由编号猜测。 |
| `DAGGER_ROBOT_DESCRIPTION_ROOT` | S3的SDK descriptions/autolife_s1目录 | S3使用已审查的`robot_v3_1.urdf`和对应SRDF；模型描述哈希不符时拒绝启动，需复核校准。 |
| `DAGGER_BACKEND` | `owned` | `owned` 启动并管理一套独占宿主；`attach` 仅连接已有**兼容**宿主，不创建/替换控制器。详见模式示例。 |
| `DAGGER_PUBLISH` | `0` | 0不发布硬件动作；1允许用户按键触发推理控制/复位。设1本身不会自动开始，但A/C、X/R可能产生真实运动。 |
| `DAGGER_SERVER_URL` | 统一入口无默认，必填 | Thor服务URL，例如 `http://192.168.8.224:8777`。IP/端口是部署示例，以当前服务为准；不支持旧SOMA/Outcome协议。 |
| `DAGGER_TOKEN_FILE` | `/home/ubuntu/.config/autolife_hg_dagger/groot_server.token` | Thor认证token文件，须与服务匹配；不要把token写进URL或提交到Git。 |
| `GROOT_REMOTE_TOKEN` | 未设置 | 兼容的环境变量认证入口，非空时预检和模型桥接均优先使用它而不是token文件。建议统一使用文件，避免Shell历史泄漏或旧环境变量覆盖凭据。 |
| `DAGGER_DEPENDENCY_ROOT` | `/home/ubuntu/ros2_ws/src` | 包含已审核 `openarmx_teleop_vr_306_v4` 的目录。源文件须通过哈希检查，启动器在 `.runtime/` 副本中应用补丁，不改同事原代码。 |
| `DAGGER_TOOLS_ROOT` | 集合仓库自动取根目录；独立库为 `/home/ubuntu/Autolife_VLA_Tools` | 保留旧Tools目录兼容入口；DAgger预检现在复用仓库内HTTP客户端，不再为读health要求安装整套Tools。 |
| `DAGGER_WEB_PORT` | `8447` | V4扩展网页的HTTPS端口，1～65535。改变端口不能解决两套控制器争用关节的问题。 |
| `DAGGER_ROS_SETUP` | 有工作区时加载工作区，否则系统ROS | 显式指定的脚本必须存在；支持新机器没有`ros2_ws`的安装布局。 |
| `DAGGER_ROS_PY` | `/usr/bin/python3` | ROS监督器、桥接和attach使用的Python。 |
| `DAGGER_VR_PY` | `/home/ubuntu/ros2_ws/venvs/openarmx_v4_placo/bin/python` | 增量遥操作控制器Python，需安装Placo等依赖。 |
| `DAGGER_WEB_PY` | `/home/ubuntu/ros2_ws/venvs/hg_dagger_web/bin/python` | HTTPS/WebXR进程Python。 |

DAgger在未显式设置时使用 `WITH_HEAD=1 WITH_UPPER_WAIST=1 WITH_WAIST=0 WITH_DEPTH=0`；
强制使用 `ACTION_MODE=joint`、`START_HAND_PRODUCER=0`，要求所选相机全部可用。
`WITH_DEPTH=1` 只增加录制深度，当前模型仍只接收RGB。

**便捷脚本与统一入口默认值不同：** `start_dagger.sh` 预设Thor地址、
`OUTPUT_BASE_DIR=/home/ubuntu/nas/dagger`、`WITH_DEPTH=1` 和 `TASK_TEXT='Pick the laundry bag.'`；
同时固定21维组合。默认仍不允许硬件发布。可覆盖的环境变量以该脚本为准，
不要把它的预设误认为其他三种模式的默认值。

### 1.9 环境及排障专用参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `ROBOT_ID` | 主机名末尾数字，否则283 | 生成机器人话题后缀；在 `autolife-robot-300` 上为300。不要混淆IP和机器人编号。DAgger另需匹配`DAGGER_ROBOT_PROFILE`。 |
| `ROBOT_TYPE` | `autolife_s1` | 新数据集的机器人类型标签；S3显式设置`autolife_s3`。不改变关节顺序、单位或控制方式，也不把S3坐标转换成S2。旧数据续录不会自动改标签。 |
| `ROS_DOMAIN_ID` | `0` | ROS通信域，必须与机器人服务一致；不是机器人编号。DAgger接受0～232，并据此生成话题后缀。 |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | ROS中间件；本工具当前使用CycloneDDS本机通信配置。 |
| `ROBOT_PY` | `/home/ubuntu/miniconda3/envs/robot_env/bin/python` | 相机生产/桥接、普通VR辅助程序解释器。 |
| `LEROBOT_PY` | `/home/ubuntu/miniconda3/envs/lerobot/bin/python` | LeRobot录制器解释器。仅改解释器路径不等于完成ROS/native library环境迁移。 |
| `CAMERA_ONLY` | `0` | 1为头部RGB链路测试，关闭手相机生产并请求state作为action回退；仍需关节数据。若要测试实际回退路径，应配合 `ACTION_MODE=joint`，并清除 `WITH_DEPTH` 等额外开关。不是专家数据采集配置。 |
| `FALLBACK_ACTION_TO_STATE` | `0` | 缺少joint目标时允许用实测state替代action，仅用于链路调试，不应当作专家目标；DAgger必须保持0。 |
| `OMP_NUM_THREADS` / `MKL_NUM_THREADS` / `OPENBLAS_NUM_THREADS` | 通用入口未设置时为1 | 限制底层数学库线程，避免小计算创建过多线程；DAgger录制隔离脚本会强制这些库为1线程。 |

默认本机回环DDS。覆盖优先级：`COLLECT_CYCLONEDDS_URI` → `CYCLONEDDS_URI` → 内置配置；
录制器和辅助进程使用同一有效配置，自定义时须确保与机器人服务互通。

帮助只依赖系统Python标准库，不加载ROS/LeRobot，也不会启动采集：

```bash
bash start_lerobot_official_collect.sh --help
bash start_lerobot_official_collect.sh WITH_DEPTH --help
bash start_lerobot_official_collect.sh MAX_SYNC_DELTA_SEC --help
bash start_lerobot_official_collect.sh DAGGER_BACKEND --help

# 录制器的底层参数使用 --kebab-case；不等同于启动脚本的环境变量。
python3 record_lerobot_official.py --help
python3 record_lerobot_official.py --max-sync-delta-sec --help
```

`--camera`、`--depth-camera`、`--duration` 等是录制器直接入口参数，
统一Shell启动器不会自动转发任意Python参数，不要直接拼在任务名后。
`--check` / `--serve` 是DAgger入口选项，不是普通三种模式的通用选项。


<a id="setup"></a>
## 2. 环境准备

依赖ROS 2 Jazzy、已运行的关节反馈和相机SHM服务。`LEROBOT_PY` 需要LeRobot 0.6+的
DatasetWriter/resume接口、ROS、OpenCV、PyArrow和编码依赖；深度需要高位深HEVC。
VR辅助、相机桥和备用生产者使用 `ROBOT_PY`；纯SHM键盘录制不启动这些辅助时，不要求额外robot_env。
工具不自动安装或重启硬件服务。

新机器人从零配置请看 [部署与 SSH 密钥登录指南](docs/NEW_ROBOT_DEPLOYMENT_zh.md)，
包括电脑/机器人分步命令、独立采集环境、只读检查与首次人工录制。

独立仓库首次安装：

```bash
cd /home/ubuntu
git clone git@github.com:lee1354774070-ai/CB_Autolife_Data_Collector.git
cd CB_Autolife_Data_Collector
```

集合仓库则进入 `/home/ubuntu/Autolife_VLA_Tools/lerobot_data_collector`。不要从两个副本同时采同一目录。
以下以 **S3-345** 为例，其他机器修改编号/型号；解释器不在默认位置时设置Python绝对路径：

```bash
cd /home/ubuntu/CB_Autolife_Data_Collector
export ROBOT_ID=345 ROS_DOMAIN_ID=0 ROBOT_TYPE=autolife_s3
source scripts/source_ros_env.sh
set +u
unset VR_CONTROL SUBTASKS_JSON ACTION_MODE
export OUTPUT_BASE_DIR=/home/ubuntu/nas14
export START_HAND_PRODUCER=0 IMAGE_SOURCE=shm
export COLLECT_FPS=30 MAX_SYNC_DELTA_SEC=0.03 SYNC_IMAGE_BUFFER_SIZE=16
export SYNC_REFERENCE_CAMERA=hand_left
```

`set +u` 只关闭Shell未定义变量报错，不改变电机使能。
S2/S3共用main和已有兼容环境，不需要专门的S3推理预览；无ros2_ws时只加载系统ROS。
启动首行显示实际代码目录，请据此排除旧副本。

<a id="modes"></a>
## 3. 模式与遥操作版本

| 模式 | 机器人遥操作 | 头显入口 | 录制控制 |
|---|---|---|---|
| `keyboard` | Autolife原厂 | 原厂独立App，机器人地址:3000 | 终端Enter/S/D/Q |
| `vr` | Autolife原厂，不替换控制器 | 同上，不使用8447 | GL+GR加面键 |
| `subtask` | Autolife原厂，不替换控制器 | 同上，不使用8447 | GL+GR加面键，B标注 |
| `dagger` | 适配后的V4增量运行副本 | 浏览器 `https://机器人IP:8447` | 面键与Grip接管 |

前三种保持原厂遥操作运行。**3000是原厂App服务，不是DAgger网页。**
DAgger owned要求独占控制；attach仅连接已适配兼容宿主，不兼容任意原厂/同事旧程序。
控制冲突时核实具体任务，不通过重启ARM、清错、换端口或跳过校验绕过保护。

<a id="keyboard"></a>
## 4. 键盘数采

完成环境准备后：

```bash
COLLECTOR_MODE=keyboard ACTION_MODE=status_target \
WITH_HEAD=0 WITH_UPPER_WAIST=0 WITH_WAIST=0 WITH_DEPTH=0 \
WITH_EEF_STATE=0 WITH_EEF_ACTION=0 \
TASK_TEXT="Pick up the water bottle." \
bash start_lerobot_official_collect.sh pick_water
```

等待有效相机和操作提示，再按Enter开始遥操作。

| 终端键 | 作用 |
|---|---|
| Enter | 开始新episode，不自动开启下一条 |
| S | 保存并暂停，等待 `[SAVED]` 和各任务累计条数 |
| D | 丢弃当前条，不影响以前保存的数据 |
| N | 仅有子任务计划时标记当前片段 |
| Q | 收尾退出；有效待处理数据尝试保存，invalid不保存 |
| Ctrl+C | 请求正常收尾，不等于已经保存成功 |

推荐：**S确认保存 → Q → 等待正常退出 → 下一批 → 重新启用控制 → Enter**。
编码可能需要较长时间，退出会等待完成，不再因固定30秒期限终止录制器。
录制/编码时不要删除目录、关机或强杀。

四种组合替换上述开关，不同schema使用不同目录：

| 组合 | WITH_HEAD | WITH_UPPER_WAIST | WITH_WAIST | WITH_DEPTH | joint维度 |
|---|---:|---:|---:|---:|---:|
| 手臂+夹爪、RGB | 0 | 0 | 0 | 0 | 16 |
| 手臂+夹爪、RGB-D | 0 | 0 | 0 | 1 | 16 |
| 全关节、RGB | 1 | 0 | 1 | 0 | 23 |
| 全关节、RGB-D | 1 | 0 | 1 | 1 | 23 |

<a id="vr"></a>
## 5. 单人VR数采

连接**原厂App**并确认遥操作正常；Collector只增加录制控制：

```bash
unset VR_CONTROL SUBTASKS_JSON ACTION_MODE
COLLECTOR_MODE=vr WITH_HEAD=1 WITH_UPPER_WAIST=1 WITH_WAIST=0 \
WITH_DEPTH=0 WITH_EEF_STATE=0 WITH_EEF_ACTION=0 \
VR_START_DELAY_SEC=3 VR_SPEECH=1 \
TASK_TEXT="Put the towel in the basket." \
bash start_lerobot_official_collect.sh towel_vr
```

全程按住 **GL+GR**，点按并松开一个面键：
- A：倒计时后开始，B/Y可取消倒计时。
- B：保存并暂停，收到成功提示后才开始下一条。
- Y：只丢弃，不复位。
- X：原厂复位尚未接入，只提示不可用；先保存/丢弃，再使用原厂复位。

终端键盘仍可用，Q退出；未确认的命令不重复排队。
语音使用机器人TTS。**原厂App暂不支持本工具的振动或子任务进度叠加**；
发布反馈事件不等于头显实际收到反馈。

<a id="subtask"></a>
## 6. 子任务数采

仍使用原厂App，提前定义子任务顺序：

```bash
unset VR_CONTROL ACTION_MODE
COLLECTOR_MODE=subtask WITH_HEAD=1 WITH_UPPER_WAIST=1 WITH_WAIST=0 \
WITH_DEPTH=0 WITH_EEF_STATE=0 WITH_EEF_ACTION=0 \
TASK_TEXT="Put the towel in the basket." \
SUBTASKS_JSON='["右手拿起毛巾","双手交接毛巾","左手放下毛巾"]' \
VR_A_LONG_PRESS_SEC=1.0 \
bash start_lerobot_official_collect.sh towel_subtask
```

GL+GR+A开始；每完成一个子任务，GL+GR+**短按B并松开**。
中间标记不暂停、不编码，继续下一动作；最后一次标记自动保存整条。
GL+GR+**长按B达到阈值后松开**可提前保存；终端N标记、S提前保存。

总任务为 `TASK_TEXT`。`subtask_index` 是本条计划局部序号，边界以录制器已接受帧为准，
不是手柄硬件时间。未标记尾段为-1，`annotations/subtasks/episode_XXXXXX.json` 标记需人工复核。
漏按/晚按需人工清洗，完成标记不证明任务成功；启用/关闭标注必须换目录。
声音为短提示，无头显进度列表。Y丢弃、X限制及退出方式同普通VR。

<a id="dagger"></a>
## 7. DAgger纠错采集

使用**经审核的V4增量副本和匹配Thor服务**，不是原厂App。
固定21维joint+3RGB；depth/EEF可额外录制，不改变模型输入。
S3普通采集通过不等于S3 DAgger验收通过，仍需核对URDF、控制权接口和模型。

以下为已配置 **S2-300** 示例，不能原样用于345：

```bash
unset VR_CONTROL SUBTASKS_JSON ACTION_MODE
export COLLECTOR_MODE=dagger DAGGER_BACKEND=owned
export ROBOT_ID=300 ROS_DOMAIN_ID=0 DAGGER_ROBOT_PROFILE=s2_v2_2
export WITH_HEAD=1 WITH_UPPER_WAIST=1 WITH_WAIST=0 WITH_DEPTH=0
export WITH_EEF_STATE=0 WITH_EEF_ACTION=0
export DAGGER_SERVER_URL=http://192.168.8.224:8777
export DAGGER_TOKEN_FILE=/home/ubuntu/.config/autolife_hg_dagger/groot_server.token
export DAGGER_DEPENDENCY_ROOT=/home/ubuntu/ros2_ws/src
export DAGGER_WEB_PORT=8447 OUTPUT_BASE_DIR=/home/ubuntu/nas/dagger
export TASK_TEXT='Pick the laundry bag.'

DAGGER_PUBLISH=0 bash start_lerobot_official_collect.sh dagger_check --check
DAGGER_PUBLISH=0 bash start_lerobot_official_collect.sh dagger_dryrun
```

`--check` 只读检查依赖/ROS冲突/Thor协议；第二条启动无硬件发布的待命会话。
Q退出后，仅在现场安全、急停可用且获运动授权时启动真实控制：

```bash
DAGGER_PUBLISH=1 bash start_lerobot_official_collect.sh dagger_live
```

浏览器连接 `https://机器人当前IP:8447`，等待就绪后主动按A。

| VR键 | 作用 |
|---|---|
| A | 等待录制回执及硬件就绪后放行模型 |
| 任一Grip新按下 | 立即切人工权限，增量控制，不等待HTTP；之后不自动交还模型 |
| B | 保存并停用本轮，不复位 |
| Y | 丢弃本轮，不复位 |
| X | 全身复位并张开夹爪；有待处理数据/忙碌时拒绝，先B/Y并释放Grip |

面键无需双Grip；终端为 **C开始、A保存、X/D丢弃、R复位、Q退出**。
DAgger退出丢弃未确认条目，要保留必须先B确认。接管振动依赖WebXR支持。
控制回执不证明到位，保存不证明任务成功；训练须显式使用 `dagger.action_train_mask`。

接入已有兼容宿主，另开终端并匹配机器人、目录、任务、发布权限：

```bash
COLLECTOR_MODE=dagger DAGGER_BACKEND=attach \
DAGGER_PUBLISH=1 WITH_DEPTH=0 OUTPUT_BASE_DIR=/home/ubuntu/nas/dagger \
TASK_TEXT='Pick the laundry bag.' \
bash start_lerobot_official_collect.sh dagger_live --check
```

去掉 `--check` 进入交互；**attach的Q仅断开本终端，宿主继续运行**。
`--serve` 只用于owned后台宿主；旧程序需一次性迁移，不能热替换。
便捷 `start_dagger.sh` 默认开启depth，桌面入口 `scripts/open_dagger_launcher.sh` 同属DAgger；
其预设不等于统一入口默认值，使用前明确设置。

Thor须支持 `controller_submission_receipts=true`；阶段夹爪策略只支持审核过的完整任务文本。
详情见 [DAgger实现](INSTRUCTION_zh.md#dagger)，不是任意任务通用策略。

<a id="data"></a>
## 8. 续录、输出和故障处理

同一输出父目录、目录名和schema，在S保存、Q正常退出后再次启动即续录。
目录名不必等于任务文本；交替修改 `TASK_TEXT` 可将多任务按episode顺序存入同一目录：

```bash
TASK_TEXT="Pick up the water bottle." COLLECTOR_MODE=keyboard \
bash start_lerobot_official_collect.sh hotel_water
# 第一场S保存、Q正常退出后：
TASK_TEXT="Put the water bottle in the box." COLLECTOR_MODE=keyboard \
bash start_lerobot_official_collect.sh hotel_water
```

`task_index` 区分总任务。每条Parquet/视频可独立成文件，官方meta负责索引，**不需手动拼接**。
改变FPS、关节/EEF、相机、depth、Action来源、子任务或DAgger字段时换新目录。

```text
task_name/
  dataset/
    meta/                   信息、任务、episode索引、统计、collector_schema.json
    data/                   Parquet
    videos/                 RGB / 可选depth
    annotations/subtasks/   可选子任务侧文件
    sync_log.jsonl          软件同步诊断
    .collector-work/        当前私有暂存，运行时禁止删除
    .collector-commit.json  仅提交/恢复期间存在
  logs/                     每次启动独立数字序号，不覆盖旧日志
  .official_*               进程、FIFO、回执及锁
```

<a id="recovery"></a>
### 故障处理

| 现象 | 处理 |
|---|---|
| 初始化前退出 | 看 `logs/N.record_lerobot_official.log` 最后异常；外层提示只是结果 |
| 相机不足/右手缺流 | 核实原厂JPEG SHM持续更新，不靠降低MIN_CAMERAS绕过 |
| `EPISODE INVALID` | 停止本条操作，D/Y丢弃；查sync_log中的reason，解决源流/同步/存储问题 |
| 保存回执超时 | 结果未知，查原请求回执和日志，不重发save、删暂存或复位 |
| 同步日志写入失败 | 本条无效且禁止再开始；解决空间/权限后正常退出、重启采集 |
| 写入/编码失败 | 仅当前暂存无效，已保存文件不变；能否续采以回执为准 |
| 提交结果不确定/遗留journal | 停止会话，重启按journal恢复；不能删标记假装成功 |
| `.collector-interrupted-*` | 未提交暂存，需人工检查；占空间但不计入已保存数据 |
| 重复启动被拒绝 | 原终端正常退出，等待保存子进程结束，不删锁文件绕过 |

30FPS要求参考源流接近30FPS，软件不能补回相机未产生的帧。
`waiting ticks` 只表示等待匹配，不等于跳帧；软件时间差不证明硬件曝光同步。
结束并确认无写入者后才可删除整组目录；只删logs不删除正式数据。

<a id="conversion"></a>
### Joint / EEF离线转换

结束源数据采集，用LeRobot环境运行；目标必须是不存在、非嵌套的新目录：

```bash
PY=/home/ubuntu/miniconda3/envs/lerobot/bin/python
SRC=/home/ubuntu/nas14/towel_joint_eef/dataset
"$PY" convert_dataset.py "$SRC" /home/ubuntu/nas14/export_joint --state joint --action joint
"$PY" convert_dataset.py "$SRC" /home/ubuntu/nas14/export_eef --state eef --action eef
"$PY" convert_dataset.py "$SRC" /home/ubuntu/nas14/export_mixed --state joint --action eef+gripper
```

state/action分别选择 `joint | eef | eef+gripper | all`：
joint保留已录关节/夹爪，eef仅14维，eef+gripper为16维，all不裁剪。
未录制EEF不能生成EEF；需要collector_schema，不猜历史索引。

转换同步更新数据、统计、Arrow/HF维度及逐维action mask；任务/子任务、帧顺序、视频偏移不变，
视频独立复制、不重编码。不做IK/FK、失败示范筛除、重新标注或重采样，不自动适配推理工具。

<a id="references"></a>
## 9. 更多说明

- [新机器人部署与SSH密钥登录](docs/NEW_ROBOT_DEPLOYMENT_zh.md)
- [中文实现](INSTRUCTION_zh.md) / [English design](INSTRUCTION.md)
- [300测试与历史](ROBOT300_TESTING_zh.md)
- [本次重构与345五组静止采集验证](docs/validation/20261019_refactor_345.md)
- [345 Action来源与EEF实测](docs/validation/20261019_s3_345.md)
- [S3历史采集记录](docs/validation/20261016_s3_collection.md)
- [遥操作快照来源](teleop_snapshot/README.md)

历史验证只适用于其版本、机器和配置，不代替当前机器的现场验收。
