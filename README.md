# 云蝶 AutoLife 数据采集器

> **S3 适配分支：`codex/s3-collector`**。347 已完成普通、单人 VR、子任务的录制按键与数据格式验证。运动采集和长时负载待验收；DAgger 仅通过无动作测试，尚不能用于 S3 真机控制。详见 [S3 使用与验收范围](S3_TESTING_zh.md)。

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

### 1.1 录制内容：深度、头部、腰部

默认录制 **14维手臂 + 2维夹爪 + 三路 RGB**。头部、腰部和深度均为可选扩展。

| 参数 | 通用默认值 | 功能及限制 |
|---|---:|---|
| `WITH_DEPTH` | `0` | `1` 增加 `rgbd_head_depth`，不增加关节维度。保存原生深度视频，不是将深度伪彩色当 RGB。需要相应 SHM 流及支持深度编码的 LeRobot。 |
| `WITH_HEAD` | `0` | `1` 追加 `neck_roll, neck_pitch, neck_yaw`，共3维。 |
| `WITH_UPPER_WAIST` | `0` | `1` 仅追加 `waist_pitch, waist_yaw`，即腰部俯仰、旋转；**不包含腿部升降相关关节**。 |
| `WITH_WAIST` | `0` | `1` 追加 `leg_ankle, leg_knee, waist_pitch, waist_yaw`，共4维，包含腿部关节。 |
| `ACTION_MODE` | `status_target` | `status_target` 从状态包读取目标，适合普通遥操作采集；`joint` 从关节/夹爪命令话题读取；`eef` 为15维末端目标调试格式，不能当作同维关节 action 使用。DAgger **必须为 `joint`**，由启动器设置。 |

`WITH_UPPER_WAIST=1` 与 `WITH_WAIST=1` **不能同时出现**。
开关控制的是保存哪些维度，不是禁止机器人其余关节运动。

`status_target` 中，手臂/头/腰使用状态包中的控制目标；状态包没有单独的夹爪目标，
两维夹爪沿用实测位置作为标签。若需要明确的夹爪命令标签，应先确认命令话题并使用 `joint`。

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

每个组合都可另加 `WITH_DEPTH=0/1`。**当前 DAgger 只接受21维组合**，不是23维。
新建数据集后，改变关节、相机、深度、action格式或标注字段时，应换新目录，不能直接追加到旧 schema。

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
| `MAX_STATE_AGE_SEC` | `0.15` 秒 | 关节数据接收后的新鲜度限制；限制插值端点过旧及关节流停滞。 |
| `MAX_STATE_INTERPOLATION_GAP_SEC` | `0.05` 秒 | 用于关节插值的前、后两个状态样本之间最大跨度。扩大后可跨更长缺口插值，但时间准确性下降。 |
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
| `STATE_READY_TIMEOUT_SEC` | 10秒 | 等待首个完整关节状态包；没有完整包，不开始相机轮询流程。 |
| `CAMERA_WARMUP_SEC` | 10秒 | 统一启动器传给录制器的相机准备等待时间。直接运行Python的默认值不同，以该入口帮助为准。 |
| `CAMERA_DETECTION_TIMEOUT_SEC` | 15秒 | 外层启动器等待相机/数据集初始化结果。提高相机warmup时，应同时给这个外层等待留足时间。 |
| `CONTROL_ACK_TIMEOUT_SEC` | 300秒 | 等待开始、标记、保存、丢弃等命令的真实回执；不是图像同步阈值。结果未知时先查日志，不盲目重复提交。 |

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
| `HG_DAGGER_RECORDER_CPUS` | `12-19` | 仅DAgger：录制器CPU亲和性，另用nice=5。是300的配置，不可直接套到其他CPU布局。 |

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

除注明外，仅对 `COLLECTOR_MODE=dagger` 生效。当前适配机器人300、ROS domain0、
21维头部+上腰、三路RGB和兼容Thor GR00T baseline/frame协议。

| 参数 | 默认值 | 功能及注意事项 |
|---|---|---|
| `DAGGER_BACKEND` | `owned` | `owned` 启动并管理一套独占宿主；`attach` 仅连接已有**兼容**宿主，不创建/替换控制器。详见模式示例。 |
| `DAGGER_PUBLISH` | `0` | 0不发布硬件动作；1允许用户按键触发推理控制/复位。设1本身不会自动开始，但A/C、X/R可能产生真实运动。 |
| `DAGGER_SERVER_URL` | 统一入口无默认，必填 | Thor服务URL，例如 `http://192.168.8.224:8777`。IP/端口是部署示例，以当前服务为准；不支持旧SOMA/Outcome协议。 |
| `DAGGER_TOKEN_FILE` | `/home/ubuntu/.config/autolife_hg_dagger/groot_server.token` | Thor认证token文件，须与服务匹配；不要把token写进URL或提交到Git。 |
| `GROOT_REMOTE_TOKEN` | 未设置 | 兼容的环境变量认证入口，非空时预检和模型桥接均优先使用它而不是token文件。建议统一使用文件，避免Shell历史泄漏或旧环境变量覆盖凭据。 |
| `DAGGER_DEPENDENCY_ROOT` | `/home/ubuntu/ros2_ws/src` | 包含已审核 `openarmx_teleop_vr_306_v4` 的目录。源文件须通过哈希检查，启动器在 `.runtime/` 副本中应用补丁，不改同事原代码。 |
| `DAGGER_TOOLS_ROOT` | 集合仓库自动取根目录；独立库为 `/home/ubuntu/Autolife_VLA_Tools` | 用于加载我们自己的部署客户端。必须是含 `deploy/` 的完整Tools目录，不是模型目录或Collector子目录。 |
| `DAGGER_WEB_PORT` | `8447` | V4扩展网页的HTTPS端口，1～65535。改变端口不能解决两套控制器争用关节的问题。 |
| `DAGGER_ROS_SETUP` | `/home/ubuntu/ros2_ws/install/setup.bash` | 机器人ROS工作区环境脚本。 |
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
| `ROBOT_ID` | 主机名末尾数字，否则283 | 生成机器人话题后缀；在 `autolife-robot-300` 上为300。不要混淆IP和机器人编号。DAgger只接受300。 |
| `ROS_DOMAIN_ID` | `0` | ROS通信域，必须与机器人服务一致；不是机器人编号。DAgger只接受0。 |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | ROS中间件；本工具当前使用CycloneDDS本机通信配置。 |
| `ROBOT_PY` | `/home/ubuntu/miniconda3/envs/robot_env/bin/python` | 相机生产/桥接、普通VR辅助程序解释器。 |
| `LEROBOT_PY` | `/home/ubuntu/miniconda3/envs/lerobot/bin/python` | LeRobot录制器解释器。仅改解释器路径不等于完成ROS/native library环境迁移。 |
| `CAMERA_ONLY` | `0` | 1为头部RGB链路测试，关闭手相机生产并请求state作为action回退；仍需关节数据。若要测试实际回退路径，应配合 `ACTION_MODE=joint`，并清除 `WITH_DEPTH` 等额外开关。不是专家数据采集配置。 |
| `FALLBACK_ACTION_TO_STATE` | `0` | 缺少joint目标时允许用实测state替代action，仅用于链路调试，不应当作专家目标；DAgger必须保持0。 |
| `OMP_NUM_THREADS` / `MKL_NUM_THREADS` / `OPENBLAS_NUM_THREADS` | 通用入口未设置时为1 | 限制底层数学库线程，避免小计算创建过多线程；DAgger录制隔离脚本会强制这些库为1线程。 |

通用录制子进程使用启动脚本内的本机回环DDS配置，不要以为仅导出 `CYCLONEDDS_URI`
就能把所有节点改成跨机通信。DAgger控制侧接受 `COLLECT_CYCLONEDDS_URI` / `CYCLONEDDS_URI`，
但自定义后仍必须与录制器及机器人服务互通；默认保持不改。

查看帮助不会启动采集，适合核对安装版本：

```bash
bash start_lerobot_official_collect.sh --help
bash start_lerobot_official_collect.sh WITH_DEPTH --help
bash start_lerobot_official_collect.sh MAX_SYNC_DELTA_SEC --help
bash start_lerobot_official_collect.sh DAGGER_BACKEND --help

# 录制器的底层参数使用 --kebab-case；不等同于启动脚本的环境变量。
/home/ubuntu/miniconda3/envs/lerobot/bin/python record_lerobot_official.py --help
/home/ubuntu/miniconda3/envs/lerobot/bin/python record_lerobot_official.py --max-sync-delta-sec --help
```

`--camera`、`--depth-camera`、`--duration` 等是录制器直接入口参数，
统一Shell启动器不会自动转发任意Python参数，不要直接拼在任务名后。
`--check` / `--serve` 是DAgger入口选项，不是普通三种模式的通用选项。

<a id="setup"></a>
## 2. 环境准备

需已配置 ROS 2 Jazzy、机器人关节反馈和相机SHM服务。
`robot_env` 提供ROS/OpenCV等依赖；`lerobot` 提供 **LeRobot 0.6+ DatasetWriter/resume接口**、
PyArrow及视频编码依赖。深度还需高位深视频支持。
本仓库不会自动安装、启动或重启原厂硬件服务。

独立仓库安装：

```bash
cd /home/ubuntu
git clone https://github.com/lee1354774070-ai/CB_Autolife_Data_Collector.git
cd /home/ubuntu/CB_Autolife_Data_Collector
```

集合仓库用户改为进入 `/home/ubuntu/Autolife_VLA_Tools/lerobot_data_collector`。
不要同时从两个副本启动。启动首行的 `AutoLife Collector | mode=... | 实际代码目录`
可以确认当前运行的是哪个副本。

以下为**已配置好的300机器人**的新终端准备命令，各模式示例均在此之后执行：

```bash
cd /home/ubuntu/CB_Autolife_Data_Collector
source scripts/source_ros_env.sh
set +u
unset VR_CONTROL SUBTASKS_JSON ACTION_MODE

export ROBOT_ID=300 ROS_DOMAIN_ID=0
export OUTPUT_BASE_DIR=/home/ubuntu/nas
export START_HAND_PRODUCER=0 IMAGE_SOURCE=shm
export COLLECT_FPS=30 SYNC_REFERENCE_CAMERA=hand_left
export MAX_SYNC_DELTA_SEC=0.03 SYNC_IMAGE_BUFFER_SIZE=16
```

`set +u` 关闭当前Shell的“未定义变量报错”，避免ROS环境脚本触发unbound variable，
与模型开关、录制或电机使能无关。环境辅助脚本预设300/domain0，其他机器人在source后
显式设置自己的 `ROBOT_ID` 和通信域；当前DAgger适配不能直接用于其他机器人。

<a id="modes"></a>
## 3. 四种模式使用哪套遥操作

**采集按键控制与机器人遥操作不是同一个东西。** `vr` 表示Collector由手柄操作，
不表示它会启动或切换一套V4遥操作。

| Collector模式 | 实际使用的遥操作版本 | 头显入口 | Collector负责什么 |
|---|---|---|---|
| `keyboard` | **Autolife原厂遥操作** | 原厂独立App，填写机器人服务地址 | 终端控制开始/保存/丢弃，原厂负责手臂遥操作。 |
| `vr` | **同一套Autolife原厂遥操作** | 同上，不用8447 | 监听原厂手柄话题，增加单人录制按键及语音；不替换原厂控制器。 |
| `subtask` | **同一套Autolife原厂遥操作** | 同上，不用8447 | 在单人录制基础上标记有序子任务，手臂仍由原厂控制。 |
| `dagger` | **适配后的 `openarmx_teleop_vr_306_v4` 增量遥操作运行副本** | 我们的HTTPS/WebXR扩展页，默认8447 | 管理模型/人工权限、增量接管、录制和回执，不使用原厂遥操作执行这条任务。 |

300原厂App的连接地址示例为 `192.168.10.2:3000`；Wi-Fi网口当前为
`192.168.8.122:3000`。按头显可达的网络选择，IP会变化。
**3000是原厂App的服务地址；8447是DAgger浏览器页面，两者不能混用。**

前三种模式保持原厂遥操作运行，不要另开V4控制器或8446/8447来“补充”控制。
DAgger `owned` 启动前先正常退出其他手臂控制任务；原厂管理界面可保留，
但其关节控制模块可能占用发布端点。冲突时核对具体任务，不要清错或重启ARM服务绕过检查。

`DAGGER_BACKEND=attach` 只接入已有的适配宿主：
**不是兼容任意原厂App，也不是直接接入未修改的同事版本。**
`teleop_snapshot/` 是审查用的已打补丁快照，不应充当启动器要再次打补丁的原始依赖目录。

<a id="keyboard"></a>
## 4. 键盘数采

**遥操作：Autolife原厂App。录制控制：当前终端。**
先完成[环境准备](#setup)，确认原厂遥操作、关节反馈和三路RGB正常。

```bash
unset VR_CONTROL SUBTASKS_JSON
COLLECTOR_MODE=keyboard \
WITH_HEAD=0 WITH_UPPER_WAIST=0 WITH_WAIST=0 WITH_DEPTH=0 \
ACTION_MODE=status_target COLLECT_FPS=30 \
TASK_TEXT="Pick up the water bottle." \
bash start_lerobot_official_collect.sh pick_water_rgb
```

这是16维、三路RGB、不带深度的示例。等待启动器打印有效相机和交互提示后：

| 终端按键 | 功能 |
|---|---|
| Enter | 开始新episode，再进行遥操作。 |
| S | 保存当前条目并暂停，等待成功回执和累计条数。 |
| D | 丢弃当前条目并暂停，不删除以前保存的数据。 |
| Q | 保存有效待处理数据并正常退出；建议先S确认，再Q收尾。 |
| Ctrl+C | 中断并尝试清理；不要将它当作保存成功保证。 |

一条完成后按S，确认成功再布置下一条场景，按Enter继续。启动后不会自动开始下一条。

### 常见四种录制配置

以下分别使用独立目录，避免不同schema混写。每次只运行一组，退出后再测试下一组。
总任务以 `TASK_TEXT` 显式设置；这些关节/深度组合也适用于 `vr / subtask`。

```bash
# 16维：手臂+夹爪，不带深度
COLLECTOR_MODE=keyboard WITH_HEAD=0 WITH_UPPER_WAIST=0 WITH_WAIST=0 WITH_DEPTH=0 \
TASK_TEXT="Pick up the water bottle." \
bash start_lerobot_official_collect.sh bottle_16d_rgb

# 16维：手臂+夹爪，带深度
COLLECTOR_MODE=keyboard WITH_HEAD=0 WITH_UPPER_WAIST=0 WITH_WAIST=0 WITH_DEPTH=1 \
TASK_TEXT="Pick up the water bottle." \
bash start_lerobot_official_collect.sh bottle_16d_rgbd

# 23维：全量关节，不带深度
COLLECTOR_MODE=keyboard WITH_HEAD=1 WITH_UPPER_WAIST=0 WITH_WAIST=1 WITH_DEPTH=0 \
TASK_TEXT="Pick up the water bottle." \
bash start_lerobot_official_collect.sh bottle_23d_rgb

# 23维：全量关节，带深度
COLLECTOR_MODE=keyboard WITH_HEAD=1 WITH_UPPER_WAIST=0 WITH_WAIST=1 WITH_DEPTH=1 \
TASK_TEXT="Pick up the water bottle." \
bash start_lerobot_official_collect.sh bottle_23d_rgbd
```

只要腰部俯仰/旋转、不需要腿部时，将最后两组改为
`WITH_HEAD=1 WITH_UPPER_WAIST=1 WITH_WAIST=0`，得到21维，并使用新的目录名。

<a id="vr"></a>
## 5. 单人 VR 数采

**遥操作：Autolife原厂App，不是V4。录制控制：双Grip+面键。**
头显进入原厂沉浸式遥操作后，Collector读取 `/control_topic_0_300`。
停在菜单时即使话题有消息，按键也可能始终是松开状态。

```bash
unset VR_CONTROL SUBTASKS_JSON
COLLECTOR_MODE=vr \
WITH_HEAD=1 WITH_UPPER_WAIST=1 WITH_WAIST=0 WITH_DEPTH=0 \
ACTION_MODE=status_target COLLECT_FPS=30 \
VR_INPUT_TOPIC=/control_topic_0_300 VR_SPEECH=1 VR_START_DELAY_SEC=3 \
TASK_TEXT="Put the towel in the basket." \
bash start_lerobot_official_collect.sh towel_vr_21d_rgb
```

这是21维、三路RGB的示例；需要深度时设置 `WITH_DEPTH=1` 并换相应数据目录。
启动后应看到 `[VR] Listening: /control_topic_0_300`。

保持 **GL+GR 两个握持键按住**，点按并松开面键；面键松开时双Grip仍需按住：

| 面键 | 功能 |
|---|---|
| A | 倒计时后请求开始；收到录制回执后提示开始。 |
| B | 保存当前条目；先提示保存中，成功后播报条数和帧数。 |
| Y | 丢弃当前条目，不复位、不退出。 |
| X | 原厂复位接口尚未接入，使用原厂自己的复位功能；不会启动另一套控制器补偿。 |

B/Y可取消倒计时。等待保存/丢弃回执后再开始下一条。
语音通过机器人现有TTS服务播放；实际音量/扬声器需现场确认。
终端Enter/S/D/Q仍可用，退出时先保存或丢弃，再Q。
当前原厂独立App没有Collector的可视进度面板，不要以为更新Collector就会出现HUD。

<a id="subtask"></a>
## 6. 子任务数采

**遥操作：Autolife原厂App，与单人数采相同。**
一条episode保持一个总任务，内部连续分段标注；不会在每个子任务间停止录制。

```bash
unset VR_CONTROL SUBTASKS_JSON
COLLECTOR_MODE=subtask \
WITH_HEAD=1 WITH_UPPER_WAIST=1 WITH_WAIST=0 WITH_DEPTH=0 \
ACTION_MODE=status_target COLLECT_FPS=30 \
VR_INPUT_TOPIC=/control_topic_0_300 VR_SPEECH=1 VR_START_DELAY_SEC=3 \
VR_A_LONG_PRESS_SEC=1.0 \
TASK_TEXT="Put the towel in the basket." \
SUBTASKS_JSON='["右手拿起毛巾","双手交接毛巾","左手放下毛巾"]' \
bash start_lerobot_official_collect.sh towel_subtask_21d_rgb
```

启动后应看到 `VR annotation` 和原厂输入话题。仍需GL+GR；短按/长按均在 **B松开时**判定：

| 操作 | 功能 |
|---|---|
| A | 倒计时后开始整条录制。 |
| 第一次短B | 将已采集片段标为“右手拿起毛巾”，继续第二阶段，确认后播报“下一步”。 |
| 第二次短B | 标记“双手交接毛巾”，继续第三阶段。 |
| 最后阶段短B | 标记“左手放下毛巾”，自动保存整条；等待保存成功回执。 |
| 长B达到阈值后松开 | 提前保存；不同时执行短B标记，未确认尾段留待人工审核。 |
| Y | 丢弃整条；下一条从第一个阶段开始。 |
| X | 与普通VR相同，原厂复位未接入。 |
| 终端N / S / D / Q | 标记当前阶段 / 提前保存 / 丢弃 / 收尾退出。 |

忘记标记或没有完成全部阶段时，可长B提前保存，不要为补标而连续点按伪造边界。
标记以**录制器处理请求时已经写入的帧**为边界，不是精确的手柄硬件按下时间。

数据中 `task` 仍为总任务；`subtask_index=0/1/2` 表示已确认阶段，未确认尾段为 `-1`。
阶段文本、帧边界和 `needs_review` 保存在
`dataset/annotations/subtasks/episode_*.json`。
此标注不意味着训练器自动使用子任务损失或自动将子任务文本替换总任务。

开启/关闭标注或修改计划时使用新数据目录，不将不同计划混进同一旧目录。
原厂App没有显示进度列表；当前以“下一步/已保存/标注未完成”等语音和终端回执确认。

<a id="dagger"></a>
## 7. DAgger 纠错数采

**遥操作：经过适配的V4增量控制副本，不是原厂App。**
`owned` 会启动一套独占宿主；头显使用浏览器进入8447扩展网页。
默认不发布硬件动作。当前仅支持300/domain0、21维、三路RGB，depth可选但不送入当前模型。

### 7.1 准备与只读预检

先正常结束其他手臂控制任务，并确认Thor服务已经由维护人员启动。
Collector不会因为端口不同就允许两个控制器同时控制机器人。
依赖或哈希校验失败时检查版本，不使用跳过校验方式继续。

```bash
unset VR_CONTROL SUBTASKS_JSON ACTION_MODE
export COLLECTOR_MODE=dagger DAGGER_BACKEND=owned
export ROBOT_ID=300 ROS_DOMAIN_ID=0
export WITH_HEAD=1 WITH_UPPER_WAIST=1 WITH_WAIST=0 WITH_DEPTH=0
export COLLECT_FPS=30 IMAGE_POLL_FPS=60
export DAGGER_SERVER_URL=http://192.168.8.224:8777
export DAGGER_TOKEN_FILE=/home/ubuntu/.config/autolife_hg_dagger/groot_server.token
export DAGGER_DEPENDENCY_ROOT=/home/ubuntu/ros2_ws/src
export DAGGER_TOOLS_ROOT=/home/ubuntu/Autolife_VLA_Tools
export DAGGER_WEB_PORT=8447
export OUTPUT_BASE_DIR=/home/ubuntu/nas/dagger
export TASK_TEXT='Pick the laundry bag.'

# 只读检查依赖、ROS图和Thor协议；不启动控制器、录制器或运动。
DAGGER_PUBLISH=0 bash start_lerobot_official_collect.sh dagger_check --check
```

### 7.2 无动作与真实控制

```bash
# 启动无硬件发布的待命会话，检查完在这个终端用Q退出。
DAGGER_PUBLISH=0 bash start_lerobot_official_collect.sh dagger_dryrun
```

无动作模式不等于实机任务验证。确认上述会话已退出、机器人周围安全、急停可用后，
才启动真实控制：

```bash
DAGGER_PUBLISH=1 bash start_lerobot_official_collect.sh dagger_live_01
```

头显浏览器打开 `https://<机器人当前IP>:8447`，300当前示例为
`https://192.168.8.122:8447`，不是原厂App中的3000地址。
进入或重连保持待命，必须主动按A才开始。

| VR操作 | 功能 |
|---|---|
| A | 确认录制就绪后开始模型推理，不使用普通VR的倒计时。 |
| 任一GL/GR | 立即选择人工控制权限；握持时增量遥操作，模型请求在后台退出。 |
| B | 保存当前有效条目，不自动复位。 |
| Y | 丢弃当前条目，不自动复位。 |
| X | 仅全身复位并张开夹爪；仍有未处理数据时拒绝。会真实运动，先保存/丢弃并释放Grip。 |

面键无需双Grip组合；Grip专门用于接管。接管后由人完成本条，不自动交还模型；
下一条再次主动按A。只有接管振动，其余状态/错误显示在扩展头显界面。
终端键位是 **C开始、A保存、X/D丢弃、R复位、Q退出**，
与普通采集的Enter/S/D不同。

DAgger退出会丢弃未确认条目；想保留时先B保存并确认。
保存结果未知时先查状态，不重复提交。DAgger不是对子任务标注模式的叠加开关。

### 7.3 接入已有兼容宿主

只有已经运行我们适配后的宿主时才能用 `attach`；
它不会停止同事的程序，也不会将未适配的程序自动变成兼容版本。
`OUTPUT_BASE_DIR`、任务目录、任务文本、depth和硬件发布权限必须与宿主匹配。

另开终端并先完成环境准备，以下示例连接上面允许真实控制的 `dagger_live_01`：

```bash
unset VR_CONTROL SUBTASKS_JSON ACTION_MODE
export COLLECTOR_MODE=dagger DAGGER_BACKEND=attach
export ROBOT_ID=300 ROS_DOMAIN_ID=0
export OUTPUT_BASE_DIR=/home/ubuntu/nas/dagger
export TASK_TEXT='Pick the laundry bag.'
export WITH_DEPTH=0 DAGGER_PUBLISH=1

# 只检查匹配关系，不发控制指令。
bash start_lerobot_official_collect.sh dagger_live_01 --check

# 交互接入；C/A/X或D/R的含义与owned终端一致。
bash start_lerobot_official_collect.sh dagger_live_01
```

attach终端的 **Q只断开本终端，宿主和VR继续运行**，不会退出整套DAgger。
不能通过attach修改已有宿主的帧率、关节维度或模型地址；需按正常流程结束后重新配置宿主。
`--serve` 仅供owned后台宿主使用，没有交互终端，不要与attach组合。

### 7.4 便捷入口、桌面与Thor限制

`start_dagger.sh` 使用同一套后端，但有[前文列出的300预设](#parameters)。
例如明确关闭其默认depth并做只读检查：

```bash
WITH_DEPTH=0 DAGGER_PUBLISH=0 bash start_dagger.sh dagger_quick --check
```

桌面启动器：

```bash
DAGGER_PUBLISH=1 bash scripts/open_dagger_launcher.sh
```

打开窗口本身不开始推理，但设置1允许后续按钮触发运动。省略时默认无动作模式；
已安装桌面快捷方式可能预设为1，应核对启动提示。
桌面Shift+A开始、Shift+B保存、Shift+X复位、Shift+Y丢弃，退出前先处理当前数据。
该窗口用于DAgger，不是原厂单人采集或子任务采集入口。

Thor必须提供 `controller_submission_receipts=true`。
当前阶段夹爪策略 `phase_aware` 仅接受以下完整任务文本：
- `Pick the laundry bag.`
- `Place the laundry bag in the upper compartment of the delivery robot.`

该策略抓取实测达到120°后锁持，放置明确预测≤60°时释放，左右独立判断。
动作修改发生在服务端digest前，机器人端不改写模型动作或ACK前缀；
`/controller_ack` 只证明提交目标，不证明电机到位。
新任务需先核对模型和夹爪策略，不能仅替换文本就宣称支持。

`tools/start_thor_baseline.sh` 会启动Thor服务，`GROOT_GRIPPER_PHASE_AWARE=0`
可选择连续夹爪模式。这属于服务维护操作，不是每次采集都要运行的步骤。

<a id="data"></a>
## 8. 续录、输出和故障处理

### 8.1 保存位置与续录

```text
<OUTPUT_BASE_DIR>/<task_name>/
├── dataset/
│   ├── meta/
│   ├── data/
│   ├── videos/
│   ├── annotations/subtasks/    # 仅启用子任务时
│   └── sync_log.jsonl
└── logs/
    ├── 1.record_lerobot_official.log
    └── 1.vr_control.log         # 单人/子任务辅助程序启用时
```

停止后用相同目录和兼容配置启动可续录，episode编号接续。
日志按启动次数递增。同一数据根目录不能同时运行两个采集器；
不要并发编辑/训练正在发布的数据集，也不要在运行中删除目录或日志。

`task_name` 决定目录，`TASK_TEXT` 决定任务标签。不同总任务要保存在同一兼容数据集中时，
可**串行**复用同一个目录名、改变 `TASK_TEXT`，由LeRobot的 `task_index` 区分。
例如普通模式完成“拿瓶子”后先S、Q并等待完全退出，再运行“放瓶子”：

```bash
COLLECTOR_MODE=keyboard WITH_HEAD=0 WITH_UPPER_WAIST=0 WITH_WAIST=0 WITH_DEPTH=0 \
TASK_TEXT="Pick up the water bottle." \
bash start_lerobot_official_collect.sh water_tasks

# 上一次已保存、Q退出后再执行；中间导航不录入这两个操作episode。
COLLECTOR_MODE=keyboard WITH_HEAD=0 WITH_UPPER_WAIST=0 WITH_WAIST=0 WITH_DEPTH=0 \
TASK_TEXT="Put the water bottle in the box." \
bash start_lerobot_official_collect.sh water_tasks
```

以上要求帧率、schema、action模式和相机配置兼容。子任务计划不同应分目录，
不要把这条总任务续录示例套用于混合不同标注计划。

每条episode先写入 `dataset/.collector-work/`，编码及文件关闭成功后再发布到正式目录。
新episode使用新的Parquet/视频文件，不追加覆盖已提交的旧文件。
LeRobot按元数据读取连续episode，**无需手工拼接MP4**。

DAgger保存整条模型和人工过程，附加 `dagger.control_source`、`dagger.train_mask`、
会话及接管标记。训练纠错数据时优先筛选 `dagger.train_mask=1`；
模型段/保持段不冒充专家标签。工具不自动训练或上传数据。

<a id="recovery"></a>
### 8.2 常见报错与恢复

| 现象 | 优先检查 | 不应采取的做法 |
|---|---|---|
| `Recorder exited before camera initialization completed` | 读取本次 `logs/<序号>.record_lerobot_official.log` 的最后异常；检查完整状态包、所选相机、库版本和数据schema。 | 只根据启动器这一行判断是“相机损坏”。 |
| `missing_camera` / `stale_camera` / `unsynchronized_camera` | 查看 `sync_log.jsonl` 中具体相机、接收年龄、时间差；检查源流更新及负载。 | 反复提高同步容差或降低所需相机数掩盖故障。 |
| `image_buffer_overflow` | 图像FIFO积压、处理吞吐和源时间戳更新；与写图队列区分。 | 把FIFO无限增大，或静默丢帧继续保存。 |
| 存储限额、写图队列满、写入/编码失败 | 剩余磁盘、当前episode预算、写图积压及编码日志。 | 删除正在使用的暂存文件、重复提交未知结果的保存。 |
| 手柄按键不生效 | 原厂沉浸式遥操是否激活，输入话题是否正确，面键松开时是否仍按着GL+GR。 | 普通模式临时启动第二套V4控制器。 |
| 语音没有声音 | `VR_SPEECH`、原厂TTS服务和实际输出声卡/音量。 | 把日志中“已发送TTS”当作现场已经听到。 |
| DAgger依赖/命令发布者冲突 | 依赖哈希及正在运行的控制任务，按正常流程退出占用者。 | 跳过校验、清错、重启ARM服务来绕过冲突。 |

出现 `EPISODE INVALID` 后，当前条目不算保存成功，已保存数据保留。
先处理故障，再按普通键盘D或VR的Y丢弃并确认；DAgger使用其专用丢弃键位。
只有收到可继续的回执才开始下一条；要求退出重启或结果未知时先核查日志。

异常退出后，重启先检查保存事务；未提交暂存可能保留为
`.collector-interrupted-<时间戳>`，不计为已保存数据。
确认无需恢复且采集完全退出后再清理；不能在运行中删除
`.collector-work` 或 `.collector-commit.json`。
容量预算不是操作系统磁盘配额，其他进程仍可能抢占空间。

<a id="references"></a>
## 9. 更多说明

- [实现与数据格式（中文）](INSTRUCTION_zh.md) / [Technical details](INSTRUCTION.md)
- [VR语音、文字及振动反馈边界](VR_FEEDBACK.md)
- [300测试与验收流程](ROBOT300_TESTING_zh.md)
- [S3 环境适配、测试与使用](S3_TESTING_zh.md)
- [存储保护测试记录](docs/validation/20261012_storage_guards.md)
- [已适配V4源码快照](teleop_snapshot/README.md)

本地测试、模拟回执、无动作预检和真实运动验收不是同一层证据。
现场长时间稳定性、极端负载及硬件断线应单独验证，不能用一次预检通过替代。
