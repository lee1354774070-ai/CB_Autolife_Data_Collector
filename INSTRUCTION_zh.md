# Collector 实现说明

日常命令及完整环境参数见 [README](README.md)；本文解释实现边界和扩展约束。
[English design](INSTRUCTION.md)。历史实测保留在 `docs/validation/`，不混入当前行为说明。

## 模块与职责

| 模块 | 职责 |
|---|---|
| `start_lerobot_official_collect.sh` | 选择模式，锁定session目录，准备ROS/解释器，管理自身子进程和终端按键 |
| `collector_modes.py` | 无副作用解析四种互斥模式及旧开关冲突 |
| `recorder_config.py` / `launcher_help.py` / `cli_help.py` | 标准库参数校验、环境参数帮助、单参数帮助；不加载ROS/模型 |
| `record_lerobot_official.py` | 单线程ROS采样、图像FIFO、时间匹配、episode状态机与保存协调 |
| `robot_schema.py` / `eef_schema.py` | 关节顺序、物理映射、位姿校验/插值、joint/EEF布局清单 |
| `camera_config.py` / `shm_camera.py` / `time_sync.py` | 相机定义、SHM快照、解码、时间戳与匹配纯函数 |
| `episode_storage.py` | 空间预算、私有暂存、提交journal、回滚和单写入者锁 |
| `dagger/capture_image_writer.py` | 四种模式共用的有界异步写图器；目录名不表示仅供DAgger |
| `collector_control.py` | 非阻塞FIFO、关联回执、摄像头就绪检测及摘要 |
| `vr_collector_control.py` / `vr_reset.py` | 手势、倒计时、异步确认、TTS；已适配V4的受保护复位 |
| `subtask_annotations.py` | 有序边界、逐帧标签和sidecar |
| `dagger/` / `dagger_labels.py` | 宿主/attach、权限仲裁、Thor桥接、来源标签与训练mask |
| `convert_dataset.py` | 在新目录独立投影state/action及对应统计量、mask |
| `hand_camera_producer.py` / `shm_camera_topic_bridge.py` | 可选备用JPEG生产者和SHM到ROS桥；默认复用原厂SHM |
| `tests/` | 无硬件回归、隔离ROS测试和显式运行的真机验证工具 |

`dagger/control/` 是经审核控制基础，`dagger/assets/` 是随版本打包的配置/Web资源。
`teleop_snapshot/` 是已打补丁的审查快照，不能再次当作补丁输入。
`SOURCE.json` 和依赖哈希保留来源；不是可随意删除的临时文件。
`__pycache__/`、`.runtime/` 属于生成物，不提交；运行期间不要删除在用的runtime。

## 配置与启动

普通/VR/子任务使用原厂控制器；DAgger使用独占增量控制副本。
入口先校验模式、开关及任务目录名，再用session锁防止两个启动器竞争FIFO。
数据集另有独立写锁，控制客户端持有从发送到回执的IPC锁；三把锁的作用不同，不能互相替代。

新session先接收完整joint状态，再启动可选相机生产者。已有SHM可提前探测，但不录制。
相机和数据集初始化后才开放按键，不因历史meta存在就认为本次就绪。
普通模式不自动使能、复位、重启硬件；无需模型和Thor。

S2/S3共用main。ROBOT_ID和ROS_DOMAIN_ID选择话题，ROBOT_TYPE只标记新数据集；
编号不能推断运动学型号。已有venv/Conda可通过解释器路径复用，不建立S3专属采集环境。
DAgger另需经审核的DAGGER_ROBOT_PROFILE、URDF/SRDF、控制接口和模型。
DDS覆盖优先级为COLLECT_CYCLONEDDS_URI、CYCLONEDDS_URI、内置本机回环配置。

## 数据契约

policy向量顺序：左臂7、右臂7、左夹爪、右夹爪、可选头3、可选上腰2或完整腰腿4。
基础16维，头+上腰21维，全量23维。两种腰部开关互斥。
关节state/action名称和顺序一致；历史名称仅在物理布局兼容时保留。
控制器q23顺序为腰腿4、左臂7、右臂7、夹爪2、头3，由robot_schema集中转换。

| Action来源 | 语义 |
|---|---|
| `status_target` | 常规三种模式默认；身体取状态包控制目标，夹爪因无独立目标字段取实测位置 |
| `joint` | DAgger必需，取已验证控制器的身体/夹爪目标发布记录；非DAgger仅用于已核实的命令源 |

原厂App可驱动机器人，不代表默认joint命令话题有消息；不能只看发布者数量。
DAgger禁止实测state回退。已移除的15维eef模式不会自动转换为joint。
图像特征是 `observation.images.rgbd_head_color/hand_left/hand_right`，可加rgbd_head_depth。
RGB写入前统一为RGB HWC；depth保留uint16毫米输入，交给LeRobot原生深度编码/量化。

WITH_EEF_STATE/ACTION独立追加14维，不改变joint前缀：左XYZ+XYZW、右XYZ+XYZW。
数据来自原厂current/target位姿报告；米、机器人坐标系、单位四元数，不添加TCP偏移或FK。
只接受完整双臂有限数值与非零四元数。实测位置线性插值，姿态最短弧SLERP；
目标因果保持，不能用实测值或零替代。14维不含夹爪。

meta/collector_schema.json记录名称、索引、单位和来源。
续录校验FPS、feature、深度、joint/EEF布局、子任务及DAgger字段；不能混写不同schema。
存在字段不等于物理语义已标定，S2/S3也不能仅因同维度就混用训练数据。

## 图像与时间同步

共享内存由原厂视觉服务写，Collector只读。备用hand producer只在设备空闲且显式配置时使用。
双手只用hand_left_jpeg/hand_right_jpeg，JPEG解码为BGR再转RGB，不探测旧直接BGR路径。
头部彩色及深度保持原生路径。

读者识别legacy `<qiiiii` 和SHM2协议。SHM2按metadata公布的active slot取当前快照，
不是调用SDK sequential读取，也不遍历所有历史slot。复制前后metadata不一致则拒绝该快照，
下一轮再试；这不能证明未遗漏任何生产者帧，连续性由后续参考帧间隔检查约束。
一次轮询先批量读取metadata、复制新帧，再解码；相同源时间戳不会重复进FIFO。

Unix时间戳归一化后参与匹配；非epoch/不可信遥测时间回退接收时刻。
接收时刻包含轮询与调度延迟，软件时间差不是硬件曝光同步证据。

1. 取参考FIFO内严格晚于上一锚点、已等待匹配窗口的最早帧。
2. 其他相机取最近时间，检查MAX_SYNC_DELTA_SEC及接收年龄。
3. joint/EEF state取两侧端点插值，检查端点年龄与最大跨度。
4. action取参考时刻之前的新鲜目标，不使用未来目标。
5. 完整行加入私有episode，然后移除已消费图像；失败不以跳帧补救。

等待窗口为max(同步误差, state插值跨度)，默认50ms；并非每帧处理必须在30ms内完成。
默认30FPS参考间隔误差上限45%，可接受约18.3到48.3ms。
不能只改数据FPS就假设会降采样硬件源流。FIFO默认每相机16帧，信号缓存每类64条。
启动前参考FIFO清空，其他相机最多保留一帧跨起点邻居；仍检查时效，不输出起录前参考帧。

等待未来state/参考帧只增加waiting ticks；停流、过旧、无因果action、同步超限、
参考间隔异常或录制期FIFO溢出使整条invalid。关闭中的warmup滚动不是有效episode丢帧。

## 写入与故障隔离

写图队列四种模式共用，默认64任务/256MiB原始输入，满时立即失败而非阻塞或丢帧。
DAgger无压缩PNG、其他模式轻压缩，depth TIFF；后台异常回传主录制器。
内存预算不包含FIFO、编码器和所有进程开销。

磁盘监控后台每0.5秒刷新；逐帧只查缓存及累计字节，快照超过2秒不新鲜就停止本条。
默认空闲底线2GiB，另留一份当前条目原始大小供编码；单条预算8GiB/600秒，先到先作废。
不是系统配额，也不保证任何分辨率都能录满600秒。

每条只写 `.collector-work`，从正式数据复制meta而不复制/追加历史视频和Parquet。
保存先等待写图、编码、关闭writer、完成标注并刷新同步日志，再提交：
- fsync新文件和目录，写提交journal；
- 新数据以拒绝覆盖的硬链接发布，备份并切换meta；
- 记录committed后清理，后续清理失败不撤销已提交数据。

提交前失败恢复旧meta，仅删除与暂存inode一致的新链接。
无法确认结果时停止会话，不自动重试/丢弃；重启按journal恢复。
遗留未提交暂存改名为collector-interrupted目录供检查，不自动删除。
要求同一文件系统支持hardlink、rename、flock和fsync；不支持时拒绝。
meta切换期间不支持并发训练或编辑，本机制不保证任意掉电/硬盘损坏场景。

失败先在内存标invalid，即使第一帧尚未计数。D/Y丢弃仅当前暂存。
同步日志保持缓冲写，避免每帧flush；起录和提交前flush发现错误则拒绝成功，
错误锁存到会话重启，防止下一条悄悄无日志采集。已提交后诊断失败不撤销已有数据。

正常Q/EOF/中断先停止自身VR helper，再等待recorder完成，不因固定30秒强杀编码。
DAgger的外层控制栈另有安全停机流程；未确认轨迹不自动保存。
不关原厂服务，不影响其他任务进程；采集仍需用户明确开始下一条。

## 控制与子任务

FIFO只接受单条原子消息，拒绝普通文件、符号链接、多行注入及超过PIPE_BUF的写入。
保存回执携带request_id，只有匹配结果才算确认；忙碌拒绝新命令，超时视为未知，不自动重试。
状态以原子JSON替换；invalid事件与回执均失败时仍保持内存中的invalid状态。

原厂VR：双Grip+面键释放触发，A开始、B保存、Y丢弃；X原厂复位未接入。
格式错误、失联、多键或松Grip取消手势。倒计时非阻塞，要求新鲜输入。
异步worker等待回执，ROS继续收消息；回执缓存读盘不在按键回调。
语音走原厂TTS，未接入原厂振动协议，也没有原厂App内进度叠加。

子任务短B标记、长B松开提前保存，最后标记保存。边界为已接受帧数的exclusive end。
中间标记只追加边界、后台写小回执，不遍历图片、编码、暂停或重置FIFO。
保存时回填subtask_index与sidecar。未确认尾段-1、needs_review=true。
task_index始终是总任务；subtask_index只在本episode计划内解释。
stock LeRobot不会自动将该字段转成子任务语言，chunk跨边界策略需训练侧决定。

<a id="dagger"></a>
## DAgger

owned启动单一兼容宿主；attach只读取新鲜状态、调用其操作接口，不产生第二套发布器。
绑定host/session、机器人、DDS域、数据FIFO；请求有ID和有效期，未知结果不重放。
attach退出仅断开。未改造旧栈不能热替换；运行副本经哈希校验生成，同事源码不修改。

控制预检检查真实move/joint/gripper输入与独占服务，不将target EEF报告误认为运动输入。
硬件使能ACK不等于就绪，需新鲜ARMED、enabled、ready并且非pending后才放行模型。
A先等recorder关联回执；首帧前有最长30秒的新鲜动作/来源屏障，之后不放宽同步。

任一Grip新按下立刻换authority epoch，本地撤销旧模型输出，不等HTTP。
唯一V4出口在锁内清旧目标/在途IK，增量mapper重新锚定实测关节及当前手柄。
旧session/epoch的迟到模型、人工目标和释放指令拒绝；接管不自动还权。
V4关节/碰撞/轨迹/watchdog保护保留，不能用较小目标跟随误差推断动作跳变。

只有V4出口发布硬件指令，并发出含原始时间/来源的controller_output回执。
代理动作保留原时间和权限，旧保持目标不会因刷新代理时间变成人工示范。
B确认任务成功并撤销输出后保存；Y丢弃；X只复位，有待处理数据或忙碌则拒绝。
操作员确认的成功结果写入annotations/dagger/episode_XXXXXX.json，与数据事务一起提交：
task_success=true，success_source为autonomous（本条从未接管）或human_intervention（曾接管）。
原有逐帧纠错掩码不改；失败/丢弃不发布成功标注，历史无标注条目保持未知。
结果不确定时保留原请求ID核对，不重复save，也不在保存中复位。

| 字段 | 训练语义 |
|---|---|
| dagger.control_source | 0模型、1人工有效控制、2保持/交接 |
| dagger.is_intervention / intervention_id | 已接管标志和人工段号，不是loss mask |
| dagger.authority_epoch | 控制权代次 |
| dagger.train_mask | 整帧AND：身体与夹爪都满足人工来源才为1 |
| dagger.action_train_mask | 与action同序，分身体/夹爪验证；追加EEF的14维固定0 |
| dagger.anchor_timestamp_ns | 图像锚点 |
| dagger.arm_command_timestamp_ns / gripper_command_timestamp_ns | 原始目标时间，不能用代理刷新时间替代 |

纠错训练必须对chunk每个时间步、每个维度应用mask；只有部分维度纠错时不能先按旧标量mask过滤。
模型-only条目可保存但mask全0。保存、提交回执不代表到位或任务成功。

当前只支持Thor GR00T baseline/frame的controller_submission_receipts协议。
controller_ack验证目标前缀/digest，只证明提交，不推进verifier历史。客户端不改动作。
phase_aware夹爪策略只适用于完整任务文本：
`Pick the laundry bag.` 与 `Place the laundry bag in the upper compartment of the delivery robot.`。
抓取达到120度后锁持，放置预测不大于60度释放；其他任务先核对策略，不直接套用。

状态文件20Hz后台读取，超过1秒不更新撤销输出；DDS图缓存5Hz刷新，过期600ms拒绝输出。
trace有界2048队列，诊断丢失统计单独记录，不影响训练标签。
仅接管触发WebXR振动，依赖头显支持；模型仍是串行chunk，未实现RTC，不能承诺零卡顿。

## 离线转换

convert_dataset以源数据共享锁阻止并发续录，拒绝未完事务、符号链接和已有/嵌套输出。
按批投影Parquet、Arrow/HF布局、全局及逐episode统计量；count不裁剪。
action mask按同索引投影；任务、标注、帧序与视频偏移不变，视频独立复制。
全部fsync后用Linux RENAME_NOREPLACE发布新目录，失败清理仅自己的私有输出。
这是列选择，不是重采样、IK/FK、专家重标注或相对动作计算。

## 验证与维护

```bash
PYTHONPATH=. /path/to/lerobot/python -B -m unittest discover -s tests -p 'test_*.py'
node tests/test_feedback.js
bash -n start_lerobot_official_collect.sh
```

ROS测试只在独立domain211运行；真机录制用独立目录标记NOT TRAINING DATA。
软件回归、真实传感器静止录制、现场VR/DAgger运动是不同证据层级，不能互相替代。
修改schema、资源哈希、存储事务或控制权契约必须补对应回归。
不要删安全测试或降低同步/存储阈值来换取表面通过。
