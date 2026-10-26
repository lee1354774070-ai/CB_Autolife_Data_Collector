# 新机器人部署与 SSH 密钥登录

本文说明如何把 Collector 部署到一台已有原厂运行环境的机器人。
以 **S3-347、`192.168.8.28`、Linux 用户 `ubuntu`** 为例；换机器时修改 IP、机器人编号和型号。
不要把机器编号当成型号，也不要直接沿用其他机器的 DAgger 控制配置。

命令分为“电脑执行”和“机器人执行”。电脑指保存源码和 SSH 私钥的工作电脑。
初始化不需要移动关节、启动录制、重启手臂服务或修改开机自启动。

## 1. 部署前确认

机器人应已具备：

- Ubuntu 24.04 / ROS 2 Jazzy，Python 3.12 与 ROS Python 接口匹配。
- 原厂手臂、视觉和遥操作服务。Collector 是消费者，不代替硬件驱动。
- 可用的关节状态话题和三路相机 SHM；深度录制另需深度流。
- 可写的数据盘和足够空间。目录名 `nas14` 不代表已挂载 NAS。
- SSH 服务及有权登录的账号。由设备管理者提供初始密码，勿写入仓库。

本次标准目录：

| 用途 | 347 路径 |
|---|---|
| Collector | `/home/ubuntu/CB_Autolife_Data_Collector` |
| 独立采集 Python | `/home/ubuntu/miniconda3/envs/lerobot/bin/python` |
| 原厂辅助 Python | `/home/ubuntu/miniconda3/envs/robot_env/bin/python` |
| 默认数据父目录 | `/home/ubuntu/nas14` |
| 本次安装日志 | `/home/ubuntu/collector_deployment/20261020` |

## 2. 配置 SSH 免密登录

### 2.1 检查或生成电脑密钥

**电脑执行：**

```bash
ls -l ~/.ssh/id_ed25519.pub
```

已有密钥就复用，**不要重新生成并覆盖**。没有密钥时执行：

```bash
ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519 -C "collector-workstation"
```

建议给私钥设置口令，用桌面密钥代理或 `ssh-add ~/.ssh/id_ed25519` 缓存解锁。
这样免去重复输入机器人的密码，也不必把私钥设置成无口令。

- `id_ed25519` 是私钥，只保留在电脑上。
- `id_ed25519.pub` 是公钥，可以添加到机器人的 `authorized_keys`。
- 首次连接时核对机器人主机指纹，确认不是连错设备。可让管理员在机器人终端运行
  `ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub` 提供指纹。

### 2.2 安装公钥

**电脑执行：**

```bash
ssh-copy-id -i ~/.ssh/id_ed25519.pub ubuntu@192.168.8.28
ssh -o BatchMode=yes -i ~/.ssh/id_ed25519 ubuntu@192.168.8.28 hostname
```

第一次按提示输入机器人密码。第二条成功输出 `autolife-robot-347`，才说明密钥登录正常。
`ssh-copy-id` 追加公钥，保留其他人的密钥；不要用重定向覆盖 `authorized_keys`。

若拒绝登录，检查用户名、目标 IP、公钥是否装在该用户下，以及权限：

**机器人执行，仅在需要修复权限时：**

```bash
chmod 700 ~/.ssh
chmod 600 ~/.ssh/authorized_keys
```

不需要关闭密码登录。新机器 SSH 主机密钥与旧 IP 记录冲突时，先让管理员核实设备/指纹，
确认后再处理旧记录；不要用 `StrictHostKeyChecking=no` 绕过检查。

### 2.3 增加连接别名

**电脑编辑 `~/.ssh/config`，保留已有配置并增加：**

```sshconfig
Host S3-347_CB robot-347
  HostName 192.168.8.28
  User ubuntu
  IdentityFile ~/.ssh/id_ed25519
  IdentitiesOnly yes
  ServerAliveInterval 30
  ServerAliveCountMax 3
```

之后连接和传文件：

```bash
ssh robot-347
scp ./example.txt robot-347:/home/ubuntu/
```

机器人 IP 改变时只修改这一段的 `HostName`，重新确认主机身份。
**机器人 SSH 登录和 GitHub 权限是两件事**，不要把电脑的 GitHub 私钥复制到机器人。

## 3. 安装 Collector 源码

### 3.1 机器人可以访问 GitHub

**机器人执行，目标目录必须尚不存在：**

```bash
cd /home/ubuntu
git clone --branch main --single-branch \
  https://github.com/lee1354774070-ai/CB_Autolife_Data_Collector.git
cd CB_Autolife_Data_Collector
git rev-parse --short HEAD
git status --short
```

公开仓库用 HTTPS 只读克隆即可，不需要在机器人上添加 GitHub SSH 私钥。
已有同名目录时先检查它的 Git 状态、实际入口和本地修改，不覆盖、不执行 `reset --hard`。

### 3.2 机器人不能访问 GitHub

可从电脑的**独立 Collector 仓库**制作 Git bundle，保留提交历史。
以下命令需在已确认并更新到所需版本的独立仓库运行，不是在 VLA 集合仓库根目录运行：

**电脑执行：**

```bash
git status --short
git rev-parse --short main
git bundle create ../collector_main.bundle main
rsync -av ../collector_main.bundle robot-347:/home/ubuntu/
```

bundle 只包含已提交的 `main`，不包含尚未提交的修改。

**机器人执行，目标目录必须尚不存在：**

```bash
git clone --branch main /home/ubuntu/collector_main.bundle \
  /home/ubuntu/CB_Autolife_Data_Collector
git -C /home/ubuntu/CB_Autolife_Data_Collector remote set-url origin \
  https://github.com/lee1354774070-ai/CB_Autolife_Data_Collector.git
git -C /home/ubuntu/CB_Autolife_Data_Collector rev-parse --short HEAD
```

核对与电脑上的提交一致。347 本次采用这一方法部署。

## 4. 建立独立采集环境

以下安装组合面向 **x86_64、Ubuntu 24.04、ROS Jazzy**。ARM / Jetson 需要对应平台的依赖，
不要直接套用 x86 CPU wheel 安装命令。不要升级或覆盖原厂 `robot_env` 和系统 ROS。

### 4.1 新建 Conda 环境

**机器人执行：**

```bash
/home/ubuntu/miniconda3/bin/conda env list
```

如果已有 `lerobot`，先检查版本和用途，不重建、不直接升级。确认是新机器且尚无该环境时：

```bash
/home/ubuntu/miniconda3/bin/conda create -y -n lerobot python=3.12 pip
PY=/home/ubuntu/miniconda3/envs/lerobot/bin/python
```

本例假设 Miniconda 已由设备管理员安装。不存在时，先安装与架构匹配的 Conda 发行版，
或指定另一个经过验证的 Python 3.12 环境；不要将 `sudo pip install` 用于补齐依赖。
离线创建依赖完整的 Conda 包缓存，仅有已有环境不等于所有安装包都缓存好了。

### 4.2 安装采集依赖

只在这个新建环境中执行：

```bash
"$PY" -m pip install torch==2.7.1 torchvision==0.22.1 \
  --index-url https://download.pytorch.org/whl/cpu

"$PY" -m pip install \
  "lerobot==0.6.0" "datasets==4.8.5" "pyarrow==25.0.0" \
  "av==15.1.0" "numpy==2.2.6" "pandas>=2,<3" "jsonlines>=4,<5" \
  "opencv-python-headless==4.13.0.92"

"$PY" -m pip check
```

这里使用独立 CPU 版 PyTorch，满足采集与数据集读写，不是训练/推理环境。
LeRobot 核心包加上显式的数据依赖即可；本安装使用 PyAV，不安装需要另外匹配
PyTorch/FFmpeg 版本的 TorchCodec，也不安装模型、CUDA 或全部机器人 extras。
PyAV wheel 自带其编解码依赖，运行 Collector 不要求另装系统 `ffmpeg` 命令。

`rclpy` 和 ROS 消息由系统 ROS 环境提供，不要用 pip 安装另一份来替代。
若安装源中的版本已不可用，不要直接放宽所有约束，先在隔离环境重新验证。

**下载很慢时：**可以在同为 Linux x86_64 / Python 3.12 的电脑上先下载大包，再传输。
347 本次使用此方式；不同架构/Python版本的 wheel 不能混用。

电脑执行：

```bash
python3 -m pip download --only-binary=:all: --no-deps --dest collector_wheels \
  pyarrow==25.0.0 av==15.1.0 opencv-python-headless==4.13.0.92
rsync -av collector_wheels/ robot-347:/home/ubuntu/collector_wheels/
```

机器人用本地 wheel 替代第二条安装命令，CPU PyTorch 仍按前一步安装：

```bash
"$PY" -m pip install /home/ubuntu/collector_wheels/*.whl \
  "lerobot==0.6.0" "datasets==4.8.5" "numpy==2.2.6" \
  "pandas>=2,<3" "jsonlines>=4,<5"
"$PY" -m pip check
```

此目录只放已核对版本的安装包。可在传输前后用 `sha256sum collector_wheels/*.whl`
核对文件，安装日志另存；不要将大包加入源码仓库。

### 4.3 环境与接口检查

**机器人执行：**

```bash
cd /home/ubuntu/CB_Autolife_Data_Collector
set +u
source scripts/source_ros_env.sh
export LEROBOT_PY=/home/ubuntu/miniconda3/envs/lerobot/bin/python
export ROBOT_PY=/home/ubuntu/miniconda3/envs/robot_env/bin/python

LD_LIBRARY_PATH="$(dirname "$(dirname "$LEROBOT_PY")")/lib:${LD_LIBRARY_PATH:-}" \
"$LEROBOT_PY" -c 'import rclpy, cv2, av, pyarrow; from lerobot.datasets.lerobot_dataset import LeRobotDataset; print("Collector imports OK")'

bash start_lerobot_official_collect.sh --help
bash start_lerobot_official_collect.sh WITH_DEPTH --help
```

`set +u` 只关闭 Shell 未定义变量报错，避免 ROS setup 脚本报错；与电机使能无关。
启动器会为子进程设置 ROS 与环境库路径，通常不需要修改 `.bashrc` 或全局 `LD_LIBRARY_PATH`。
`ROBOT_PY` 用于 VR 辅助、可选图像桥/生产者；纯 SHM 键盘采集主要使用 `LEROBOT_PY`。

## 5. 只读检查硬件输入

**机器人执行：**

```bash
systemctl --user status arm-control-service.service vision-service.service --no-pager
df -h /home/ubuntu /dev/shm
ls -l /dev/shm/camera_metadata_struct_{rgbd_head_color,rgbd_head_depth,hand_left_jpeg,hand_right_jpeg}

cd /home/ubuntu/CB_Autolife_Data_Collector
set +u
source scripts/source_ros_env.sh
export ROBOT_ID=347 ROS_DOMAIN_ID=0
ros2 topic list
ros2 topic echo --once /topic_arm_whole_body_and_gripper_current_joints_status_0_347
```

服务 `active (running)`、SHM 文件存在，只代表进程/文件存在，**不代表有新帧**。
还需核对 metadata 时间戳持续变化、帧能解码、状态包包含所选关节及目标。
相机源可能是 SHM2 多槽发布，Collector 读取当前发布槽并做一致性校验；
双手使用 `hand_left_jpeg / hand_right_jpeg`，不会回退到旧 BGR 手相机流。

状态或相机缺失时先排查原厂服务，不要通过降低 `MIN_CAMERAS`、放宽同步阈值、
开启第二个相机生产者或重启手臂来绕过故障。服务重启应由现场人员确认后操作。

## 6. 第一次人工录制

只有环境和输入检查通过、现场准备好后才执行。使用全新目录，验证数据不要混进训练集。

**机器人执行：**

```bash
cd /home/ubuntu/CB_Autolife_Data_Collector
set +u
source scripts/source_ros_env.sh
unset VR_CONTROL SUBTASKS_JSON ACTION_MODE

ROBOT_ID=347 ROS_DOMAIN_ID=0 ROBOT_TYPE=autolife_s3 \
COLLECTOR_MODE=keyboard ACTION_MODE=status_target \
START_HAND_PRODUCER=0 IMAGE_SOURCE=shm \
WITH_HEAD=0 WITH_UPPER_WAIST=0 WITH_WAIST=0 WITH_DEPTH=0 \
WITH_EEF_STATE=0 WITH_EEF_ACTION=0 \
COLLECT_FPS=30 MAX_SYNC_DELTA_SEC=0.03 SYNC_IMAGE_BUFFER_SIZE=16 \
OUTPUT_BASE_DIR=/home/ubuntu/collector_validation/manual_347 \
TASK_TEXT="VALIDATION ONLY - NOT TRAINING DATA" \
bash start_lerobot_official_collect.sh rgb_16d_first_check
```

等待初始化完成，**Enter 开始 → S 保存 → 等待 `[SAVED]` → Q 正常退出**。
不要在保存/编码过程中关机、强杀或删除目录。

默认采集 14 维手臂 + 2 维夹爪 + 三路 RGB。后续开头/腰/深度/EEF时换新 schema 目录。
实测应覆盖保存、视频解码、Parquet/metadata、一条续录以及同步日志，
不能只用“启动成功”认定 30 FPS 稳定，也不能用短条验证代替长时间负载测试。

`task_name` 是输出子目录；`TASK_TEXT` 是训练任务文本。
正式数据路径自行改为已确认可写、空间足够的数据盘，不要放进 Git 源码目录。

## 7. 单人、子任务与 DAgger 的边界

| 模式 | 新机器还需确认 |
|---|---|
| `keyboard` | 原厂遥操作可用；Collector 终端控制开始/保存/丢弃。 |
| `vr` | 原厂 App / 手柄消息 `/control_topic_0_347`、辅助 Python、TTS；GL+GR 配合面键。 |
| `subtask` | 与 VR 相同，另设置有序 `SUBTASKS_JSON`；短 B 标记，长 B 提前保存。 |
| `dagger` | 单独审核的 V4 增量控制、机器人 profile、模型/Thor 服务及控制权限；不能把 300 的配置只改编号后直接运行。 |

前三种使用原厂遥操作和 `status_target`；DAgger 使用独立控制链路与 `joint` action。
原厂独立 App 不加载 8447 网页，不能以软件反馈事件存在就宣称头显振动/显示已接通。
完整参数和按键见 [README](../README.md)，DAgger 现场接管验收须单独完成。

## 8. 之后更新与留档

**机器人执行：**

```bash
cd /home/ubuntu/CB_Autolife_Data_Collector
git status --short
git rev-parse HEAD
git fetch origin
git log --oneline HEAD..origin/main
```

确认没有在录制/编码，工作区无需要保留的未提交修改，审查更新后再执行：

```bash
git merge --ff-only origin/main
```

发现同事修改或不能快进时停下来处理差异，不覆盖本地文件。
不要把 `rsync --delete` 指向含数据、日志、配置或模型的目录。
版本更新不等于需要重装环境，更不等于需要重启硬件服务。

每次验收至少记录：机器/IP、源码 commit、Python/LeRobot 版本、schema、测试范围与结果。
可以在独立日志目录保存 `pip freeze`；不要将密码、私钥、token、数据集或大量安装日志提交 Git。

347 本次部署结果见 [部署与只读验证记录](validation/20261020_s3_347_deployment.md)。
