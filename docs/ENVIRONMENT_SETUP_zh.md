# 一键部署 Collector 环境

在**机器人终端的Collector目录**执行。支持Linux x86_64、ROS 2 Jazzy/Python 3.12，需要已安装Conda和系统ROS；不适用于Thor/Jetson ARM。

```bash
# 自动检测NVIDIA GPU，新建名为collector的独立环境
python3 scripts/setup_collector_env.py
```

有可用NVIDIA GPU时安装CUDA版PyTorch；没有时安装CPU版。已经检测到GPU但安装/实际GPU探针失败时会报错，不会把失败伪装为GPU已启用。
脚本不安装Conda、ROS、SDK、驱动、原厂遥操作或DAgger依赖，不停止任务、不重启服务、不读取相机、不发送控制指令。

## 常用命令

```bash
# 强制GPU；没有可用GPU/驱动时在安装前退出
python3 scripts/setup_collector_env.py --name collector_gpu --backend nvjpeg

# 强制CPU，适用于只有Intel核显的机器人
python3 scripts/setup_collector_env.py --name collector_cpu --backend cpu

# 克隆已有环境，在副本中安装；不修改原lerobot环境
python3 scripts/setup_collector_env.py --name collector_gpu --clone lerobot --backend nvjpeg

# 先查看安装命令，不创建环境、不下载
python3 scripts/setup_collector_env.py --name collector_gpu --backend nvjpeg --dry-run

# 检查已有环境，不安装包、不修改环境配置
python3 scripts/setup_collector_env.py --name collector_gpu --backend nvjpeg --check

python3 scripts/setup_collector_env.py --help
```

默认安装环境位于Conda根目录的`envs/<name>`。可用`--conda /path/bin/conda`指定Conda，
用`--ros-setup /opt/ros/jazzy/setup.bash`指定ROS入口。`--clone`接受已有环境名或绝对路径。

**已有同名环境不会被覆盖。** 安装失败时保留现场；仅由此脚本创建的未完成环境可在原命令后加`--resume`续装，名称、后端、克隆来源和配置必须一致。
如果环境创建阶段就失败、没有生成Python解释器，请使用新环境名，检查旧目录后自行处理，脚本不会自动删除。
同名环境不能同时运行两个安装程序。`base`、`lerobot`、`robot_env`、`cuda_env`只允许`--check`，不允许作为安装目标。

## 安装与验证内容

固定采集基线：LeRobot 0.6.0、torch 2.7.1、torchvision 0.22.1、datasets 4.8.5、
PyArrow 25.0.0、PyAV 15.1.0、NumPy 2.2.6、OpenCV headless 4.13.0.92。
GPU版使用cu126，CPU版使用cpu；其余间接依赖仍由pip解析，不是完整离线锁文件。
torch/torchvision版本组合来自[PyTorch官方安装说明](https://pytorch.org/get-started/previous-versions/#v2-7-1)。

不安装LeRobot全部模型训练extra，也不安装本工具不需要的torchcodec，以避免无关框架升级。
克隆模式会复制源环境；新建模式需要联网获取Conda与pip包。安装前要求GPU至少16 GiB、CPU至少6 GiB空闲空间。
这是最低预检，不是磁盘配额；大型克隆源可能需要更多空间。

安装后自动检查：

- `pip check`依赖一致性。
- ROS消息类型、录制器和LeRobot真实导入。
- GPU模式实际解码一张合成JPEG，CPU模式明确使用OpenCV。
- 在临时目录生成一条三帧合成LeRobot数据，保存后核对Parquet行数及MP4实际解码帧数，随后清理临时数据。

检查过程不创建ROS节点、不连接硬件。通过不代表真实相机同步、手柄、DAgger、深度视频或硬件视频编码已经验收。
录制JPEG解码和保存时NVENC/VAAPI是两个独立功能，详见[GPU JPEG](GPU_JPEG_zh.md)及[视频编码](GPU_ENCODING_zh.md)。

## 用新环境采集

安装成功会为**新环境本身**配置`LEROBOT_PY`。重新激活环境后生效，不修改`.bashrc`或启动器默认路径：

```bash
conda activate collector_gpu
# 保留原来的任务、模式、关节和相机参数
JPEG_DECODER=nvjpeg TASK_TEXT="Pick up the water bottle." \
bash start_lerobot_official_collect.sh pick_water
```

也可以不激活，明确传入解释器，路径按本机Conda位置修改：

```bash
LEROBOT_PY=/home/ubuntu/miniconda3/envs/collector_gpu/bin/python \
JPEG_DECODER=nvjpeg TASK_TEXT="Pick up the water bottle." \
bash start_lerobot_official_collect.sh pick_water
```

`JPEG_DECODER=nvjpeg`要求GPU成功；默认`auto`允许启动时回退CPU。`ROBOT_PY`仍指向原厂`robot_env`，
用于VR/相机辅助，不要一起改成新环境。首次采集前仍须检查机器人、相机及关节服务状态。

安装器的`--check`不会为外部创建的环境设置`LEROBOT_PY`；这种情况使用上面的显式路径命令。

## 验证范围

当前合并版验证及限制见[VALIDATION](VALIDATION.md)。安装器使用合成输入检查环境，不能代替真实遥操作或DAgger运动验收。
