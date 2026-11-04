# 340集成整理与验证（2026-11-04）

基于Lee的`90b7b08`继续提交，保留原采集和持久化保存实现。此记录是软件验证，不能替代真机与现场声音验收。

## 合入内容

- 集合版公共录制入口集成电脑网页播报：`PC_SPEECH_WEB=1`默认开启，点击终端完整链接后启用本机声音。使用当前录制的日志/已发布回执，不使用OCR或固定子任务次数；后台排队与落盘成功分别反馈。
- `VR_DEBUG_INPUT=0`默认精简手柄调试输出，采集确认和异常保留；后台保存时子任务标注未完成会明确提醒。
- 原厂单人/子任务固定`status_target`，避免误用不连续的独立目标话题；DAgger保持`joint`及原mask契约。
- DAgger默认使用已审核打包快照，只在私有运行副本加入任务姿态加载，避免重复打权限补丁；源文件、适配补丁和缓存仍严格校验。启动器标题显示实际机器人编号。
- `MAX_EPISODE_TEMP_GB=0`可选关闭单条估算字节上限；主线默认仍为8 GiB，磁盘余量、队列、时长保护保留。
- 新增回归测试和备份/临时采集目录忽略规则。没有提交机器人数据、视频、日志、备份包或凭据。

340本机的头部第二轴上限34度未推广为默认值，主线仍是25度；其既有配置哈希作为兼容变体保留。340本机关闭单条预算的设置也未替换主线8 GiB默认值。此次整理没有改动运行中的340采集或遥操服务。

## 保持的上游功能

`jpeg_decoding.py`、`video_encoding.py`、`shm_camera.py`、`first_frames.py`、`background_saves.py`、`collector_modes.py`、`subtask_annotations.py`、`camera_session.py`和环境安装器均与`90b7b08`逐字节一致。保留GPU优先JPEG解码、可选硬件视频编码、首帧导出、深度语义、后台队列、F封口/Q退出和可配置B防连按。

安装器只有测试夹具变化：磁盘容量使用模拟值，低磁盘拒绝路径仍单独覆盖，避免测试受宿主机/tmp容量影响；生产磁盘检查没有放宽。

## 本轮验证

- 340隔离副本：529项Python测试，524通过、5项真实CUDA专用测试跳过，无失败。
- 使用真实LeRobot/FFmpeg/PyAV完成合成数据的RGBD保存、读回、四模式、标签/mask和历史数据保护测试；不使用真实相机输入。
- ROS域211：子任务按键流程、S2/300与S3/340 DAgger流程均通过，硬件调用0、模型请求0。
- 188个Python文件AST解析、11个Shell脚本语法检查通过。
- 两项既有前端反馈测试、网页事件顺序/重复/断线测试、可选PowerShell协议自测通过。

复现时source `scripts/source_ros_env.sh`，设置`ROS_DOMAIN_ID=211 CUDA_VISIBLE_DEVICES=-1 HF_HUB_OFFLINE=1`后运行AGENTS.md中的测试；需要让测试PATH能找到现有FFmpeg。DAgger隔离测试使用`DAGGER_DEPENDENCY_ROOT=$PWD/teleop_snapshot`，分别设置`ROBOT_ID=300 DAGGER_ROBOT_PROFILE=s2_v2_2`和`ROBOT_ID=340 DAGGER_ROBOT_PROFILE=s3_v3_1`。

## GPU环境核查与修复

340原来运行采集的环境为`torch 2.7.1+cpu`、`torchvision 0.22.1+cpu`，日志确认`JPEG_DECODER=auto`回退OpenCV，视频使用CPU。问题是默认解释器选错，不是GPU代码丢失。

340已经存在安装完成的`collector_gpu_v2`环境（torch 2.7.1+cu126、torchvision 0.22.1+cu126），本次复用它，没有升级或替换原`lerobot`环境：

- 环境检查通过：pip依赖、ROS导入、实际nvJPEG探针、LeRobot保存、Parquet和视频读回。
- 在该CUDA环境运行同一529项回归：全部通过，无跳过，包括5项实际GPU JPEG检查。
- 使用既有`robot_env/bin/ffmpeg`真实NVENC运行四模式合成RGBD保存、作废、丢弃、续录和读回集成测试，通过；深度仍走原编码路径。
- 仅340本机启动脚本的默认`LEROBOT_PY`改为该CUDA解释器，原子替换并在Git目录外备份。当前CPU采集不热切换，下次启动生效。GitHub公共默认路径未硬编码为340环境。

其他部署或显式指定340环境：在原命令前增加以下环境变量，其他任务/模式参数保持原样：

```bash
LEROBOT_PY=/home/ubuntu/miniconda3/envs/collector_gpu_v2/bin/python
JPEG_DECODER=nvjpeg
```

实际使用时将变量加入原`env ... bash start_lerobot_official_collect.sh ...`命令，或者用`export`导出。`auto`允许初始化失败回退CPU；`nvjpeg`要求GPU实际可用。录制器日志应出现`JPEG decoder: nvjpeg on cuda:0`。

RGB视频硬件编码仍为可选项，默认CPU不变；340已验证的参数为：

```bash
VIDEO_ACCELERATOR=nvenc
VIDEO_FFMPEG_BIN=/home/ubuntu/miniconda3/envs/robot_env/bin/ffmpeg
```

需要回到原环境时显式指定`LEROBOT_PY=/home/ubuntu/miniconda3/envs/lerobot/bin/python JPEG_DECODER=opencv VIDEO_ACCELERATOR=cpu`。切换JPEG解码或视频编码后使用新数据集目录，避免混用不同像素/压缩实现。

这些是合成图像的软件和真实硬件编解码验证，没有读取现场相机或发送机器人动作；未测量真实采集CPU降低比例、NVENC整体保存收益或机器人闭环表现。Windows实际音频仍需现场确认，操作见[电脑播报说明](PC_SPEECH_zh.md)。
