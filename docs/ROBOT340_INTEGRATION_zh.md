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

## GPU核查与限制

340此次检查的录制环境为`torch 2.7.1+cpu`、`torchvision 0.22.1+cpu`，现有采集日志确认`JPEG_DECODER=auto`回退OpenCV，`VIDEO_ACCELERATOR=cpu`。GPU优化代码已存在，但该环境尚不能执行nvJPEG；不得将代码存在等同于GPU已启用。CUDA环境和NVENC性能需要另行验证，不在采集中升级原环境。

本轮不证明真实机器人运动、Windows实际音频或GPU加速收益；电脑播报操作见[使用说明](PC_SPEECH_zh.md)。
