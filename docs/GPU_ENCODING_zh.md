# RGB 保存阶段硬件编码

## 作用范围

`VIDEO_ACCELERATOR=nvenc` 可在具备可用 NVIDIA 驱动与 FFmpeg NVENC 的采集机器上，
使用 GPU 的固定功能视频编码器生成 RGB H.264 视频。RTX 4090 支持此类能力，但需要在
**运行 Collector 的那台机器**上实际验证；另一台电脑有 GPU 不会自动加速机器人。

这不是“所有采集放到 GPU”。SHM JPEG解码由独立的JPEG_DECODER选择，详见[GPU JPEG](GPU_JPEG_zh.md)。当前以下工作仍在 CPU：同步、
PNG 暂存、RGB→YUV 转换、输出视频完整解码验证、Depth 视频以及 Parquet/元数据保存。
NVENC 不等于 CUDA 推理，不以 GPU 显存占满或 CUDA 利用率高作为验收条件。
普通 NVENC 不能代替现有保留深度语义的 12-bit 编码；本适配器不会把 depth 当 RGB。

VIDEO_ACCELERATOR默认cpu，在同步和后台保存中均不探测视频硬件；JPEG_DECODER独立控制GPU解码。启用硬件前需完成目标机测试。

## 配置与失败行为

| 配置 | 行为 |
| --- | --- |
| `VIDEO_ACCELERATOR=cpu` | 原 LeRobot CPU 路径，默认值 |
| `VIDEO_ACCELERATOR=nvenc` | 明确要求 NVENC，探测失败则启动失败 |
| `VIDEO_ACCELERATOR=vaapi` | 明确要求 VAAPI，探测失败则启动失败 |
| `VIDEO_ACCELERATOR=auto` | 启动时依次探测 NVENC、VAAPI；全部失败才选 CPU 并告警 |
| `VIDEO_FFMPEG_BIN=/path/to/ffmpeg` | 硬件路径使用的 FFmpeg 可执行文件 |
| `VIDEO_HW_QP=23` | H.264 constant QP，0–51；与 CPU CRF 不等价 |
| `VIDEO_HW_DEVICE=/dev/dri/renderD128` | VAAPI render device；NVENC 不使用此参数 |
| `VIDEO_ENCODING_TIMEOUT_SEC=600` | 每路 RGB 的输入检查、编码及完整读回共享预算；不是整个 episode 保存上限 |

硬件编码仅支持 RGB `h264/yuv420p`，不能与 `VIDEO_CRF`、`VIDEO_PRESET` 混用。
使用 `ENCODER_THREADS` 明确限制 PNG 解码/CPU 编码线程；采集器默认值为 2。
`DEPTH_ENCODER_THREADS` 单独限制 x265 工作线程，默认 2，保留 lossless 参数。
启动器不会自动把 Conda 的 bin 加入 PATH。如果 FFmpeg 只装在采集环境内，需将
`VIDEO_FFMPEG_BIN` 设置为该环境中 FFmpeg 的绝对路径；不能只设置 `VIDEO_ACCELERATOR`。
启动时每个后端最多 15 秒预算，覆盖全部实际 RGB 分辨率的短片编码与完整解码，
并检查色块顺序、颜色范围、帧数、分辨率与时间戳。仅查看 `ffmpeg -encoders` 不算验证。

**开始录制后不自动换后端。** 编码错误、超时、漏帧、损坏或读回失败会使保存失败，
不发布成功回执、不提交半成品、不用 CPU 重新编码来掩盖 GPU 异常。
原图只在本路视频完整验证通过后消费；整个 episode 仍由现有事务处理。
已存在的输出不覆盖，失败仅清理本次创建的私有临时输出。

## 色彩与数据契约

输入必须是逐帧连续的 RGB PNG；不接受 BGR、深度、尺寸变化或不完整序列。
不缩放、不补帧、不丢帧。输出显式使用 BT.601/SMPTE170M、limited range、YUV420P、
无 B 帧，并保存真实编码回执到 episode 事务侧文件。

这里沿用 LeRobot/PyAV 的 BT.601 读回约定。已在本机合成测试中发现部分 PyAV 版本对
BT.709 标签的默认 RGB 读回不符合预期，不能仅写入颜色标签就假定训练颜色正确。
启动色块验证用于捕获通道互换及明显颜色错误，不等于真实相机色度标定或无损编码。

续采只允许现有 RGB metadata 是 `h264/yuv420p` 的数据集；不兼容时使用原编码方式
或新目录。硬件 QP 与原 CRF 的码率、失真不同，即使格式兼容，也应先由团队验收画质。

## 独立合成基准

不连接 ROS、SHM、相机，也不发送动作。新输出目录标注 `NOT_TRAINING`，不可用于训练。

```bash
# 仅 CPU，确认基准与验证依赖可用；显式指定目标环境的 FFmpeg。
python -B scripts/benchmark_video_encoding.py \
  --backend cpu --ffmpeg /path/to/ffmpeg \
  --output /tmp/collector-encoding-cpu-new

# 仅在已经确认可占用 GPU、机器人静止且无人遥操时进行硬件测试。
python -B scripts/benchmark_video_encoding.py \
  --backend nvenc --compare-cpu --ffmpeg /path/to/ffmpeg \
  --cameras 3 --frames 90 --width 640 --height 480 --threads 1 \
  --output /tmp/collector-encoding-nvenc-new
```

结果 `report.json` 分开列出编码时间、完整验证时间、总耗时、CPU 秒和输出字节数。
三路顺序处理，避免未经目标机验证就提高并发峰值。CPU 基线为 FFmpeg/libx264
`veryfast`，**不是 LeRobot 整体保存时间**。合成基准不能证明真实 VR 卡顿、JPEG
损坏或相机同步问题已消失，也不能在未运行目标 GPU 测试时声称获得具体加速倍数。

## 集成边界与检查

`video_encoding.py` 不依赖 ROS，提供：

- `select_encoder(args, rgb_sizes, logger)`：启动选择，CPU 返回 `None`。
- `validate_resume_encoding(info)`：现有 RGB 视频格式兼容检查。
- `accelerated_rgb_encoding(dataset, encoder)`：只包装当前 writer 的一次顺序保存。

要求 LeRobot 0.6 风格 `_encode_temporary_episode_video(video_key, episode_index)`、
`batch_encoding_size=1`、非 streaming。接口不符则失败，不修改全局 LeRobot 方法。
深度由原函数处理；退出上下文时恢复原 writer，即使编码失败也是如此。

离线单元验证：

```bash
python -B -m unittest discover -s tests -p test_video_encoding.py -v
```

测试包括序列完整性、尺寸/深度拒绝、超时/编码故障、保存竞争不覆盖、完整读回
帧数/时戳/格式校验、实例恢复及原生深度路由。有 FFmpeg/PyAV 时额外运行小型
**CPU** 色块编码与全量读回；默认测试不探测 GPU。
