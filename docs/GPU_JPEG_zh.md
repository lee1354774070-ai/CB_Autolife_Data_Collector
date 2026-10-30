# 录制时默认优先 GPU JPEG 解码

默认 `JPEG_DECODER=auto`，在可用的NVIDIA CUDA环境优先使用nvJPEG，启动初始化/探针失败才回退OpenCV。把同一次SHM轮询中已经复制的JPEG批量解码，再传回CPU作为连续的HWC uint8 RGB图像。不会为了凑批次等待其他相机。

这项功能只作用于 `IMAGE_SOURCE=shm` 的JPEG流，四种Collector模式共用。原生RGB/BGR和uint16深度继续走原来的读取流程；图像FIFO、时间戳、同步阈值和关节语义不变。ROS `sensor_msgs/Image` 已在上游解码，auto/opencv不再解码或初始化CUDA，显式nvjpeg被拒绝。

## 启动

在含本次更新的Collector目录运行，保留原任务所需的头/腰/深度和模式参数：

```bash
# 默认GPU优先，原启动命令无需增加参数
TASK_TEXT="Pick up the water bottle." \
bash start_lerobot_official_collect.sh pick_water

# 固定NVIDIA GPU，不允许启动时回退
JPEG_DECODER=nvjpeg JPEG_DEVICE=cuda:0 bash start_lerobot_official_collect.sh pick_water

# 保持旧OpenCV解码行为，完全不初始化CUDA
JPEG_DECODER=opencv bash start_lerobot_official_collect.sh pick_water

bash start_lerobot_official_collect.sh JPEG_DECODER --help
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `JPEG_DECODER` | `auto` | 默认GPU优先，预检失败时整场使用CPU；`nvjpeg`要求GPU预检通过；`opencv`固定CPU |
| `JPEG_DEVICE` | `cuda:0` | CUDA可见设备编号；不控制保存阶段的NVENC/VAAPI视频编码器 |

录制器启动时输出 `JPEG decoder: nvjpeg on cuda:0; batched decode with CPU RGB output` 表示GPU初始化和JPEG探针通过。实际相机尺寸在每次解码时继续校验。

`JPEG decoder: opencv (CPU startup fallback)`表示本场回退CPU，并打印原因；这不是GPU已启用。
`sync_log.jsonl`记录`image_decoder_selected`，每条`episode_started.image_decoding`记录实际后端与设备。
关节状态就绪后才初始化GPU和启动SHM轮询；相机warmup计时从初始化完成后开始。
默认SHM auto/nvjpeg的外层`CAMERA_DETECTION_TIMEOUT_SEC`为45秒，显式设置优先；录制中的同步阈值保持不变。

录制中GPU解码错误使当前episode无效并停止录制进程，不静默换后端或复用旧帧。OpenCV路径保留原行为。nvJPEG需要匹配版本的CUDA版torch、torchvision及NVIDIA驱动，CUDA需至少11.6；只有Intel核显的机器会在auto模式下使用OpenCV。不要仅为解码无条件升级机器人的整个LeRobot环境。

## 像素一致性与性能边界

nvJPEG使用GPU辅助解码，部分JPEG工作仍在CPU；本工具还需要把RGB结果传回CPU写图。因此应测量完整解码、排列转换和回传耗时。临时PNG、深度处理和保存流程不会因打开本参数全部移到GPU。

**nvJPEG与OpenCV的JPEG输出不是逐像素等价。** 色度上采样和解码实现可能导致颜色边缘差异，幅度依赖输入的采样格式与内容。必须用目标机器的原始相机JPEG比较像素、时延和CPU开销，不能把合成图或视频重新压缩的JPEG等同于原始SHM码流。

正式启用还需检查模型行为；要求延续旧像素语义时显式设置`JPEG_DECODER=opencv`。建议切换后端时使用新数据目录，避免无意识混用不同解码行为。

## 离线测量

```bash
conda activate lerobot
# 使用真实相机JPEG样本，只读文件，不连接机器人或发布ROS
python scripts/benchmark_jpeg_decoding.py \
  --jpeg /path/left.jpg /path/right.jpg --hz 30 --iterations 300 --runs 1 \
  --output /path/new_jpeg_benchmark.json

# 无样本时用合成图像做功能/性能对比
python scripts/benchmark_jpeg_decoding.py --width 640 --height 480 --runs 3
```

输出包含每批CPU时间、端到端平均/P95/P99/最大耗时、30Hz单核占用估算、超期次数、像素差异及PyTorch显存计数。显存计数不包含CUDA上下文和nvJPEG内部申请；CPU数字也不包含视觉服务或整个Collector。

连续吞吐、30Hz间歇调用和实时采集的负载不同。间歇调用受GPU时钟和调度影响，应以端到端P95/P99和超期次数验收，不能只比较平均解码耗时。

## 验证范围

当前合并版本的测试记录见[验证范围](VALIDATION.md)，命令见[测试指南](../ROBOT300_TESTING_zh.md)。四种模式的合成状态/图像与控制事件回归，不代表真实手柄、硬件编码器性能或DAgger运动验收。旧版本的过程记录保留在Git历史，不作为当前版本的验收结论。

参考：[torchvision decode_jpeg](https://docs.pytorch.org/vision/0.26/generated/torchvision.io.decode_jpeg.html)、[NVIDIA nvJPEG](https://docs.nvidia.com/cuda/nvjpeg/)。
