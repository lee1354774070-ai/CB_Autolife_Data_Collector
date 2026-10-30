# Collector 验证指南

代码统一在独立仓库维护，机器人目录按实际部署路径指定；不要从旧验证目录启动生产任务。
当前验证记录见 [VALIDATION](docs/VALIDATION.md)。旧过程记录可从Git历史查看。

## 软件回归（不控制机器人）

在本机或机器人独立代码目录执行，需要ROS 2 Jazzy及LeRobot环境：
```bash
set +u
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=211 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export HF_HUB_OFFLINE=1 HF_DATASETS_OFFLINE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
PY=${LEROBOT_PY:-/home/ubuntu/miniconda3/envs/lerobot/bin/python}
export LD_LIBRARY_PATH="$(dirname "$(dirname "$PY")")/lib:${LD_LIBRARY_PATH:-}"
"$PY" -B -m unittest discover -s tests -p 'test_*.py'
node tests/test_feedback.js
bash -n start_lerobot_official_collect.sh
```

GPU可用且允许占用时，增加COLLECTOR_TEST_NVJPEG=1运行实际nvJPEG测试；
COLLECTOR_TEST_VIDEO_ACCELERATOR=nvenc或vaapi运行硬件保存集成。
测试跳过不等于通过，不能降低同步或存储阈值换取通过。

覆盖：四种模式的合成录制、后台队列/恢复/失败、关节及EEF、子任务边界、
DAgger结果和逐维mask、PNG首帧、视频/Parquet读回、SHM一致性、写盘故障、环境安装保护。
测试只在域211注入合成信号，不能改成真机控制域。
额外ROS链路脚本仅在依赖匹配时显式运行，参见各脚本help；不强杀其他任务规避冲突。

## 现场验收（需单独授权）

普通、单人、子任务连接原厂遥操作；DAgger连接审核过的增量副本。
先检查相机、关节和控制冲突，再由现场人员按README启动。每种模式先录5至10秒。

| 模式 | 核对 |
|---|---|
| keyboard | Enter开始、S保存、D丢弃；F封口仍待命；Q退出并等保存队列完成 |
| vr | 双Grip+A开始、B保存、Y丢弃，语音与回执一致 |
| subtask | 逐阶段短B，最后阶段保存；长B提前保存；持续松开达到重触发间隔 |
| dagger | A推理、Grip立即接管、B成功、短Y丢弃、长Y失败保存、X仅复位；终端K失败保存 |

未确认复位目标、控制权、模型/机器人schema时，不执行DAgger运动。
相机会话只有明确配置后才切换视觉服务，测试时说明是否会重启，不能影响别人的会话。
原厂模式X复位未接入；不要启动另一套控制器补偿。

## 数据验收

- save_queued只代表暂存；后台完成回执及meta一致才计入成功数。
- 核对episode数量、连续Parquet索引、三路RGB及可选depth的首/中/末帧。
- 首帧PNG按实际episode编号存在，失败条不得污染历史首帧。
- 子任务边界合法，未知尾段-1；DAgger结果标签与专家mask分开检查。
- 验证失败保留raw job，重启可恢复；不要人工重发保存或删除队列。
- 故障注入只针对合成数据，不损坏真实已保存文件。
- 真实相机时序、VR操作、网络、CPU负载和长期稳定性需要独立现场证据。
