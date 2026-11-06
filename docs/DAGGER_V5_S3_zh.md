# S3 V5增量遥操接入DAgger

V5是可选后端，默认仍为V4。普通、单人、子任务采集不进入V5代码。

来源：[openarmx_teleop_vr_v5](https://github.com/liangxinfeng/openarmx_teleop_vr_v5/tree/robot345-v5-20261030)，固定提交 `266cfb4167cf7333601936f56a4536883fd4ee1f`。
源仓库声明许可证 CC-BY-NC-SA-4.0；本适配以补丁和源文件哈希清单记录来源，不重新打包整套遥操。

## 安装与边界

将该分支放在独立目录 `/home/ubuntu/openarmx_teleop_vr_v5`，按其 `scripts/setup_v5_mink_env.sh` 建立独立Mink环境。
不要覆盖原厂SDK、V4环境或运行中的采集目录。源版本变化必须重新审核，不能跳过哈希。

新增选择：

```bash
export DAGGER_TELEOP_BACKEND=v5
export DAGGER_V5_SOURCE_ROOT=/home/ubuntu/openarmx_teleop_vr_v5
export ROBOT_ID=340 ROBOT_TYPE=autolife_s3 DAGGER_ROBOT_PROFILE=s3_v3_1
```

DAgger在`.runtime/v5-*`建立经校验的私有副本。为兼容现有监督器、网页、录制和输出回执，副本保留旧的`openarmx_teleop_vr_306_v4`模块/话题名称；实际求解器为V5 Mink，原始V5映射/求解文件不变，只替换兼容命名。界面日志中的306/V4不是目标机器人编号，硬件话题后缀由`ROBOT_ID`和ROS域确定。

补丁增加：模型/专家epoch隔离、即时接管重锚、异步IK提交前复核、输出来源回执、专家状态下才允许腰部跟随、任务快照复位、未使能退出零命令。使用V5原生配套简化URDF/SRDF，不拿完整SDK网格和另一套碰撞排除表混用。21个受控关节的轴、坐标变换、上下限与340实际SDK逐项一致；这个静态检查不能代替实机碰撞验收。

V5手柄闭合百分比保持上游默认90%（逻辑角325°），配置按当前机器人编号独立保存，不读写345的设置。模型端使用下述binary330规则，客户端不修改proposal中的动作。

## 340验证部署

当前独立目录：`/home/ubuntu/collector_validation/dagger_v5_20261106/collector`。
入口：`/home/ubuntu/collector_validation/dagger_v5_20261106/start_dagger_v5.sh`。

入口已配置：

- 参考推理入口：`/home/ubuntu/s3_s2_minimal_020000`。
- 模型服务：`http://192.168.8.224:8783`；checkpoint：`hotel_trash_clean233_020000`。
- prompt：`Clear the trash from the table.`；夹爪阶段：`binary330`，由Thor执行。
- 复用本机`~/.config/groot340.token`，不复制或打印token。
- 姿态：从上述推理入口使用的`~/.config/autolife_robot_control_center/vr_reset_pose.json`生成独立快照`table_trash_020000`，全身21关节加左右夹爪10°。后续修改原姿态不会自动覆盖此快照，更新时须重新核验。
- 录制：头部、上腰、深度开启；GPU采集环境`collector_gpu_v2`；默认输出到独立`test_data`，不混入正式训练数据。
- 默认`DAGGER_PUBLISH=0`；启动后保持DISARMED，按A才开始。

先预检（只读，不退出任何控制组件）：

```bash
bash /home/ubuntu/collector_validation/dagger_v5_20261106/start_dagger_v5.sh v5_check --check
```

确认原厂遥操/导航/其他推理已退出，预检通过，再由现场操作者运行：

```bash
DAGGER_PUBLISH=1 bash /home/ubuntu/collector_validation/dagger_v5_20261106/start_dagger_v5.sh v5_live_test_dagger
```

头显访问 `https://192.168.8.59:8447`。只运行这一套控制器。

1. 启动后确认状态为未开始；先检查全身复位目标，X只复位，不保存/丢弃。
2. A开始模型与录制；检查双臂、头/腰模型输出和夹爪，无手柄接管时应无VR腰部跟随。
3. 按单侧Grip接管，先小幅平移再旋转；确认末端跟手、另一侧不突跳、扳机有效。接管后允许腰部跟随；松开Grip保持，重新握持从当前位置重新建立基准。本模式由人完成，不自动交还模型。
4. B确认成功并保存；无接管保存为自主成功，有接管保存为纠错成功。等待真实落盘反馈，后台排队不等于保存成功。
5. Y短按松开仅丢弃；Y按满1秒失败保存。检查启动器/头显结果反馈及纠错掩码。
6. 终端F封口并继续运行；Q退出。终端沿用C开始、A成功保存、K失败保存、X/D丢弃、R复位；头显仍使用上述ABXY，勿混淆。保存/丢弃不隐含复位，下一条前用X按任务姿态复位。
7. 测试后审查视频、深度、时间戳和action/mask，再开始正式采集。

出现冲突/活动控制输入时先正常退出原控制端，不跳过检查，不清错或重启ARM。
回退：停止本入口，继续使用原来的集合版目录/入口；默认V4和原厂服务没有被改写。

## 已完成的软件验证（2026-11-06）

- 集合版530项测试，14项环境相关跳过；V5上游相关65项测试通过。
- V5真实Mink零位移、异步IK、旧模型命令拒绝、接管、腰部门控、夹爪及全身任务复位规划通过；硬件调用0。
- 隔离ROS监督/FIFO测试通过：开始、接管、成功/失败保存、丢弃与慢存储；硬件调用0。软件接管回调约0.55ms，不代表头显到电机延迟。
- 五组件整栈隔离启动，网页HTTP200，保持DISARMED，硬件消息0，正常退出。
- 新DAgger桥只读真实推理：40×21，digest匹配，约1.04秒；discard并关闭，会话无动作ACK，硬件调用0。
- 真机控制权切换、物理跟手和实录数据尚待现场验证。现有控制端有活动输入时，正式预检会阻止启动。

测试脚本：`tests/test_dagger_v5_runtime.py`、`tests/v5_controller_smoke.py`、`tests/dagger_ros_smoke.py`、`tests/v5_stack_smoke.py`、`tests/v5_thor_readonly.py`。
后三个ROS测试必须使用各脚本指定的隔离条件；只读推理脚本仅观测真实域，不发布动作。
