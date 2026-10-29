# AutoLife Collector 协作约定

## 入口与目录

- 统一入口 `start_lerobot_official_collect.sh`，`COLLECTOR_MODE=keyboard|vr|subtask|dagger`。
- `record_lerobot_official.py` 管理同步采集、FIFO回执和LeRobot数据保存。
- `recorder_config.py` 集中参数校验；`launcher_help.py` 复用参数定义生成帮助，修改参数时同步核对环境变量映射。
- `vr_collector_control.py`、`subtask_annotations.py` 管理单人按键、语音及子任务标记。
- `dagger/control/` 为已验证控制基础；外层 `dagger/` 负责权限、部署与集合版适配。
- `dagger/robot_profile.py`区分机器人编号和型号；不能恢复只允许300的判断，也不能用编号猜型号。
- S2/S3共用main，ROBOT_TYPE只标记数据集；不在采集器中添加S2模型到S3的推理转换。旧S3预览工具已移除，生产DAgger仍须拒绝预览服务并校验控制接口。
- `dagger/assets/` 为随版本发布的配置与网页；哈希在 `dependencies.py` 和 `SOURCE.json`。
- `teleop_snapshot/` 为已打补丁的审查快照，不是待打补丁的运行依赖。
- 300正式部署在 `/home/ubuntu/CB_Autolife_Data_Collector`；部署路径不等于修改硬件的授权。

## 环境和验证

无构建步骤。机器人依赖ROS2 Jazzy、既有robot_env/lerobot环境、相机SHM。
DAgger额外依赖审核过的V4、Placo/Web环境和Thor baseline/frame服务。
新终端先 `source scripts/source_ros_env.sh`，再 `set +u`。

```bash
/home/ubuntu/miniconda3/envs/lerobot/bin/python -B -m unittest discover -s tests -p 'test_*.py'
node tests/test_feedback.js
bash -n start_lerobot_official_collect.sh
ROS_DOMAIN_ID=211 /usr/bin/python3 tests/vr_ros_smoke.py --subtasks
ROS_DOMAIN_ID=211 ROBOT_ID=345 DAGGER_ROBOT_PROFILE=s3_v3_1 DAGGER_DEPENDENCY_ROOT=/path/to/reviewed/dependencies /usr/bin/python3 tests/dagger_ros_smoke.py
```

完整流程和证据见 `ROBOT300_TESTING_zh.md`。不在真实控制域注入合成动作。
上面的DAgger示例仅用于隔离测试；必须先核验依赖版本，不把隔离域测试当作345的硬件控制验收。
测试数据放独立目录并标记不可训练；已保存用户数据不因整理代码而改名、覆盖或删除。

## 保持不变的契约

- 普通/单人/子任务使用原厂输入；DAgger使用独立增量控制器，不能同时运行两套。
- 常规数采默认`ACTION_MODE=status_target`；DAgger强制`joint`并使用控制器输出代理。旧15维`ACTION_MODE=eef`已删除，不恢复；保留独立`WITH_EEF_STATE`/`WITH_EEF_ACTION`追加及离线转换。
- 单人面键需GL+GR；DAgger面键无需双Grip，Grip用于人工接管。
- A开始、B保存/阶段标记、X仅复位、Y丢弃；DAgger手柄Y改为短按松开丢弃、长按满1秒失败保存。终端键位按README保留。
- 原厂X复位未接入，不调用另一套控制器补偿；错误和未确认结果不能显示成功。
- DAgger X和启动器独立复位使用本轮任务姿态快照，参考多任务计划的 task_poses/poses_file；检查所有保存关节及夹爪，不退回原厂HOME规划或全局姿态。独立复位不启动Thor/录制/VR，保留控制权、版本和碰撞检查；自定义任务必须明确选择姿态，不按文字猜测。无robot_id的历史多任务计划只可用于300。
- 21维顺序、相机字段、FIFO回执、epoch/session门控与训练mask不得随意修改。
- DAgger B/finish确认任务成功；手柄Y长按满1秒、启动器fail或终端K确认任务失败并保存；F仅封口。手柄Y短按松开丢弃；启动器/浏览器Shift+Y仍直接丢弃，右摇杆不负责失败保存。outcome区分autonomous_success/human_corrected_success/failure，接管记录与原纠错mask独立。只写同目录侧文件，不建分类文件夹；侧文件与episode事务提交，存储失败/丢弃不发布，历史缺失不回填。结果未知时不能重发保存或改变原标签。FIFO第三段JSON可选，旧命令/回执兼容。
- `dagger.train_mask`保留整帧AND语义；`dagger.action_train_mask`按身体/双夹爪来源分别屏蔽。新旧DAgger schema不能在同一数据目录混写。
- 可选EEF action追加在joint后；逐维mask同步扩展，EEF部分保持0，不能从joint来源推断EEF专家身份。离线转换必须同步投影action mask及统计量。
- 模型动作只在Thor端digest前修改；客户端不能裁剪或重写proposal/ACK前缀。
- 保留低CPU写入器、队列上限和同步阈值；不能通过丢帧或放宽阈值掩盖异常。
- 不清错、不停止或重启ARM服务来规避冲突。真机运动、服务切换需当前任务明确授权。
- 用户已授权DAgger每次启动自检修复：仅正常退出核实空闲的控制中心关节模块和Void-Cog后台；保留界面/导航。未知/忙碌/录制中/状态不明不得退出，不强杀；--check只读，attach不修复，其他数采模式不变。
- 版本校验失败先检查差异，不增加跳过校验开关。更新打包资源时同时核对来源、哈希和回归。
- 正式版不新增个人后缀；旧名称仅保留在兼容入口、竞争进程检测和历史来源中。

- 后台保存必须区分save_queued与save；仅落盘计成功。F/Q等待队列；失败保留raw job。四种模式同一机制；深度单位、12位量化、纠错掩码不得降级。
