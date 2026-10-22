# 2026-10-22 DAgger三类任务结果标签

同一数据集、同一保存事务，不建分类文件夹、不复制或重编码视频。
B/finish确认成功；新增启动器fail按钮、窗口Shift+F、owned/attach终端F确认失败并保存；Y仍只丢弃，X仅复位。

每条annotations/dagger/episode_XXXXXX.json新增outcome：autonomous_success、human_corrected_success或failure。原task_success、human_takeover、成功时的success_source保持兼容；失败时task_success=false、success_source=null，仍记录是否接管。
标注来源继续是operator_confirmation；结果不改变dagger.train_mask或dagger.action_train_mask。Parquet schema不变。历史文件不回写，缺失结果按未标注处理。

保存回执success表示文件提交结果，与task_success独立；录制损坏、磁盘失败和丢弃不能发布结果标签。未确认保存再次核对原请求，后续按钮不能重发保存或重标原结果。

## 无动作证据

300独立验证目录：/home/ubuntu/collector_validation/20261022_dagger_outcomes；正式采集数据未用于测试。

- unit.log：292项Python测试通过，无跳过。
- 真实LeRobot合成episode保存及Parquet回读覆盖自主成功、接管成功、未接管失败、接管失败；失败条正常提交但task_success=false，原逐维mask分别保持模型全0/身体19维纠错、夹爪2维保持0，未新增分类目录。
- 覆盖非法布尔值、跨trial拒绝、保存失败不发布、Y丢弃清除暂存、legacy save/FIFO兼容、EEF组合及转换、保存超时后改按其他结果不重标。
- ros_s2.log与ros_s3.log：隔离domain211的真实supervisor/mapper/FIFO回归，包含未接管/接管后的fail服务，硬件调用0、模型请求0；S3是配置回归，不是现场验收。
- attach.log：新增F指令成功、重复请求保护、Q仅断开附加终端，宿主保留。
- 虚拟显示GUI检查：失败按钮和急停按钮可见，业务按钮初始禁用，窗口打开不启动采集；hardware_calls=0。
- 接管振动JS测试通过。

现场按A/B完成自主成功一条、Grip后B完成人工纠正成功一条、启动器失败保存一条、Y丢弃一条；核对前三条outcome及第四条不新增数据。
未代替操作员执行真机动作；长时间录制负载仍需现场持续验收。

## 手柄失败键补充

手柄单独向下按住右摇杆1秒，沿用fail保存入口；无需Grip，A/B/X/Y的按下/松开行为保持不变。头显上方说明及启动器提示同步更新。
检测复用原有VR包，在同一控制锁内、立即Grip处理后运行，不新增订阅/图像处理或网络请求，不增加振动。双摇杆桌面组合、短按和只推方向不触发；追踪丢失、300ms以上输入中断、忙碌、待确认保存和跨trial要求重新松开再按。

- 299项Python测试通过，无跳过；新增7项真实检测函数边界测试。
- S2/S3隔离ROS均通过：实际_on_vr_input连续右摇杆包触发一次save，携带task_success=false；未接管和接管后的失败均保留正确human_takeover，硬件调用0、模型请求0。
- 原ABXY、快速扳机与连续条目回归通过。模拟握持接管回调约0.31/0.50ms，不代表硬件延迟。
- 接管振动JS检查通过；前端只改说明，不改WebXR传输或姿态计算。
- 实际头显摇杆操作仍需现场验收；未执行真机动作。
