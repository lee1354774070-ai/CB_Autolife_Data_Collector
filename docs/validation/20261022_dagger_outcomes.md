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
