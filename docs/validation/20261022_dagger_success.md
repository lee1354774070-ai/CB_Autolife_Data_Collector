# 2026-10-22 DAgger任务成功标记

基线main：0bbaa841e19eaebf9ea9a6f22865aadd68e0def7。S2/S3共用本次实现，未合并历史S3预览分支。

## 行为与存储

- B、启动器“确认任务成功并保存”、终端A均表示操作员确认任务成功。原有A开始、Grip接管、Y丢弃、X仅复位保持不变。
- 本条从未接管：task_success=true、success_source=autonomous、human_takeover=false。
- 本条曾接管：task_success=true、success_source=human_intervention、human_takeover=true。依据监督器锁存的接管记录，不根据纠错帧数猜测；接管但暂时没有可训练动作也属于接管成功。
- 标注来源annotation_source=operator_confirmation，包含episode_index、trial_id、任务、帧数、原始录制有效纠错帧数和保存请求ID。
- 侧文件dataset/annotations/dagger/episode_XXXXXX.json与数据、视频一起由EpisodeStore提交，失败不发布，丢弃清除暂存，提交日志负责中断恢复。保存回执新增可选task_outcome；回执success仍只表示文件操作结果。
- dagger.train_mask和dagger.action_train_mask计算及字段均不变，模型段仍为0，追加EEF维度继续沿用现有掩码规则。Parquet schema未增加字段。
- 自主成功可作为人工筛选过的自模仿数据池；接管成功继续用原掩码选择有效纠错。训练端按episode_index连接侧文件；模型成功不改写为专家来源。
- 历史条目、旧客户端普通save没有侧文件时按未标注处理，不补写。只在原有schema相同时续录；旧掩码schema升级限制仍有效。
- FIFO旧格式save request_id兼容；新版可追加紧凑JSON。命令名归一化，request_id及JSON原样保留。单次写入不得超过PIPE_BUF，关联请求ID及trial_id防止跨条目误标。
- 升级需正常结束旧宿主后重启新版整套DAgger，attach或GUI单独升级不能给旧宿主增加成功标记。

## 验证

独立副本：/home/ubuntu/collector_validation/20261022_dagger_success_labels/source；日志在其父目录。测试不修改正式部署、既有数据或ARM服务。

- unit_final.log：286项Python测试全部通过，无跳过。
- 真实LeRobot合成数据保存/Parquet回读：自主成功3帧动作掩码全0；接管成功3帧保留身体19维、夹爪2维为0；旧整帧掩码保持0。两个成功侧文件与episode索引0/1及纠错帧数0/3一致。
- 所有EEF开关及S2/S3数据型号组合通过保存、续录和离线转换；成功侧文件转换前后字节一致。
- 覆盖短暂接管且纠错帧数0、跨trial拒绝、非法JSON、真实FIFO读入保持大小写、旧save兼容、保存失败/丢弃不发布成功、原保存数据保持不变及中断恢复。
- ros_s2_final.log、ros_s3_final.log：在300的隔离ROS域211分别使用300/S2和345/S3配置；按键、接管、来源锁存、连续条目重置、保存不重发、丢弃和复位门控通过。硬件调用0，实际模型请求0；不是345现场验收。
- attach.log：附加终端预检、单次命令、断开保留宿主、旧契约拒绝通过。
- 接管振动JavaScript测试1项通过；shell/JavaScript/Python语法、源码与diff检查通过。

ROS完整流程使用300已有的审核依赖副本/home/ubuntu/collector_validation/20260928_modes_dryrun_01/dependencies，五项哈希均匹配。300现场controller.yaml仍有自定义复位姿态，保留版本检查，不自动覆盖或跳过。

## 现场复核

1. 正常退出旧宿主，整体升级后用新任务名开始。A开始、全程不接管，任务完成后B确认；检查模型自主成功提示和第0条侧文件。
2. X复位后A开始、Grip接管完成任务，B确认；检查人工接管成功提示及第1条侧文件，原纠错掩码仍区分模型/有效人工动作。
3. X复位后A开始，再Y丢弃；已保存episode和侧文件数量均不增加。
4. 正常退出后续录，检查已有数据和侧文件保持不变。旧无侧文件条目继续按未知处理。

本次完成源码及无动作验证，未进行新的真机运动、头显实操或长时负载验收。
