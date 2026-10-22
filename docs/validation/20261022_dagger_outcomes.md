# 2026-10-22 DAgger三类任务结果标签

同一数据集、同一保存事务，不建分类文件夹、不复制或重编码视频。
B/finish确认成功；手柄Y长按1秒、启动器fail按钮、窗口Shift+F、owned/attach终端F确认失败并保存；Y短按松开丢弃，X仅复位。

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

## 手柄Y短按/长按及同步反馈

按用户最新约定，替换右摇杆入口：手柄Y短按松开丢弃；按住满1秒确认任务失败并保存。按下时不丢弃，长按保存后松开不再丢弃，持续按住只触发一次。启动器/浏览器Shift+Y仍直接丢弃，启动器Shift+F/终端F仍失败保存；普通VR/子任务键位不变。
检测复用原VR包，在同一控制锁内、立即Grip处理后运行，不新增订阅、图像处理或网络请求。追踪丢失、300ms以上输入中断、忙碌、待确认保存和跨trial要求重新松开再按。浏览器键盘包增加source标识，避免误套手柄长按。
网页说明、头显上方常亮说明、各模式提示、启动器页脚和终端说明同步。头显下方及启动器显示同一notice：正在保存/已保存任务失败/保存错误；先发布正在保存，再等待硬件停用及录制回执。仅Grip接管振动。

300验证目录：/home/ubuntu/collector_validation/20261022_dagger_y_hold。

- unit.log：302项Python测试通过，无跳过，包含真实LeRobot保存/标签/mask回读及Y短按、长按、阈值松开、取消、失联、忙碌、跨trial与启动器反馈。
- ros_s2.log、ros_s3.log：实际_on_vr_input输入流均通过；Y按下未丢弃，长按仅保存一次，松开不丢弃；未接管及接管后的失败均保留正确task_success=false和human_takeover。原快速扳机/ABX/保存回执回归通过，硬件调用0、模型请求0。
- node tests/test_feedback.js：2项通过；常亮Y说明、底部保存中/已保存/错误内容和颜色通过，仍仅接管振动。JS语法及资源哈希通过。
- 真实头显Y操作仍需现场验收，不将隔离测试作为真机动作验收。未启用或重启ARM服务，未执行真机动作。

## 每次启动自检修复

验证目录：`/home/ubuntu/collector_validation/20261022_dagger_startup_repair`。

- `unit.log`：隔离ROS域下314项Python测试全部通过，无跳过；包含忙碌/未知模块拒绝、状态改变取消、正常退出竞态、退出超时不强杀及启动器错误反馈。
- `--check`只读发现控制中心关节模块及Void-Cog后台冲突，所有原进程保留。
- 实际待命启动只正常退出已锁定空闲的关节模块，以及无任务/循环/录制的Void-Cog后台和看门狗。停止前重读ROS状态，PID身份变化、未知发布者、动作输入、忙碌或未确认状态均拒绝；退出超时不强杀。
- 实际联调补齐状态回调的JSON导入，并修复看门狗在读取/proc期间自行退出导致的误报。对应竞态加入回归测试。
- 300待命启动到网页8447、三RGB相机与录制器ready；`standby_ready.json`记录DISARMED、hardware_publish=false、recording=false、recorded_frames=0，`standby_observation.json`的电机命令计数为空。
- `duplicate_start.log`确认第二套启动被拒绝；原待命实例保留，随后正常退出所有自建节点，最终0帧、0条episode。
- 控制中心界面/导航及Void-Cog前端保留。未清错、复位、启用或重启ARM，未开始模型推理；被暂停的关节模块和Void-Cog后台不会自动恢复。

这是启动/退出的软件验收，未执行真实运动或长时间录制；此前相机时间戳异常不在本次修复范围。
