# 启动器切换 checkpoint 和任务

300桌面两个启动器共用 `gui/checkpoint_panel.py` 和 `dagger/inference_options.py`：

- **云蝶DAgger启动器**：先保存/丢弃本条并停止采集服务；点击“选择 checkpoint…”，刷新列表，选模型并“应用所选模型”。等待显示“实际加载：… · 就绪”。任务框可选择六个已有任务或直接输入英文指令。启动采集服务后仍需按A/“开始推理+录制”，不会自动运动。
- **多任务推理启动器**：先结束本轮并等待客户端退出；点击同名按钮切换。已核实模型写入原 `laundry_cycle_plan.json` 的 `checkpoint`；原任务顺序、导航点、位姿、人工确认和中途接管保持。下次开始前仍核对Thor实际模型，不再强制034000。

刷新显示的是服务端实际加载结果。只在下拉框选中并未切换；按“应用”并收到就绪才算完成。两个窗口使用同一个8777服务，另一端改模型后，旧计划会被拦截，应刷新核实。正在推理/存在未关闭session时拒绝切换，同一个模型不重载。

## 任务与夹爪阶段

| 阶段 | 行为 |
|---|---|
| 自动 | 仅精确识别六个已有任务和旧的 `Pick the laundry bag.`；未知文本拒绝开始，提示显式选阶段 |
| 抓取 / pick | 初始跟随连续模型动作；下一轮实测某侧≥120°才锁该侧360°，不因chunk后部预测提前闭合 |
| 放置 / place | 已闭合侧维持360°；模型某帧≤60°才从该帧释放10°，仅执行/提交前缀中的释放进入后续状态 |
| 连续 / continuous | 退出阶段锁持，使用已有连续限速/防抖模式；适用于需要中途反复开合的任务，夹爪可能比阶段模式慢 |

所有动作变化仍在Thor生成 `physical_actions` 后、digest前完成。机器人客户端不改动作或ACK。任务文本原样传给模型并用于采集；选对夹爪阶段不等于模型支持该任务。

终端示例（只读预检）：

```bash
COLLECTOR_MODE=dagger DAGGER_PUBLISH=0 \
DAGGER_GRIPPER_PHASE=pick \
DAGGER_EXPECTED_CHECKPOINT=laundry_bag_six_tasks_continual_034000 \
TASK_TEXT='Pick the laundry bag from the washing machine.' \
bash start_lerobot_official_collect.sh checkpoint_check --check
```

## 安装与回滚边界

模型管理目前配置为Thor `192.168.8.224:8777`，只管理 `groot-n1.7` baseline容器；8778和ARM服务不受影响。列表来自 `/home/wayne/groot_n1_7_runtime/models` 的完整目录，校验21维顺序、RGB输入、horizon、处理器JSON及权重文件完整长度。新checkpoint需先放在该目录，再刷新；本功能不从H200下载权重。

300使用专用 `~/.ssh/id_ed25519_collector_checkpoint`，Thor对应公钥设置 `restrict,command="/usr/bin/python3 /home/wayne/groot_n1_7_runtime/collector_checkpoint_control.py"`，只允许JSON `list/switch`，没有交互shell或端口转发。私钥和token不入库。Thor helper源为 `dagger/thor_checkpoint_control.py`，需能通过既有 `sudo -n` 运行容器管理脚本；主机密钥必须通过已认证渠道核实。

外部项目补丁存于 `integrations/checkpoint_options/`：Thor服务端、300普通推理客户端、多任务启动器分别应用，须先 `patch --dry-run` 核对当前代码。不能把Thor旧客户端整个覆盖到300；300已有的新鲜关节反馈等待修复必须保留。多任务启动器需能导入正式集合仓库 `/home/ubuntu/CB_Autolife_Data_Collector`。

切换采用已有 `manage.sh`，直接挂载目标目录，不复制数GB权重。服务端短期维护锁禁止新session在关闭容器前插入。加载成功后原子更新 `models/baseline` 符号链接；已选名称与实际模型不一致会拒绝推理。加载失败会在确认可安全切换时恢复旧模型；若服务状态未知或另有session，不强行重启，界面显示失败及Thor日志路径。切换可能需要数分钟。

回滚业务选项：在模型窗口应用之前的checkpoint；连续夹爪模式可单独选择。回滚程序：结束所有推理/采集后，恢复部署备份中的启动器、客户端和Thor三个源文件，按原参数重启推理服务。只恢复GUI不会撤销服务器逻辑；切勿停止/清错ARM来回滚。

## 验证

- 集合版离线回归、新的模型互斥/回滚/路径校验测试。
- `xvfb-run -a /usr/bin/python3 tests/checkpoint_gui_smoke.py`：模拟后端验证刷新、忙碌拒绝、应用及显示，不接触机器人。
- Thor阶段规则、完整digest/ACK、controller-submission receipt回归。
- 实时三RGB+关节观测只读推理，逐个discard并close；不发执行ACK、不发布机器人动作。
- 真机验收由现场完成：先检查显示的checkpoint/task/阶段，再A开始；分别验证抓取锁持、放置释放、人工接管和保存标签。模型切换本身不做复位或开爪。
