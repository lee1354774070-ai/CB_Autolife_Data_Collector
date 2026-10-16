# S3 推理适配：只读预览阶段

日期：2026-10-16。分支：`s3-collector`。原S2的`main`和部署保持不变。
本版复用已验证的GR00T N1.7 baseline权重，完成关节约定转换；没有宣称S2模型在S3上的抓取成功率。

## 改动位置

- `s3_thor_preview.py`：独立Thor服务入口，S3实测状态先转换为S2关节约定，再送入原模型。模型完成反归一化生成physical_actions后转换回S3，再进入原夹爪阶段处理和proposal digest。
- 复用 `robot_schema.py` 的21维顺序；严格检查shape、有限值和19个身体关节的S3 URDF硬限位。超限报出关节名称和数值，不裁剪动作。
- 夹爪仍用10～360°，保留已验证的按任务阶段锁持/释放逻辑。抓取由下一次实测≥120°才锁持；放置在预测≤60°的对应帧释放。左右独立。
- 只支持baseline、K=1和精确匹配的两个laundry instruction。没有加入SOMA、历史引导或新依赖。
- 服务mode是 `policy_only_baseline_s3_preview`，生产DAgger客户端会拒绝连接；`/ack`、`/controller_ack`、`/cancel`明确拒绝。只读测试使用health/start/discard/close，不导入ROS或控制SDK。
- `tests/remote/s3_saved_readonly.py`：从已有S3数据集取首/中/尾观测，不修改原数据；检查返回动作、限位和digest，逐次discard并关闭自己创建的session，报告保存到新的独立目录。

## 运动学依据与适用边界

从SDK的S2 v2.2及S3 v3.1 URDF递推到同一世界坐标系，比较每个关节的旋转轴。
需要反号的索引（从0开始）为：`1,2,3,4,5,7,8,9,11,12,16,17`；其他维度保持不变。
S3输入和模型输出使用同一个可逆符号映射。SDK已经负责原始电机方向和比例，不再次处理电机编码器值。
347的ARM配置确认夹爪对外范围仍为10～360°；这不等于负载下夹紧效果已验收。

来源SHA256：

- SDK `robot_v2_2.urdf`：`0a2c883e19aa1af25c04fa91b6f669a46640f181ceba631fa1b78d2fa138cae0`
- SDK `robot_v3_1.urdf`：`f6d4de074adc1afdabc4960b8eda39ce0fe99148dd6c217d253cf1189a236179`
- 已有V4快照 `robot_v2_2_simplified.urdf`：`792931382c9e317b833e661c49ab45dbdd64185671b5bb9f00868ee90ef2fecc`

200组固定随机种子的共同限位内姿态验证：转换后双腕相对于各自零位的世界旋转差最大约0.0222°。
**两代连杆长度和关节位置不同**，这200组姿态的腕部位置差最大约0.10米。符号映射没有做末端位置重定向，也没有修复视角/数据分布差异。
不能把相同21维或限位检查通过当作真机轨迹安全与任务成功的证明。

## 测试结果

- 已验证的采集适配：347隔离ROS域211中227项Python测试通过；网页仅接管振动测试通过。
- 新增9项S3测试在本地和Thor实际运行环境全部通过，包括输入/输出映射、不修改原数组、非法输入、限位拒绝、夹爪锁持/释放、discard不锁定未执行释放、真实proposal digest以及执行回执拒绝。
- 用347已保存的真实静态VR观测（共364帧，取第0/182/363帧）请求独立8779服务。3次均返回40×21，全部身体关节在S3硬限位内、夹爪在10～360°内、digest一致、discard/close成功。
- 左肘实测约−110°；适配前旧模型首帧为+52.5～+70.0°，适配后本次三个首帧为−92.62°、−91.99°、−100.21°。这是不同随机推理样本，不是固定种子的模型性能对比。
- 三次端到端耗时约2577/1405/334毫秒；Thor端处理约1967/330/314毫秒。首轮冷启动和网络抖动仍需处理，未通过长时延迟验收。
- 首帧身体关节相对实测的最大差约34.86°、36.05°、32.15°。只检查了硬限位；后续必须验证轨迹速度、碰撞和实机坐标标定，不能直接整帧下发。
- 未发送真机动作或执行ACK。347原ARM/vision父进程1754/77725保持运行；原Thor服务进程2585/2645未重启。

数据报告在347：
`/home/ubuntu/collector_validation/20261016_s3_347/dagger_validation/s3_mapping_01/readonly-inference.json`。
临时服务token副本已删除；报告和Git提交不含token。原始数据集未改动。

## 复现

已有Thor验证部署：`/home/wayne/s3_inference_validation_20261016/collector`。
使用独立容器 `s3-groot-preview-20261016`，端口8779，模型只读挂载到 `/model`。
该容器验证后已停止；需要再次回放时在Thor运行：

```bash
sudo docker start s3-groot-preview-20261016
sudo docker logs --tail 20 s3-groot-preview-20261016
```

等待出现 `S3 preview ready`。新环境部署时需使用已有GR00T兼容环境，将本分支和原推理runtime同时加入PYTHONPATH，使用独立端口；入口为：

```bash
PYTHONPATH=/collector:/workspace python -B -m s3_thor_preview \
  --model-dir /model --token-file /run/secrets/groot_token \
  --host 0.0.0.0 --port 8779
```

347回放（`S3_PREVIEW_TOKEN_FILE`需指向已安全部署的同一服务token；不要写入Git或命令参数值）：

```bash
cd /home/ubuntu/collector_validation/20261016_s3_347/collector
source scripts/source_ros_env.sh
set +u
export PYTHONPATH="$PWD:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="/home/ubuntu/miniconda3/envs/robot_env/lib:${LD_LIBRARY_PATH:-}"
export HF_HUB_OFFLINE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
: "${S3_PREVIEW_TOKEN_FILE:?请先指定已部署的服务token文件路径}"
../venv/bin/python -B tests/remote/s3_saved_readonly.py \
  --dataset /home/ubuntu/nas/s3_347_vr_01/dataset \
  --token-file "$S3_PREVIEW_TOKEN_FILE" \
  --url http://192.168.8.224:8779 \
  --output-dir ../dagger_validation/s3_mapping_next
```

输出目录必须不存在。任务文本取自数据，必须是现有精确匹配的抓取或放置instruction。
本脚本只回放已有数据，不启动录制、不驱动手臂。
测试完成后在Thor停止**这个独立容器**：

```bash
sudo docker stop --timeout 10 s3-groot-preview-20261016
```

原8777/8778服务不需要停止、重启或重新配置；回滚S3预览仅需停用该入口。没有把预览集成到生产DAgger启动器。
