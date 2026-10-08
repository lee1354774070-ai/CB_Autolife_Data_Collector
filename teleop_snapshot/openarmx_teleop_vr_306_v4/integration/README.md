# 306 V4 閬ユ搷浼犺緭鏃犲姩浣滈泦鎴愭祴璇?
`transport_integration_client.py` 鍙悜 `dry_run:=true` 鐨勬ˉ鍙戦€佹棤鎵嬫焺浣嶅Э銆?鎵€鏈夋寜閽噴鏀剧殑鏁版嵁锛屼笉鐢熸垚鏈烘鑷傜洰鏍囥€傚畠妫€鏌ワ細

- 绗竴浠?WebRTC DataChannel 鐨?generation 涓?ACK锛?- 绗簩浠ｈ繛鎺ユ浛鎹㈢涓€浠ｈ繛鎺ワ紝鏃т唬鏁版嵁鍜屾棫 offer 涓嶄細褰卞搷鏂拌繛鎺ワ紱
- WebSocket 鍥為€€鍜?WebRTC 鍏辩敤鍏ㄥ眬 sequence锛岄噸澶?鏃у抚浼氳涓㈠純锛?- WebSocket 鍥為€€鍚庡綋鍓?WebRTC 鑳界户缁帴鏀跺拰 ACK 鏈€鏂板抚銆?
## 杩愯

缁堢涓€锛堝彧鑳戒互 dry-run 鍚姩锛夛細

```bash
cd /home/ubuntu/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch openarmx_teleop_vr_306_v4 full_vr_teleop.launch.py dry_run:=true
```

缁堢浜岋細

```bash
/home/ubuntu/miniconda3/envs/robot_env/bin/python \
  /home/ubuntu/ros2_ws/src/openarmx_teleop_vr_306_v4/integration/transport_integration_client.py \
  --base-url https://127.0.0.1:8446
```

鎴愬姛鏃舵渶鍚庝竴琛屽簲涓猴細

```text
ALL TRANSPORT INTEGRATION CHECKS PASSED (no motion targets sent)
```

瀹㈡埛绔粯璁ゆ嫆缁濋潪鏈満鍦板潃銆傛祴璇曡繙绔?dry-run 妗ユ椂蹇呴』鏄惧紡澧炲姞
`--allow-remote`锛屼絾鎺ㄨ崘鐩存帴鍦?306 鏈満杩愯锛岄伩鍏嶆妸缃戠粶闂娣峰叆妗ョ殑閫昏緫楠岃瘉銆?
