"""Live RGB-D + synthetic signals, isolated domain211; never use for training.

Exercises the real MZJ supervisor -> provenance topics -> collection recorder ->
FIFO save/discard -> Parquet/videos. No controller, Thor, or motor publisher.
"""
import json
import os
from pathlib import Path
import sys
import time
import numpy as np
import cv2
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger
from record_lerobot_official import OfficialLeRobotRecorder, parse_args
from dagger.supervisor import CollectorDaggerSupervisor


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '211':
        raise SystemExit('requires isolated domain 211')
    args = parse_args()
    assert args.dagger and args.with_depth and args.with_head and args.with_upper_waist
    base = args.output_dir.parent
    base.mkdir(parents=True, exist_ok=True)
    (base / 'SYNTHETIC_SIGNALS_DO_NOT_TRAIN.txt').write_text('Live cameras, synthetic joints/actions; NO hardware calls.\n')
    fifo = base / '.official_recording_control'
    os.mkfifo(fifo)
    args.control_fifo, args.status_file = str(fifo), str(base / '.official_recording_status.json')
    args.episode_event_file = str(base / '.official_episode_event.json')
    cv2.setNumThreads(1)
    rclpy.init(args=['--ros-args', '-p', f'trace_root:={base / "dagger_trace"}',
        '-p', f'rgb_collector_fifo:={fifo}', '-p', f'rgbd_collector_fifo:={fifo}',
        '-p', 'human_finish_only:=true'])
    recorder = OfficialLeRobotRecorder(args)
    supervisor = CollectorDaggerSupervisor()
    supervisor._forward_enable = lambda enabled: (True, 'synthetic no-hardware enable')
    executor = SingleThreadedExecutor()
    executor.add_node(recorder); executor.add_node(supervisor)
    grip = [False]
    packet = lambda value: String(data=json.dumps(value))
    state = {key: {'position': values} for key, values in (
        ('leg_waist_joint_state', [0.] * 4), ('left_arm_joint_state', [0.] * 7),
        ('right_arm_joint_state', [0.] * 7), ('neck_joint_state', [0.] * 3),
        ('left_gripper_state', [10.]), ('right_gripper_state', [10.]))}
    body = dict(left_arm_target_joints_position=[0.] * 7, right_arm_target_joints_position=[0.] * 7,
                leg_waist_target_joints_position=[0.] * 4, neck_target_joints_position=[0.] * 3)
    grippers = dict(left_gripper_target_joints_position=[10.], right_gripper_target_joints_position=[10.])
    def inputs():
        stamp = time.time_ns()
        recorder._state_cb(packet({**state, 'timestamp_ns': stamp}))
        supervisor._on_joint_feedback(packet(state))
        supervisor._on_controller_status(packet(dict(state='ARMED', hardware_ready=True,
            hardware_enabled=True, hardware_enable_pending=False, target_source='cartesian_ik')))
        supervisor._on_vr_input(packet(dict(leftController={'gripActive': grip[0]}, rightController={'gripActive': False})))
        supervisor._last_policy_monotonic_ns = time.monotonic_ns()
        if grip[0] and supervisor._machine.mode.value == 'EXPERT_ACTIVE':
            supervisor._on_expert_eef(packet(dict(pos_left_in_robot=[0., 0., .8],
                authority_epoch=supervisor._machine.authority_epoch, collector_session_id=supervisor._session_id)))
        for gripper, command in ((False, body), (True, grippers)):
            supervisor._on_controller_output(packet(dict(command=command, gripper=gripper,
                timestamp_ns=stamp, origin_authority_epoch=supervisor._machine.authority_epoch,
                session_id=supervisor._session_id)))
    supervisor.create_timer(.01, inputs)
    def pump_until(predicate, timeout=30):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
            recorder.process_control_commands()
            if recorder.episode_invalid:
                raise RuntimeError('live RGB-D invalid: ' + str(recorder.episode_invalid_reason))
        assert predicate(), 'timeout: ' + str(supervisor._save_notice)
    def command(action):
        response = supervisor._on_launcher_action(action, Trigger.Response())
        assert response.success, response.message
        pump_until(lambda: not supervisor._button_worker_active, 150)
    try:
        recorder.start_control_listener()
        pump_until(lambda: recorder.all_requested_cameras_seen() and bool(recorder.state_buffer), 10)
        recorder.create_dataset()
        for trial in range(3):
            grip[0] = False
            command('start')
            assert supervisor._machine.mode.value == 'POLICY_ACTIVE'
            pump_until(lambda: recorder.current_episode_frames >= 35)
            if trial == 0:
                grip[0] = True
                pump_until(lambda: recorder.current_episode_frames >= 100)
            command('discard' if trial == 2 else 'finish')
            assert not supervisor._intervention_id, supervisor._save_notice
            print('RGBD_TRIAL_PASS', trial+1, json.dumps(supervisor._save_notice, ensure_ascii=False), flush=True)
        assert recorder.saved_episodes == 2 and recorder.discarded_episodes == 1
        assert recorder.episodes_invalidated == 0
    finally:
        supervisor.shutdown_session()
        recorder.finish()
        executor.shutdown(timeout_sec=2)
        supervisor.destroy_node(); recorder.destroy_node(); rclpy.shutdown()
    import pyarrow.parquet as pq
    tables = [pq.read_table(path) for path in sorted(args.output_dir.glob('data/**/*.parquet'))]
    import pyarrow as pa
    data = pa.concat_tables(tables).to_pandas()
    for episode, rows in data.groupby('episode_index'):
        mask = np.array([int(np.asarray(x).item()) for x in rows['dagger.train_mask']])
        source = np.array([int(np.asarray(x).item()) for x in rows['dagger.control_source']])
        assert np.all(mask[source != 1] == 0)
        assert (mask.sum() > 0) if episode == 0 else (mask.sum() == 0)
        assert all(len(x) == 21 for x in rows['action'])
        print('RGBD_PARQUET_PASS', json.dumps(dict(episode=int(episode), frames=len(rows), expert_frames=int(mask.sum()))), flush=True)
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    dataset = LeRobotDataset(args.repo_id, root=args.output_dir, video_backend='pyav')
    indices = set()
    for _, rows in data.groupby('episode_index'):
        indices.update((int(rows.index[0]), int(rows.index[len(rows)//2]), int(rows.index[-1])))
    for index in sorted(indices):
        row = dataset[index]
        assert row['action'].shape == (21,)
        assert all(key in row for key in ('observation.images.hand_left', 'observation.images.hand_right',
                                         'observation.images.rgbd_head_color', 'observation.images.rgbd_head_depth'))
    print('RGBD_SAVE_RESTART_DISCARD_LOADER_PASS hardware_calls=0 synthetic_signals=true', flush=True)


if __name__ == '__main__':
    main()
