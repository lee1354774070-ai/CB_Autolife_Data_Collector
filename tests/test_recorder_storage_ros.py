"""Opt-in real recorder/storage integration, isolated DDS, synthetic frames.

Source ROS and set ROS_DOMAIN_ID=211 and HF_HUB_OFFLINE=1. No executor spins,
camera reads, motion publishers, VR input or policy requests are used.
"""
import hashlib
import contextlib
import io
import itertools
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
from dagger_labels import DaggerLabels

import numpy as np

recorder = None
if os.environ.get('ROS_DOMAIN_ID') == '211':
    try:
        import record_lerobot_official as recorder
    except ImportError:
        pass


@unittest.skipIf(recorder is None, 'requires ROS + LeRobot and isolated ROS_DOMAIN_ID=211')
class RecorderStorageTest(unittest.TestCase):
    def test_eef_flags_real_frame_path_save_resume_and_conversion(self):
        from convert_dataset import convert_dataset
        from eef_schema import SCHEMA_PATH, parse_eef_pose
        from lerobot.datasets import LeRobotDataset
        recorder.rclpy.init()
        try:
            for with_state, with_action, with_dagger in itertools.product((False, True), repeat=3):
                with self.subTest(state=with_state, action=with_action, dagger=with_dagger), tempfile.TemporaryDirectory() as directory:
                    root = Path(directory) / 'dataset'
                    argv = ['recorder', '--output-dir', str(root), '--image-source', 'ros', '--vcodec', 'h264',
                            '--encoder-threads', '1', '--with-head', '--with-upper-waist']
                    if with_state:
                        argv.append('--with-eef-state')
                    if with_action:
                        argv.append('--with-eef-action')
                    if with_dagger:
                        argv.extend(['--dagger', '--action-mode', 'joint'])
                    with patch.object(sys, 'argv', argv):
                        args = recorder.parse_args()
                    node = recorder.OfficialLeRobotRecorder(args)
                    pose = parse_eef_pose({f'{side}_eef_pose': {'position': [.4, .2, .8], 'rotation': [0, 0, 0, 1]}
                                           for side in ('left', 'right')})
                    def seed(anchor):
                        for stamp in (anchor - .005, anchor + .005):
                            node.state_buffer.append(recorder.StateSample(np.zeros(21, np.float32), np.ones(21, np.float32), stamp, stamp))
                            packet = {f'{side}_eef_pose': {'position': [.4, .2, .8], 'rotation': [0, 0, 0, 1]}
                                      for side in ('left', 'right')}
                            packet['timestamp'] = stamp
                            with patch.object(recorder.time, 'time', return_value=stamp):
                                node._state_eef_cb(recorder.String(data=json.dumps(packet)))
                        packet['left_eef_pose']['position'][0] = .5
                        packet['timestamp'] = anchor - .005
                        with patch.object(recorder.time, 'time', return_value=anchor - .005):
                            node._action_eef_cb(recorder.String(data=json.dumps(packet)))
                        if with_dagger:
                            stamp = round((anchor - .005) * 1e9)
                            boundary = max(node.dagger.started_ns, stamp - 20_000_000)
                            command = dict(timestamp_ns=stamp, authority_epoch=3,
                                original_command_timestamp_ns=stamp,
                                dagger=dict(timestamp_ns=stamp + 1_000_000, authority_timestamp_ns=boundary,
                                    authority_epoch=3, control_source=1, is_intervention=1,
                                    intervention_id=1, trial_id='synthetic-eef'))
                            node.dagger.ingest(command)
                            node.dagger.ingest({**command, 'origin_authority_epoch': 2}, gripper=True)
                            groups = {group.name: [1.] * group.size for group in node.schema.groups
                                      if group.command_target_key is not None}
                            node.action_body_buffer.append(recorder.TimedSample(groups, stamp / 1e9, stamp / 1e9))
                            node.action_gripper_buffer.append(recorder.TimedSample((1., 1.), stamp / 1e9, stamp / 1e9))
                        for camera in args.cameras:
                            node._store_image(camera_name=camera, image=np.full((32, 32, 3), 120, np.uint8),
                                              stamp_sec=anchor, received_sec=anchor, width=32, height=32, is_depth_map=False)
                    try:
                        now = time.time()
                        seed(now)
                        if with_state or with_action:
                            # Fresh camera/joints alone must not create an EEF
                            # dataset shell when the controller reports nothing.
                            node.eef_state_buffer.clear()
                            node.action_eef_buffer.clear()
                            with self.assertRaisesRegex(RuntimeError, 'EEF telemetry'):
                                node.create_dataset()
                            self.assertFalse(root.exists())
                            seed(time.time())
                        node.create_dataset()
                        self.assertTrue(node.start_episode('synthetic-eef'))
                        start = time.time() + .1
                        if with_dagger:
                            seed(start - 1 / 30)
                            with patch.object(recorder.time, 'time', return_value=start):
                                node._record_tick()
                            self.assertTrue(node.dagger.ready)
                        for index in range(3):
                            anchor = start + index / 30
                            seed(anchor)
                            with patch.object(recorder.time, 'time', return_value=anchor + .06):
                                node._record_tick()
                            self.assertFalse(node.episode_invalid, node.episode_invalid_reason)
                        self.assertEqual(node.current_episode_frames, 3)
                        self.assertTrue(node.save_current_episode('synthetic-eef'))
                    finally:
                        node.finish()
                        node.destroy_node()
                    data = LeRobotDataset('local/eef-smoke', root=root, video_backend='pyav')
                    first = data[0]
                    self.assertEqual(tuple(first['observation.state'].shape), (21 + 14 * with_state,))
                    self.assertEqual(tuple(first['action'].shape), (21 + 14 * with_action,))
                    if with_state:
                        np.testing.assert_allclose(first['observation.state'][21:].numpy(), pose)
                    if with_action:
                        self.assertAlmostEqual(float(first['action'][21]), .5)
                    if with_dagger:
                        np.testing.assert_array_equal(first['dagger.action_train_mask'].numpy(),
                                                      [1] * 14 + [0, 0] + [1] * 5 + [0] * (14 * with_action))
                    layout = json.loads((root / SCHEMA_PATH).read_text())
                    self.assertEqual(len(layout['features']['action']['groups']['eef']), 14 * with_action)
                    convert_dataset(root, Path(directory) / 'training', state='eef' if with_state else 'joint',
                                    action='eef+gripper' if with_action else 'joint')
                    # A second real recorder reopens this root without losing
                    # the sidecar during create/resume/empty-workspace cleanup.
                    node = recorder.OfficialLeRobotRecorder(args)
                    try:
                        seed(time.time())
                        node.create_dataset()
                        self.assertEqual(node.saved_episodes, 1)
                        changed = json.loads((root / 'meta/info.json').read_text())
                        changed['features']['action']['shape'][0] += 14
                        with self.assertRaisesRegex(RuntimeError, 'action size'):
                            node._validate_resume_schema(changed)
                    finally:
                        node.finish()
                        node.destroy_node()
                    self.assertEqual(json.loads((root / SCHEMA_PATH).read_text()), layout)
        finally:
            recorder.rclpy.shutdown()

    def test_nonfinite_limits_and_unsafe_encoding_modes_are_rejected(self):
        for option, value in (('--max-sync-delta-sec', 'nan'), ('--image-poll-fps', 'inf'),
                              ('--max-episode-temp-gb', '0'), ('--min-free-disk-gb', '-1'),
                              ('--max-episode-seconds', 'nan'), ('--depth-max', 'nan'),
                              ('--batch-encoding-size', '2'), ('--streaming-encoding', None)):
            argv = ['recorder', '--output-dir', '/unused', option] + ([value] if value else [])
            with self.subTest(option=option), patch.object(sys, 'argv', argv), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    recorder.parse_args()
                self.assertEqual(error.exception.code, 2)
        with patch.object(sys, 'argv', ['recorder', '--output-dir', '/unused', '--action-mode', 'eef', '--with-eef-state']), contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                recorder.parse_args()
        with patch.object(sys, 'argv', ['recorder', '--output-dir', '/unused', '--with-eef-action', '--fallback-action-to-state']), contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                recorder.parse_args()

    def test_four_workflows_save_invalidate_discard_and_resume(self):
        import pyarrow.parquet as pq
        import av
        recorder.rclpy.init()
        try:
            for mode in ('keyboard', 'vr', 'subtask', 'dagger'):
                with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                    root = Path(directory) / 'dataset'
                    argv = ['recorder', '--output-dir', str(root), '--image-source', 'ros',
                            '--image-writer-threads', '2', '--vcodec', 'h264',
                            '--task-name', 'SYNTHETIC STORAGE TEST - NOT TRAINING DATA',
                            '--status-file', str(Path(directory) / 'status.json'),
                            '--episode-event-file', str(Path(directory) / 'event.json')]
                    if mode == 'subtask':
                        argv += ['--subtasks-json', '["pick", "place"]']
                    if mode in ('vr', 'dagger'):
                        argv += ['--with-depth']
                    if mode == 'dagger':
                        argv += ['--dagger', '--action-mode', 'joint', '--with-head', '--with-upper-waist']
                    with patch.object(sys, 'argv', argv):
                        args = recorder.parse_args()
                    node = recorder.OfficialLeRobotRecorder(args)
                    def seed():
                        now = time.time()
                        for name in args.cameras:
                            depth = name in args.depth_cameras
                            image = np.full((64, 64, 1 if depth else 3), 1000 if depth else 80,
                                            dtype=np.uint16 if depth else np.uint8)
                            node._store_image(camera_name=name, image=image, stamp_sec=now,
                                              received_sec=now, width=64, height=64, is_depth_map=depth)
                        node.state_buffer.append(recorder.StateSample(
                            np.zeros(node.schema.size, dtype=np.float32),
                            np.zeros(node.schema.size, dtype=np.float32), now, now))
                    def add():
                        frame = {'task': args.task_name}
                        for name, feature in node.dataset.features.items():
                            if name.startswith('observation.images.'):
                                frame[name] = node.latest_images[name.removeprefix('observation.images.')].image_hwc
                            elif name in ('observation.state', 'action', 'subtask_index') or name.startswith('dagger.'):
                                frame[name] = np.zeros(feature['shape'], dtype=feature['dtype'])
                        if mode == 'dagger':
                            stamp = 1_800_000_000_000_000_000
                            labels = DaggerLabels()
                            packet = dict(timestamp_ns=stamp+20_000_000, authority_epoch=3,
                                original_command_timestamp_ns=stamp+20_000_000,
                                dagger=dict(timestamp_ns=stamp+22_000_000, authority_timestamp_ns=stamp,
                                    authority_epoch=3, control_source=1, is_intervention=1,
                                    intervention_id=1, trial_id='synthetic'))
                            labels.ingest(packet)
                            labels.ingest({**packet, 'origin_authority_epoch':2,
                                           'original_command_timestamp_ns':stamp-1_000_000}, gripper=True)
                            values = labels.frame((stamp+30_000_000)/1e9,
                                                   {'action_body':-10, 'action_gripper':-10}, .5)
                            frame.update({k:np.asarray(v,dtype=np.int64).reshape(-1) for k,v in values.items()})
                        node.storage_budget.admit(frame)
                        node.dataset.add_frame(frame)
                        node.current_episode_frames += 1
                        node.frames_written += 1
                        node.session_frames_written += 1
                    try:
                        seed()
                        node.create_dataset()
                        if mode == 'dagger':
                            self.assertEqual(json.loads(Path(args.status_file).read_text())['event'], 'ready')
                        self.assertTrue(node.start_episode('test'))
                        self.assertIsNotNone(node.storage_budget.started)
                        for _ in range(3):
                            add()
                        self.assertTrue(node.save_current_episode('test'))
                        saved = {p: hashlib.sha256(p.read_bytes()).hexdigest()
                                 for group in ('data', 'videos', 'meta', 'annotations')
                                 for p in (root / group).rglob('*') if p.is_file()}
                        self.assertEqual(node.saved_episodes, 1)
                        seed()
                        self.assertTrue(node.start_episode('failed-write'))
                        def fail(*_):
                            raise OSError('injected disk full')
                        node.dataset.writer.image_writer._write = fail
                        # Partial first-row failure is possible as soon as the
                        # next camera observes the first worker's exception.
                        try:
                            add()
                        except OSError:
                            pass
                        with self.assertRaises(OSError):
                            node.dataset.writer.image_writer.wait_until_done()
                        node._record_tick()
                        self.assertTrue(node.episode_invalid)
                        self.assertFalse(node.is_recording)
                        self.assertEqual(node.saved_episodes, 1)
                        self.assertFalse(node.start_episode('must-reject'))
                        self.assertTrue(node.discard_current_episode('test'))
                        for p, digest in saved.items():
                            self.assertEqual(hashlib.sha256(p.read_bytes()).hexdigest(), digest)
                        seed()
                        self.assertTrue(node.start_episode('after-discard'))
                        for _ in range(3):
                            add()
                        self.assertTrue(node.save_current_episode('test'))
                        self.assertEqual(node.saved_episodes, 2)
                        self.assertEqual(sum(pq.read_metadata(p).num_rows for p in (root / 'data').rglob('*.parquet')), 6)
                        if mode == 'dagger':
                            table = pq.read_table(sorted((root / 'data').rglob('*.parquet')),
                                                  columns=['dagger.train_mask','dagger.action_train_mask'])
                            self.assertEqual(table['dagger.train_mask'].to_pylist(), [0]*6)
                            self.assertEqual(table['dagger.action_train_mask'].to_pylist(),
                                             [[1]*14+[0,0]+[1]*5]*6)
                        for path in (root / 'videos').rglob('*.mp4'):
                            with av.open(str(path)) as video:
                                self.assertEqual(sum(1 for _ in video.decode(video=0)), 3)
                    finally:
                        node.finish()
                        node.destroy_node()
        finally:
            recorder.rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
