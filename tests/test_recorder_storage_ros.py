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
    @unittest.skipUnless(os.environ.get('COLLECTOR_TEST_NVJPEG') == '1', 'requires opt-in CUDA JPEG decode')
    def test_nvjpeg_shm_poll_sync_and_real_dataset_save(self):
        from PIL import Image
        from shm_camera import ShmFrame, PIXEL_FORMAT_MJPEG
        recorder.rclpy.init()
        try:
            with tempfile.TemporaryDirectory() as directory:
                root = Path(directory) / 'dataset'
                with patch.object(sys, 'argv', ['recorder', '--output-dir', str(root),
                                               '--vcodec', 'h264',
                                               '--encoder-threads', '1']):
                    args = recorder.parse_args([*sys.argv[1:], '--no-background-save'])
                node = recorder.OfficialLeRobotRecorder(args)
                node.shm_readers = {}  # Synthetic SHM functions below, never hardware.
                self.assertIsNone(node.jpeg_decoder)  # No CUDA work before joint readiness.
                self.assertEqual(args.jpeg_decoder, 'auto')
                def seed(anchor):
                    for stamp in (anchor - .005, anchor + .005):
                        node.state_buffer.append(recorder.StateSample(
                            np.zeros(16, np.float32), np.ones(16, np.float32), stamp, stamp))
                    node.start_image_capture()
                    frames = {}
                    for i, name in enumerate(args.cameras):
                        bgr = np.full((64, 64, 3), 40 + 60 * i, np.uint8)
                        _, encoded = recorder.cv2.imencode('.jpg', bgr)
                        stamp = round(anchor * 1e9)
                        frames[name] = ShmFrame(stamp, 64, 64, 3, PIXEL_FORMAT_MJPEG,
                                                len(encoded), encoded.tobytes())
                    with patch.object(recorder, 'read_shm_metadata', side_effect=lambda spec: (stamp,)), \
                         patch.object(recorder, 'read_shm_frame', side_effect=lambda spec, meta: frames[spec.name]), \
                         patch.object(recorder.time, 'time', return_value=anchor):
                        node._poll_shm_images()
                try:
                    seed(time.time())
                    self.assertEqual(node.image_decoding['backend'], 'nvjpeg')
                    node.create_dataset()
                    self.assertTrue(node.start_episode('synthetic-gpu-jpeg'))
                    start = time.time() + .1
                    for i in range(3):
                        anchor = start + i / 30
                        seed(anchor)
                        with patch.object(recorder.time, 'time', return_value=anchor + .06):
                            node._record_tick()
                    self.assertFalse(node.episode_invalid, node.episode_invalid_reason)
                    self.assertEqual(node.current_episode_frames, 3)
                    self.assertTrue(node.save_current_episode('synthetic-gpu-jpeg'))
                    for position, value in [('head', 40), ('left', 100), ('right', 160)]:
                        with Image.open(root / 'first_frame' / f'{position}_frame' / f'{position}_episode_00000.png') as image:
                            np.testing.assert_array_equal(np.asarray(image), np.full((64, 64, 3), value, np.uint8))
                    saved = {p: hashlib.sha256(p.read_bytes()).hexdigest()
                             for folder in ('meta', 'data', 'videos', 'first_frame')
                             for p in (root / folder).rglob('*') if p.is_file()}
                    self.assertTrue(node.start_episode('injected-gpu-failure'))
                    with patch.object(node.jpeg_decoder, 'decode', side_effect=RuntimeError('injected GPU failure')):
                        with self.assertRaisesRegex(RuntimeError, 'GPU JPEG decode failed'):
                            seed(time.time() + 1)
                    self.assertTrue(node.episode_invalid)
                    self.assertEqual(node.episode_invalid_reason, 'jpeg_decode_failed')
                finally:
                    node.finish()
                    node.destroy_node()
                self.assertEqual(json.loads((root / 'meta/info.json').read_text())['total_episodes'], 1)
                for path, digest in saved.items():
                    self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), digest)
                events = [json.loads(line) for line in (root / 'sync_log.jsonl').read_text().splitlines()]
                selected = next(e for e in events if e['event'] == 'image_decoder_selected')
                self.assertEqual(selected['backend'], 'nvjpeg')
                self.assertEqual(selected['requested_backend'], 'auto')
                self.assertEqual(selected['device'], 'cuda:0')
                self.assertEqual(next(e for e in events if e['event'] == 'episode_started')['image_decoding']['backend'], 'nvjpeg')
        finally:
            recorder.rclpy.shutdown()

    def test_eef_flags_real_frame_path_save_resume_and_conversion(self):
        from convert_dataset import convert_dataset
        from eef_schema import SCHEMA_PATH, parse_eef_pose
        from lerobot.datasets import LeRobotDataset
        recorder.rclpy.init()
        try:
            combinations = itertools.product((False, True), (False, True), (False, True),
                                             ('autolife_s1', 'autolife_s3'))
            for with_state, with_action, with_dagger, robot_type in combinations:
                with self.subTest(state=with_state, action=with_action, dagger=with_dagger, robot_type=robot_type), tempfile.TemporaryDirectory() as directory:
                    root = Path(directory) / 'dataset'
                    argv = ['recorder', '--output-dir', str(root), '--image-source', 'ros', '--vcodec', 'h264',
                            '--encoder-threads', '1', '--with-head', '--with-upper-waist', '--robot-type', robot_type]
                    if with_state:
                        argv.append('--with-eef-state')
                    if with_action:
                        argv.append('--with-eef-action')
                    if with_dagger:
                        argv.extend(['--dagger', '--action-mode', 'joint'])
                    with patch.object(sys, 'argv', argv):
                        args = recorder.parse_args([*sys.argv[1:], '--no-background-save'])
                    node = recorder.OfficialLeRobotRecorder(args)
                    topics = {subscription.topic_name for subscription in node.subscriptions}
                    self.assertEqual(args.action_arm_topic in topics, with_dagger)
                    self.assertEqual(args.action_gripper_topic in topics, with_dagger)
                    self.assertEqual(args.state_eef_topic in topics, with_state)
                    self.assertEqual(args.action_eef_topic in topics, with_action)
                    self.assertNotIn(f'/topic_arm_target_robot_height_z_{recorder.TOPIC_NODE_ID}', topics)
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
                            start += 1 / 30  # Saved anchors must follow the provenance barrier.
                        for index in range(3):
                            anchor = start + index / 30
                            seed(anchor)
                            with patch.object(recorder.time, 'time', return_value=anchor + .06):
                                node._record_tick()
                            self.assertFalse(node.episode_invalid, node.episode_invalid_reason)
                        self.assertEqual(node.current_episode_frames, 3)
                        outcome = dict(trial_id=node.dagger.trial_id, task_success=True, human_takeover=True) if with_dagger else None
                        self.assertTrue(node.save_current_episode('synthetic-eef', dagger_outcome=outcome))
                    finally:
                        node.finish()
                        node.destroy_node()
                    data = LeRobotDataset('local/eef-smoke', root=root, video_backend='pyav')
                    self.assertEqual(data.meta.robot_type, robot_type)
                    first = data[0]
                    self.assertEqual(tuple(first['observation.state'].shape), (21 + 14 * with_state,))
                    self.assertEqual(tuple(first['action'].shape), (21 + 14 * with_action,))
                    np.testing.assert_array_equal(first['observation.state'][:21].numpy(), np.zeros(21))
                    np.testing.assert_array_equal(first['action'][:21].numpy(), np.ones(21))
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
                    converted_info = json.loads((Path(directory) / 'training/meta/info.json').read_text())
                    self.assertEqual(converted_info['robot_type'], robot_type)
                    if with_dagger:
                        sidecar = Path('annotations/dagger/episode_000000.json')
                        self.assertEqual((root / sidecar).read_bytes(), (Path(directory) / 'training' / sidecar).read_bytes())
                    for source in (root / 'first_frame').rglob('*.png'):
                        copied = Path(directory) / 'training' / source.relative_to(root)
                        self.assertEqual(copied.read_bytes(), source.read_bytes())
                    # A second real recorder reopens this root without losing
                    # the sidecar during create/resume/empty-workspace cleanup.
                    node = recorder.OfficialLeRobotRecorder(args)
                    try:
                        seed(time.time())
                        node.create_dataset()
                        self.assertEqual(node.saved_episodes, 1)
                        changed = json.loads((root / 'meta/info.json').read_text())
                        self.assertEqual(changed['robot_type'], robot_type)
                        changed['features']['action']['shape'][0] += 14
                        with self.assertRaisesRegex(RuntimeError, 'action size'):
                            node._validate_resume_schema(changed)
                    finally:
                        node.finish()
                        node.destroy_node()
                    self.assertEqual(json.loads((root / SCHEMA_PATH).read_text()), layout)
        finally:
            recorder.rclpy.shutdown()

    def test_robot_type_environment_and_explicit_argument(self):
        with patch.dict(os.environ, {"ROBOT_TYPE": "autolife_s3"}):
            with patch.object(sys, "argv", ["recorder", "--output-dir", "/unused"]):
                self.assertEqual(recorder.parse_args().robot_type, "autolife_s3")
            with patch.object(sys, "argv", ["recorder", "--output-dir", "/unused",
                                           "--robot-type", "explicit_test"]):
                self.assertEqual(recorder.parse_args().robot_type, "explicit_test")

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
        with patch.object(sys, 'argv', ['recorder', '--output-dir', '/unused', '--with-eef-action', '--fallback-action-to-state']), contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                recorder.parse_args()

    def test_action_mode_defaults_and_eef_append_flags(self):
        for mode in (None, 'status_target', 'joint'):
            for with_state, with_action in itertools.product((False, True), repeat=2):
                with self.subTest(mode=mode, state=with_state, action=with_action):
                    argv = ['recorder', '--output-dir', '/unused']
                    if mode is not None:
                        argv.extend(['--action-mode', mode])
                    if with_state:
                        argv.append('--with-eef-state')
                    if with_action:
                        argv.append('--with-eef-action')
                    with patch.object(sys, 'argv', argv):
                        args = recorder.parse_args()
                    self.assertEqual(args.action_mode, mode or 'status_target')
                    self.assertEqual(args.with_eef_state, with_state)
                    self.assertEqual(args.with_eef_action, with_action)
                    self.assertFalse(hasattr(args, 'action_height_topic'))

    def test_removed_eef_mode_and_height_option_are_rejected(self):
        for extra in ([], ['--with-eef-state'], ['--with-eef-action'],
                      ['--with-eef-state', '--with-eef-action']):
            with self.subTest(extra=extra):
                error = io.StringIO()
                argv = ['recorder', '--output-dir', '/unused', '--action-mode', 'eef', *extra]
                with patch.object(sys, 'argv', argv), contextlib.redirect_stderr(error):
                    with self.assertRaises(SystemExit) as result:
                        recorder.parse_args()
                self.assertEqual(result.exception.code, 2)
                self.assertIn("invalid choice: 'eef'", error.getvalue())
        error = io.StringIO()
        argv = ['recorder', '--output-dir', '/unused', '--action-height-topic', '/unused']
        with patch.object(sys, 'argv', argv), contextlib.redirect_stderr(error):
            with self.assertRaises(SystemExit) as result:
                recorder.parse_args()
        self.assertEqual(result.exception.code, 2)
        self.assertIn('unrecognized arguments: --action-height-topic', error.getvalue())

    def test_four_workflows_save_invalidate_discard_and_resume(self):
        import pyarrow.parquet as pq
        import av
        from PIL import Image
        from first_frames import RGB_POSITIONS
        from extract_first_frames import extract_first_frames
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
                    # Opt-in hardware integration uses only synthetic images
                    # on isolated DDS. The normal suite remains CPU-portable.
                    if accelerator := os.environ.get('COLLECTOR_TEST_VIDEO_ACCELERATOR'):
                        argv += ['--video-accelerator', accelerator, '--video-ffmpeg-bin',
                                 os.environ.get('COLLECTOR_TEST_FFMPEG', 'ffmpeg')]
                    # Tiny CPU fixtures are below some hardware encoders'
                    # minimum dimensions. Hardware checks use camera-sized RGB.
                    height, width = (480, 640) if accelerator else (64, 64)
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
                            image = np.full((height, width, 1 if depth else 3), 1000 if depth else 80,
                                            dtype=np.uint16 if depth else np.uint8)
                            node._store_image(camera_name=name, image=image, stamp_sec=now,
                                              received_sec=now, width=width, height=height, is_depth_map=depth)
                        node.state_buffer.append(recorder.StateSample(
                            np.zeros(node.schema.size, dtype=np.float32),
                            np.zeros(node.schema.size, dtype=np.float32), now, now))
                    def add(source=1):
                        frame = {'task': args.task_name}
                        for name, feature in node.dataset.features.items():
                            if name.startswith('observation.images.'):
                                frame[name] = node.latest_images[name.removeprefix('observation.images.')].image_hwc
                                if name.removeprefix('observation.images.') not in args.depth_cameras:
                                    frame[name] = np.full_like(frame[name], 80 + 30 * node.saved_episodes + 10 * node.current_episode_frames)
                            elif name in ('observation.state', 'action', 'subtask_index') or name.startswith('dagger.'):
                                frame[name] = np.zeros(feature['shape'], dtype=feature['dtype'])
                        if mode == 'dagger':
                            stamp = 1_800_000_000_000_000_000
                            labels = DaggerLabels()
                            packet = dict(timestamp_ns=stamp+20_000_000, authority_epoch=3,
                                original_command_timestamp_ns=stamp+20_000_000,
                                dagger=dict(timestamp_ns=stamp+22_000_000, authority_timestamp_ns=stamp,
                                    authority_epoch=3, control_source=source, is_intervention=source,
                                    intervention_id=source, trial_id=f'synthetic-{node.saved_episodes}'))
                            labels.ingest(packet)
                            labels.ingest({**packet, 'origin_authority_epoch':2,
                                           'original_command_timestamp_ns':stamp-1_000_000}, gripper=True)
                            values = labels.frame((stamp+30_000_000)/1e9,
                                                   {'action_body':-10, 'action_gripper':-10}, .5)
                            frame.update({k:np.asarray(v,dtype=np.int64).reshape(-1) for k,v in values.items()})
                            node.dagger.trial_id = packet['dagger']['trial_id']
                            node.dagger.expert_frames += int(any(values['dagger.action_train_mask']))
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
                            add(source=0)
                        outcome = dict(trial_id=node.dagger.trial_id, task_success=True, human_takeover=False) if mode == 'dagger' else None
                        self.assertTrue(node.save_current_episode('test', dagger_outcome=outcome))
                        self.assertEqual(json.loads(Path(args.status_file).read_text())['event'], 'save_queued')
                        self.assertEqual(node.saved_episodes, 0)
                        seed()
                        self.assertTrue(node.start_episode('concurrent-capture'))
                        add(source=0)
                        self.assertTrue(node._poll_background_saves(wait=True))
                        self.assertTrue(node.is_recording)
                        self.assertEqual(node.current_episode_frames, 1)
                        self.assertTrue(node.discard_current_episode('test concurrent discard'))
                        saved = {p: hashlib.sha256(p.read_bytes()).hexdigest()
                                 for group in ('data', 'videos', 'meta', 'annotations', 'first_frame')
                                 for p in (root / group).rglob('*') if p.is_file()}
                        self.assertEqual(node.saved_episodes, 1)
                        # Encoder/first-frame failures are injected in the actual
                        # worker tests; parent monkeypatches do not cross processes.
                        if node.hardware_video_encoder is not None:
                            self.assertTrue((root / 'annotations/video_encoding/episode_000000.json').is_file())
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
                        outcome = dict(trial_id=node.dagger.trial_id, task_success=True, human_takeover=True) if mode == 'dagger' else None
                        self.assertTrue(node.save_current_episode('test', dagger_outcome=outcome))
                        self.assertTrue(node.seal_batch('seal-with-pending-save'))
                        self.assertEqual(node.background_saves.pending, 0)
                        self.assertEqual(node.saved_episodes, 2)
                        self.assertEqual(len(list((root / 'first_frame').rglob('*.png'))), 6)
                        for position in RGB_POSITIONS.values():
                            for episode in range(2):
                                path = root / 'first_frame' / f'{position}_frame' / f'{position}_episode_{episode:05d}.png'
                                with Image.open(path) as image:
                                    self.assertEqual(image.mode, 'RGB')
                                    np.testing.assert_array_equal(np.asarray(image), np.full((height, width, 3), 80 + 30 * episode, np.uint8))
                        self.assertEqual(sum(pq.read_metadata(p).num_rows for p in (root / 'data').rglob('*.parquet')), 6)
                        if mode == 'dagger':
                            table = pq.read_table(sorted((root / 'data').rglob('*.parquet')),
                                                  columns=['dagger.train_mask','dagger.action_train_mask'])
                            self.assertEqual(table['dagger.train_mask'].to_pylist(), [0]*6)
                            self.assertEqual(table['dagger.action_train_mask'].to_pylist(),
                                             [[0]*21]*3 + [[1]*14+[0,0]+[1]*5]*3)
                            paths = sorted((root / 'annotations/dagger').glob('*.json'))
                            self.assertEqual(len(paths), 2)
                            outcomes = [json.loads(p.read_text()) for p in paths]
                            self.assertEqual([x['success_source'] for x in outcomes], ['autonomous','human_intervention'])
                            self.assertEqual([x['episode_index'] for x in outcomes], [0,1])
                            self.assertEqual([x['expert_frames'] for x in outcomes], [0,3])
                            self.assertTrue(all(x['task_success'] for x in outcomes))
                            for takeover in (False, True):
                                seed()
                                self.assertTrue(node.start_episode('task-failure'))
                                for _ in range(3):
                                    add(source=int(takeover))
                                outcome = dict(trial_id=node.dagger.trial_id, task_success=False, human_takeover=takeover)
                                self.assertTrue(node.save_current_episode('failure-save', dagger_outcome=outcome))
                                self.assertTrue(node._poll_background_saves(wait=True))
                                label = json.loads((root / f'annotations/dagger/episode_{node.saved_episodes-1:06d}.json').read_text())
                                self.assertIs(label['task_success'], False)
                                self.assertEqual(label['outcome'], 'failure')
                                self.assertEqual(label['human_takeover'], takeover)
                                self.assertIsNone(label['success_source'])
                                receipt = json.loads(Path(args.status_file).read_text())
                                self.assertIs(receipt['success'], True)
                                self.assertEqual(receipt['task_outcome'], label)
                            table = pq.read_table(sorted((root / 'data').rglob('*.parquet')),
                                                  columns=['dagger.action_train_mask'])
                            self.assertEqual(table['dagger.action_train_mask'].to_pylist(),
                                [[0]*21]*3 + [[1]*14+[0,0]+[1]*5]*3 + [[0]*21]*3 + [[1]*14+[0,0]+[1]*5]*3)
                            self.assertEqual({p.name for p in (root / 'annotations/dagger').iterdir()},
                                             {f'episode_{i:06d}.json' for i in range(4)})
                        for path in (root / 'videos').rglob('*.mp4'):
                            with av.open(str(path)) as video:
                                self.assertEqual(sum(1 for _ in video.decode(video=0)), 3)
                        if node.hardware_video_encoder is not None:
                            from lerobot.datasets import LeRobotDataset
                            dataset = LeRobotDataset('local/hardware-save-test', root=root, video_backend='pyav')
                            self.assertEqual(len(dataset), node.saved_episodes * 3)
                            for index in (0, 2, 3, 5):
                                row = dataset[index]
                                self.assertEqual(tuple(row['observation.state'].shape), (node.schema.size,))
                                for camera in args.cameras:
                                    image = row[f'observation.images.{camera}']
                                    self.assertEqual(tuple(image.shape[-2:]), (height, width))
                                    if camera in args.depth_cameras:
                                        self.assertLess(abs(float(image.float().mean()) - 1000), 10)
                                    else:
                                        expected = (80 + 30 * (index // 3) + 10 * (index % 3)) / 255
                                        self.assertLess(abs(float(image.mean()) - expected), .03)
                    finally:
                        node.finish()
                        node.destroy_node()
                    # Backfill only temporary synthetic fixtures, after releasing
                    # the recorder lock. Exercise actual v3 writer metadata/codecs.
                    before = {p: hashlib.sha256(p.read_bytes()).hexdigest()
                              for group in ('meta', 'data', 'videos', 'annotations')
                              for p in (root / group).rglob('*') if p.is_file()}
                    kept = {}
                    for path in (root / 'first_frame').rglob('*.png'):
                        if path.name.endswith('00001.png'):
                            path.unlink()
                        else:
                            kept[path] = path.read_bytes()
                    self.assertEqual(extract_first_frames(root, report=lambda text: None),
                                     {'episodes': node.saved_episodes, 'saved': 3,
                                      'skipped': (node.saved_episodes - 1) * 3})
                    for position in RGB_POSITIONS.values():
                        with Image.open(root / 'first_frame' / f'{position}_frame' / f'{position}_episode_00001.png') as image:
                            self.assertEqual(image.size, (width, height))
                            self.assertEqual(image.mode, 'RGB')
                            self.assertLess(abs(float(np.asarray(image).mean()) - 110), 4)
                    for path, value in kept.items():
                        self.assertEqual(path.read_bytes(), value)
                    for path, digest in before.items():
                        self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), digest)
        finally:
            recorder.rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
