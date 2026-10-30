"""Exercise real start/reset method bodies without importing ROS or LeRobot.

Only the surrounding hardware/dataset dependencies are replaced. AST extraction
keeps these tests tied to production methods rather than copies of their logic.
"""

import ast
import os
import shutil
import json
import time
import tempfile
import threading
import unittest
from concurrent.futures import Future, ThreadPoolExecutor
from collections import Counter, deque
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import Mock as BaseMock, patch

def Mock(*args, **kwargs):
    kwargs.setdefault("background_saves", None)  # These are synchronous-path fixtures.
    kwargs.setdefault("active_episode_index", None)
    kwargs.setdefault("_last_shm_poll_sec", time.time())
    kwargs.setdefault("shm_readers", {})
    return BaseMock(*args, **kwargs)
from queue import SimpleQueue, Empty
from subtask_annotations import SubtaskAnnotations, check_pending_annotations, prepare_annotation
from dagger_labels import DaggerLabels, success_annotation, validate_features as validate_dagger_features
from episode_storage import StorageLimitError
from first_frames import stage_first_frames
from video_encoding import accelerated_rgb_encoding
from shm_camera import PIXEL_FORMAT_MJPEG


SOURCE = Path(__file__).resolve().parents[1] / 'record_lerobot_official.py'
tree = ast.parse(SOURCE.read_text())
methods = [item for cls in tree.body if isinstance(cls, ast.ClassDef)
           for item in cls.body if isinstance(item, ast.FunctionDef)
           and item.name in {'start_episode', '_reset_episode_state', 'mark_subtask', 'seal_batch',
                             'save_current_episode', 'discard_current_episode', 'finish', '_clear_warmup_images',
                             '_invalidate_episode', '_record_tick', '_record_frame', '_log_sync', '_flush_sync_log',
                             '_poll_background_saves'}]
methods.extend(item for cls in tree.body if isinstance(cls, ast.ClassDef)
               for item in cls.body if isinstance(item, ast.FunctionDef)
               and item.name in {'_write_command_status', 'process_control_commands', '_control_fifo_loop', '_validate_resume_schema', '_poll_shm_images', 'start_image_capture'})
methods.extend(item for item in tree.body if isinstance(item, ast.FunctionDef) and item.name in {'write_json_atomic', 'make_depth_encoder'})
namespace = {'time': time, 'Path': Path, 'os': os, 'json': json, 'prepare_annotation': prepare_annotation,
             'argparse': __import__('argparse'),
             'Any': Any, 'Empty': Empty, 'validate_dagger_features': validate_dagger_features,
             'shutil': shutil, 'StorageLimitError': StorageLimitError,
             'success_annotation': success_annotation,
             'PIXEL_FORMAT_MJPEG': PIXEL_FORMAT_MJPEG,
             'accelerated_rgb_encoding': accelerated_rgb_encoding, 'stage_first_frames': stage_first_frames}
exec(compile(ast.Module(body=methods, type_ignores=[]), str(SOURCE), 'exec'), namespace)


class RecorderControlsTest(unittest.TestCase):
    def test_background_failure_invalidates_only_active_capture(self):
        for active in (False, True):
            node = Mock(is_recording=active, episode_invalid=False, saved_episodes=3,
                        _background_error_logged=False)
            node.background_saves = Mock()
            node.background_saves.poll.side_effect = RuntimeError('injected encoder failure')
            self.assertFalse(namespace['_poll_background_saves'](node))
            self.assertEqual(node.saved_episodes, 3)
            if active:
                self.assertEqual(node._invalidate_episode.call_args.args[0], 'background_save_failed')
            else:
                node._invalidate_episode.assert_not_called()

    def test_seal_does_not_stop_or_save_pending_data(self):
        with tempfile.TemporaryDirectory() as tmp:
            node = Mock(is_recording=False, save_failed=False, episode_invalid=False,
                        saved_episodes=10, stop_requested=False)
            node.args = SimpleNamespace(output_dir=tmp)
            node.has_pending_episode.return_value = False
            node.episode_store.journal.exists.return_value = False
            self.assertTrue(namespace['seal_batch'](node, 'seal-1'))
            self.assertEqual(json.loads((Path(tmp)/'meta/collection_seal.json').read_text())['total_episodes'], 10)
            self.assertFalse(node.stop_requested)
            node.save_current_episode.assert_not_called()
            node.is_recording = True
            self.assertFalse(namespace['seal_batch'](node, 'seal-2'))
            self.assertEqual(json.loads((Path(tmp)/'meta/collection_seal.json').read_text())['request_id'], 'seal-1')
            node.save_current_episode.assert_not_called()

    def test_dagger_depth_tuning_keeps_quantizer_and_lossless_encoding(self):
        for dagger, codec in [(True, 'hevc'), (False, 'hevc'), (True, 'other')]:
            args = SimpleNamespace(with_depth=True, dagger=dagger, depth_vcodec=codec,
                                   depth_min=.05, depth_max=10., depth_shift=3.5, depth_use_log=True)
            with patch.dict(namespace, DepthEncoderConfig=lambda **kw:kw):
                result = namespace['make_depth_encoder'](args)
                self.assertEqual([result[k] for k in ('depth_min','depth_max','shift','use_log')],
                                 [.05,10.,3.5,True])
                if codec == 'hevc':
                    self.assertEqual(result['preset'], 'ultrafast')
                    self.assertEqual(result['extra_options']['x265-params'],
                                     'lossless=1:pools=2:frame-threads=1:bframes=0:scenecut=0')
                else:
                    self.assertNotIn('extra_options', result)
                args.with_depth = False
                self.assertIsNone(namespace['make_depth_encoder'](args))

    def test_dagger_start_excludes_delayed_pre_label_camera_on_every_trial(self):
        from time_sync import oldest_ready_sample
        from test_dagger_labels import packet, STAMP
        node = Mock()
        node.args = SimpleNamespace(dagger_start_timeout_sec=10, max_action_hold_sec=.5)
        node.dagger = DaggerLabels()
        for trial in range(4):
            now = STAMP / 1e9 + trial * 10 + .025
            node.started_sec = now - 2
            node.dagger.reset()
            node.dagger.started_ns = round((now - 2) * 1e9)
            body = packet()
            offset = trial * 10_000_000_000
            body['timestamp_ns'] += offset
            body['original_command_timestamp_ns'] += offset
            body['dagger']['timestamp_ns'] += offset
            body['dagger']['authority_timestamp_ns'] += offset
            body['dagger']['trial_id'] = f'trial-{trial}'
            node.dagger.ingest(body)
            node.dagger.ingest(body, gripper=True)
            node.action_body_buffer = node.action_gripper_buffer = [SimpleNamespace(stamp_sec=now)]
            with patch.object(time, 'time', return_value=now):
                namespace['_record_frame'](node)
            self.assertTrue(node.dagger.ready)
            # Delivered after clearing, but captured before the first label.
            late = SimpleNamespace(stamp_sec=now - .010, received_sec=now + .001)
            fresh = SimpleNamespace(stamp_sec=now + .020, received_sec=now + .021)
            with self.assertRaisesRegex(ValueError, 'missing or stale'):
                node.dagger.frame(late.stamp_sec, {}, .5)
            chosen = oldest_ready_sample([late, fresh], now + .1, .03, node.last_reference_stamp_sec)
            self.assertIs(chosen, fresh)
            delta = (body['timestamp_ns'] / 1e9 - fresh.stamp_sec) * 1000
            labels = node.dagger.frame(fresh.stamp_sec, {'action_body': delta, 'action_gripper': delta}, .5)
            self.assertEqual(labels['dagger.action_train_mask'], [1] * 21)
    def test_backend_selection_waits_for_state_and_starts_exactly_one_timer(self):
        for decoder in (None, SimpleNamespace(device='cuda:0')):
            node = Mock()
            node.image_decoding = {'backend': 'pending'}
            node.state_buffer = []
            node.args = SimpleNamespace(jpeg_decoder='auto', jpeg_device='cuda:0', image_poll_fps=120)
            selector = Mock(return_value=decoder)
            with patch.dict(namespace, select_jpeg_decoder=selector):
                with self.assertRaisesRegex(RuntimeError, 'joint-state packet'):
                    namespace['start_image_capture'](node)
                selector.assert_not_called()
                node.create_timer.assert_not_called()
                node.state_buffer.append(object())
                namespace['start_image_capture'](node)
                namespace['start_image_capture'](node)
                selector.assert_called_once()
                node.create_timer.assert_called_once_with(1/120, node._poll_shm_images)
                self.assertIs(node.jpeg_decoder, decoder)
                self.assertEqual(node.image_decoding['backend'], 'nvjpeg' if decoder else 'opencv')

    def test_strict_gpu_failure_does_not_enable_camera_polling(self):
        node = Mock()
        node.image_decoding = {'backend': 'pending'}
        node.state_buffer = [object()]
        node.args = SimpleNamespace(jpeg_decoder='nvjpeg', jpeg_device='cuda:0')
        with patch.dict(namespace, select_jpeg_decoder=Mock(side_effect=RuntimeError('no GPU'))):
            with self.assertRaisesRegex(RuntimeError, 'no GPU'):
                namespace['start_image_capture'](node)
        node.create_timer.assert_not_called()
        self.assertEqual(node.image_decoding['backend'], 'pending')

    def test_ros_images_never_initialize_nvjpeg(self):
        node = Mock()
        node.image_decoding = {'backend': 'upstream_ros'}
        selector = Mock()
        with patch.dict(namespace, select_jpeg_decoder=selector):
            namespace['start_image_capture'](node)
        selector.assert_not_called()
        node.create_timer.assert_not_called()

    def test_jpeg_only_poll_deduplicates_and_never_probes_bgr(self):
        from camera_config import CAMERA_SPECS
        spec = CAMERA_SPECS['hand_left']
        node = Mock()
        node._last_shm_poll_sec = time.time()
        node.shm_readers = {}
        node.jpeg_decoder = None
        node.args = SimpleNamespace(cameras=['hand_left'], depth_cameras=[])
        node.dataset = None
        node.last_shm_timestamps = {spec.meta_path: 1}
        metadata = Mock(return_value=(1,))
        frame = SimpleNamespace(timestamp_ns=2, width=1, height=1)
        with patch.dict(namespace, read_shm_metadata=metadata, read_shm_frame=Mock(return_value=frame),
                        frame_to_hwc=Mock(return_value=object()), shm_timestamp_sec=lambda a, b: b,
                        CAMERA_SPECS=CAMERA_SPECS):
            for _ in range(100):
                namespace['_poll_shm_images'](node)
            node._store_image.assert_not_called()
            self.assertEqual(metadata.call_count, 100)
            self.assertTrue(all(call.args == (spec,) for call in metadata.call_args_list))
            metadata.return_value = (2,)
            namespace['_poll_shm_images'](node)
            node._store_image.assert_called_once()
            namespace['_poll_shm_images'](node)
            node._store_image.assert_called_once()
            self.assertEqual(node.last_shm_timestamps[spec.meta_path], 2)
            node._store_image.reset_mock()
            metadata.reset_mock(return_value=True)
            metadata.return_value = None
            namespace['_poll_shm_images'](node)
            node._store_image.assert_not_called()
            metadata.assert_called_once_with(spec)
            node._store_image.reset_mock()
            metadata.reset_mock()
            node.dataset, node.active_cameras = object(), []
            namespace['_poll_shm_images'](node)
            node._store_image.assert_not_called()
            metadata.assert_not_called()

    def test_copy_all_cameras_before_decode_preserves_depth_generation(self):
        from camera_config import CAMERA_SPECS
        cameras = ['hand_left', 'hand_right', 'rgbd_head_depth']
        node = Mock()
        node._last_shm_poll_sec = time.time()
        node.shm_readers = {}
        node.jpeg_decoder = None
        node.args = SimpleNamespace(cameras=cameras, depth_cameras=['rgbd_head_depth'])
        node.dataset = None
        node.last_shm_timestamps = {}
        decoded = []
        def read(spec, metadata):
            # A JPEG decode lets the producer overwrite the depth snapshot.
            if spec == CAMERA_SPECS['rgbd_head_depth'] and decoded:
                return None
            return SimpleNamespace(timestamp_ns=1, width=1, height=1)
        def decode(*args, **kwargs):
            decoded.append(True)
            return object()
        with patch.dict(namespace, CAMERA_SPECS=CAMERA_SPECS,
                        read_shm_metadata=lambda spec: (1,), read_shm_frame=read,
                        frame_to_hwc=decode, shm_timestamp_sec=lambda a, b: b):
            namespace['_poll_shm_images'](node)
        self.assertEqual([c.kwargs['camera_name'] for c in node._store_image.call_args_list],
                         cameras)

    def test_copy_race_retry_is_bounded_and_keeps_new_source_timestamp(self):
        from camera_config import CAMERA_SPECS
        for succeeds in (False, True):
            node = Mock()
            node._last_shm_poll_sec = time.time()
            node.shm_readers = {}
            node.jpeg_decoder = None
            node.args = SimpleNamespace(cameras=['rgbd_head_depth'], depth_cameras=['rgbd_head_depth'])
            node.dataset = None
            node.last_shm_timestamps = {}
            frame = SimpleNamespace(timestamp_ns=2, width=1, height=1)
            read = Mock(side_effect=[None, frame if succeeds else None])
            with patch.dict(namespace, CAMERA_SPECS=CAMERA_SPECS,
                            read_shm_metadata=Mock(side_effect=[(1,), (2,)]),
                            read_shm_frame=read, frame_to_hwc=lambda *a, **k: object(),
                            shm_timestamp_sec=lambda stamp, now: stamp):
                namespace['_poll_shm_images'](node)
            self.assertEqual(read.call_count, 2)
            if succeeds:
                self.assertEqual(node._store_image.call_args.kwargs['stamp_sec'], 2)
            else:
                node._store_image.assert_not_called()

    def test_gpu_poll_batches_only_copied_jpegs_and_keeps_timestamps(self):
        from camera_config import CAMERA_SPECS
        cameras = ['hand_left', 'hand_right', 'rgbd_head_depth']
        node = Mock()
        node.args = SimpleNamespace(cameras=cameras, depth_cameras=['rgbd_head_depth'])
        node.dataset, node.last_shm_timestamps = None, {}
        copied = []
        outputs = [object(), object()]
        def read(spec, metadata):
            copied.append(spec.name)
            return SimpleNamespace(timestamp_ns=123, width=2, height=1, data=spec.name.encode(),
                                   pixel_format=2 if spec.name.endswith('depth') else PIXEL_FORMAT_MJPEG)
        def decode(frames):
            self.assertEqual(copied, cameras)
            self.assertEqual(frames, [(b'hand_left', 2, 1), (b'hand_right', 2, 1)])
            return outputs
        node.jpeg_decoder.decode.side_effect = decode
        with patch.dict(namespace, CAMERA_SPECS=CAMERA_SPECS, read_shm_metadata=lambda spec: (123,),
                        read_shm_frame=read, frame_to_hwc=Mock(return_value='depth'),
                        shm_timestamp_sec=lambda stamp, now: stamp):
            namespace['_poll_shm_images'](node)
            calls = node._store_image.call_args_list
            self.assertEqual([c.kwargs['image'] for c in calls], [*outputs, 'depth'])
            self.assertEqual([c.kwargs['stamp_sec'] for c in calls], [123]*3)
            namespace['_poll_shm_images'](node)
            self.assertEqual(node._store_image.call_count, 3)

    def test_gpu_decode_error_invalidates_and_does_not_publish_partial_batch(self):
        from camera_config import CAMERA_SPECS
        node = Mock()
        node.args = SimpleNamespace(cameras=['hand_left'], depth_cameras=[])
        node.dataset, node.last_shm_timestamps, node.is_recording = None, {}, True
        node.jpeg_decoder.decode.side_effect = RuntimeError('injected GPU failure')
        frame = SimpleNamespace(timestamp_ns=1, width=2, height=1, data=b'bad', pixel_format=PIXEL_FORMAT_MJPEG)
        with patch.dict(namespace, CAMERA_SPECS=CAMERA_SPECS, read_shm_metadata=lambda spec: (1,),
                        read_shm_frame=lambda *a: frame):
            with self.assertRaisesRegex(RuntimeError, 'GPU JPEG decode failed'):
                namespace['_poll_shm_images'](node)
        node._store_image.assert_not_called()
        node._invalidate_episode.assert_called_once()
        self.assertEqual(node._invalidate_episode.call_args.args[0], 'jpeg_decode_failed')
        self.assertEqual(node.last_shm_timestamps, {})

    def recorder(self):
        node = Mock()
        node.dagger = None
        node.hardware_video_encoder = None
        node.shm_readers = {}
        node.receipt_writer_closed = False
        node.mark_status_future = None
        node.pending_start_payload = None
        node.dataset = object()
        node.image_decoding = {'backend': 'opencv', 'device': None}
        node.subtasks = SubtaskAnnotations(())
        node.save_failed = False
        node.sync_log_failed = False
        node.storage_budget = Mock()
        node.is_recording = False
        node.episode_invalid = False
        node.episodes_invalidated = 0
        node.current_episode_frames = 0
        node.saved_episodes = 3
        node.session_frames_written = 42
        node.reported_image_overflows = {'left'}
        node.has_pending_episode.return_value = False
        node._acquire_motion_lock.return_value = True
        node.sync_reference_camera = 'left'
        node.image_buffers = {'left': deque([SimpleNamespace(stamp_sec=10)]), 'right': deque([object()])}
        node.latest_images = {name: samples[-1] for name, samples in node.image_buffers.items()}
        node._clear_warmup_images = lambda: namespace['_clear_warmup_images'](node)
        node._reset_episode_state = lambda: namespace['_reset_episode_state'](node)
        return node

    def test_failed_sync_log_never_acknowledges_start_or_allows_a_retry(self):
        node = self.recorder()
        node.args = SimpleNamespace(episode_event_file=None)
        node.sync_log = Mock()
        node.sync_log.write_event.side_effect = OSError('disk full')
        node._log_sync = lambda event: namespace['_log_sync'](node, event)
        node._flush_sync_log = lambda: namespace['_flush_sync_log'](node)
        node._invalidate_episode = lambda *a, **kw: namespace['_invalidate_episode'](node, *a, **kw)
        self.assertFalse(namespace['start_episode'](node, 'start', 'request'))
        self.assertTrue(node.sync_log_failed)
        self.assertFalse(node.is_recording)
        self.assertTrue(node.episode_invalid)
        self.assertEqual(node._write_command_status.call_args.args, ('start', False, 'request'))
        node.sync_log.close.assert_not_called()
        namespace['_reset_episode_state'](node)
        self.assertFalse(namespace['start_episode'](node, 'retry', 'second'))
        self.assertIn('sync log failed', node._write_command_status.call_args.kwargs['message'])

    def test_buffered_log_failure_prevents_publication_but_preserves_saved_data(self):
        with tempfile.TemporaryDirectory() as root:
            node = self.annotation_recorder(root)
            node.sync_log = Mock()
            node.sync_log.flush.side_effect = OSError('late disk failure')
            node._flush_sync_log = lambda: namespace['_flush_sync_log'](node)
            self.assertFalse(node.save_current_episode('save', 'failed'))
            node.episode_store.commit.assert_not_called()
            self.assertTrue(node.episode_invalid)
            self.assertTrue(node.sync_log_failed)
            self.assertEqual(node.saved_episodes, 3)

    def test_warmup_reset_keeps_latest_neighbor_even_after_previous_clear(self):
        node = self.recorder()
        right = node.latest_images['right']
        for _ in range(3):
            namespace['_clear_warmup_images'](node)
            self.assertFalse(node.image_buffers['left'])
            self.assertEqual(list(node.image_buffers['right']), [right])

    def test_start_clears_reference_keeps_one_neighbor_and_acks(self):
        node = self.recorder()
        neighbor = node.image_buffers['right'][-1]
        node.image_buffers['right'].appendleft(object())
        self.assertTrue(namespace['start_episode'](node, 'test', 'request-1'))
        self.assertTrue(node.is_recording)
        self.assertFalse(node.image_buffers['left'])
        self.assertEqual(list(node.image_buffers['right']), [neighbor])
        self.assertEqual(node.last_reference_stamp_sec, 10)
        self.assertIsNone(node.last_episode_anchor_stamp_sec)
        self.assertFalse(node.reported_image_overflows)
        node._write_command_status.assert_called_once_with(
            'start', True, 'request-1', episode_index=3, message='episode started')

    def test_start_keeps_storage_duration_timer_armed(self):
        node = self.recorder()
        node.storage_budget.start.side_effect = lambda: setattr(node.storage_budget, 'started', 123)
        self.assertTrue(namespace['start_episode'](node, 'test'))
        self.assertEqual(node.storage_budget.started, 123)

    def test_start_flushes_old_diagnostics_before_establishing_capture_boundary(self):
        node = self.recorder()
        def flush_old_log():
            self.assertFalse(node.is_recording)
            self.assertEqual(node.image_buffers['left'][-1].stamp_sec, 10)
        node._flush_sync_log.side_effect = flush_old_log
        self.assertTrue(namespace['start_episode'](node, 'test'))
        node._flush_sync_log.assert_called_once()

    def test_disk_guard_rejects_start_and_releases_motion_lock(self):
        node = self.recorder()
        node.storage_budget.start.side_effect = StorageLimitError('disk_reserve_low')
        self.assertFalse(namespace['start_episode'](node, 'test'))
        self.assertFalse(node.is_recording)
        node._release_motion_lock.assert_called_once()

    def test_diagnostic_disk_failure_does_not_prevent_invalidation(self):
        node = self.recorder()
        node.args = SimpleNamespace(episode_event_file='event.json')
        node._log_sync.side_effect = OSError('full log disk')
        with patch.dict(namespace, write_json_atomic=Mock(side_effect=OSError('full event disk'))):
            namespace['_invalidate_episode'](node, 'disk_reserve_low', 1)
        self.assertTrue(node.episode_invalid)
        self.assertFalse(node.is_recording)
        self.assertEqual(node.episodes_invalidated, 1)

    def test_each_rejected_start_is_acknowledged_without_clearing_frames(self):
        for field, value in [('dataset', None), ('is_recording', True), ('episode_invalid', True), ('current_episode_frames', 5)]:
            with self.subTest(field=field):
                node = self.recorder()
                setattr(node, field, value)
                self.assertFalse(namespace['start_episode'](node, 'test', 'rejected'))
                self.assertEqual(len(node.image_buffers['left']), 1)
                self.assertEqual(node._write_command_status.call_args.args, ('start', False, 'rejected'))

    def test_motion_lock_rejection_is_acknowledged(self):
        node = self.recorder()
        node._acquire_motion_lock.return_value = False
        self.assertFalse(namespace['start_episode'](node, 'test', 'busy'))
        node._write_command_status.assert_called_once_with('start', False, 'busy', message='robot motion lock is busy')

    def annotation_recorder(self, root):
        node = self.recorder()
        node.args = SimpleNamespace(task_name='put towel in basket', fps=30, output_dir=root, episode_event_file=None)
        node.subtasks = SubtaskAnnotations(('pick', 'handover', 'place'))
        node.is_recording = True
        node.stop_requested = False
        node.current_episode_frames = 3
        node.frames_written = 3
        node.has_pending_episode.return_value = True
        node.dataset = Mock()
        node.dataset.features = {}
        node._saved_dataset = node.dataset
        work = Path(root) / '.collector-work'
        work.mkdir(exist_ok=True)
        def clear_work():
            if work.exists():
                shutil.rmtree(work)
        def publish(_):
            if (work / 'annotations').exists():
                shutil.copytree(work / 'annotations', Path(root) / 'annotations', dirs_exist_ok=True)
            clear_work()
        node.episode_store = SimpleNamespace(work=work, journal=Path(root) / '.collector-commit.json',
                                            commit=Mock(side_effect=publish), discard=Mock(side_effect=clear_work), close=Mock())
        def open_workspace():
            work.mkdir(exist_ok=True)
            node.dataset = node._saved_dataset
        node._open_episode_dataset.side_effect = open_workspace
        node._invalidate_episode = lambda *a, **kw: namespace['_invalidate_episode'](node, *a, **kw)
        node.dataset.meta.video_keys = []
        node.dataset.writer.episode_buffer = {'size': 3, 'subtask_index': [-1] * 3}
        node.task_episode_counts = Counter()
        node.session_saved_by_task = Counter()
        node.session_discarded_by_task = Counter()
        node.discarded_episodes = 0
        node.episode_invalid_reason = None
        node.sync_log = None
        node.motion_lock_handle = None
        node.save_current_episode = lambda *a: namespace['save_current_episode'](node, *a)
        node.discard_current_episode = lambda *a: namespace['discard_current_episode'](node, *a)
        return node


    def test_success_annotations_are_transactional_and_do_not_require_expert_frames(self):
        for takeover in (False, True):
            with self.subTest(takeover=takeover), tempfile.TemporaryDirectory() as root:
                node = self.annotation_recorder(root)
                node.subtasks = SubtaskAnnotations(())
                node.dagger = DaggerLabels()
                node.dagger.trial_id = 'trial-1'
                outcome = dict(trial_id='trial-1', task_success=True, human_takeover=takeover)
                self.assertTrue(namespace['save_current_episode'](node, 'command:save', 'req', dagger_outcome=outcome))
                path = Path(root) / 'annotations/dagger/episode_000003.json'
                result = json.loads(path.read_text())
                self.assertTrue(result['task_success'])
                self.assertEqual(result['success_source'], 'human_intervention' if takeover else 'autonomous')
                self.assertEqual(result['expert_frames'], 0)  # A brief takeover may have no trainable frame.
                self.assertEqual(result['episode_index'], 3)
                self.assertEqual(node._write_command_status.call_args.kwargs['task_outcome'], result)

    def test_bad_or_wrong_trial_confirmation_keeps_data_and_does_not_publish(self):
        for metadata in ({'trial_id':'other','task_success':True,'human_takeover':False},
                         {'trial_id':'trial-1','task_success':True,'human_takeover':'false'}):
            with self.subTest(metadata=metadata), tempfile.TemporaryDirectory() as root:
                node = self.annotation_recorder(root)
                node.subtasks = SubtaskAnnotations(())
                node.dagger = DaggerLabels()
                node.dagger.trial_id = 'trial-1'
                self.assertFalse(namespace['save_current_episode'](node, 'save', 'req', dagger_outcome=metadata))
                node.episode_store.commit.assert_not_called()
                self.assertTrue(node.is_recording)
                self.assertFalse((Path(root) / 'annotations/dagger').exists())

    def test_failed_success_save_never_publishes_success_and_discard_removes_staging(self):
        with tempfile.TemporaryDirectory() as root:
            node = self.annotation_recorder(root)
            node.subtasks = SubtaskAnnotations(())
            node.dagger = DaggerLabels()
            node.dagger.trial_id = 'trial-1'
            node.dataset.save_episode.side_effect = OSError('injected disk full')
            outcome = dict(trial_id='trial-1', task_success=True, human_takeover=True)
            self.assertFalse(namespace['save_current_episode'](node, 'save', 'req', dagger_outcome=outcome))
            self.assertFalse((Path(root) / 'annotations/dagger').exists())
            node.episode_store.commit.assert_not_called()
            self.assertNotIn('task_outcome', node._write_command_status.call_args.kwargs)
            self.assertTrue(node.discard_current_episode('discard'))
            self.assertFalse((Path(root) / '.collector-work/annotations/dagger').exists())

    def test_failure_tag_is_published_only_after_a_successful_storage_commit(self):
        for disk_ok in (False, True):
            with self.subTest(disk_ok=disk_ok), tempfile.TemporaryDirectory() as root:
                node = self.annotation_recorder(root)
                node.subtasks = SubtaskAnnotations(())
                node.dagger = DaggerLabels()
                node.dagger.trial_id = 'trial-1'
                if not disk_ok:
                    node.dataset.save_episode.side_effect = OSError('injected disk full')
                outcome = dict(trial_id='trial-1', task_success=False, human_takeover=True)
                self.assertEqual(namespace['save_current_episode'](node, 'failure-save', 'req', dagger_outcome=outcome), disk_ok)
                path = Path(root) / 'annotations/dagger/episode_000003.json'
                self.assertEqual(path.exists(), disk_ok)
                if disk_ok:
                    result = json.loads(path.read_text())
                    self.assertEqual(result['outcome'], 'failure')
                    self.assertIs(result['task_success'], False)
                    self.assertIs(node._write_command_status.call_args.args[1], True)
                else:
                    self.assertNotIn('task_outcome', node._write_command_status.call_args.kwargs)


    def test_real_fifo_listener_preserves_trial_and_request_case(self):
        with tempfile.TemporaryDirectory() as root:
            path=Path(root)/'commands'
            os.mkfifo(path)
            node=Mock()
            node.args=SimpleNamespace(control_fifo=str(path))
            node.stop_requested=False
            node.control_queue=SimpleQueue()
            metadata=dict(trial_id='Trial-ABC',task_success=True,human_takeover=True)
            worker=threading.Thread(target=namespace['_control_fifo_loop'],args=(node,))
            worker.start()
            with path.open('w') as writer:
                writer.write('SAVE Request-ABC '+json.dumps(metadata)+'\n')
                writer.flush()
                received=node.control_queue.get(timeout=2)
                node.stop_requested=True
            worker.join(timeout=2)
            self.assertFalse(worker.is_alive())
            node.control_queue.put(received)
            node.mark_status_future=None
            namespace['process_control_commands'](node)
            node.save_current_episode.assert_called_once_with('command:save','Request-ABC',dagger_outcome=metadata)

    def test_legacy_save_remains_unlabelled_and_fifo_metadata_is_optional(self):
        with tempfile.TemporaryDirectory() as root:
            node = self.annotation_recorder(root)
            node.subtasks = SubtaskAnnotations(())
            node.dagger = DaggerLabels()
            node.dagger.trial_id = 'trial-1'
            self.assertTrue(node.save_current_episode('legacy-save'))
            self.assertFalse((Path(root) / 'annotations/dagger').exists())
        node = Mock()
        node.mark_status_future = None
        node.control_queue = SimpleQueue()
        node.control_queue.put('save old-request')
        metadata = dict(trial_id='trial-1', task_success=True, human_takeover=False)
        node.control_queue.put('save new-request ' + json.dumps(metadata))
        node.control_queue.put('save bad-request {broken')
        node.control_queue.put('save null-request null')
        namespace['process_control_commands'](node)
        self.assertEqual(node.save_current_episode.call_count, 2)
        self.assertEqual(node.save_current_episode.call_args_list[0].args, ('command:save', 'old-request'))
        self.assertEqual(node.save_current_episode.call_args_list[1].kwargs, {'dagger_outcome':metadata})
        self.assertEqual(node._write_command_status.call_count, 2)

    def test_marks_last_autosaves_one_episode_and_resets(self):
        with tempfile.TemporaryDirectory() as root:
            node = self.annotation_recorder(root)
            for count in (1, 2, 3):
                node.current_episode_frames = count
                self.assertTrue(namespace['mark_subtask'](node, f'mark-{count}'))
            node.dataset.save_episode.assert_called_once_with(parallel_encoding=False)
            self.assertEqual(node.saved_episodes, 4)
            self.assertEqual(node.subtasks.ends, [])
            self.assertFalse(node.is_recording)
            receipt = node._write_command_status.call_args
            self.assertEqual(receipt.args, ('save', True, 'mark-3'))
            self.assertTrue(receipt.kwargs['subtask_status']['complete'])
            self.assertTrue((Path(root) / 'annotations/subtasks/episode_000003.json').is_file())

    def test_mark_without_frames_or_disabled_or_invalid_is_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            for field, value in [('current_episode_frames', 0), ('is_recording', False),
                                 ('episode_invalid', True), ('subtasks', SubtaskAnnotations(()))]:
                node = self.annotation_recorder(root)
                setattr(node, field, value)
                self.assertFalse(namespace['mark_subtask'](node, 'reject'))
                node.dataset.save_episode.assert_not_called()
                self.assertEqual(node._write_command_status.call_args.args, ('mark_subtask', False, 'reject'))

    def test_early_save_keeps_unconfirmed_tail_and_acknowledges_review(self):
        with tempfile.TemporaryDirectory() as root:
            node = self.annotation_recorder(root)
            node.subtasks.mark(1)
            self.assertTrue(node.save_current_episode('command:save', 'early'))
            labels = node.dataset.writer.episode_buffer['subtask_index']
            self.assertEqual([int(x[0]) for x in labels], [0, -1, -1])
            receipt = node._write_command_status.call_args.kwargs['subtask_status']
            self.assertFalse(receipt['complete'])
            self.assertTrue(receipt['needs_review'])

    def test_failed_save_invalidates_workspace_without_touching_saved_metadata(self):
        with tempfile.TemporaryDirectory() as root:
            node = self.annotation_recorder(root)
            node.dataset.save_episode.side_effect = RuntimeError('disk failure')
            self.assertFalse(node.save_current_episode('save', 'failed'))
            self.assertFalse(node.save_failed)
            self.assertFalse(node.stop_requested)
            self.assertTrue(node.episode_invalid)
            self.assertEqual(node.saved_episodes, 3)
            check_pending_annotations(Path(root))
            with self.assertRaisesRegex(RuntimeError, 'Unfinished episode save'):
                check_pending_annotations(node.episode_store.work)
            self.assertFalse(namespace['start_episode'](node, 'retry'))
            self.assertTrue(node.discard_current_episode('discard'))
            node.dataset.save_episode.assert_called_once()
            self.assertFalse(node.episode_invalid)

    def test_discard_resets_annotation_without_publishing_manifest(self):
        with tempfile.TemporaryDirectory() as root:
            node = self.annotation_recorder(root)
            node.subtasks.mark(1)
            self.assertTrue(node.discard_current_episode('discard'))
            self.assertFalse(node.subtasks.ends)
            self.assertFalse(list(Path(root).rglob('*.json')))
            node.dataset.save_episode.assert_not_called()

    def test_discard_removes_only_pending_video_images_after_writer_finishes(self):
        with tempfile.TemporaryDirectory() as root:
            node = self.annotation_recorder(root)
            key = 'observation.images.rgbd_head_depth'
            pending = node.episode_store.work / 'images' / key / 'episode-000003'
            saved = Path(root) / 'images' / key / 'episode-000002'
            for directory in (pending, saved):
                directory.mkdir(parents=True)
                (directory / 'frame-000000.png').write_bytes(b'frame')
            # Simulate a late write during the mandatory writer join.
            node._close_episode_dataset.side_effect = lambda: (pending / 'last.png').write_bytes(b'late')
            self.assertTrue(node.discard_current_episode('discard'))
            self.assertFalse(pending.exists())
            self.assertTrue((saved / 'frame-000000.png').is_file())

    def test_first_frame_partial_failure_is_invalid_and_discardable(self):
        with tempfile.TemporaryDirectory() as root:
            node = self.annotation_recorder(root)
            node.current_episode_frames = 0
            node._record_frame.side_effect = OSError('disk full after first camera')
            namespace['_record_tick'](node)
            self.assertTrue(node.episode_invalid)
            self.assertFalse(node.is_recording)
            self.assertFalse(node.stop_requested)
            self.assertTrue(node.discard_current_episode('discard'))
            node._close_episode_dataset.assert_called_once()
            node.episode_store.discard.assert_called_once()

    def test_delayed_log_failure_is_detected_before_next_frame_without_join(self):
        with tempfile.TemporaryDirectory() as root:
            node = self.annotation_recorder(root)
            node.sync_log = Mock()
            node.sync_log.check.side_effect = OSError('injected delayed log error')
            namespace['_record_tick'](node)
            self.assertTrue(node.sync_log_failed)
            self.assertTrue(node.episode_invalid)
            self.assertEqual(node.episode_invalid_reason, 'sync_log_write_failed')
            node._record_frame.assert_not_called()
            node.sync_log.close.assert_not_called()
            node.sync_log.flush.assert_not_called()

    def test_failed_public_discard_keeps_video_images(self):
        with tempfile.TemporaryDirectory() as root:
            node = self.annotation_recorder(root)
            pending = node.episode_store.work / 'images/cam/episode-000003'
            pending.mkdir(parents=True)
            node._close_episode_dataset.side_effect = RuntimeError('writer cannot close')
            self.assertFalse(node.discard_current_episode('discard'))
            self.assertTrue(pending.is_dir())

    def test_slow_receipt_io_does_not_block_recording_or_queue_more_commands(self):
        with tempfile.TemporaryDirectory() as root:
            node = self.annotation_recorder(root)
            node.args.status_file = Path(root) / 'status.json'
            node.control_queue = SimpleQueue()
            node.mark_status_future = None
            writing, release = threading.Event(), threading.Event()

            def slow_write(path, value):
                writing.set()
                if not release.wait(5):
                    raise RuntimeError('test receipt was not released')
                path.write_text(json.dumps(value))

            with ThreadPoolExecutor(max_workers=1) as executor, patch.dict(namespace, write_json_atomic=slow_write):
                node.mark_status_writer = executor
                try:
                    namespace['_write_command_status'](node, 'mark_subtask', True, 'req', message='confirmed')
                    self.assertTrue(writing.wait(1))
                    self.assertFalse(node.mark_status_future.done())
                    node.control_queue.put('mark_subtask next')
                    namespace['process_control_commands'](node)
                    self.assertFalse(node.control_queue.empty())
                    self.assertTrue(node.is_recording)
                    # This can run while the receipt thread is still blocked.
                    node.current_episode_frames += 1
                    self.assertEqual(node.current_episode_frames, 4)
                finally:
                    release.set()
                node.mark_status_future.result(timeout=1)

    def test_start_receipt_waits_for_log_in_worker_not_capture_thread(self):
        with tempfile.TemporaryDirectory() as root:
            node = self.annotation_recorder(root)
            node.args.status_file = Path(root) / 'status.json'
            node.sync_log = Mock()
            barrier = Future()
            node.sync_log.barrier.return_value = barrier
            with ThreadPoolExecutor(max_workers=1) as executor:
                node.mark_status_writer = executor
                try:
                    namespace['_write_command_status'](node, 'start', True, 'start-1', message='episode started')
                    self.assertFalse(node.mark_status_future.done())
                    self.assertFalse(node.args.status_file.exists())
                    node.sync_log.flush.assert_not_called()
                    # Capture remains free to append while disk flush is pending.
                    node.current_episode_frames += 1
                    self.assertEqual(node.current_episode_frames, 4)
                finally:
                    barrier.set_result(None)
                node.mark_status_future.result(timeout=1)
            self.assertEqual(json.loads(node.args.status_file.read_text())['request_id'], 'start-1')

    def test_log_barrier_failure_never_publishes_success_or_escapes_callback(self):
        for admission_failure in (False, True):
            with self.subTest(admission_failure=admission_failure), tempfile.TemporaryDirectory() as root:
                node = self.annotation_recorder(root)
                node.args.status_file = Path(root) / 'status.json'
                node.control_queue = SimpleQueue()
                node.sync_log = Mock()
                error = OSError('injected log failure')
                if admission_failure:
                    node.sync_log.barrier.side_effect = error
                else:
                    barrier = Future()
                    barrier.set_exception(error)
                    node.sync_log.barrier.return_value = barrier
                with ThreadPoolExecutor(max_workers=1) as executor:
                    node.mark_status_writer = executor
                    namespace['_write_command_status'](node, 'start', True, 'start-1', message='episode started')
                    with self.assertRaises(OSError):
                        node.mark_status_future.result(timeout=1)
                namespace['process_control_commands'](node)
                self.assertTrue(node.episode_invalid)
                self.assertFalse(node.is_recording)
                self.assertFalse(node.args.status_file.exists())
                node.sync_log.close.assert_not_called()

    def test_delayed_receipt_io_failure_is_fail_closed(self):
        with tempfile.TemporaryDirectory() as root:
            node = self.annotation_recorder(root)
            node.args.status_file = Path(root) / 'status.json'
            node.control_queue = SimpleQueue()
            with ThreadPoolExecutor(max_workers=1) as executor, patch.dict(
                    namespace, write_json_atomic=Mock(side_effect=OSError('full disk'))):
                node.mark_status_writer = executor
                namespace['_write_command_status'](node, 'start', True, 'start-1', message='episode started')
                with self.assertRaises(OSError):
                    node.mark_status_future.result(timeout=1)
                namespace['process_control_commands'](node)
            self.assertTrue(node.episode_invalid)
            self.assertFalse(node.is_recording)

    def test_shutdown_observes_failed_receipt_instead_of_saving_valid_episode(self):
        with tempfile.TemporaryDirectory() as root:
            node = self.annotation_recorder(root)
            failed = Future()
            failed.set_exception(OSError('injected receipt failure'))
            node.mark_status_future = failed
            node.save_current_episode = Mock()
            node.discard_current_episode = Mock()
            namespace['finish'](node)
            self.assertTrue(node.episode_invalid)
            node.save_current_episode.assert_not_called()
            node.discard_current_episode.assert_called_once_with('shutdown:invalid_episode')

    def test_invalidation_cannot_be_overwritten_by_slow_start_success(self):
        with tempfile.TemporaryDirectory() as root:
            node = self.annotation_recorder(root)
            node.args.status_file = Path(root) / 'status.json'
            node._write_command_status = lambda *a, **kw: namespace['_write_command_status'](node, *a, **kw)
            writing, release = threading.Event(), threading.Event()
            seen = []
            def slow_write(path, value):
                if value['success']:
                    writing.set()
                    if not release.wait(5):
                        raise RuntimeError('test receipt was not released')
                seen.append(value)
                path.write_text(json.dumps(value))
            with ThreadPoolExecutor(max_workers=1) as executor, patch.dict(namespace, write_json_atomic=slow_write):
                node.mark_status_writer = executor
                try:
                    node._write_command_status('start', True, 'start-1', message='episode started')
                    self.assertTrue(writing.wait(1))
                    namespace['_invalidate_episode'](node, 'camera_failed', time.time())
                    self.assertTrue(node.episode_invalid)
                    self.assertFalse(node.mark_status_future.done())
                finally:
                    release.set()
                node.mark_status_future.result(timeout=1)
            final = json.loads(node.args.status_file.read_text())
            self.assertEqual([value['success'] for value in seen], [True, False])
            self.assertFalse(final['success'])
            self.assertTrue(final['episode_invalid'])
            self.assertEqual(final['request_id'], 'start-1')

    def test_resume_rejects_annotation_schema_change_before_opening_writer(self):
        with tempfile.TemporaryDirectory() as root:
            for plan, features in [(('pick',), {}), ((), {'subtask_index': {'dtype': 'int64', 'shape': [1]}}),
                                   (('pick',), {'subtask_index': {'dtype': 'float32', 'shape': [1]}})]:
                node = self.annotation_recorder(root)
                node.subtasks = SubtaskAnnotations(plan)
                with self.assertRaisesRegex(RuntimeError, 'Subtask annotation schema differs'):
                    namespace['_validate_resume_schema'](node, {'features': features})


if __name__ == '__main__':
    unittest.main()
