"""Offline extraction on real encoded video shards, without ROS or a robot."""

import fcntl
from fractions import Fraction
import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import av
from PIL import Image
import pyarrow as pa
import pyarrow.parquet as pq

from extract_first_frames import extract_first_frames, main
from first_frames import RGB_POSITIONS, first_frame_path


class ExtractFirstFramesTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name) / 'dataset'
        self.meta = self.root / 'meta/episodes/chunk-000/file-000.parquet'
        self.meta.parent.mkdir(parents=True)
        self.keys = [f'observation.images.{name}' for name in RGB_POSITIONS]
        self.info = {
            'codebase_version': 'v3.0', 'total_episodes': 2, 'fps': 30,
            'video_path': 'videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4',
            'features': {key: {'dtype': 'video'} for key in self.keys},
        }
        # Depth is deliberately absent on disk; it must never be decoded here.
        self.info['features']['observation.images.rgbd_head_depth'] = {
            'dtype': 'video', 'info': {'is_depth_map': True}}
        self.rows = [{'episode_index': i} for i in range(2)]
        for camera, key in enumerate(self.keys):
            video = self.root / self.info['video_path'].format(video_key=key, chunk_index=0, file_index=0)
            self.write_video(video, camera)
            for index, row in enumerate(self.rows):
                row.update({f'videos/{key}/chunk_index': 0, f'videos/{key}/file_index': 0,
                            f'videos/{key}/from_timestamp': index * 4 / 30,
                            f'videos/{key}/to_timestamp': (index + 1) * 4 / 30})
        self.write_metadata()

    def tearDown(self):
        self.temp.cleanup()

    def write_video(self, path, camera):
        path.parent.mkdir(parents=True, exist_ok=True)
        with av.open(str(path), 'w') as container:
            stream = container.add_stream('libx264rgb', rate=30)
            stream.width, stream.height = 64, 48
            stream.pix_fmt = 'rgb24'
            stream.options = {'crf': '0', 'preset': 'fast', 'g': '30', 'sc_threshold': '0', 'bf': '2'}
            for index in range(8):
                image = Image.new('RGB', (64, 48), self.color(camera, index))
                frame = av.VideoFrame.from_image(image)
                frame.pts, frame.time_base = index, Fraction(1, 30)
                for packet in stream.encode(frame):
                    container.mux(packet)
            for packet in stream.encode():
                container.mux(packet)

    @staticmethod
    def color(camera, frame):
        return (30 + frame * 20, 50 + camera * 60, 190)

    def write_metadata(self):
        (self.root / 'meta/info.json').write_text(json.dumps(self.info))
        pq.write_table(pa.Table.from_pylist(self.rows), self.meta)

    def extract(self, **kwargs):
        return extract_first_frames(self.root, report=lambda text: None, **kwargs)

    def snapshots(self):
        return sorted((self.root / 'first_frame').rglob('*.png'))

    def hash_originals(self):
        return {path.relative_to(self.root): hashlib.sha256(path.read_bytes()).hexdigest()
                for group in ('meta', 'videos') for path in (self.root / group).rglob('*') if path.is_file()}

    def test_shared_video_uses_each_episode_offset_and_preserves_originals(self):
        before = self.hash_originals()
        self.assertEqual(self.extract(), {'episodes': 2, 'saved': 6, 'skipped': 0})
        for camera, position in enumerate(RGB_POSITIONS.values()):
            for index in range(2):
                with Image.open(self.root / first_frame_path(position, index)) as image:
                    self.assertEqual(image.format, 'PNG')
                    self.assertEqual(image.mode, 'RGB')
                    self.assertEqual(image.getpixel((0, 0)), self.color(camera, index * 4))
        self.assertEqual(self.hash_originals(), before)
        self.assertEqual(len(self.snapshots()), 6)

    def test_repeated_run_keeps_original_png_and_only_fills_missing(self):
        destination = self.root / first_frame_path('head', 0)
        destination.parent.mkdir(parents=True)
        Image.new('RGB', (64, 48), (1, 2, 3)).save(destination)
        before = destination.read_bytes(), destination.stat().st_mtime_ns
        self.assertEqual(self.extract()['saved'], 5)
        self.assertEqual(self.extract(), {'episodes': 2, 'saved': 0, 'skipped': 6})
        self.assertEqual((destination.read_bytes(), destination.stat().st_mtime_ns), before)
        destination = self.root / first_frame_path('right', 1)
        destination.unlink()
        self.assertEqual(self.extract(), {'episodes': 2, 'saved': 1, 'skipped': 5})

    def test_selection_and_separate_shards_keep_dataset_indices(self):
        self.rows[1]['episode_index'] = 100001
        for camera, key in enumerate(self.keys):
            self.rows[1][f'videos/{key}/chunk_index'] = 2
            self.rows[1][f'videos/{key}/file_index'] = 9
            self.rows[1][f'videos/{key}/from_timestamp'] = 0
            self.write_video(self.root / self.info['video_path'].format(
                video_key=key, chunk_index=2, file_index=9), camera)
        self.write_metadata()
        self.assertEqual(self.extract(episodes=[100001]), {'episodes': 1, 'saved': 3, 'skipped': 0})
        self.assertEqual(len(self.snapshots()), 3)
        self.assertTrue(all('episode_100001' in path.name for path in self.snapshots()))
        with Image.open(self.root / first_frame_path('left', 100001)) as image:
            self.assertEqual(image.getpixel((0, 0)), self.color(1, 0))

    def test_partial_camera_dataset(self):
        self.info['features'] = {self.keys[1]: {'dtype': 'video'}}
        self.write_metadata()
        self.assertEqual(self.extract()['saved'], 2)
        self.assertTrue(all(path.parent.name == 'left_frame' for path in self.snapshots()))

    def test_bad_video_or_offset_does_not_publish_partial_episode(self):
        self.extract(episodes=[0])
        saved = {path: path.read_bytes() for path in self.snapshots()}
        for bad_offset in (10, float('nan'), -1, 5 / 30):
            with self.subTest(offset=bad_offset):
                # End stays at 4.5 frames: the last case selects a later frame
                # beyond the episode's interval and must also be rejected.
                self.rows[1][f'videos/{self.keys[2]}/from_timestamp'] = bad_offset
                self.rows[1][f'videos/{self.keys[2]}/to_timestamp'] = 4.5 / 30
                self.write_metadata()
                with self.assertRaisesRegex(RuntimeError, 'Episode 1 extraction failed'):
                    self.extract()
                self.assertEqual({path: path.read_bytes() for path in self.snapshots()}, saved)
                self.assertEqual(list(self.root.glob('.first-frame-extract-*')), [])
        self.rows[1][f'videos/{self.keys[2]}/from_timestamp'] = 4 / 30
        self.write_metadata()
        video = self.root / self.info['video_path'].format(video_key=self.keys[2], chunk_index=0, file_index=0)
        video.write_bytes(b'broken video')
        with self.assertRaisesRegex(RuntimeError, 'Episode 1 extraction failed'):
            self.extract()
        self.assertEqual({path: path.read_bytes() for path in self.snapshots()}, saved)

    def test_nearest_keyframe_is_not_accepted_for_missing_target(self):
        for key in self.keys:
            self.rows[1][f'videos/{key}/from_timestamp'] = 10
            self.rows[1][f'videos/{key}/to_timestamp'] = 11
        self.write_metadata()
        with self.assertRaisesRegex(RuntimeError, 'No first frame within'):
            self.extract(episodes=[1])
        self.assertEqual(self.snapshots(), [])

    def test_publication_failure_and_interrupt_roll_back_only_new_images(self):
        self.extract(episodes=[0])
        saved = {path: path.read_bytes() for path in self.snapshots()}
        original_link = os.link
        for exception in (OSError('disk full'), KeyboardInterrupt()):
            def fail_last(source, destination):
                if 'right_frame' in str(destination):
                    raise exception
                original_link(source, destination)
            with self.subTest(error=type(exception).__name__):
                with patch('extract_first_frames.os.link', side_effect=fail_last):
                    with self.assertRaises(KeyboardInterrupt if isinstance(exception, KeyboardInterrupt) else RuntimeError):
                        self.extract()
                self.assertEqual({path: path.read_bytes() for path in self.snapshots()}, saved)
                self.assertEqual(list(self.root.glob('.first-frame-extract-*')), [])

    def test_active_dataset_and_unfinished_transaction_refused(self):
        lock_path = self.root.parent / ('.' + self.root.name + '.collector.lock')
        with lock_path.open('a+') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaisesRegex(RuntimeError, 'Dataset is in use'):
                self.extract()
        for name in ('.collector-work', '.collector-commit.json'):
            path = self.root / name
            path.touch()
            with self.assertRaisesRegex(RuntimeError, 'Unfinished collector'):
                self.extract()
            path.unlink()
        self.assertEqual(self.snapshots(), [])

    def test_unsafe_paths_and_symlinks_are_refused(self):
        for template in ('../outside.mp4', '/tmp/outside.mp4'):
            self.info['video_path'] = template
            self.write_metadata()
            with self.assertRaisesRegex(RuntimeError, 'Unsafe dataset path'):
                self.extract()
        outside = Path(self.temp.name) / 'outside'
        outside.mkdir()
        (self.root / 'first_frame').symlink_to(outside, target_is_directory=True)
        with self.assertRaisesRegex(RuntimeError, 'Symlink not supported'):
            self.extract()
        self.assertEqual(list(outside.iterdir()), [])

    def test_existing_bad_snapshot_is_reported_not_overwritten(self):
        path = self.root / first_frame_path('head', 0)
        path.parent.mkdir(parents=True)
        path.write_bytes(b'bad PNG')
        with self.assertRaisesRegex(RuntimeError, 'Episode 0 extraction failed'):
            self.extract()
        self.assertEqual(path.read_bytes(), b'bad PNG')

    def test_bad_schema_count_and_selection_are_reported(self):
        self.info['total_episodes'] = 3
        self.write_metadata()
        with self.assertRaisesRegex(ValueError, 'Episode count mismatch'):
            self.extract()
        self.info['total_episodes'] = 2
        self.write_metadata()
        with self.assertRaisesRegex(ValueError, 'not found'):
            self.extract(episodes=[99])
        with self.assertRaisesRegex(ValueError, 'nonnegative'):
            self.extract(episodes=[-1])
        self.info['codebase_version'] = 'v2.1'
        self.write_metadata()
        with self.assertRaisesRegex(ValueError, 'Only LeRobot v3.0'):
            self.extract()
        self.assertEqual(self.snapshots(), [])

    def test_cli_help_and_selection(self):
        with patch('sys.stdout'), self.assertRaises(SystemExit) as caught:
            main(['--help'])
        self.assertEqual(caught.exception.code, 0)
        with patch('sys.stdout'):
            main([str(self.root), '--episodes', '1'])
        self.assertEqual(len(self.snapshots()), 3)
        self.assertTrue(all('episode_00001' in path.name for path in self.snapshots()))


if __name__ == '__main__':
    unittest.main()
