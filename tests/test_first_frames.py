"""Original RGB frame-zero export without ROS, cameras or video re-encoding."""

import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from PIL import Image
from first_frames import RGB_POSITIONS, stage_first_frames


class FirstFramesTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.work = Path(self.temp.name) / '.collector-work'
        self.work.mkdir()
        self.features = {f'observation.images.{camera}': {'dtype': 'video'} for camera in RGB_POSITIONS}
        self.features['observation.images.rgbd_head_depth'] = {'dtype': 'video', 'info': {'is_depth_map': True}}
        self.dataset = SimpleNamespace(features=self.features, writer=SimpleNamespace(_get_image_file_path=self.path))
        for index, key in enumerate(list(self.features)[:3]):
            source = self.path(7, key, 0)
            source.parent.mkdir(parents=True)
            Image.new('RGB', (32, 24), (index * 50, 80, 190)).save(source)
            Image.new('RGB', (32, 24), (255, 0, 0)).save(self.path(7, key, 1))

    def tearDown(self):
        self.temp.cleanup()

    def path(self, episode_index, image_key, frame_index):
        return self.work / 'images' / image_key / f'episode-{episode_index:06d}' / f'frame-{frame_index:06d}.png'

    def test_preserves_exact_first_png_after_encoder_deletes_source(self):
        stage_first_frames(self.dataset, self.work, 7)
        self.assertEqual(len(list((self.work / 'first_frame').rglob('*.png'))), 3)
        for camera, position in RGB_POSITIONS.items():
            source = self.path(7, f'observation.images.{camera}', 0)
            destination = self.work / 'first_frame' / f'{position}_frame' / f'{position}_episode_00007.png'
            content = source.read_bytes()
            self.assertTrue(os.path.samefile(source, destination))
            source.unlink()
            self.assertEqual(destination.read_bytes(), content)
            with Image.open(destination) as image:
                self.assertEqual(image.mode, 'RGB')
                self.assertEqual(image.size, (32, 24))
                self.assertEqual(image.getpixel((0, 0))[1:], (80, 190))

    def test_partial_camera_mode_only_exports_active_rgb(self):
        self.dataset.features = {'observation.images.hand_left': {'dtype': 'video'}}
        stage_first_frames(self.dataset, self.work, 7)
        self.assertEqual([p.name for p in (self.work / 'first_frame').iterdir()], ['left_frame'])

    def test_existing_export_is_never_replaced(self):
        stage_first_frames(self.dataset, self.work, 7)
        before = {p: p.read_bytes() for p in (self.work / 'first_frame').rglob('*.png')}
        with self.assertRaises(FileExistsError):
            stage_first_frames(self.dataset, self.work, 7)
        self.assertTrue(all(path.read_bytes() == data for path, data in before.items()))

    def test_missing_corrupt_or_non_rgb_first_frame_fails_closed(self):
        key = 'observation.images.rgbd_head_color'
        source = self.path(7, key, 0)
        source.unlink()
        with self.assertRaisesRegex(RuntimeError, 'Missing or unsafe'):
            stage_first_frames(self.dataset, self.work, 7)
        source.write_bytes(b'corrupt')
        with self.assertRaises(OSError):
            stage_first_frames(self.dataset, self.work, 7)
        Image.new('L', (32, 24)).save(source)
        with self.assertRaisesRegex(RuntimeError, 'not an RGB PNG'):
            stage_first_frames(self.dataset, self.work, 7)

    def test_destination_symlink_cannot_write_outside_workspace(self):
        outside = Path(self.temp.name) / 'outside'
        outside.mkdir()
        (self.work / 'first_frame').symlink_to(outside, target_is_directory=True)
        with self.assertRaisesRegex(RuntimeError, 'Symlink not allowed'):
            stage_first_frames(self.dataset, self.work, 7)
        self.assertEqual(list(outside.iterdir()), [])


if __name__ == '__main__':
    unittest.main()
