"""Real LeRobot save/resume publication with tiny synthetic RGB episodes.

Requires the collector's LeRobot environment. No hardware topics are opened.
"""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np

from episode_storage import EpisodeStore
try:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.configs import RGBEncoderConfig
    from dagger.capture_image_writer import CaptureImageWriter
except ImportError:
    LeRobotDataset = None


@unittest.skipIf(LeRobotDataset is None, 'requires LeRobot writer and encoder')
class LeRobotStorageTest(unittest.TestCase):
    def test_save_fail_discard_resume_keeps_previous_files_readable(self):
        import pyarrow.parquet as pq
        import cv2
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / 'dataset'
            encoder = RGBEncoderConfig(vcodec='h264')
            features = {
                'observation.state': {'dtype': 'float32', 'shape': (1,), 'names': ['joint']},
                'action': {'dtype': 'float32', 'shape': (1,), 'names': ['joint']},
                'observation.images.cam': {'dtype': 'video', 'shape': (32, 32, 3),
                                           'names': ['height', 'width', 'channels']},
            }
            dataset = LeRobotDataset.create('local/storage_test', fps=30, root=root, features=features,
                                            rgb_encoder=encoder, image_writer_threads=0)
            dataset.finalize()
            store = EpisodeStore(root)
            def open_episode():
                path = store.prepare()
                if json.loads((root / 'meta/info.json').read_text())['total_episodes']:
                    result = LeRobotDataset.resume('local/storage_test', root=path, rgb_encoder=encoder)
                else:
                    result = LeRobotDataset.create('local/storage_test', root=path, fps=30,
                                                   features=features, rgb_encoder=encoder)
                result.writer.image_writer = CaptureImageWriter(num_threads=1)
                return result
            def add(dataset, frames=3):
                for i in range(frames):
                    dataset.add_frame({'observation.state': np.array([i], dtype=np.float32),
                                       'action': np.array([i], dtype=np.float32),
                                       'observation.images.cam': np.full((32, 32, 3), i * 30, dtype=np.uint8),
                                       'task': 'synthetic storage validation; not training data'})
            def hashes():
                return {p.relative_to(root): hashlib.sha256(p.read_bytes()).hexdigest()
                        for group in ('data', 'videos') for p in (root / group).rglob('*') if p.is_file()}
            try:
                dataset = open_episode()
                add(dataset)
                dataset.save_episode(parallel_encoding=False)
                dataset.finalize()
                store.commit(0)
                saved = hashes()
                self.assertTrue(saved)
                meta = (root / 'meta/info.json').read_bytes()

                dataset = open_episode()
                # A late worker error is still visible BEFORE LeRobot saves.
                with patch('dagger.capture_image_writer.write_capture_image', side_effect=OSError('ENOSPC')):
                    dataset.writer.image_writer._write = lambda *_: (_ for _ in ()).throw(OSError('ENOSPC'))
                    add(dataset, 1)
                    with self.assertRaises(OSError):
                        dataset.writer.image_writer.wait_until_done()
                    with self.assertRaises(OSError):
                        dataset.writer.image_writer.stop()
                dataset.writer.image_writer = None
                dataset.finalize()
                store.discard()
                self.assertEqual(hashes(), saved)
                self.assertEqual((root / 'meta/info.json').read_bytes(), meta)

                # Failure after Parquet/video generation never touched old files.
                dataset = open_episode()
                add(dataset)
                dataset.save_episode(parallel_encoding=False)
                dataset.finalize()
                store.discard()
                self.assertEqual(hashes(), saved)
                self.assertEqual((root / 'meta/info.json').read_bytes(), meta)

                dataset = open_episode()
                add(dataset)
                dataset.save_episode(parallel_encoding=False)
                dataset.finalize()
                store.commit(1)
                self.assertEqual(json.loads((root / 'meta/info.json').read_text())['total_episodes'], 2)
                for relative, digest in saved.items():
                    self.assertEqual(hashes()[relative], digest)
                self.assertEqual(sum(pq.read_metadata(p).num_rows for p in (root / 'data').rglob('*.parquet')), 6)
                reader = LeRobotDataset('local/storage_test', root=root, video_backend='pyav')
                self.assertEqual(len(reader), 6)
                for index in (0, 3):
                    item = reader[index]
                    self.assertEqual(item['observation.images.cam'].shape, (3, 32, 32))
                    self.assertEqual(int(item['episode_index']), index // 3)
                for p in (root / 'videos').rglob('*.mp4'):
                    reader = cv2.VideoCapture(str(p))
                    try:
                        self.assertTrue(reader.read()[0])
                    finally:
                        reader.release()
            finally:
                store.close()


if __name__ == '__main__':
    unittest.main()
