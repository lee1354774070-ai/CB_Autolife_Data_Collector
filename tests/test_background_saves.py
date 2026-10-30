"""No ROS/hardware: real four-camera encoding, queue recovery and publication."""
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import patch
import time

from background_saves import BackgroundSaves
from episode_storage import EpisodeStore
try:
    import numpy as np
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.datasets.video_utils import RGBEncoderConfig, DepthEncoderConfig
    from dagger.capture_image_writer import CaptureImageWriter
except ImportError:
    LeRobotDataset = None


@unittest.skipIf(LeRobotDataset is None, 'requires robot LeRobot environment')
class BackgroundSaveTest(unittest.TestCase):
    def test_queue_four_cameras_labels_recovery_and_history(self):
        self._exercise_queue(hardware_adapter=False)

    def test_hardware_adapter_worker_and_first_frame_publication(self):
        # CPU FFmpeg exercises the same adapter and subprocess boundary without
        # requiring a GPU. Opt-in recorder tests cover actual NVENC/VAAPI.
        self._exercise_queue(hardware_adapter=True)

    def _exercise_queue(self, hardware_adapter):
        import pyarrow.parquet as pq
        from dataclasses import asdict
        with tempfile.TemporaryDirectory(prefix='background-save-test-') as tmp:
            root = Path(tmp) / 'dataset'
            rgb = RGBEncoderConfig(vcodec='h264')
            depth = DepthEncoderConfig(preset='ultrafast', extra_options={'x265-params':'lossless=1:pools=2:frame-threads=1:bframes=0'})
            features = {'observation.state':dict(dtype='float32',shape=(21,),names=None),
                        'action':dict(dtype='float32',shape=(21,),names=None),
                        'subtask_index':dict(dtype='int64',shape=(1,),names=None),
                        'dagger.action_train_mask':dict(dtype='float32',shape=(21,),names=None)}
            for key in ['hand_left','hand_right','rgbd_head_color','rgbd_head_depth']:
                features['observation.images.'+key] = dict(dtype='video',shape=(32,32,1 if key.endswith('depth') else 3),names=['height','width','channels'])
            features['observation.images.rgbd_head_depth']['info'] = {'is_depth_map':True, 'video.output_unit':'mm'}
            shell = LeRobotDataset.create('local/background_test',root=root,fps=30,features=features,
                                          rgb_encoder=rgb,depth_encoder=depth)
            shell.finalize()
            store = EpisodeStore(root)
            options=dict(repo_id='local/background_test',rgb=asdict(rgb),depth=asdict(depth))
            if hardware_adapter:
                from video_encoding import HardwareVideoEncoder
                binary = shutil.which('ffmpeg') or str(Path(sys.prefix) / 'bin/ffmpeg')
                if not Path(binary).is_file():
                    self.skipTest('requires FFmpeg; never probes GPU')
                options['hardware_rgb'] = asdict(HardwareVideoEncoder('cpu', binary))
            q=BackgroundSaves(store,options)
            def capture(index):
                ds=LeRobotDataset.create('local/background_test',root=q.new_capture(),fps=30,features=features,
                                         rgb_encoder=rgb,depth_encoder=depth,image_writer_threads=0)
                ds.writer.image_writer=CaptureImageWriter(num_threads=1)
                for i in range(8):
                    frame={'task':'synthetic validation only', 'observation.state':np.full(21,i,dtype=np.float32),
                           'action':np.full(21,i+index,dtype=np.float32),'subtask_index':np.array([i//2],dtype=np.int64),
                           'dagger.action_train_mask':np.array([i%2]*21,dtype=np.float32)}
                    for key in ['hand_left','hand_right','rgbd_head_color','rgbd_head_depth']:
                        frame['observation.images.'+key]=(np.full((32,32,1),1000+i,dtype=np.uint16) if key.endswith('depth') else np.full((32,32,3),i*20,dtype=np.uint8))
                    ds.add_frame(frame)
                return ds
            try:
                ds=capture(0)
                q.submit(ds,0,dict(frames=8,task='test',expert_frames=4),
                         annotation={'episode_index':0,'frames':8},outcome={'outcome':'human_corrected_success'})
                deadline=time.monotonic()+5
                while q.worker is None and time.monotonic()<deadline:time.sleep(.01)
                self.assertIsNotNone(q.worker)
                q.worker.kill()  # Only this test's isolated encoder, before publication.
                with self.assertRaises(RuntimeError):q.poll(wait=True)
                self.assertEqual(json.loads((root/'meta/info.json').read_text())['total_episodes'],0)
                self.assertEqual(len(list(q.root.glob('*/buffer.npz'))),1)
                q.close()
                q=BackgroundSaves(store,options)
                ds=capture(1)
                q.submit(ds,1,dict(frames=8,task='test',expert_frames=4))
                with self.assertRaisesRegex(RuntimeError,'queue full'):q.new_capture()
                with patch('background_saves.shutil.rmtree', side_effect=OSError('injected cleanup failure')):
                    try:
                        results=q.poll(wait=True)  # Simulate crash before committed raw cleanup.
                    except RuntimeError as exc:
                        self.fail(str(exc) + '\n' + (q.root/'encoder.log').read_text()[-5000:])
                q.close()
                q=BackgroundSaves(store,options)
                self.assertEqual(q.pending,0)
                self.assertEqual(list(q.root.glob('*/job.json')),[])
                self.assertEqual([x['episode_index'] for x in results],[0,1])
                self.assertIsNone(q.error)
                self.assertEqual(json.loads((root/'meta/info.json').read_text())['total_episodes'],2)
                for e in range(2):
                    table=pq.read_table(root/f'data/chunk-000/file-{e:03d}.parquet').to_pydict()
                    self.assertEqual(table['episode_index'],[e]*8)
                    self.assertEqual(table['subtask_index'],[0,0,1,1,2,2,3,3])
                    self.assertEqual(table['dagger.action_train_mask'][1],[1.]*21)
                import av
                for f in (root/'videos').rglob('*.mp4'):
                    with av.open(str(f)) as c:self.assertEqual(sum(1 for _ in c.decode(video=0)),8)
                reader=LeRobotDataset('local/background_test',root=root,video_backend='pyav')
                for i in [0,7,8,15]:self.assertEqual(tuple(reader[i]['observation.images.rgbd_head_depth'].shape),(1,32,32))
                self.assertTrue((root/'annotations/subtasks/episode_000000.json').exists())
                self.assertTrue((root/'annotations/dagger/episode_000000.json').exists())
                from PIL import Image
                for index in range(2):
                    for position in ('head', 'left', 'right'):
                        with Image.open(root / 'first_frame' / f'{position}_frame' / f'{position}_episode_{index:05d}.png') as image:
                            np.testing.assert_array_equal(np.asarray(image), np.zeros((32,32,3), np.uint8))
                if hardware_adapter:
                    self.assertEqual(len(list((root/'annotations/video_encoding').glob('*.json'))), 2)
                q.close()
                # Fail the worker on an actually missing raw first frame. The
                # publication and all historical files must remain unchanged.
                import hashlib
                before = {p:hashlib.sha256(p.read_bytes()).digest()
                          for group in ('meta','data','videos','annotations','first_frame')
                          for p in (root/group).rglob('*') if p.is_file()}
                q=BackgroundSaves(store,options,autostart=False)
                ds=capture(2)
                q.submit(ds,2,dict(frames=8,task='test',expert_frames=4))
                next(Path(ds.root).rglob('frame-000000.png')).unlink()
                q.start()
                with self.assertRaises(RuntimeError):q.poll(wait=True)
                self.assertEqual(json.loads((root/'meta/info.json').read_text())['total_episodes'],2)
                self.assertEqual(len(list(q.root.glob('*/buffer.npz'))),1)
                for p,digest in before.items():
                    self.assertEqual(hashlib.sha256(p.read_bytes()).digest(),digest)
                with self.assertRaisesRegex(RuntimeError,'background save failed'):q.new_capture()
            finally:
                q.close();store.close()


if __name__=='__main__':unittest.main()
