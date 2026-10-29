"""Offline RGBD replay: compares synchronous save with durable background queue.

Never reads SHM or publishes ROS. Writes only to a new explicit --output folder.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time
import resource

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import av
import numpy as np
import pyarrow.parquet as pq
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.video_utils import RGBEncoderConfig, DepthEncoderConfig
from lerobot.datasets.depth_utils import dequantize_depth
from background_saves import BackgroundSaves, probe_rgb_encoder
from dagger.capture_image_writer import CaptureImageWriter
from episode_storage import EpisodeStore


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--episode',type=int,default=32)
    p.add_argument('--frames',type=int,default=300)
    p.add_argument('--strategy',choices=('both','baseline','background'),default='both')
    a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    info=json.loads((a.source/'meta/info.json').read_text())
    data=pq.read_table(a.source/f'data/chunk-000/file-{a.episode:03d}.parquet').to_pydict()
    n=min(a.frames,len(data['index']))
    features={k:v for k,v in info['features'].items() if k not in ('index','episode_index','task_index','frame_index','timestamp')}
    cameras=[k for k in features if k.startswith('observation.images.')]
    base_rgb=RGBEncoderConfig(vcodec='h264')
    base_depth=DepthEncoderConfig()
    fast_rgb=RGBEncoderConfig(vcodec='h264_nvenc',extra_options={'bf':'0'})
    probe_rgb_encoder(fast_rgb,features)
    fast_depth=DepthEncoderConfig(preset='ultrafast',extra_options={'x265-params':'lossless=1:pools=4:frame-threads=1:bframes=0:scenecut=0'})
    for cfg in (base_depth,fast_depth):
        d=info['features']['observation.images.rgbd_head_depth']['info']
        for k in ('depth_min','depth_max','shift','use_log'):
            if 'video.'+k in d:setattr(cfg,k,d['video.'+k])
    report=[]
    def create(path,rgb,depth):
        ds=LeRobotDataset.create('local/background_benchmark',root=path,fps=30,features=features,
                                 robot_type='offline_benchmark_not_training',rgb_encoder=rgb,depth_encoder=depth,
                                 encoder_threads=None,image_writer_threads=0)
        ds.writer.image_writer=CaptureImageWriter(2)
        return ds
    def fill(ds, realtime=False):
        containers=[av.open(str(a.source/f'videos/{k}/chunk-000/file-{a.episode:03d}.mp4')) for k in cameras]
        for c in containers:c.streams.video[0].codec_context.thread_count=1
        streams=[c.decode(video=0) for c in containers]
        ticks=[];start=time.monotonic()
        try:
            for i in range(n):
                if realtime:time.sleep(max(0,start+i/30-time.monotonic()))
                tick=time.monotonic();ticks.append(tick)
                frame={'task':'offline performance validation; not training data'}
                for k,v in features.items():
                    if k not in cameras:
                        frame[k]=np.asarray(data[k][i],dtype=v['dtype']).reshape(v['shape'])
                for k,stream in zip(cameras,streams):
                    f=next(stream)
                    if k.endswith('depth'):
                        im=dequantize_depth(f,depth_min=base_depth.depth_min,depth_max=base_depth.depth_max,
                            shift=base_depth.shift,use_log=base_depth.use_log,output_unit='mm',output_tensor=False,output_channel_last=True)
                        frame[k]=np.asarray(im,dtype=np.uint16)
                    else:frame[k]=f.to_ndarray(format='rgb24')
                ds.add_frame(frame)
            return {'capture_seconds':time.monotonic()-start,'frames':n,
                    'tick_interval_max_ms':float(np.max(np.diff(ticks))*1000) if len(ticks)>1 else 0}
        finally:
            for c in containers:c.close()
    for background in (False,True):
        if a.strategy != "both" and background != (a.strategy == "background"):
            continue
        root=a.output/('background' if background else 'baseline')
        shell=create(root,base_rgb,base_depth);shell.writer.image_writer.stop();shell.writer.image_writer=None;shell.finalize()
        store=EpisodeStore(root);queue=None
        try:
            if background:
                queue=BackgroundSaves(store,dict(repo_id='local/background_benchmark',rgb=asdict(fast_rgb),depth=asdict(fast_depth)))
                ds=create(queue.new_capture(),base_rgb,base_depth)
            else:ds=create(store.prepare(),base_rgb,base_depth)
            fill(ds)
            t=time.monotonic();cpu=time.process_time();child=resource.getrusage(resource.RUSAGE_CHILDREN)
            if background:
                queue.submit(ds,0,dict(task='benchmark',frames=n))
                accepted=time.monotonic()-t
                next_ds=create(queue.new_capture(),base_rgb,base_depth)
                # Real image read/write at 30 Hz while the previous save encodes.
                concurrent=fill(next_ds,realtime=True)
                next_ds.writer.image_writer.stop();next_ds.writer.image_writer=None;next_ds.finalize()
                assert next_ds.writer.episode_buffer['size'] == n
                results=queue.poll(wait=True)
                encode=results[0]['encode_seconds']
                durable=results[0].get('committed_at',results[0]['wall_time'])-results[0]['queued_at']+accepted
            else:
                ds.writer.image_writer.wait_until_done();ds.save_episode(parallel_encoding=False);ds.finalize();store.commit(0)
                accepted=time.monotonic()-t;durable=accepted;encode=None;concurrent={}
            after=resource.getrusage(resource.RUSAGE_CHILDREN)
            result=dict(background=background,frames=n,ready_for_next_seconds=accepted,durable_seconds=durable,
                        worker_encode_seconds=encode,parent_cpu_seconds=time.process_time()-cpu,
                        worker_cpu_seconds=results[0].get('worker_cpu_seconds') if background else None,
                        child_cpu_seconds=after.ru_utime+after.ru_stime-child.ru_utime-child.ru_stime,
                        concurrent=concurrent,bytes=sum(p.stat().st_size for p in (root/'videos').rglob('*.mp4')))
            # Full decode and final LeRobot loader reads before accepting timing.
            for f in (root/'videos').rglob('*.mp4'):
                with av.open(str(f)) as c:
                    c.streams.video[0].codec_context.thread_count=1
                    assert sum(1 for _ in c.decode(video=0))==n
            reader=LeRobotDataset('local/background_benchmark',root=root,video_backend='pyav')
            for i in [0,n//2,n-1]:assert reader[i]['observation.images.rgbd_head_depth'].shape==(1,480,640)
            table=pq.read_table(root/'data/chunk-000/file-000.parquet').to_pydict()
            for key in ('action','observation.state','subtask_index'):
                if key in data:
                    np.testing.assert_array_equal(np.asarray(table[key]),np.asarray(data[key][:n]))
            result['numeric_labels_match_source'] = True
            report.append(result);print(json.dumps(result),flush=True)
            (a.output/'report.json').write_text(json.dumps(report,indent=2))
        finally:
            if queue:queue.close()
            store.close()

    if a.strategy == 'both':
        depth_path='videos/observation.images.rgbd_head_depth/chunk-000/file-000.mp4'
        with av.open(str(a.output/'baseline'/depth_path)) as left, av.open(str(a.output/'background'/depth_path)) as right:
            left.streams.video[0].codec_context.thread_count=1
            right.streams.video[0].codec_context.thread_count=1
            count=0
            for x,y in zip(left.decode(video=0),right.decode(video=0)):
                np.testing.assert_array_equal(x.to_ndarray(),y.to_ndarray())
                count+=1
            assert count==n
        (a.output/'depth_equality.json').write_text(json.dumps(dict(frames=count,decoded_pixels_identical=True)))


if __name__=='__main__':main()
