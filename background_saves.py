"""Durable bounded episode outbox. One low-priority encoder process, no ROS.

Capture directories never move while image writers hold paths. Only a single
worker publishes, using EpisodeStore's existing journal. Ready jobs survive
parent/worker crashes; a committed job id prevents duplicate publication.
"""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import threading
import uuid
from concurrent.futures import ThreadPoolExecutor

from episode_storage import EpisodeStore, durable_json, sync_dir


def probe_rgb_encoder(config, features):
    """Codec presence alone does not prove the camera size/options can encode."""
    import io
    import av
    import numpy as np
    for feature in features.values():
        if feature.get('dtype') != 'video' or feature.get('info', {}).get('is_depth_map'):
            continue
        shape = feature['shape']
        height, width = (shape[1], shape[2]) if shape[0] == 3 else shape[:2]
        output = av.open(io.BytesIO(), mode='w', format='matroska')
        try:
            stream = output.add_stream(config.vcodec, rate=30)
            stream.width, stream.height, stream.pix_fmt = width, height, config.pix_fmt
            stream.options = config.get_codec_options(encoder_threads=2, as_strings=True)
            frame = av.VideoFrame.from_ndarray(np.zeros((height,width,3),np.uint8), format='rgb24')
            for packet in stream.encode(frame):
                output.mux(packet)
            for packet in stream.encode():
                output.mux(packet)
        finally:
            output.close()


class BackgroundSaves:
    def __init__(self, store, options, limit=2, autostart=True):
        self.store, self.options, self.limit = store, options, limit
        self._ready = threading.Event()
        if autostart:
            self._ready.set()
        self.root = store.root / '.collector-queue'
        self.root.mkdir(exist_ok=True)
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='save-dispatch')
        self.jobs = []
        self.error = None
        self.worker = None
        self.worker_log = None
        self.completed = []
        self.reserved_bytes = 0
        self.base_count = json.loads((store.root / 'meta/info.json').read_text())['total_episodes']
        for job in sorted(self.root.glob('*/job.json'), key=lambda p: json.loads(p.read_text())['episode_index']):
            spec = json.loads(job.read_text())
            marker = store.root / 'annotations/background' / (spec['job_id'] + '.json')
            if marker.exists():
                # Publication is authoritative, including a crash before receipt.
                shutil.rmtree(job.parent)
                continue
            self._dispatch(job)

    @property
    def pending(self):
        return len(self.jobs)

    def new_capture(self):
        if self.error:
            raise RuntimeError('background save failed; pending data retained: ' + self.error)
        if self.pending >= self.limit:
            raise RuntimeError('background save queue full; wait for durable save')
        path = self.root / uuid.uuid4().hex / 'capture'
        path.parent.mkdir()
        return path

    def submit(self, dataset, episode_index, receipt, annotation=None, outcome=None):
        import numpy as np
        if self.pending >= self.limit:
            raise RuntimeError('background save queue full; current episode retained')
        capture = Path(dataset.root)
        job = capture.parent / 'job.json'
        if job.exists():
            raise RuntimeError('episode already queued; refusing duplicate save')
        if not isinstance(episode_index, int) or episode_index < 0:
            raise ValueError('invalid reserved episode index')
        dataset.writer.image_writer.stop()
        buffer = dataset.writer.episode_buffer
        if buffer['size'] <= 0 or buffer['size'] != receipt['frames']:
            raise ValueError('episode frame receipt differs from buffer')
        data = {}
        for key, value in buffer.items():
            if key in dataset.meta.camera_keys:
                # Persist relative paths; allow replay after restoring a dataset.
                data[key] = np.asarray([str(Path(v).relative_to(capture)) for v in value])
            else:
                data[key] = np.asarray(value)
            if data[key].dtype.hasobject:
                raise ValueError('unsupported object buffer: ' + key)
        arrays = capture.parent / 'buffer.npz'
        with arrays.open('wb') as f:
            np.savez(f, **data)
            f.flush()
            os.fsync(f.fileno())
        dataset.writer.image_writer = None
        # No save() here: numeric rows and images remain replayable even if
        # encoding later fails. Finalize only closes the empty metadata writer.
        dataset.finalize()
        for p in capture.rglob('*'):
            if p.is_symlink():
                raise RuntimeError('symlink in capture workspace')
            if p.is_file():
                with p.open('rb') as f:
                    os.fsync(f.fileno())
        for p in sorted((p for p in capture.rglob('*') if p.is_dir()), key=lambda p: len(p.parts), reverse=True):
            sync_dir(p)
        sync_dir(capture)
        spec = dict(version=1, job_id=capture.parent.name, episode_index=episode_index,
                    receipt=receipt, annotation=annotation, outcome=outcome,
                    options=self.options, queued_at=time.time(),
                    depth_units={k:dataset.meta.features[k].get("info", {}).get("depth_unit") for k in dataset.meta.depth_keys})
        durable_json(job, spec)
        sync_dir(self.root)
        self._dispatch(job)
        return spec['job_id']

    def _dispatch(self, job):
        self.reserved_bytes += json.loads(job.read_text())["receipt"].get("reserved_bytes", 0)
        self.jobs.append((job, self.pool.submit(self._run, job)))

    def start(self):
        self._ready.set()

    def _run(self, job):
        self._ready.wait()
        if self.error:
            raise RuntimeError(self.error)
        log = self.root / 'encoder.log'
        env = {**os.environ, 'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1',
               'MKL_NUM_THREADS': '1', 'NUMEXPR_NUM_THREADS': '1'}
        try:
            if self.worker is None:
                self.worker_log = (self.root / 'encoder.log').open('a')
                self.worker = subprocess.Popen([sys.executable, str(Path(__file__).resolve()),
                    '--serve', str(self.store.root), str(self.store._lock.fileno())],
                    pass_fds=(self.store._lock.fileno(),), env=env, stdin=subprocess.PIPE,
                    stdout=self.worker_log, stderr=subprocess.STDOUT, text=True, start_new_session=True)
            self.worker.stdin.write(str(job) + '\n')
            self.worker.stdin.flush()
            while not (job.parent / 'result.json').exists():
                if self.worker.poll() is not None:
                    raise RuntimeError(f'encoder exited {self.worker.returncode}; see {self.root / "encoder.log"}')
                time.sleep(.05)
            result = json.loads((job.parent / 'result.json').read_text())
            # Thousands of raw files must never be deleted on the capture tick.
            marker = self.store.root / 'annotations/background' / (job.parent.name + '.json')
            if not marker.exists():
                raise RuntimeError('encoder receipt lacks durable publication marker')
            shutil.rmtree(job.parent)
            return result
        except Exception as exc:
            self.error = f'{exc}; inspect {log}'
            raise RuntimeError(self.error) from exc

    def poll(self, wait=False):
        results = []
        while self.jobs and (wait or self.jobs[0][1].done()):
            job, future = self.jobs[0]
            try:
                result = future.result()  # Failure leaves raw job and index reserved.
            except Exception:
                if results:
                    break  # Deliver successful predecessors before reporting failure.
                raise
            self.jobs.pop(0)
            self.reserved_bytes -= result.get("reserved_bytes", 0)
            results.append(result)
            self.completed.append(result)
            self.completed = self.completed[-50:]
        return results

    def close(self):
        self._ready.set()
        self.pool.shutdown(wait=True)
        if self.worker is not None:
            if self.worker.stdin:
                self.worker.stdin.close()
            self.worker.wait()
            self.worker = None
        if self.worker_log is not None:
            self.worker_log.close()
            self.worker_log = None


def configure_worker():
    os.nice(10)
    if hasattr(os, 'sched_getaffinity'):
        cpus = sorted(os.sched_getaffinity(0))
        frequencies = {}
        for cpu in cpus:
            try:
                frequencies[cpu] = int(Path(f'/sys/devices/system/cpu/cpu{cpu}/cpufreq/cpuinfo_max_freq').read_text())
            except OSError:
                pass
        if len(frequencies) == len(cpus):
            # Avoid ultra-low-power cores, while preferring later efficiency
            # cores over control workloads on the high-performance cores.
            threshold = .7 * max(frequencies.values())
            cpus = [cpu for cpu in cpus if frequencies[cpu] >= threshold]
        os.sched_setaffinity(0, cpus[-min(4, len(cpus)):])
    print('Background encoder affinity:', sorted(os.sched_getaffinity(0)), flush=True)


def encode_job(root, job, lock_fd):
    # Preserve the same flock across parent death: a new recorder cannot race
    # the still-running encoder. No second lock acquisition or force-unlock.
    cpu_started = time.process_time()
    root, job = Path(root).resolve(), Path(job).resolve()
    lock_path = root.parent / ('.' + root.name + '.collector.lock')
    if not os.path.samestat(os.fstat(lock_fd), lock_path.stat()):
        raise RuntimeError('invalid inherited dataset lock')
    import numpy as np
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.datasets.video_utils import RGBEncoderConfig, DepthEncoderConfig
    spec = json.loads(job.read_text())
    if spec['version'] != 1 or job.parent.parent != root / '.collector-queue' or spec['job_id'] != job.parent.name:
        raise RuntimeError('invalid queue job')
    store = EpisodeStore.__new__(EpisodeStore)
    store.root, store.work, store.journal = root, root / '.collector-work', root / '.collector-commit.json'
    store._lock = os.fdopen(os.dup(lock_fd), 'a')
    marker = root / 'annotations/background' / (spec['job_id'] + '.json')
    if marker.exists():
        durable_json(job.parent / 'result.json', json.loads(marker.read_text()))
        return
    store.recover()
    info = json.loads((root / 'meta/info.json').read_text())
    expected = spec['episode_index']
    if info['total_episodes'] != expected:
        raise RuntimeError('queued episode index differs from committed metadata')
    options = spec['options']
    rgb = RGBEncoderConfig(**options['rgb'])
    depth = DepthEncoderConfig(**options['depth']) if options['depth'] else None
    path = store.prepare()
    kwargs = dict(repo_id=options['repo_id'], root=path, rgb_encoder=rgb, depth_encoder=depth,
                  encoder_threads=2, image_writer_threads=0, streaming_encoding=False)
    if expected:
        ds = LeRobotDataset.resume(**kwargs)
    else:
        ds = LeRobotDataset.create(**kwargs, fps=info['fps'], features=info['features'],
                                   robot_type=info.get('robot_type'), use_videos=True,
                                   **{k: info[k] for k in ('video_files_size_in_mb', 'data_files_size_in_mb') if k in info})
    for key in ds.meta.depth_keys:
        unit = spec.get('depth_units', {}).get(key)
        if unit not in ('m', 'mm'):
            raise RuntimeError('missing depth input unit; refusing ambiguous encoding')
        existing = ds.meta.features[key].setdefault('info', {}).get('depth_unit')
        if existing is not None and existing != unit:
            raise RuntimeError('depth input units changed')
        ds.meta.features[key]['info']['depth_unit'] = unit
    capture = job.parent / 'capture'
    with np.load(job.parent / 'buffer.npz', allow_pickle=False) as z:
        buffer = {k: z[k].tolist() for k in z.files}
    n = int(buffer['size'])
    buffer['episode_index'] = expected
    for key in ds.meta.camera_keys:
        paths = []
        for i, relative in enumerate(buffer[key]):
            source = (capture / relative).resolve()
            if capture not in source.parents or not source.is_file():
                raise RuntimeError('missing or unsafe raw image')
            dest = ds.writer._get_image_file_path(expected, key, i)
            dest.parent.mkdir(parents=True, exist_ok=True)
            os.link(source, dest)
            paths.append(str(dest))
        if len(paths) != n:
            raise RuntimeError('incomplete raw image sequence')
        buffer[key] = paths
    for key, feature in ds.meta.features.items():
        if key not in ds.meta.camera_keys and key in buffer and isinstance(buffer[key], list):
            if feature['dtype'] != 'string':
                buffer[key] = np.asarray(buffer[key], dtype=feature['dtype'])
    ds.writer.episode_buffer = buffer
    started = time.monotonic()
    # Child encoders inherit the same four-CPU affinity; depth and GPU RGB can overlap.
    ds.save_episode(parallel_encoding=True)
    ds.finalize()
    for group, value in [('subtasks', spec['annotation']), ('dagger', spec['outcome'])]:
        if value is not None:
            p = path / 'annotations' / group / f'episode_{expected:06d}.json'
            p.parent.mkdir(parents=True, exist_ok=True)
            durable_json(p, value)
    schema = capture / 'meta/collector_schema.json'
    if schema.exists():
        shutil.copy2(schema, path / 'meta/collector_schema.json')
    result = {**spec['receipt'], 'episode_index': expected, 'job_id': spec['job_id'],
              'event': 'save', 'success': True, 'encode_seconds': time.monotonic() - started,
              'queued_at': spec['queued_at'], 'wall_time': time.time(),
              'worker_cpu_seconds': time.process_time()-cpu_started,
              'worker_cpus': sorted(os.sched_getaffinity(0))}
    p = path / 'annotations/background' / marker.name
    p.parent.mkdir(parents=True, exist_ok=True)
    durable_json(p, result)
    store.commit(expected)
    result['committed_at'] = time.time()
    durable_json(job.parent / 'result.json', result)
    store.close()


def watch_status(path, pid):
    seen, last_error = set(), None
    while True:
        try:
            status = json.loads(Path(path).read_text())
            for result in status.get('completed', []):
                if result['job_id'] not in seen:
                    seen.add(result['job_id'])
                    print(f"[SAVED BACKGROUND] Episode {result['episode_index']} | {result['frames']} frames | 已落盘", flush=True)
            if status.get('error') and status['error'] != last_error:
                last_error = status['error']
                print('[BACKGROUND SAVE FAILED] 暂存数据保留：' + last_error, flush=True)
        except FileNotFoundError:
            pass
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return
        time.sleep(.5)


if __name__ == '__main__':
    if sys.argv[1] == '--watch':
        watch_status(sys.argv[2], int(sys.argv[3]))
    else:
        configure_worker()
        if sys.argv[1] == '--serve':
            for line in sys.stdin:
                if line.strip():
                    encode_job(sys.argv[2], line.strip(), int(sys.argv[3]))
        else:
            encode_job(sys.argv[1], sys.argv[2], int(sys.argv[3]))
