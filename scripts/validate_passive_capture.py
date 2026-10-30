#!/usr/bin/env python3
"""Opt-in real-input, no-motion collector check in a NEW non-training folder.

No launcher, VR controller, FIFO listener, control publisher or model is used.
The real recorder/writer and unchanged synchronization guards run on existing
SHM + status streams. Every trial is discarded, NEVER committed as an episode.
Run only after the operator has stopped the ordinary collection session.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--collector-root', type=Path, required=True)
    parser.add_argument('--output-parent', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--trials', type=int, default=3)
    parser.add_argument('--bounded-native-pools', action='store_true')
    opts = parser.parse_args()
    if not 1 <= opts.seconds <= 60 or not 1 <= opts.trials <= 5:
        parser.error('bounded validation requires seconds 1..60 and trials 1..5')
    opts.output_parent.mkdir(parents=True, exist_ok=True)
    base = Path(tempfile.mkdtemp(prefix='NOT_TRAINING_', dir=opts.output_parent))
    (base / 'NOT_TRAINING.txt').write_text('Passive validation only. No saved episodes. Do not train.\n')
    sys.path.insert(0, str(opts.collector_root.resolve()))
    import record_lerobot_official as mod
    mod.cv2.setNumThreads(1)
    if opts.bounded_native_pools:
        import torch
        import pyarrow as pa
        torch.set_num_threads(1)
        torch.set_num_interop_threads(1)
        pa.set_cpu_count(1)
        pa.set_io_thread_count(1)
    sys.argv = ['passive-validation', '--output-dir', str(base / 'dataset'),
        '--repo-id', 'local/NOT_TRAINING_passive_validation',
        '--task-name', 'NOT_TRAINING passive no-motion validation',
        '--subtasks-json', '["test phase one", "test phase two"]',
        '--with-head', '--with-upper-waist', '--with-depth',
        '--robot-type', 'autolife_s3', '--image-source', 'shm', '--fps', '30',
        '--image-poll-fps', '120', '--min-cameras', '4',
        '--sync-reference-camera', 'hand_left', '--action-mode', 'status_target',
        '--max-sync-delta-sec', '.03', '--max-image-age-sec', '.15',
        '--max-state-age-sec', '.15', '--max-state-interpolation-gap-sec', '.05',
        '--max-frame-interval-error-ratio', '.45', '--image-writer-threads', '4',
        '--image-writer-max-pending', '64', '--image-writer-max-mb', '256',
        '--max-episode-temp-gb', '8', '--episode-event-file', str(base / 'last_event.json')]
    args = mod.parse_args()
    assert not args.motion_lock_file and not args.control_fifo and not args.dagger
    stats = {}
    for name in ('_poll_shm_images', '_state_cb', '_record_tick'):
        original = getattr(mod.OfficialLeRobotRecorder, name)
        values = stats[name] = []
        def measured(self, *a, _fn=original, _values=values, **kw):
            start = time.monotonic()
            try:
                return _fn(self, *a, **kw)
            finally:
                _values.append((time.monotonic() - start) * 1000)
        setattr(mod.OfficialLeRobotRecorder, name, measured)
    mod.rclpy.init()
    node = mod.OfficialLeRobotRecorder(args)
    executor = mod.SingleThreadedExecutor()
    executor.add_node(node)
    result = {'output': str(base), 'collector': str(opts.collector_root),
              'no_motion': True, 'saved_episodes': 0, 'trials': []}
    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
    try:
        deadline = time.monotonic() + 15
        while not (node.state_buffer and node.all_requested_cameras_seen()):
            if node.state_buffer:
                node.start_image_capture()
            if time.monotonic() >= deadline:
                raise RuntimeError('real telemetry or cameras not ready')
            executor.spin_once(timeout_sec=.02)
        node.create_dataset()
        for index in range(opts.trials):
            spin_for(1)
            if not node.start_episode('authorized-passive-validation'):
                raise RuntimeError('test episode start refused')
            start = time.monotonic()
            while time.monotonic() - start < opts.seconds and node.is_recording:
                executor.spin_once(timeout_sec=.01)
            trial = {'index': index, 'elapsed_sec': round(time.monotonic() - start, 3),
                     'frames': node.current_episode_frames,
                     'invalid': node.episode_invalid,
                     'reason': getattr(node, 'episode_invalid_reason', None)}
            result['trials'].append(trial)
            print(json.dumps(trial), flush=True)
            node.discard_current_episode('passive-validation-never-save')
    finally:
        # Explicitly discard before finish(), whose ordinary behavior saves.
        if node.dataset is not None and (node.is_recording or node.has_pending_episode()):
            node.discard_current_episode('passive-validation-finally-never-save')
        node.finish()
        executor.remove_node(node)
        node.destroy_node()
        mod.rclpy.shutdown()
        for name, values in stats.items():
            values.sort()
            result[name] = {'count': len(values),
                'p99_ms': round(values[min(len(values)-1, int(len(values)*.99))], 3) if values else None,
                'max_ms': round(max(values), 3) if values else None}
        meta = base / 'dataset/meta/info.json'
        if meta.exists():
            result['saved_episodes'] = json.loads(meta.read_text())['total_episodes']
        (base / 'result.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2), flush=True)
    if result['saved_episodes'] or any(t['invalid'] for t in result['trials']):
        raise SystemExit(2)


if __name__ == '__main__':
    main()
