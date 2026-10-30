#!/usr/bin/env python3
"""Backfill head/left/right first-frame PNGs from saved LeRobot v3 videos.

No ROS, model or robot is needed. Stop collection before running this tool.
Existing valid PNGs are kept, including lossless originals saved by the recorder.
Video extracts retain the stored video's compression loss, not original pixels.
"""

from __future__ import annotations

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import tempfile

from episode_storage import sync_dir
from first_frames import RGB_POSITIONS, first_frame_path


def local_path(root: Path, relative: str | Path) -> Path:
    """Reject metadata paths/output links that could escape the dataset."""
    relative = Path(relative)
    if relative.is_absolute() or '..' in relative.parts:
        raise ValueError(f'Unsafe dataset path: {relative}')
    path = root
    for part in relative.parts:
        path = path / part
        if path.is_symlink():
            raise ValueError(f'Symlink not supported: {path}')
    return path


def episode_rows(root: Path, info: dict, cameras: dict):
    """Read only indices and video offsets, never load per-frame data/stats."""
    import pyarrow.parquet as pq

    directory = local_path(root, 'meta/episodes')
    files = sorted(directory.glob('chunk-*/file-*.parquet'))
    columns = ['episode_index'] + [
        f'videos/{key}/{field}' for key in cameras
        for field in ('chunk_index', 'file_index', 'from_timestamp', 'to_timestamp')
    ]
    total = 0
    for path in files:
        local_path(root, path.relative_to(root))
        with pq.ParquetFile(path) as parquet:
            if not set(columns).issubset(parquet.schema_arrow.names):
                raise ValueError(f'Missing episode/video metadata columns: {path}')
            total += parquet.metadata.num_rows
    if total != info['total_episodes']:
        raise ValueError(f'Episode count mismatch: metadata={total}, info={info["total_episodes"]}')
    seen = set()
    for path in files:
        with pq.ParquetFile(path) as parquet:
            for batch in parquet.iter_batches(batch_size=256, columns=columns):
                for row in batch.to_pylist():
                    index = row['episode_index']
                    if not isinstance(index, int) or index < 0 or index in seen:
                        raise ValueError(f'Invalid or duplicate episode_index: {index}')
                    seen.add(index)
                    yield row


def decode_first_frame(path: Path, timestamp: float, end: float, fps: float):
    """Seek to the preceding keyframe, then match the episode's video offset.

    A shard may contain many episodes. Its first decoded/keyframe image is not
    necessarily this episode's first frame. Keep at most one candidate frame,
    limit decoder threads, and refuse offsets further than half a frame away.
    """
    import av

    if not math.isfinite(timestamp) or not math.isfinite(end) or not 0 <= timestamp < end:
        raise ValueError(f'Invalid video interval [{timestamp}, {end}): {path}')
    tolerance = 0.5 / fps
    with av.open(str(path)) as container:
        if len(container.streams.video) != 1:
            raise ValueError(f'Expected one video stream: {path}')
        stream = container.streams.video[0]
        stream.thread_count = 1
        container.seek(max(0, round(timestamp / stream.time_base) - 1),
                       backward=True, any_frame=False, stream=stream)
        closest, delta = None, float('inf')
        for frame in container.decode(stream):
            if frame.pts is None:
                continue
            at = float(frame.pts * frame.time_base)
            # A tiny offset-rounding error is allowed, but not a frame from
            # the next episode or an unrelated keyframe returned by seek().
            distance = abs(at - timestamp)
            if at < end and distance < delta:
                closest, delta = frame, distance
            if at >= timestamp:
                break
        if closest is None or delta >= tolerance:
            raise ValueError(f'No first frame within {tolerance:.6f}s of {timestamp:.6f}s: {path}')
        if closest.is_corrupt:
            raise ValueError(f'Corrupt first frame: {path}')
        return closest.to_image().convert('RGB')


def export_episode(root: Path, info: dict, cameras: dict, row: dict) -> tuple[int, int]:
    """Stage one episode before linking; never replace a pre-existing PNG.

    Failure/interrupt removes only links created by this call, verified by inode.
    A hard kill can leave a subset of complete PNGs; rerunning fills missing ones.
    Metadata, video, data and existing snapshots are never rewritten.
    """
    from PIL import Image

    index = row['episode_index']
    pending, skipped = [], 0
    for key, position in cameras.items():
        destination = local_path(root, first_frame_path(position, index))
        if destination.exists():
            with Image.open(destination) as image:
                if image.format != 'PNG' or image.mode != 'RGB':
                    raise ValueError(f'Existing snapshot is not an RGB PNG: {destination}')
                image.verify()
            skipped += 1
        else:
            pending.append((key, destination))
    if not pending:
        return 0, skipped

    with tempfile.TemporaryDirectory(prefix='.first-frame-extract-', dir=root) as temporary:
        staged, published = [], []
        complete = False
        try:
            for key, destination in pending:
                prefix = f'videos/{key}/'
                indices = {name: row[prefix + name] for name in ('chunk_index', 'file_index')}
                if any(not isinstance(value, int) or value < 0 for value in indices.values()):
                    raise ValueError(f'Invalid video shard indices for {key}: {indices}')
                source = local_path(root, info['video_path'].format(video_key=key, **indices))
                image = decode_first_frame(source, float(row[prefix + 'from_timestamp']),
                                           float(row[prefix + 'to_timestamp']), float(info['fps']))
                stage = Path(temporary) / destination.name
                with image, stage.open('xb') as output:
                    image.save(output, format='PNG')
                    output.flush()
                    os.fsync(output.fileno())
                staged.append((stage, destination))
            for stage, destination in staged:
                local_path(root, destination.relative_to(root))
                destination.parent.mkdir(parents=True, exist_ok=True)
                os.link(stage, destination)  # Refuses races/collisions, unlike replace().
                published.append((stage, destination))
                sync_dir(destination.parent)
            sync_dir(root / 'first_frame')
            sync_dir(root)
            complete = True
        finally:
            if not complete:
                for stage, destination in reversed(published):
                    if not destination.is_symlink() and destination.exists() and os.path.samefile(stage, destination):
                        destination.unlink()
                        sync_dir(destination.parent)
    return len(pending), skipped


def extract_first_frames(source, *, episodes=None, report=print) -> dict:
    """Extract all saved episodes, or selected zero-based episode indices."""
    root = Path(source).expanduser().resolve(strict=True)
    selected = None if episodes is None else set(episodes)
    if selected is not None and (not selected or any(not isinstance(i, int) or i < 0 for i in selected)):
        raise ValueError('Episode indices must be nonnegative integers')
    lock_path = root.parent / ('.' + root.name + '.collector.lock')
    with lock_path.open('a+') as lock:
        try:
            # Exclusive: unlike read-only conversion, we publish files in root.
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('Dataset is in use; stop collection/conversion/extraction first') from None
        if any(os.path.lexists(root / name) for name in ('.collector-work', '.collector-commit.json')):
            raise RuntimeError('Unfinished collector transaction; resolve it before extraction')
        info = json.loads(local_path(root, 'meta/info.json').read_text(encoding='utf-8'))
        if info.get('codebase_version') != 'v3.0':
            raise ValueError('Only LeRobot v3.0 video datasets are supported')
        fps = float(info['fps'])
        if not math.isfinite(fps) or fps <= 0 or info['total_episodes'] < 1:
            raise ValueError('Dataset must have positive FPS and saved episodes')
        cameras = {f'observation.images.{name}': position for name, position in RGB_POSITIONS.items()
                   if f'observation.images.{name}' in info['features']}
        if not cameras:
            raise ValueError('No supported RGB cameras: rgbd_head_color, hand_left, hand_right')
        for key in cameras:
            feature = info['features'][key]
            if feature.get('dtype') != 'video' or feature.get('info', {}).get('is_depth_map', False):
                raise ValueError(f'Expected an RGB video feature: {key}')
        result = {'episodes': 0, 'saved': 0, 'skipped': 0}
        remaining = set() if selected is None else selected.copy()
        for row in episode_rows(root, info, cameras):
            index = row['episode_index']
            if selected is not None and index not in selected:
                continue
            remaining.discard(index)
            try:
                saved, skipped = export_episode(root, info, cameras, row)
            except Exception as exc:
                raise RuntimeError(f'Episode {index} extraction failed: {exc}') from exc
            result['episodes'] += 1
            result['saved'] += saved
            result['skipped'] += skipped
            report(f'Episode {index:05d}: saved={saved}, existing={skipped}')
        if remaining:
            raise ValueError(f'Episode indices not found: {sorted(remaining)}')
        return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', type=Path, help='Saved LeRobot v3 dataset root containing meta/ and videos/')
    parser.add_argument('--episodes', type=int, nargs='+', metavar='INDEX',
                        help='Only these zero-based episode indices; default: all saved episodes')
    args = parser.parse_args(argv)
    try:
        result = extract_first_frames(args.dataset, episodes=args.episodes)
    except Exception as exc:
        parser.exit(1, f'ERROR: {exc}\nDataset/video files and pre-existing PNGs are unchanged.\n')
    print(f'Done: episodes={result["episodes"]}, saved={result["saved"]}, existing={result["skipped"]}')
    print(f'Output: {args.dataset.expanduser().resolve() / "first_frame"}')


if __name__ == '__main__':
    main()
