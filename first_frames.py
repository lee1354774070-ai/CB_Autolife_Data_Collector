"""Retain recorded RGB frame zero inside the episode's private transaction."""

import os
from pathlib import Path


RGB_POSITIONS = {
    'rgbd_head_color': 'head',
    'hand_left': 'left',
    'hand_right': 'right',
}


def first_frame_path(position: str, episode_index: int) -> Path:
    """Shared naming for original capture PNGs and offline video extracts."""
    return Path('first_frame') / f'{position}_frame' / f'{position}_episode_{episode_index:05d}.png'


def stage_first_frames(dataset, work: Path, episode_index: int) -> None:
    """Run after image writes finish, before video encoding removes the PNGs.

    A same-filesystem hardlink preserves the original lossless RGB PNG without
    another pixel buffer, image encode or file copy. LeRobot can then delete its
    temporary image name; EpisodeStore publishes/rolls back the retained name
    with the rest of the episode. No image is read from the live camera cache.
    """
    from PIL import Image

    root = Path(work).resolve()
    for camera, position in RGB_POSITIONS.items():
        key = f'observation.images.{camera}'
        if key not in dataset.features:
            continue  # Preserve camera-only/partial-camera collection modes.
        if dataset.features[key].get('info', {}).get('is_depth_map', False):
            raise RuntimeError(f'Expected RGB camera for first-frame export: {camera}')
        source = Path(dataset.writer._get_image_file_path(episode_index, key, frame_index=0))
        if source.is_symlink() or not source.is_file() or not source.resolve().is_relative_to(root):
            raise RuntimeError(f'Missing or unsafe recorded first frame: {source}')
        with Image.open(source) as image:
            if image.format != 'PNG' or image.mode != 'RGB':
                raise RuntimeError(f'First frame is not an RGB PNG: {source}')
            image.verify()
        parent = root / 'first_frame'
        directory = parent / f'{position}_frame'
        if parent.is_symlink() or directory.is_symlink():
            raise RuntimeError(f'Symlink not allowed in first-frame destination: {directory}')
        directory.mkdir(parents=True, exist_ok=True)
        # link() refuses existing files. All changes remain private until the
        # normal episode commit; failures follow the recorder's invalid path.
        os.link(source, root / first_frame_path(position, episode_index))
