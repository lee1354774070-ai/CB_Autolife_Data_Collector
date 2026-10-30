#!/usr/bin/env python3
"""Project joint/EEF channels independently into a NEW LeRobot v3 dataset.

Examples:
  python convert_dataset.py SOURCE DEST --state joint --action joint
  python convert_dataset.py SOURCE DEST --state joint --action eef+gripper
  python convert_dataset.py SOURCE DEST --state eef --action eef

This is lossless column selection, not IK/FK, action relabeling, or resampling.
All rows, tasks, annotations and video offsets remain unchanged. Per-action
training masks follow the same column selection as action. Videos
are copied byte-for-byte, never re-encoded. Existing/source roots are never
overwritten. Stop the collector normally before running this offline tool.
"""

from __future__ import annotations

import argparse
import copy
import ctypes
import fcntl
import json
import os
from pathlib import Path
import shutil
import tempfile

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from eef_schema import SCHEMA_PATH, SELECTIONS, VECTOR_KEYS, select_indices, validate_layout
from episode_storage import sync_dir
from cli_help import show_requested_parameter_help


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def project_layout(info, layout, selections):
    validate_layout(info, layout)
    new_info, new_layout = copy.deepcopy(info), copy.deepcopy(layout)
    projections = {}
    for key, selection in zip(VECTOR_KEYS, selections, strict=True):
        entry = layout["features"][key]
        indices = select_indices(entry, selection)
        projections[key] = (indices, len(entry["names"]))
        names = [entry["names"][i] for i in indices]
        new_info["features"][key].update(names=names, shape=[len(names)])
        projected = new_layout["features"][key]
        projected["names"] = names
        projected["groups"] = {
            group: [i for i, original in enumerate(indices) if original in members]
            for group, members in entry["groups"].items()
        }
        if not projected["groups"]["eef"]:
            projected.pop("eef", None)
    # A per-action mask is part of the action schema, not an unrelated label.
    # Preserve its values and statistics using exactly the same projection.
    mask_key = "dagger.action_train_mask"
    mask = info["features"].get(mask_key)
    if mask is not None:
        if (mask.get("dtype") != "int64" or mask.get("names") != layout["features"]["action"]["names"]
                or list(mask.get("shape", [])) != [projections["action"][1]]):
            raise ValueError("DAgger action mask does not match the source action layout")
        projections[mask_key] = projections["action"]
        new_info["features"][mask_key].update(
            names=list(new_info["features"]["action"]["names"]),
            shape=list(new_info["features"]["action"]["shape"]))
    validate_layout(new_info, new_layout)
    return new_info, new_layout, projections


def project_stats(stats, projections):
    """Per-channel stats commute with selection; slice, do not approximate.

    This preserves global/episode quantiles exactly, as well as mean/std/min/
    max. Count describes rows, not dimensions, so it MUST NOT be sliced.
    """
    result = copy.deepcopy(stats)
    for key, (indices, width) in projections.items():
        if key not in stats:
            raise ValueError(f"Missing statistics for {key}")
        for name, value in stats[key].items():
            if name == "count":
                continue
            array = np.asarray(value)
            if array.shape != (width,):
                raise ValueError(f"Unsupported statistic shape: {key}/{name}: {array.shape}")
            result[key][name] = array[indices].tolist()
    return result


def _project_parquet(source, destination, projections):
    """Rewrite bounded row batches, including Arrow/Hugging Face dimensions."""
    with pq.ParquetFile(source) as parquet:
        schema = parquet.schema_arrow
        fields = list(schema)
        for name, (indices, width) in projections.items():
            index = schema.get_field_index(name)
            if index < 0:
                raise ValueError(f"Missing column {name} in {source}")
            old = schema.field(index)
            if pa.types.is_fixed_size_list(old.type):
                if old.type.list_size != width:
                    raise ValueError(f"Wrong list width: {source}: {name}")
                kind = pa.list_(old.type.value_type, len(indices))
            elif pa.types.is_list(old.type):
                kind = pa.list_(old.type.value_type)
            elif pa.types.is_large_list(old.type):
                kind = pa.large_list(old.type.value_type)
            else:
                raise ValueError(f"Expected a vector column: {source}: {name}")
            fields[index] = pa.field(name, kind, nullable=old.nullable, metadata=old.metadata)
        metadata = dict(schema.metadata or {})
        if b"huggingface" in metadata:
            # Use the library's Arrow type mapping, rather than guessing which
            # installed datasets release calls fixed vectors Sequence or List.
            from datasets import Features
            hf = json.loads(metadata[b"huggingface"])
            mapped = Features.from_arrow_schema(pa.schema(fields)).to_dict()
            for name in projections:
                hf["info"]["features"][name] = mapped[name]
            metadata[b"huggingface"] = json.dumps(hf).encode()
        output_schema = pa.schema(fields, metadata=metadata)
        destination.parent.mkdir(parents=True, exist_ok=True)
        rows = 0
        with pq.ParquetWriter(destination, output_schema, compression="zstd") as writer:
            for batch in parquet.iter_batches(batch_size=4096):
                columns = list(batch.columns)
                for name, (indices, width) in projections.items():
                    index = schema.get_field_index(name)
                    values = np.asarray(columns[index].to_pylist())
                    if values.shape != (batch.num_rows, width) or not np.isfinite(values).all():
                        raise ValueError(f"Malformed/nonfinite vector in {source}: {name}")
                    columns[index] = pa.array(values[:, indices].tolist(), type=fields[index].type)
                writer.write_batch(pa.RecordBatch.from_arrays(columns, schema=output_schema))
                rows += batch.num_rows
        return rows


def _episode_projections(path, projections):
    columns = pq.read_schema(path).names
    result = {}
    for key, projection in projections.items():
        prefix = f"stats/{key}/"
        found = [name for name in columns if name.startswith(prefix) and name != prefix + "count"]
        if not found:
            raise ValueError(f"Missing per-episode statistics: {path}: {key}")
        result.update({name: projection for name in found})
    return result


def _check_references(root, info, episode_files):
    """Check v3 shard references, without assuming one episode per file."""
    videos = [key for key, feature in info["features"].items() if feature["dtype"] == "video"]
    checked = set()
    count = 0
    for path in episode_files:
        with pq.ParquetFile(path) as parquet:
            for batch in parquet.iter_batches(batch_size=4096):
                for episode in batch.to_pylist():
                    for key in [None, *videos]:
                        prefix = "data" if key is None else f"videos/{key}"
                        template = info["data_path" if key is None else "video_path"]
                        relative = Path(template.format(video_key=key,
                                        chunk_index=int(episode[prefix + "/chunk_index"]),
                                        file_index=int(episode[prefix + "/file_index"])))
                        if relative.is_absolute() or ".." in relative.parts:
                            raise ValueError(f"Unsafe shard reference: {relative}")
                        if relative not in checked:
                            if not (root / relative).is_file():
                                raise ValueError(f"Missing referenced shard: {relative}")
                            checked.add(relative)
                    count += 1
    if count != info["total_episodes"]:
        raise ValueError(f"Episode count mismatch: {count} != {info['total_episodes']}")


def _publish(stage, destination):
    # Linux renameat2(RENAME_NOREPLACE) atomically publishes a whole dataset
    # while also refusing an empty destination created by a racing process.
    libc = ctypes.CDLL(None, use_errno=True)
    rename = libc.renameat2
    rename.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    rename.restype = ctypes.c_int
    if rename(-100, os.fsencode(stage), -100, os.fsencode(destination), 1) != 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error), str(destination))


def convert_dataset(source, destination, *, state="all", action="all"):
    source, destination = Path(source).expanduser().resolve(), Path(destination).expanduser()
    if os.path.lexists(destination):
        raise FileExistsError(f"Destination already exists; nothing overwritten: {destination}")
    destination = destination.resolve()
    if source == destination or source in destination.parents or destination in source.parents:
        raise ValueError("Source and destination must be separate, non-nested directories")
    if destination.exists():
        raise FileExistsError(f"Destination already exists; nothing overwritten: {destination}")
    if not (source / SCHEMA_PATH).is_file():
        raise ValueError(f"Missing {SCHEMA_PATH}; cannot infer EEF indices from old/third-party vectors")
    # Same advisory lock as EpisodeStore: a running collector fails fast here.
    # The lock lives outside the dataset; no source dataset file is modified.
    lock_path = source.parent / ("." + source.name + ".collector.lock")
    with lock_path.open("a+") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_SH | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("Collector owns the source dataset; save/quit and wait for exit first") from None
        if (source / ".collector-work").exists() or (source / ".collector-commit.json").exists():
            raise RuntimeError("Source has an unfinished collector transaction; resolve it before conversion")
        info, layout = read_json(source / "meta/info.json"), read_json(source / SCHEMA_PATH)
        if str(info.get("codebase_version", "")) != "v3.0":
            raise ValueError("Only LeRobot v3.0 datasets are supported")
        if int(info.get("total_episodes", 0)) < 1 or int(info.get("total_frames", 0)) < 1:
            raise ValueError("Source dataset must contain saved episodes")
        new_info, new_layout, projections = project_layout(info, layout, (state, action))
        stats = project_stats(read_json(source / "meta/stats.json"), projections)
        files = []
        for group in ("meta", "data", "videos", "annotations", "first_frame"):
            parent = source / group
            if parent.is_symlink():
                raise ValueError(f"Symlink source group is not supported: {parent}")
            for path in sorted(parent.rglob("*")):
                if path.is_symlink():
                    raise ValueError(f"Symlink source file is not supported: {path}")
                if path.is_file():
                    files.append(path)
        episode_files = [path for path in files if path.is_relative_to(source / "meta/episodes") and path.suffix == ".parquet"]
        episode_paths = set(episode_files)
        _check_references(source, info, episode_files)
        destination.parent.mkdir(parents=True, exist_ok=True)
        stage = Path(tempfile.mkdtemp(prefix=f".{destination.name}.converting-", dir=destination.parent))
        published = False
        try:
            rows = 0
            for path in files:
                relative = path.relative_to(source)
                out = stage / relative
                if relative.as_posix() in ("meta/info.json", "meta/stats.json", SCHEMA_PATH):
                    continue
                if relative.parts[0] == "data" and path.suffix == ".parquet":
                    rows += _project_parquet(path, out, projections)
                elif path in episode_paths:
                    _project_parquet(path, out, _episode_projections(path, projections))
                else:
                    out.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(path, out)  # Independent bytes, no source hardlinks.
            if rows != info["total_frames"]:
                raise ValueError(f"Frame count mismatch: {rows} != {info['total_frames']}")
            new_layout["conversion"] = dict(source=str(source), state=state, action=action,
                                            operation="column_selection", videos="byte_copy")
            write_json(stage / "meta/info.json", new_info)
            write_json(stage / "meta/stats.json", stats)
            write_json(stage / SCHEMA_PATH, new_layout)
            _check_references(stage, new_info, [stage / p.relative_to(source) for p in episode_files])
            # Seal the private output before its atomic directory publication.
            for path in stage.rglob("*"):
                if path.is_file():
                    with path.open("rb") as stream:
                        os.fsync(stream.fileno())
            for path in sorted((p for p in stage.rglob("*") if p.is_dir()), key=lambda p: len(p.parts), reverse=True):
                sync_dir(path)
            sync_dir(stage)
            _publish(stage, destination)
            published = True
            sync_dir(destination.parent)
        finally:
            if not published and stage.exists():
                shutil.rmtree(stage)
        return new_info


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("source", type=Path, help="Saved source dataset root (contains meta/info.json and collector_schema.json)")
    parser.add_argument("destination", type=Path, help="NEW output dataset root; existing/nested paths are refused")
    for key in ("state", "action"):
        parser.add_argument(f"--{key}", choices=SELECTIONS, default="all",
                            help="Independent selection: joint includes recorded grippers/head/waist; eef=14 pose values only; "
                                 "eef+gripper=poses then recorded left/right grippers; all=unchanged (default)")
    show_requested_parameter_help(parser)
    args = parser.parse_args()
    try:
        info = convert_dataset(args.source, args.destination, state=args.state, action=args.action)
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(1, f"Conversion failed: {exc}\nSource dataset was not modified. "
                    "If the destination exists, inspect it before retrying; it will never be overwritten.\n")
    print(f"Saved {info['total_episodes']} episodes / {info['total_frames']} frames to {args.destination}")
    print(f"state={info['features']['observation.state']['shape'][0]}, action={info['features']['action']['shape'][0]}; videos copied without re-encoding")


if __name__ == "__main__":
    main()
