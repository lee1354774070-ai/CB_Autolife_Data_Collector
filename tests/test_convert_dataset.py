"""Real v3 save/load and out-of-place conversion; all signals are synthetic."""

import copy
import hashlib
import importlib.util
import itertools
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from eef_schema import SCHEMA_PATH, SELECTIONS, VECTOR_KEYS, capture_schema, select_indices
from robot_schema import build_robot_schema


@unittest.skipUnless(importlib.util.find_spec("lerobot") and importlib.util.find_spec("pyarrow"), "requires the LeRobot environment")
class ConvertDatasetTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from lerobot.datasets import LeRobotDataset
        from lerobot.configs import RGBEncoderConfig
        from convert_dataset import write_json
        cls.temporary = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temporary.name)
        cls.source = cls.root / "source"
        cls.layout = capture_schema(build_robot_schema(with_head=True, with_upper_waist=True).names,
                                    with_state=True, with_action=True)
        features = {key: dict(dtype="float32", shape=(len(entry["names"]),), names=entry["names"])
                    for key, entry in cls.layout["features"].items()}
        features.update({"observation.images.hand_left": dict(dtype="video", shape=(32, 32, 3), names=["height", "width", "channel"]),
                         "subtask_index": dict(dtype="int64", shape=(1,), names=None),
                         "dagger.action_train_mask": dict(dtype="int64", shape=(35,), names=features['action']['names']),
                         "dagger.train_mask": dict(dtype="int64", shape=(1,), names=None)})
        dataset = LeRobotDataset.create("local/eef_test", root=cls.source, fps=30, features=features,
                                        use_videos=True, rgb_encoder=RGBEncoderConfig(vcodec="h264"),
                                        image_writer_threads=1, encoder_threads=1)
        for episode in range(2):
            for index in range(3):
                frame = {key: np.arange(35, dtype=np.float32) / 100 + episode + index / 10 for key in VECTOR_KEYS}
                frame.update({"task": f"task {episode}", "observation.images.hand_left": np.full((32, 32, 3), 20 + index * 50, np.uint8),
                              "dagger.action_train_mask": np.array([index % 2] * 21 + [0] * 14, np.int64),
                              "subtask_index": np.array([index - 1], np.int64), "dagger.train_mask": np.array([index % 2], np.int64)})
                dataset.add_frame(frame)
            dataset.save_episode()
        dataset.finalize()
        write_json(cls.source / SCHEMA_PATH, cls.layout)
        write_json(cls.source / "annotations/subtasks/episode_000000.json", {"ends": [1, 3], "needs_review": False})
        cls.before = cls.fingerprint(cls.source)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    @staticmethod
    def fingerprint(root):
        return {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in root.rglob("*") if path.is_file()}

    def test_all_state_action_combinations_load_in_official_reader(self):
        from convert_dataset import convert_dataset, read_json
        from lerobot.datasets import LeRobotDataset
        import pyarrow.parquet as pq
        original = pq.read_table(sorted((self.source / "data").rglob("*.parquet"))[0])
        original_stats = read_json(self.source / "meta/stats.json")
        for state, action in itertools.product(SELECTIONS, repeat=2):
            with self.subTest(state=state, action=action):
                destination = self.root / f"converted-{state}-{action}"
                info = convert_dataset(self.source, destination, state=state, action=action)
                data = LeRobotDataset("local/converted", root=destination, video_backend="pyav")
                self.assertEqual(len(data), 6)
                self.assertEqual(data[0]["observation.images.hand_left"].shape, (3, 32, 32))
                self.assertEqual(data[3]["task"], "task 1")
                self.assertEqual(int(data[1]["subtask_index"]), 0)
                self.assertEqual(int(data[1]["dagger.train_mask"]), 1)
                stats = read_json(destination / "meta/stats.json")
                episodes = pq.read_table(sorted((destination / "meta/episodes").rglob("*.parquet"))[0]).to_pylist()
                for key, selection in zip(VECTOR_KEYS, (state, action)):
                    indices = select_indices(self.layout["features"][key], selection)
                    self.assertEqual(info["features"][key]["shape"], [len(indices)])
                    np.testing.assert_allclose(data[0][key].numpy(), np.asarray(original[key][0].as_py())[indices])
                    np.testing.assert_allclose(stats[key]["mean"], np.asarray(original_stats[key]["mean"])[indices])
                    self.assertEqual(stats[key]["count"], original_stats[key]["count"])
                    self.assertEqual(len(episodes[0][f"stats/{key}/mean"]), len(indices))
                mask_key = 'dagger.action_train_mask'
                indices = select_indices(self.layout['features']['action'], action)
                self.assertEqual(info['features'][mask_key]['names'], info['features']['action']['names'])
                self.assertEqual(info['features'][mask_key]['shape'], [len(indices)])
                np.testing.assert_array_equal(data[1][mask_key].numpy(), np.array([1] * 21 + [0] * 14)[indices])
                np.testing.assert_allclose(stats[mask_key]['mean'], np.asarray(original_stats[mask_key]['mean'])[indices])
                self.assertEqual(len(episodes[0][f'stats/{mask_key}/mean']), len(indices))
                for relative, digest in self.before.items():
                    if relative.startswith(("videos/", "annotations/")) or relative == "meta/tasks.parquet":
                        self.assertEqual(hashlib.sha256((destination / relative).read_bytes()).hexdigest(), digest)
        self.assertEqual(self.fingerprint(self.source), self.before)

    def test_repeated_conversion_preserves_remaining_groups(self):
        from convert_dataset import convert_dataset
        from lerobot.datasets import LeRobotDataset
        first, second = self.root / "first", self.root / "second"
        convert_dataset(self.source, first, state="eef+gripper", action="all")
        convert_dataset(first, second, state="eef", action="joint")
        dataset = LeRobotDataset("local/second", root=second, video_backend="pyav")
        self.assertEqual(tuple(dataset[0]["observation.state"].shape), (14,))
        self.assertEqual(tuple(dataset[0]["action"].shape), (21,))
        with self.assertRaisesRegex(ValueError, "not recorded"):
            convert_dataset(second, self.root / "impossible", state="all", action="eef")

    def test_write_failure_does_not_publish_or_modify_source(self):
        from convert_dataset import convert_dataset
        destination = self.root / "failure"
        with patch("convert_dataset._project_parquet", side_effect=OSError("simulated disk full")):
            with self.assertRaisesRegex(OSError, "disk full"):
                convert_dataset(self.source, destination, state="eef", action="joint")
        self.assertFalse(destination.exists())
        self.assertFalse(list(self.root.glob(".failure.converting-*")))
        self.assertEqual(self.fingerprint(self.source), self.before)

    def test_source_lock_existing_path_and_nested_paths_refused(self):
        from convert_dataset import convert_dataset
        from episode_storage import EpisodeStore
        for destination in (self.source, self.source / "nested", self.root):
            with self.assertRaises((ValueError, FileExistsError)):
                convert_dataset(self.source, destination)
        destination = self.root / "existing"
        destination.mkdir()
        with self.assertRaises(FileExistsError):
            convert_dataset(self.source, destination)
        store = EpisodeStore(self.source)
        try:
            with self.assertRaisesRegex(RuntimeError, "owns"):
                convert_dataset(self.source, self.root / "locked")
        finally:
            store.close()
        self.assertEqual(self.fingerprint(self.source), self.before)

    def test_atomic_publication_refuses_racing_destination_and_dangling_symlink(self):
        from convert_dataset import _publish, convert_dataset
        stage, destination = self.root / 'racing-stage', self.root / 'racing-dest'
        stage.mkdir()
        destination.mkdir()
        with self.assertRaises(OSError):
            _publish(stage, destination)
        self.assertTrue(stage.exists())
        self.assertTrue(destination.exists())
        dangling = self.root / 'dangling'
        dangling.symlink_to(self.root / 'must-not-create')
        with self.assertRaises(FileExistsError):
            convert_dataset(self.source, dangling)
        self.assertFalse((self.root / 'must-not-create').exists())

    def test_missing_manifest_bad_mapping_and_missing_shard_rejected(self):
        from convert_dataset import project_layout, _check_references, read_json, convert_dataset
        info = read_json(self.source / "meta/info.json")
        broken = copy.deepcopy(self.layout)
        broken["features"]["action"]["groups"]["eef"] = [0]
        with self.assertRaises(ValueError):
            project_layout(info, broken, ("all", "all"))
        for changes in ({'shape': [21]}, {'names': list(reversed(info['features']['action']['names']))}):
            broken_info = copy.deepcopy(info)
            broken_info['features']['dagger.action_train_mask'].update(changes)
            with self.assertRaisesRegex(ValueError, 'mask'):
                project_layout(broken_info, self.layout, ('all', 'all'))
        with self.assertRaisesRegex(ValueError, "Missing"):
            convert_dataset(self.root / "not-a-dataset", self.root / "no-output")
        with self.assertRaisesRegex(ValueError, "Missing referenced"):
            _check_references(self.root / "absent", info, list((self.source / "meta/episodes").rglob("*.parquet")))


if __name__ == "__main__":
    unittest.main()
