"""Policy context, epoch boundaries, latched commands and resume contracts."""

import unittest

from dagger_labels import DaggerLabels, FEATURES, validate_features


STAMP = 1_800_000_000_000_000_000


def packet(source=1, epoch=3, original=None):
    return {
        "timestamp_ns": STAMP + 20_000_000,
        "original_command_timestamp_ns": original or STAMP + 20_000_000,
        "authority_epoch": epoch,
        "dagger": {"timestamp_ns": STAMP + 22_000_000, "authority_timestamp_ns": STAMP,
                   "control_source": source, "is_intervention": int(source != 0),
                   "intervention_id": 1, "authority_epoch": epoch, "trial_id": "trial-1"},
    }


class DaggerLabelsTest(unittest.TestCase):
    def labels(self, source=1, grip_original=None, grip_epoch=3):
        labels = DaggerLabels()
        labels.started_ns = STAMP
        labels.ingest(packet(source))
        labels.ingest(packet(source, epoch=grip_epoch, original=grip_original), gripper=True)
        return labels

    def frame(self, labels):
        return labels.frame((STAMP + 30_000_000) / 1e9,
                            {"action_body": -10, "action_gripper": -10}, .5)

    def test_only_causal_expert_commands_are_training_targets(self):
        self.assertEqual(self.frame(self.labels())["dagger.train_mask"], 1)
        for source in (0, 2):
            self.assertEqual(self.frame(self.labels(source))["dagger.train_mask"], 0)
        self.assertEqual(self.frame(self.labels(grip_original=STAMP - 1000))["dagger.train_mask"], 0)
        self.assertEqual(self.frame(self.labels(grip_epoch=2))["dagger.train_mask"], 0)

    def test_missing_future_stale_and_fallback_provenance_fail_closed(self):
        for delta in ({}, {"action_state_fallback": 0}, {"action_body": 1, "action_gripper": -10}):
            with self.assertRaises(ValueError):
                self.labels().frame((STAMP + 30_000_000) / 1e9, delta, .5)
        with self.assertRaisesRegex(ValueError, "missing or stale"):
            self.labels().frame((STAMP + 1_000_000_000) / 1e9, {}, .5)
        labels = self.labels()
        labels.trial_id = "another-trial"
        with self.assertRaisesRegex(ValueError, "trial changed"):
            self.frame(labels)

    def test_current_proxy_epoch_cannot_relabel_an_old_controller_target(self):
        labels = self.labels()
        labels.arms.clear()
        labels.ingest({**packet(), "origin_authority_epoch": 2})
        self.assertEqual(self.frame(labels)["dagger.train_mask"], 0)
        labels.arms.clear()
        labels.ingest({**packet(), "origin_authority_epoch": -1})
        self.assertEqual(self.frame(labels)["dagger.train_mask"], 0)

    def test_bad_packets_never_insert_partial_labels(self):
        for field, value in (("timestamp_ns", 1800000000), ("authority_epoch", True),
                             ("original_command_timestamp_ns", STAMP + 40_000_000), ("dagger", {})):
            labels = DaggerLabels()
            data = packet()
            data[field] = value
            with self.assertRaises(ValueError):
                labels.ingest(data)
            self.assertFalse(labels.labels)
            self.assertFalse(labels.arms)

    def test_fixed_size_buffers_and_counts(self):
        labels = DaggerLabels(2)
        for _ in range(6):
            labels.ingest(packet())
        self.assertEqual(len(labels.arms), 2)
        self.assertEqual(len(labels.labels), 2)
        labels.expert_frames = 12
        labels.reset()
        self.assertEqual(labels.expert_frames, 0)
        self.assertEqual(labels.last_expert_frames, 12)
        self.assertFalse(labels.ready)

    def test_dagger_and_ordinary_roots_cannot_mix(self):
        validate_features({}, False)
        validate_features(FEATURES, True)
        for features, enabled in (({}, True), (FEATURES, False), ({"dagger.train_mask": {}}, True)):
            with self.assertRaises(ValueError):
                validate_features(features, enabled)


if __name__ == "__main__":
    unittest.main()
