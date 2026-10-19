"""Policy context, epoch boundaries, latched commands and resume contracts."""

import unittest

from dagger_labels import DaggerLabels, FEATURES, features_for_actions, validate_features
from eef_schema import EEF_NAMES


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

    def test_body_correction_is_retained_while_gripper_still_belongs_to_policy(self):
        frame = self.frame(self.labels(grip_original=STAMP - 1000, grip_epoch=2))
        self.assertEqual(frame["dagger.train_mask"], 0)  # Legacy consumers remain conservative.
        self.assertEqual(frame["dagger.action_train_mask"], [1] * 14 + [0, 0] + [1] * 5)

    def test_masks_follow_each_command_origin_without_relabelling_policy(self):
        for source in (0, 2):
            self.assertEqual(self.frame(self.labels(source))["dagger.action_train_mask"], [0] * 21)
        self.assertEqual(self.frame(self.labels())["dagger.action_train_mask"], [1] * 21)
        labels = self.labels()
        labels.arms.clear()
        labels.ingest({**packet(), "origin_authority_epoch": 2})
        self.assertEqual(self.frame(labels)["dagger.action_train_mask"], [0] * 14 + [1, 1] + [0] * 5)

    def test_old_dataset_or_wrong_vector_order_cannot_be_silently_resumed(self):
        old = {k: v for k, v in FEATURES.items() if k != "dagger.action_train_mask"}
        with self.assertRaisesRegex(ValueError, "new dataset"):
            validate_features(old, True)
        for change in ({"shape": [1]}, {"names": list(reversed(FEATURES["dagger.action_train_mask"]["names"]))}):
            features = {**FEATURES, "dagger.action_train_mask": {**FEATURES["dagger.action_train_mask"], **change}}
            with self.assertRaises(ValueError):
                validate_features(features, True)

    def test_eef_mask_stays_zero_without_independent_pose_provenance(self):
        names = FEATURES['dagger.action_train_mask']['names'] + list(EEF_NAMES)
        labels = DaggerLabels(action_names=names)
        labels.ingest(packet())
        labels.ingest(packet(), gripper=True)
        values = self.frame(labels)
        self.assertEqual(values['dagger.action_train_mask'], [1] * 21 + [0] * 14)
        self.assertEqual(values['dagger.train_mask'], 1)  # Joint-only legacy contract.
        features = {**features_for_actions(names), 'action': {'names': names}}
        validate_features(features, True)
        with self.assertRaises(ValueError):
            validate_features({**features, 'dagger.action_train_mask': FEATURES['dagger.action_train_mask']}, True)
        with self.assertRaises(ValueError):
            DaggerLabels(action_names=list(reversed(names)))
        self.assertEqual(FEATURES['dagger.action_train_mask']['shape'], (21,))

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
