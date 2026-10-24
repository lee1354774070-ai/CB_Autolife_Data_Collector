import json
from pathlib import Path
import tempfile
import unittest

from gui.dagger_counts import collection_counts, describe_counts


class CountsTest(unittest.TestCase):
    def test_committed_labels_only_and_failed_takeover_is_not_autonomous(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'dataset/meta').mkdir(parents=True)
            labels = root / 'dataset/annotations/dagger'
            labels.mkdir(parents=True)
            (root / 'dataset/meta/info.json').write_text(json.dumps({'total_episodes': 5}))
            for i, (success, human) in enumerate([(True, False), (False, False), (True, True), (False, True), (True, False), (True, False)]):
                if i == 4:  # Legacy unlabelled episode.
                    continue
                (labels / f'episode_{i:06d}.json').write_text(json.dumps(dict(
                    episode_index=i, task_success=success, human_takeover=human)))
            (root / '.official_recording_status.json').write_text(json.dumps(dict(
                event='discard', success=True, session_discarded_episodes=2)))
            counts = collection_counts(root)
            self.assertEqual(counts, dict(autonomous_success=1, autonomous_failure=1,
                human_corrected_success=1, human_failure=1, unlabelled=1,
                discarded=2, discard_confirmed=True))
            self.assertIn('本条丢弃已确认', describe_counts(counts))

    def test_no_dataset_is_not_a_save(self):
        with tempfile.TemporaryDirectory() as directory:
            self.assertEqual(collection_counts(directory)['autonomous_success'], 0)

    def test_discard_count_survives_receipt_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'dataset').mkdir()
            log = root / 'dataset/sync_log.jsonl'
            log.write_text('{"event":"episode_discarded"}\n{"event":"episode_saved"}\n{"event":"episode_discarded"}\n')
            self.assertEqual(collection_counts(root)['discarded'], 2)
            with log.open('a') as stream: stream.write('{"event":"episode_discarded"}\n')
            self.assertEqual(collection_counts(root)['discarded'], 3)

    def test_corrupt_label_is_reported_not_silently_counted(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'dataset/meta').mkdir(parents=True)
            (root / 'dataset/meta/info.json').write_text('{"total_episodes":1}')
            path = root / 'dataset/annotations/dagger/episode_000000.json'
            path.parent.mkdir(parents=True)
            path.write_text('{"episode_index":0,"task_success":"false","human_takeover":false}')
            with self.assertRaises(ValueError):
                collection_counts(root)
