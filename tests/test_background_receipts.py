"""An encoder exit and a dataset commit are independent outcomes."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock

from background_saves import BackgroundSaves
from episode_storage import EpisodeStore, durable_json


class PublicationReceiptTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        root = Path(tmp.name) / 'dataset'
        (root / 'meta').mkdir(parents=True)
        self.info = root / 'meta/info.json'
        durable_json(self.info, dict(total_episodes=0))
        self.store = EpisodeStore(root)
        self.addCleanup(self.store.close)
        self.queue = BackgroundSaves(self.store, {})
        self.addCleanup(self.queue.close)
        self.job = self.queue.new_capture().parent / 'job.json'
        name = self.job.parent.name
        durable_json(self.job, dict(job_id=name, episode_index=0, receipt=dict(frames=3)))
        stage = self.store.prepare()
        for group in ('annotations/background', 'data', 'meta'):
            (stage / group).mkdir(parents=True)
        durable_json(stage / 'meta/info.json', dict(total_episodes=1))
        (stage / 'data/synthetic').write_bytes(b'frame fixture')
        relative = Path('annotations/background', name + '.json')
        self.marker = root / relative
        self.result = dict(job_id=name, episode_index=0, frames=3, event='save', success=True)
        durable_json(stage / relative, self.result)
        self.store.commit(0)
        self.queue.worker = Mock(returncode=1)
        self.queue.worker.poll.side_effect = [None, 1]

    def test_missing_receipt_after_commit_does_not_poison_next_episode(self):
        self.assertEqual(self.queue._run(self.job), self.result)
        self.assertIsNone(self.queue.error)
        self.assertFalse(self.job.exists())
        self.assertNotEqual(self.queue.new_capture().parent, self.job.parent)
        self.assertTrue(self.marker.is_file())

    def test_uncommitted_or_mismatched_proof_cannot_acknowledge(self):
        for field, value in (('job_id', 'other'), ('episode_index', 1), ('frames', 2), ('success', False)):
            with self.subTest(field=field):
                durable_json(self.marker, {**self.result, field: value})
                with self.assertRaises(RuntimeError):
                    self.queue._confirmed_result(self.job)
        durable_json(self.marker, self.result)
        durable_json(self.store.journal, dict(version=1, phase='prepared'))
        with self.assertRaisesRegex(RuntimeError, 'unfinished'):
            self.queue._confirmed_result(self.job)
        self.store.journal.unlink()
        durable_json(self.info, dict(total_episodes=0))
        with self.assertRaisesRegex(RuntimeError, 'not confirmed'):
            self.queue._run(self.job)
        self.assertTrue(self.job.exists())
        self.assertEqual(json.loads(self.marker.read_text()), self.result)
