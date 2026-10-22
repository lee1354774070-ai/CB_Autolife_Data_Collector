"""Confirmed invalid workspaces are discardable; unknown saves are not replayed."""
import ast
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock

from dagger.control.recorder import CollectorCommandResult, CollectorFifo

SOURCE = Path(__file__).resolve().parents[1] / 'dagger/control/supervisor_node.py'
method = next(item for cls in ast.parse(SOURCE.read_text()).body if isinstance(cls, ast.ClassDef)
              for item in cls.body if isinstance(item, ast.FunctionDef)
              and item.name == '_finish_intervention_locked')
namespace = {}
exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), 'exec'), namespace)


class StorageReceiptTest(unittest.TestCase):
    def test_receipt_carries_explicit_failure_and_uncertainty(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / '.official_recording_status.json'
            fifo = CollectorFifo(str(Path(directory) / 'commands'), '')
            for uncertain in (False, True):
                path.write_text(json.dumps(dict(request_id='save-1', event='save', success=False,
                                               episode_invalid=True, publication_uncertain=uncertain)))
                result = fifo.wait_for_result(False, 'save', 'save-1', .1)
                self.assertTrue(result.acknowledged and result.episode_invalid)
                self.assertEqual(result.publication_uncertain, uncertain)


    def test_fifo_preserves_legacy_requests_and_transports_compact_success_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'commands'
            os.mkfifo(path)
            fd=os.open(path,os.O_RDWR|os.O_NONBLOCK)
            fifo=CollectorFifo(str(path),'')
            try:
                self.assertTrue(fifo.command(False,'save','legacy'))
                self.assertEqual(os.read(fd,4096).decode().splitlines(), ['save legacy'])
                metadata=dict(trial_id='trial',task_success=True,human_takeover=True)
                self.assertTrue(fifo.command(False,'save','new',metadata=metadata))
                command,request,payload=os.read(fd,4096).decode().split(maxsplit=2)
                self.assertEqual((command,request),('save','new'))
                self.assertEqual(json.loads(payload),metadata)
                with self.assertRaises(ValueError):
                    fifo.command(False,'save',metadata=metadata)
                self.assertFalse(fifo.command(False,'save','large',metadata={'padding':'x'*8192}))
            finally:
                os.close(fd)

    def test_known_invalid_failure_clears_pending_but_unknown_retains_request(self):
        for uncertain in (False, True):
            node = Mock()
            node._intervention_id = 'trial-1'
            node._active_depth = False
            node._recorder_active = True
            node._pending_collector_result = None
            node.get_parameter.return_value.value = 1
            result = CollectorCommandResult(True, False, 'save', 'save-1', 'disk full',
                                            episode_invalid=True, publication_uncertain=uncertain)
            node._collector.command_and_wait.return_value = result
            with self.assertRaisesRegex(RuntimeError, 'unresolved' if uncertain else 'invalid'):
                namespace['_finish_intervention_locked'](node, True, 'test')
            self.assertEqual(node._pending_collector_result, result if uncertain else None)
            self.assertEqual(node._intervention_id, 'trial-1')
            node._collector.command_and_wait.assert_called_once()


if __name__ == '__main__':
    unittest.main()
