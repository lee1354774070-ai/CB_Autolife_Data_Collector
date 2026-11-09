"""Real feedback/receipt methods without ROS, speech hardware or storage writes."""
import ast
import json
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
from dagger.compat import CompatibilityPublisher
from vr_feedback import feedback_packet

SOURCE = Path(__file__).resolve().parents[1] / 'dagger/supervisor.py'
parsed = ast.parse(SOURCE.read_text())
methods = {'_feedback', '_publish_state', '_finish_collection_trial', '_set_notice_locked'}
node_class = next(n for n in parsed.body if isinstance(n, ast.ClassDef) and n.name == 'CollectorDaggerSupervisor')
node_class.body = [n for n in node_class.body if isinstance(n, ast.FunctionDef) and n.name in methods]

class Base:
    def _publish_state(self): pass
    def _set_notice_locked(self, text, level): self._save_notice = dict(text=text, level=level)

scope = dict(HgDaggerSupervisor=Base, json=json, String=SimpleNamespace,
             feedback_packet=feedback_packet, CompatibilityPublisher=CompatibilityPublisher,
             CachedCollector=type('UnusedCache', (), {}))
exec(compile(ast.Module(body=[node_class], type_ignores=[]), str(SOURCE), 'exec'), scope)

class FeedbackTest(unittest.TestCase):
    def setUp(self):
        self.n = scope['CollectorDaggerSupervisor']()
        self.n._lock = threading.RLock()
        self.n._intervention_id = ''
        self.n._recorder_active = False
        self.n._collector = SimpleNamespace(background={})
        self.n._feedback_pub = Mock(); self.n._speech_pub = Mock()
        self.n._feedback_history = []; self.n._background_jobs_seen = set()
        self.n._background_error_seen = ''; self.n._save_notice = {}
        self.n._state_pub = CompatibilityPublisher(Mock(), {})

    def events(self):
        return [json.loads(call.args[0].data)['event'] for call in self.n._feedback_pub.publish.call_args_list]

    def test_completed_receipts_arriving_together_report_each_once_even_while_next_pending(self):
        completed = [dict(job_id=str(i), episode_index=i, success=True) for i in range(2)]
        self.n._collector.background = dict(pending=1, completed=completed[-1:])
        self.n._collector.completed_results = completed
        self.n._publish_state(); self.n._publish_state()
        self.assertEqual(self.events(), ['save', 'save'])
        self.assertEqual(self.n._save_notice['text'], '第2条已保存')
        self.assertEqual(len(self.n._state_pub.contract['feedback_events']), 2)

    def test_queued_receipt_does_not_announce_saved(self):
        n = self.n; n._intervention_id = 'trial'; n._pending_collector_result = None
        n._takeover_seen = True; n._session_id = 'session'
        n._forward_enable = Mock(return_value=(True, 'disabled'))
        n._finish_intervention_locked = Mock(return_value=False)
        n._last_finish_queued = True; n._trace = SimpleNamespace(barrier=lambda:None)
        n._publish_state = Mock()
        n._finish_collection_trial(True)
        self.assertEqual(self.events(), ['saving'])
        self.assertIn('尚未落盘', n._save_notice['text'])

    def test_background_and_worker_error_never_announce_success(self):
        self.n._collector.background = dict(error='disk full', completed=[])
        self.n._publish_state(); self.n._publish_state()
        self.assertEqual(self.events(), ['error'])
        self.assertIn('临时数据已保留', self.n._save_notice['text'])
        self.n._set_notice_locked('回执未知，请检查', 'error')
        self.assertEqual(self.events(), ['error', 'error'])

if __name__ == '__main__': unittest.main()
