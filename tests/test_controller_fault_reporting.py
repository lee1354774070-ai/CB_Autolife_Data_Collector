"""Root fault survives delayed status and recorder errors; no ROS/hardware."""
import ast
import collections
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
from dagger.local_policy_execution import LocalPolicyExecutionMixin

ROOT = Path(__file__).resolve().parents[1]
FAULT = 'tracking error 22.3 deg exceeds limit; output stopped'

class FaultReportingTest(unittest.TestCase):
    def test_rejected_submission_reports_original_fault_not_generic_disabled(self):
        class Base:
            def _set_hardware_fault(self, reason):
                self._hardware_enabled = False
                self._state = 'FAULT'
                self._last_hardware_fault_reason = self._reason = reason
        class Controller(LocalPolicyExecutionMixin, Base):
            pass
        n = Controller()
        n._lock = threading.RLock()
        n._local_policy_cache = collections.OrderedDict()
        n._local_policy_enabled = True
        n._collector_session_id = 'trial'
        n._collector_target_allowed = Mock(return_value=True)
        n.get_logger = Mock()
        n._publish_local_policy_receipt = Mock()
        n._set_hardware_fault(FAULT)
        n._reason = 'authority changed; current-pose anchor'
        n._on_local_policy_submission(SimpleNamespace(data=json.dumps(dict(
            collector_session_id='trial', authority_epoch=1, proposal_id='p',
            chunk_step=0, bridge_generation=1, action=[0.] * 21, units='degrees',
            timestamp_ns=time.time_ns()))))
        receipt = n._publish_local_policy_receipt.call_args.args[0]
        self.assertFalse(receipt['accepted'])
        self.assertEqual(receipt['reason'], FAULT)
        self.assertEqual(receipt['controller_state'], 'FAULT')

    def test_supervisor_keeps_original_fault_when_authority_anchor_overwrites_reason(self):
        tree = ast.parse((ROOT/'dagger/supervisor.py').read_text())
        method = next(n for c in tree.body if isinstance(c, ast.ClassDef)
                      for n in c.body if isinstance(n, ast.FunctionDef) and n.name == '_on_controller_status')
        scope = dict(time=time)
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<status>', 'exec'), scope)
        n = Mock()
        n._lock = threading.RLock()
        n._parse = lambda m: json.loads(m.data)
        n._machine.mode.value = 'POLICY_ACTIVE'
        n._machine.estop.return_value = SimpleNamespace(reason=FAULT,
            old=SimpleNamespace(value='POLICY_ACTIVE'), new=SimpleNamespace(value='ESTOP'))
        message = SimpleNamespace(data=json.dumps(dict(state='FAULT',
            reason='authority changed; current-pose anchor', last_hardware_fault_reason=FAULT)))
        scope['_on_controller_status'](n, message)
        self.assertEqual(n._last_failure_reason, FAULT)
        self.assertEqual(n._controller_status['reason'], FAULT)
        n._machine.estop.assert_called_once_with(FAULT)
        n.get_logger().error.assert_called_once_with('Controller output stopped: ' + FAULT)
        self.assertIn(FAULT, n._set_notice_locked.call_args.args[0])
        n._machine.mode.value = 'ESTOP'
        scope['_on_controller_status'](n, message)
        self.assertEqual(n._last_failure_reason, FAULT)
        n._machine.estop.assert_called_once()

    def test_launcher_prioritizes_fault_and_distinguishes_tracking_wait(self):
        tree = ast.parse((ROOT/'gui/dagger_launcher.py').read_text())
        fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'describe_state')
        scope = {}
        exec(compile(ast.Module(body=[fn], type_ignores=[]), '<launcher>', 'exec'), scope)
        for mode in ('ESTOP', 'FAILURE_HOLD'):
            text, color = scope['describe_state'](dict(mode=mode, failure_reason=FAULT,
                controller_reason='hardware output disabled',
                trial=dict(invalid_reason='missing_stale_or_noncausal_action')))
            self.assertLess(text.index(FAULT), text.index('missing_stale'))
            self.assertNotIn('hardware output disabled', text)
            self.assertEqual(color, '#ffb09a')
        text, color = scope['describe_state'](dict(mode='POLICY_ACTIVE',
            policy_bridge=dict(phase='holding', detail='action[3] error 20 deg')))
        self.assertIn('等待机械臂跟随', text)
        self.assertNotIn('异常', text)
