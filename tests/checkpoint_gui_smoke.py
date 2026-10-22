"""Run with xvfb-run; mock all checkpoint requests, never contact Thor."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import time
import tkinter as tk
from unittest.mock import patch
from gui.checkpoint_panel import CheckpointDialog

root = tk.Tk()
root.withdraw()
applied = []
allowed = False
calls = []

def request(action, checkpoint, server_url):
    calls.append((action, checkpoint))
    return {'checkpoint': checkpoint if action == 'switch' else 'model_a',
            'models': ['model_a', 'model_b'], 'health': {'ready': True, 'active_session': False}}

def settle(dialog):
    deadline = time.monotonic() + 2
    while dialog.busy and time.monotonic() < deadline:
        root.update()
        time.sleep(.01)
    assert not dialog.busy

try:
    with patch('gui.checkpoint_panel.checkpoint_request', side_effect=request):
        dialog = CheckpointDialog(root, lambda: allowed, applied.append, 'http://192.168.8.224:8777')
        settle(dialog)
        assert applied == ['model_a']
        dialog.choice.set('model_b')
        dialog.run('switch')
        assert len(calls) == 1
        allowed = True
        dialog.run('switch')
        settle(dialog)
        assert applied[-1] == 'model_b'
        assert 'model_b' in dialog.actual.cget('text')
        dialog.close()
    print('CHECKPOINT_GUI_SMOKE_OK: refresh, busy guard, selection, confirmed result')
finally:
    root.destroy()
