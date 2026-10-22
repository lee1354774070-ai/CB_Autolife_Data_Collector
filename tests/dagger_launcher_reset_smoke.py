"""Headless GUI check; Popen is intercepted and no motion is requested."""
import os
from pathlib import Path
import sys
import tempfile
import tkinter as tk
from unittest.mock import Mock, patch

assert os.environ.get('ROS_DOMAIN_ID') == '211'
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rclpy
from gui import dagger_launcher as gui

rclpy.init()
root = tk.Tk()
root.withdraw()
with tempfile.TemporaryDirectory() as directory, \
        patch.object(gui, 'LOGS', Path(directory)/'logs'), \
        patch.object(gui, 'DATA_ROOT', Path(directory)/'data'), \
        patch.object(gui.subprocess, 'Popen') as popen:
    popen.return_value = Mock(poll=Mock(return_value=None))
    launcher = gui.Launcher(root)
    try:
        launcher.poll()
        assert launcher.control_buttons['reset']['state'] == 'normal'
        assert launcher.control_buttons['start']['state'] == 'disabled'
        launcher.command('reset')
        args, kwargs = popen.call_args
        assert '--reset-only' in args[0] and '--serve' not in args[0]
        assert Path(kwargs['env']['OUTPUT_BASE_DIR']) == Path(directory)/'logs/reset'
        assert not (Path(directory)/'data').exists()
        launcher.poll()
        assert all(button['state']=='disabled' for button in launcher.control_buttons.values())
        print('LAUNCHER_RESET_GUI_PASS no_process_started=true no_dataset_created=true')
    finally:
        if launcher.reader:launcher.reader.close()
        launcher.node.destroy_node()
        root.destroy()
        rclpy.shutdown()
