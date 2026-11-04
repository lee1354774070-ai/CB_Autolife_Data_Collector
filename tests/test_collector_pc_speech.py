"""Read-only PC speech tests. No SSH, ROS, audio or robot actions."""
import ast
import contextlib
from collections import Counter
from concurrent.futures import Future, ThreadPoolExecutor
from types import SimpleNamespace
import time
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from collector_pc_speech import LogSpeech, LogTail, RECEIPT, emit_receipt, find_session, receipt_text


class SpeechTest(unittest.TestCase):
    def test_repeated_marks_and_different_saves_are_never_text_deduplicated(self):
        parser = LogSpeech('vr')
        lines = ['[VR speech] 下一步'] * 8 + ['[VR speech] 第39条已保存', '[VR speech] 第40条已保存']
        self.assertEqual(len([parser.parse(line) for line in lines]), 10)
        self.assertEqual([parser.parse(line) for line in lines][-2:], ['第39条已保存', '第40条已保存'])

    def test_countdown_errors_disabled_and_no_false_success_from_noise(self):
        parser = LogSpeech('vr')
        for text in ('三', '二', '一', '开始', '后台保存中，可以开始下一条', '已保存，标注未完成', '正在丢弃', '后台保存失败'):
            self.assertEqual(parser.parse('[VR speech disabled] ' + text), text)
        for text in ('[VR input] GL+GR=True', '[VR] B ready', 'help: 已保存', '[SAVED] Episode 2'):
            self.assertIsNone(parser.parse(text))

    def test_four_five_and_eight_subtask_totals(self):
        for total in (1, 3, 4, 5, 8):
            for done in range(1, total + 1):
                text = receipt_text(dict(event='mark_subtask', success=True, subtasks={'confirmed': done, 'total': total}))
                self.assertIn(f'第{done}步已标记', text)
                self.assertEqual('下一步' in text, done < total)

    def test_queued_failure_empty_save_and_durable_save_are_distinct(self):
        self.assertNotIn('已保存', receipt_text(dict(event='save_queued', success=True)))
        self.assertNotIn('已保存', receipt_text(dict(event='save', success=False)))
        self.assertNotIn('已保存', receipt_text(dict(event='save', success='false')))
        self.assertIsNone(receipt_text(dict(event='save', success=True, frames=0)))
        self.assertNotIn('已保存', receipt_text(dict(event='save', success=True, frames=5, publication_uncertain=True)))
        self.assertIn('标注未完成', receipt_text(dict(event='save_queued', success=True, subtasks={'enabled': True, 'complete': False})))
        self.assertEqual(receipt_text(dict(event='save', success=True, frames=42, episode_index=0)), '第1条已保存')
        self.assertIn('标注未完成', receipt_text(dict(event='save', success=True, frames=42,
                            subtasks={'enabled': True, 'complete': False})))

    def test_dagger_outcome_and_nonvr_background(self):
        self.assertIn('人工纠错成功', receipt_text(dict(event='save', success=True, frames=42,
                                      task_outcome={'outcome': 'human_corrected_success'})))
        parser = LogSpeech('recorder')
        self.assertEqual(parser.parse('[INFO] [recorder]: [SAVED BACKGROUND] Episode 39 | 100 frames'), '第40条已保存')
        self.assertIn('失败', parser.parse('background save failed; raw data retained: disk full'))
        self.assertIn('异常', parser.parse('[ERROR] EPISODE INVALID: cameras'))
        self.assertIsNone(parser.parse(RECEIPT + '{bad json'))
        self.assertIsNone(parser.parse(RECEIPT + '[]'))

    def test_receipt_emitter_retains_exact_committed_identity(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            emit_receipt(dict(event='mark_subtask', success=True, request_id='r1', wall_time=1.2,
                              subtasks={'confirmed': 4, 'total': 5}))
        result = json.loads(out.getvalue()[len(RECEIPT):])
        self.assertEqual((result['request_id'], result['wall_time']), ('r1', 1.2))
        self.assertIn('第4步', LogSpeech('recorder').parse(out.getvalue()))
        with patch('builtins.print', side_effect=BrokenPipeError):
            emit_receipt({'event': 'save', 'success': True})

    def test_tail_partial_utf8_no_history_no_same_text_loss(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'log'
            path.write_bytes(b'old complete\nold partial')
            tail = LogTail(path)
            with path.open('ab') as out:
                out.write(b' remainder\n')
            self.assertEqual(tail.read(), [])
            data = '[VR speech] 下一步\n'.encode()
            with path.open('ab') as out:
                out.write(data[:-2])
            self.assertEqual(tail.read(), [])
            with path.open('ab') as out:
                out.write(data[-2:] + data)
            lines = tail.read()
            self.assertEqual([line for _, line in lines], ['[VR speech] 下一步']*2)
            self.assertNotEqual(lines[0][0], lines[1][0])
            self.assertEqual(lines[-1][0], path.stat().st_size)
            tail.close()

    def test_tail_truncate_or_replace_never_replays_old_events(self):
        for replace in (False, True):
            with tempfile.TemporaryDirectory() as tmp:
                path = Path(tmp)/'log'; path.write_text('old\n')
                tail = LogTail(path)
                if replace:
                    path.unlink()
                path.write_text('')
                with self.assertRaises(RuntimeError): tail.read()
                tail.close()

    def test_discovery_uses_live_recorder_and_explicit_directory_not_latest(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); proc = root/'proc'; proc.mkdir()
            task = root/'task'; task.mkdir()
            def process(pid, args):
                d = proc/str(pid); d.mkdir(); (d/'cmdline').write_bytes(('\0'.join(args)+'\0').encode())
            process(11, ['python', '/repo/record_lerobot_official.py','--status-file',str(task/'.official_recording_status.json')])
            process(12, ['python','/repo/vr_collector_control.py','--base-dir',str(task)])
            (task/'.official_recording_pids').write_text(f'recorder 11 {task}/record.log\nvr_control 12 {task}/vr.log\n')
            directory, logfile, mode, _ = find_session(proc_root=proc)
            self.assertEqual((directory, logfile, mode), (task.resolve(), task/'vr.log','vr'))
            process(13, ['python','/repo/record_lerobot_official.py','--status-file',str(root/'other'/'.official_recording_status.json')])
            with self.assertRaisesRegex(RuntimeError, '2 live'): find_session(proc_root=proc)
            self.assertEqual(find_session(task, proc_root=proc)[2], 'vr')
            (proc/'12'/'cmdline').write_bytes(b'other process\0')
            self.assertEqual(find_session(task, proc_root=proc)[2], 'recorder')
            (task/'.official_recording_pids').write_text('recorder 99 wrong.log\n')
            with self.assertRaises(RuntimeError): find_session(task, proc_root=proc)

    def test_real_recorder_receipt_cannot_speak_before_barrier_or_on_write_failure(self):
        source = Path(__file__).resolve().parents[1] / 'record_lerobot_official.py'
        tree = ast.parse(source.read_text())
        method = next(item for cls in tree.body if isinstance(cls, ast.ClassDef)
                      for item in cls.body if isinstance(item, ast.FunctionDef) and item.name == '_write_command_status')
        calls = []
        barrier = Future()
        scope = {'Path': Path, 'time': time, 'emit_receipt': lambda value: calls.append(('speech', value.copy())),
                 'write_json_atomic': lambda path, value: calls.append(('write', value.copy()))}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), scope)
        with tempfile.TemporaryDirectory() as directory, ThreadPoolExecutor(max_workers=1) as worker:
            node = SimpleNamespace(args=SimpleNamespace(status_file=str(Path(directory)/'status'), task_name='test'),
                task_episode_counts=Counter(), session_saved_by_task=Counter(), session_discarded_by_task=Counter(),
                is_recording=True, episode_invalid=False, save_failed=False, dagger=None, saved_episodes=0,
                subtasks=SimpleNamespace(progress=lambda: {'confirmed': 0, 'total': 5}), mark_status_future=None,
                mark_status_writer=worker, receipt_writer_closed=False, sync_log=SimpleNamespace(barrier=lambda: barrier))
            scope['_write_command_status'](node, 'start', True, 'r1', message='started')
            self.assertEqual(calls, [])
            node.episode_invalid = True
            node.episode_invalid_reason = 'camera stalled during barrier'
            barrier.set_result(None)
            node.mark_status_future.result(timeout=2)
            self.assertEqual([kind for kind, _ in calls], ['write','speech'])
            self.assertFalse(calls[-1][1]['success'])
            self.assertNotEqual(receipt_text(calls[-1][1]), '开始录制')
            calls.clear()
            def fail_write(path, value): raise OSError('disk full')
            scope['write_json_atomic'] = fail_write
            scope['_write_command_status'](node, 'mark_subtask', True, 'r2', message='marked')
            with self.assertRaises(OSError): node.mark_status_future.result(timeout=2)
            self.assertEqual(calls, [])


if __name__ == '__main__':
    unittest.main()
