#!/usr/bin/env python3
"""Read-only NDJSON speech events for the management website's SSH backend.

This process owns neither collection nor audio: stopping it only stops log
following. No HTTP listener, ROS imports, camera access or control commands.
"""
from __future__ import annotations

import argparse
import base64
import json
import os
from pathlib import Path
import re
import stat
import time

RECEIPT = '[COLLECTOR RECEIPT] '


def emit_receipt(status):
    """Called after a recorder receipt is published, never before its barrier.

    Existing redirected recorder stdout is the journal: no extra fsync, file,
    thread, network, or speech engine in the acquisition process.
    """
    payload = {key: status.get(key) for key in (
        'event', 'success', 'request_id', 'wall_time', 'episode_index', 'frames',
        'episode_invalid', 'publication_uncertain', 'subtasks', 'task_outcome')}
    try:
        print(RECEIPT + json.dumps(payload, ensure_ascii=True), flush=True)
    except OSError:
        pass  # A detached speech/log reader must not undo a committed episode.


def receipt_text(value):
    if not isinstance(value, dict):
        return None
    event = value.get('event')
    names = {'start': '开始录制', 'mark_subtask': '子任务标记', 'save': '保存',
             'save_queued': '后台保存', 'discard': '丢弃', 'seal': '封口', 'ready': '初始化'}
    if not isinstance(event, str) or event not in names:
        return None
    if value.get('publication_uncertain'):
        return names[event] + '结果未确认，请检查采集终端'
    if value.get('success') is not True:
        return names[event] + '未完成，请检查采集终端'
    if value.get('episode_invalid') and event in ('start', 'mark_subtask', 'save', 'save_queued'):
        return '本条数据无效，请检查采集终端'
    progress = value.get('subtasks') or {}
    outcome = value.get('task_outcome') or {}
    if not isinstance(progress, dict) or not isinstance(outcome, dict):
        return '采集回执格式异常，请检查采集终端'
    if event == 'start':
        return '开始录制'
    if event == 'mark_subtask':
        done, total = progress.get('confirmed'), progress.get('total')
        if type(done) is int and type(total) is int and 0 < done <= total:
            return f'第{done}步已标记' + ('，全部标记完成' if done == total else '，下一步')
        return '子任务已标记'
    if event == 'save_queued':
        return ('后台保存中，标注未完成，可以开始下一条'
                if progress.get('enabled') and not progress.get('complete')
                else '后台保存中，可以开始下一条')
    if event == 'save':
        if type(value.get('frames')) is not int or value['frames'] <= 0:
            return None
        index = value.get('episode_index')
        text = f'第{index + 1}条已保存' if type(index) is int and index >= 0 else '已保存'
        if progress.get('enabled') and not progress.get('complete'):
            text += '，标注未完成'
        labels = {'autonomous_success': '模型自主成功', 'human_corrected_success': '人工纠错成功',
                  'failure': '任务失败数据已保留'}
        label = outcome.get('outcome')
        if isinstance(label, str) and label in labels:
            text += '，' + labels[label]
        return text
    return {'discard': '已丢弃', 'seal': '本批已封口，可以继续采集', 'ready': '采集工具已就绪'}[event]


class LogSpeech:
    def __init__(self, mode):
        self.mode = mode

    def parse(self, line):
        if self.mode == 'vr':
            # Consume only confirmed speech messages, not words in prompts/logs.
            match = re.fullmatch(r'\[VR speech(?: disabled)?\] (.+)', line.strip())
            return match.group(1) if match else None
        if line.startswith(RECEIPT):
            try:
                value = json.loads(line[len(RECEIPT):])
            except ValueError:
                return None
            return receipt_text(value)
        match = re.search(r'\[SAVED BACKGROUND\] Episode (\d+) \|', line)
        if match:
            return f'第{int(match.group(1)) + 1}条已保存'
        if 'EPISODE INVALID:' in line:
            return '录制异常，请停止本条操作并检查采集终端'
        if 'background save failed; raw data retained:' in line:
            return '后台保存失败，暂存数据保留，请检查采集终端'
        if 'command receipt unavailable (' in line:
            return '操作结果未确认，请检查采集终端'
        return None


class LogTail:
    """Follow byte offsets, preserving repeated lines and partial UTF-8 writes."""
    def __init__(self, path):
        self.path = Path(path)
        self.handle = os.fdopen(os.open(self.path, os.O_RDONLY | os.O_NONBLOCK), 'rb')
        info = os.fstat(self.handle.fileno())
        if not stat.S_ISREG(info.st_mode):
            self.handle.close()
            raise RuntimeError('speech source must be a regular log file')
        self.identity = (info.st_dev, info.st_ino)
        self.handle.seek(0, os.SEEK_END)
        self.pending = b''
        self.offset = self.handle.tell()
        # If attaching during a write, skip its incomplete historical line.
        if self.offset:
            self.handle.seek(-1, os.SEEK_END)
            self.skip_partial = self.handle.read(1) != b'\n'
        else:
            self.skip_partial = False

    def read(self):
        info = self.path.stat()
        if (info.st_dev, info.st_ino) != self.identity or info.st_size < self.handle.tell():
            raise RuntimeError('log changed or was truncated; reconnect to the intended session')
        self.pending += self.handle.read(65536)
        if len(self.pending) > 1048576:
            raise RuntimeError('oversized log line; stopping speech only')
        lines = self.pending.split(b'\n')
        self.pending = lines.pop()
        result = []
        for raw in lines:
            self.offset += len(raw) + 1
            if self.skip_partial:
                self.skip_partial = False
                continue
            result.append((self.offset, raw.rstrip(b'\r').decode('utf-8', errors='replace')))
        return result

    def close(self):
        self.handle.close()


def process_args(pid, proc_root=Path('/proc')):
    try:
        return (proc_root / str(pid) / 'cmdline').read_bytes().decode().strip('\0').split('\0')
    except (OSError, UnicodeError):
        return []


def find_session(base_dir=None, proc_root=Path('/proc')):
    """Require one live recorder owned by this user; never choose latest data."""
    candidates = []
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            args = process_args(int(entry.name), proc_root)
            if not any(Path(arg).name == 'record_lerobot_official.py' for arg in args):
                continue
            status_path = Path(args[args.index('--status-file') + 1])
            directory = status_path.parent.resolve()
            if base_dir is None or directory == Path(base_dir).resolve():
                candidates.append((int(entry.name), directory, args))
        except (OSError, ValueError, IndexError):
            continue
    if len(candidates) != 1:
        raise RuntimeError(f'found {len(candidates)} live collection sessions; start collection first, or specify --base-dir')
    pid, directory, args = candidates[0]
    entries = []
    for line in (directory / '.official_recording_pids').read_text().splitlines():
        fields = line.split(maxsplit=2)
        if len(fields) == 3 and fields[1].isdigit():
            entries.append((fields[0], int(fields[1]), Path(fields[2])))
    recorder = [path for role, number, path in entries if role == 'recorder' and number == pid]
    if len(recorder) != 1:
        raise RuntimeError('recorder PID file does not match the live process')
    vr = []
    for role, number, path in entries:
        command = process_args(number, proc_root)
        if role == 'vr_control' and any(Path(arg).name == 'vr_collector_control.py' for arg in command):
            try:
                if Path(command[command.index('--base-dir') + 1]).resolve() == directory:
                    vr.append((number, path, command))
            except (ValueError, IndexError):
                pass
    if len(vr) > 1:
        raise RuntimeError('multiple VR helpers; select one collector only')
    if vr:
        number, path, command = vr[0]
        return directory, path, 'vr', ((pid, args), (number, command))
    return directory, recorder[0], 'recorder', ((pid, args),)


def packet(kind, **fields):
    print(json.dumps(dict(kind=kind, **fields), ensure_ascii=True), flush=True)


def stream(base_dir=None, duration=None, *, emit=packet, stop=None, required_mode=None):
    """Follow one verified session, never replay history or switch on reconnect.

    seq covers every packet in this connection (including heartbeats). id is
    the source log byte offset, so equal phrases remain separate speech events.
    session_id binds the source mode, recorder PID and opened log identity.
    """
    directory, path, mode, processes = find_session(base_dir)
    if required_mode is not None and mode != required_mode:
        raise RuntimeError('waiting for the selected VR log source')
    reader = LogTail(path)
    policy = LogSpeech(mode)
    identity = f'{reader.identity[0]}:{reader.identity[1]}'
    session_id = f'{mode}:{processes[0][0]}:{identity}'
    sequence = 0

    def notify(kind, **fields):
        nonlocal sequence
        sequence += 1
        emit(kind, version=1, seq=sequence, session_id=session_id, **fields)

    started = heartbeat = time.monotonic()
    stopped_at = None
    try:
        notify('connected', base_dir=str(directory), source=str(path), mode=mode)
        while stop is None or not stop.is_set():
            for offset, line in reader.read():
                text = policy.parse(line)
                if text:
                    notify('speech', id=f'{identity}:{offset}', text=text)
            now = time.monotonic()
            if duration is not None and now - started >= duration:
                notify('ended', reason='read_only_probe_finished')
                return
            if now - heartbeat >= 2:
                notify('heartbeat')
                heartbeat = now
            alive = all(process_args(pid) == args for pid, args in processes)
            if not alive:
                stopped_at = stopped_at or now
                if now - stopped_at >= .5:
                    notify('ended', reason='collector_or_vr_helper_exited',
                           text='采集状态来源已退出，请以采集终端的最终结果为准')
                    return
            if stop is None:
                time.sleep(.1)
            else:
                stop.wait(.1)
    finally:
        reader.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--base-dir', type=Path)
    group.add_argument('--base-dir-b64', help='UTF-8 path encoded by the Windows companion, never shell code')
    parser.add_argument('--duration', type=float, help='Bounded read-only connection probe, seconds')
    args = parser.parse_args()
    if args.duration is not None and not 0 < args.duration <= 3600:
        parser.error('--duration must be in (0, 3600]')
    try:
        base_dir = args.base_dir
        if args.base_dir_b64:
            base_dir = Path(base64.b64decode(args.base_dir_b64, validate=True).decode('utf-8'))
        if base_dir is not None and not base_dir.is_absolute():
            raise ValueError('base directory must be absolute')
        stream(base_dir, args.duration)
    except KeyboardInterrupt:
        return 0
    except BrokenPipeError:
        return 0
    except (OSError, ValueError, RuntimeError) as exc:
        packet('error', text=str(exc))
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
