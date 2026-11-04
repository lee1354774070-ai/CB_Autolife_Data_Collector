#!/usr/bin/env python3
"""Collection-owned read-only web speech page; speech runs in the PC browser."""
import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import hmac
import json
import os
from pathlib import Path
import secrets
import signal
import socket
import subprocess
import threading
import time
from urllib.parse import parse_qs, urlsplit

from collector_pc_speech import stream

ASSETS = Path(__file__).with_name('web') / 'collector_speech'


class Feed:
    def __init__(self, limit=512):
        self.condition = threading.Condition()
        self.events = deque(maxlen=limit)
        self.sequence = 0
        self.info = {}
        self.closed = False

    def emit(self, kind, **fields):
        if kind == 'heartbeat':
            return
        with self.condition:
            self.sequence += 1
            item = dict(kind=kind, seq=self.sequence, **fields)
            self.events.append(item)
            if kind == 'connected':
                self.info = fields
            if kind in ('ended', 'error'):
                self.closed = True
            self.condition.notify_all()

    def snapshot(self):
        with self.condition:
            return dict(seq=self.sequence, closed=self.closed, **self.info)

    def after(self, cursor):
        with self.condition:
            if cursor > self.sequence or (self.events and cursor < self.events[0]['seq'] - 1):
                raise ValueError('播报连接落后或已更换，请核对采集终端后重新开启；不补播过期提示')
            return [event for event in self.events if event['seq'] > cursor]


class SpeechServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, feed, token):
        self.feed, self.token = feed, token
        self.stopping = threading.Event()
        super().__init__(address, Handler)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass  # Session access tokens must never enter request logs.

    def reply(self, code, body, mime):
        self.send_response(code)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Referrer-Policy', 'no-referrer')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        route = urlsplit(self.path)
        if route.path in ('/', '/app.js', '/style.css'):
            name, mime = {'/': ('index.html', 'text/html; charset=utf-8'),
                          '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
                          '/style.css': ('style.css', 'text/css; charset=utf-8')}[route.path]
            self.reply(200, (ASSETS/name).read_bytes(), mime)
            return
        query = parse_qs(route.query)
        token = query.get('token', [''])[0]
        if not hmac.compare_digest(token.encode('utf-8'), self.server.token.encode('utf-8')):
            self.reply(403, b'Forbidden', 'text/plain')
            return
        if route.path == '/state':
            self.reply(200, json.dumps(self.server.feed.snapshot()).encode(), 'application/json')
        elif route.path == '/events':
            try:
                cursor = int(query.get('after', ['-1'])[0])
                if cursor < 0:
                    raise ValueError('invalid cursor')
                self.server.feed.after(cursor)
            except ValueError:
                self.reply(409, b'Reconnect after checking the collection terminal', 'text/plain')
                return
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Referrer-Policy', 'no-referrer')
            self.end_headers()
            self.connection.settimeout(5)
            try:
                while not self.server.stopping.is_set():
                    try:
                        events = self.server.feed.after(cursor)
                    except ValueError as exc:
                        events = [dict(kind='error', text=str(exc), seq=cursor)]
                    for event in events:
                        self.wfile.write(('data: ' + json.dumps(event) + '\n\n').encode())
                        self.wfile.flush()
                        cursor = event['seq']
                        if event['kind'] in ('ended', 'error'):
                            return
                    if self.server.feed.closed:
                        return
                    self.wfile.write(b': heartbeat\n\n')
                    self.wfile.flush()
                    with self.server.feed.condition:
                        if not self.server.feed.after(cursor):
                            self.server.feed.condition.wait(2)
            except (OSError, TimeoutError):
                pass
        else:
            self.reply(404, b'Not found', 'text/plain')


def advertised_hosts():
    hosts = []
    ssh = os.environ.get('SSH_CONNECTION', '').split()
    if len(ssh) == 4:
        hosts.append(ssh[2])
    try:
        hosts.extend(subprocess.check_output(['hostname', '-I'], text=True, timeout=2).split())
    except (OSError, subprocess.SubprocessError):
        try:
            hosts.append(socket.gethostbyname(socket.gethostname()))
        except OSError:
            pass
    return list(dict.fromkeys(host for host in hosts if ':' not in host and not host.startswith('127.')))


def run(base_dir, ready_file, *, vr_control=False, bind='0.0.0.0', port=0):
    feed, token = Feed(), secrets.token_urlsafe(24)
    server = SpeechServer((bind, port), feed, token)
    stop = threading.Event()
    def monitor():
        deadline = time.monotonic() + 5
        while not stop.is_set():
            try:
                stream(base_dir, emit=feed.emit, stop=stop, required_mode='vr' if vr_control else None)
                return
            except (OSError, ValueError, RuntimeError) as exc:
                if not feed.snapshot().get('source') and time.monotonic() < deadline:
                    stop.wait(.1)
                    continue
                feed.emit('error', text='播报状态源中断：' + str(exc))
                return
    worker = threading.Thread(target=monitor, daemon=True)
    http = threading.Thread(target=server.serve_forever, daemon=True)
    ready_file = Path(ready_file)
    temporary = ready_file.with_name(ready_file.name + '.tmp')
    worker.start()
    http.start()
    try:
        info = dict(port=server.server_port, pid=os.getpid(),
                    urls=[f'http://{host}:{server.server_port}/#{token}' for host in advertised_hosts()])
        with temporary.open('w') as handle:
            os.chmod(temporary, 0o600)
            json.dump(info, handle)
        os.replace(temporary, ready_file)
        closed_at = None
        while not stop.wait(.2):
            if feed.closed:
                closed_at = closed_at or time.monotonic()
                if time.monotonic() - closed_at >= 3:
                    break
    finally:
        stop.set()
        server.stopping.set()
        with feed.condition:
            feed.condition.notify_all()
        server.shutdown()
        server.server_close()
        worker.join(timeout=1)
        ready_file.unlink(missing_ok=True)
        temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base-dir', type=Path, required=True)
    parser.add_argument('--ready-file', type=Path, required=True)
    parser.add_argument('--vr-control', choices=('0', '1'), default='0')
    args = parser.parse_args()
    # Bash background children may inherit SIGINT ignored.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        run(args.base_dir, args.ready_file, vr_control=args.vr_control == '1')
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError, RuntimeError) as exc:
        print('电脑播报页启动失败：' + str(exc), flush=True)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
