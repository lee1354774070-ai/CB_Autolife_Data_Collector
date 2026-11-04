"""Read-only browser voice transport, no ROS or hardware."""
import http.client
import json
import signal
import sys
import time
from pathlib import Path
import subprocess
import tempfile
import threading
import unittest
from unittest.mock import patch
from collector_speech_web import Feed, SpeechServer, advertised_hosts


class WebSpeechTest(unittest.TestCase):
    def setUp(self):
        self.feed = Feed()
        self.server = SpeechServer(('127.0.0.1', 0), self.feed, 'test-session-key')
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.stopping.set()
        with self.feed.condition: self.feed.condition.notify_all()
        self.server.shutdown(); self.server.server_close(); self.thread.join(timeout=2)

    def request(self, path, method='GET'):
        conn = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=2)
        conn.request(method, path); reply = conn.getresponse()
        result = reply.status, dict(reply.getheaders()), reply.read()
        conn.close(); return result

    def test_assets_have_no_third_party_dependencies_or_session_data(self):
        self.feed.emit('connected', base_dir='/private/task-data')
        for path in ('/', '/app.js', '/style.css'):
            code, headers, body = self.request(path)
            self.assertEqual(code, 200)
            self.assertNotIn(b'/private/task-data', body)
            self.assertIn("frame-ancestors 'none'", headers['Content-Security-Policy'])
            self.assertEqual(headers['Cache-Control'], 'no-store')
        self.assertEqual(self.request('/../../AGENTS.md?token=test-session-key')[0], 404)

    def test_auth_and_read_only_routes(self):
        self.assertEqual(self.request('/state')[0], 403)
        self.assertEqual(self.request('/state?token=wrong')[0], 403)
        self.assertEqual(self.request('/state?token=%E4%B8%AD')[0], 403)
        self.assertEqual(self.request('/state?token=test-session-key', 'POST')[0], 501)
        self.feed.emit('connected', base_dir='/task', mode='vr')
        code, _, body = self.request('/state?token=test-session-key')
        self.assertEqual(code, 200); self.assertEqual(json.loads(body)['base_dir'], '/task')

    def test_sse_repeated_text_unique_sequence_and_saved_order(self):
        self.feed.emit('connected', base_dir='/task')
        for _ in range(8): self.feed.emit('speech', text='下一步')
        for i in (39,40): self.feed.emit('speech', text=f'第{i}条已保存')
        self.feed.emit('ended', text='来源退出')
        conn = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=2)
        conn.request('GET','/events?token=test-session-key&after=1')
        reply = conn.getresponse(); self.assertEqual(reply.status,200)
        events = [json.loads(line[6:]) for line in reply.read().decode().splitlines() if line.startswith('data: ')]
        conn.close()
        self.assertEqual([v['seq'] for v in events],list(range(2,13)))
        self.assertEqual([v['text'] for v in events[:8]],['下一步']*8)
        self.assertEqual([v['text'] for v in events[8:10]],['第39条已保存','第40条已保存'])

    def test_overflow_bad_cursor_never_silently_replays_old_messages(self):
        self.server.feed = Feed(limit=2)
        for _ in range(4): self.server.feed.emit('speech',text='下一步')
        for cursor in (-1, 0, 100, 'x'):
            self.assertEqual(self.request(f'/events?token=test-session-key&after={cursor}')[0],409)
        self.assertEqual(self.server.feed.after(4),[])

    def test_heartbeat_does_not_fill_event_buffer(self):
        self.feed.emit('connected',base_dir='/task')
        for _ in range(1000): self.feed.emit('heartbeat')
        self.assertEqual(self.feed.snapshot()['seq'],1)

    def test_address_prefers_actual_ssh_server_address(self):
        with patch.dict('os.environ', SSH_CONNECTION='192.168.8.2 1234 192.168.8.59 22'), \
             patch('collector_speech_web.subprocess.check_output',return_value='192.168.225.169 192.168.8.59 fd::2'):
            self.assertEqual(advertised_hosts(),['192.168.8.59','192.168.225.169'])

    def test_process_sigterm_removes_ready_file_and_closes_port(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as tmp:
            ready = Path(tmp)/'speech.json'
            script = """import sys
import collector_speech_web as web
def fake_stream(base_dir, *, emit, stop, required_mode):
    emit('connected',base_dir=str(base_dir),source='synthetic-test',mode='vr')
    stop.wait()
web.stream=fake_stream
web.advertised_hosts=lambda: ['127.0.0.1']
sys.argv=['voice','--base-dir',sys.argv[1],'--ready-file',sys.argv[2]]
raise SystemExit(web.main())
"""
            process = subprocess.Popen([sys.executable,'-B','-c',script,tmp,str(ready)],cwd=root,
                                       stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            try:
                deadline=time.monotonic()+4
                while not ready.exists() and process.poll() is None and time.monotonic()<deadline:
                    time.sleep(.02)
                self.assertTrue(ready.exists())
                info=json.loads(ready.read_text())
                conn=http.client.HTTPConnection('127.0.0.1',info['port'],timeout=1)
                conn.request('GET','/'); self.assertEqual(conn.getresponse().status,200); conn.close()
                process.send_signal(signal.SIGTERM)
                out,err=process.communicate(timeout=4)
                self.assertEqual(process.returncode,0,out+err)
                self.assertFalse(ready.exists())
                conn=http.client.HTTPConnection('127.0.0.1',info['port'],timeout=1)
                with self.assertRaises(OSError): conn.connect()
                conn.close()
            finally:
                if process.poll() is None: process.kill(); process.communicate()

    def test_launcher_voice_failure_is_nonfatal_and_has_no_control_actions(self):
        root = Path(__file__).resolve().parents[1]
        code = (root/'start_lerobot_official_collect.sh').read_text()
        function = code[code.index('start_pc_speech_page() {'):code.index('cleanup_session() {')]
        with tempfile.TemporaryDirectory() as tmp:
            args = ['bash','-c','''set -eu
SCRIPT_DIR=$1; BASE_DIR=$1; LOG_PREFIX=$1/test; PIDFILE=$1/pids; VR_CONTROL=0
''' + function + '''
start_pc_speech_page
printf 'collection-can-continue'
''','voice-test',tmp]
            result = subprocess.run(args,text=True,capture_output=True,timeout=4)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertIn('collection-can-continue',result.stdout)
            self.assertIn('WARNING:',result.stderr)
            self.assertTrue((Path(tmp)/'pids').read_text().startswith('pc_speech '))


if __name__ == '__main__': unittest.main()
