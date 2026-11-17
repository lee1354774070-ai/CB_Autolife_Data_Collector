"""DAgger-owned power lease. EOF/heartbeat loss restores pre-session settings.

No network reconnect, power-mode change, motor command or boot-time service.
The machine opts in through DAGGER_SESSION_POWER=1; checks and reset-only skip it.
"""
from __future__ import annotations
import fcntl
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import threading

def command(*args):
    return subprocess.run(args, text=True, capture_output=True, check=True,
                          timeout=20).stdout.strip()


class Settings:
    def __init__(self, root):
        self.root = root
        self.journal = root / 'before.json'
        self.clocks = root / 'jetson_clocks.before'
        self.inhibitor = None

    def capture(self):
        devices = command('nmcli', '-t', '-f', 'DEVICE,TYPE,STATE', 'device').splitlines()
        wifi = {}
        for line in devices:
            interface, kind, state = line.split(':', 2)
            if kind == 'wifi' and state.startswith('connected'):
                value = command('iw', 'dev', interface, 'get', 'power_save').split()[-1]
                if value not in ('on', 'off'):
                    raise RuntimeError('Cannot read Wi-Fi power setting')
                wifi[interface] = value
        clocks = Path('/usr/bin/jetson_clocks').exists()
        if clocks:
            # jetson_clocks prompts before overwriting; this file belongs to us.
            self.clocks.unlink(missing_ok=True)
            command('sudo', '-n', '/usr/bin/jetson_clocks', '--store', str(self.clocks))
        return {'version': 1, 'wifi': wifi, 'desktop': {}, 'clocks': clocks}

    def enable(self, before):
        # A pipe-owned inhibitor disappears even if this worker is killed.
        # No persistent desktop power preference is changed.
        self.inhibitor = subprocess.Popen([
            'sudo', '-n', 'systemd-inhibit', '--what=idle:sleep',
            '--who=AutoLife-DAgger', '--why=DAgger-session', '--mode=block',
            '/usr/bin/python3', '-u', '-c',
            "import sys; print('ready', flush=True); sys.stdin.buffer.read()"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        if (not select.select([self.inhibitor.stdout], [], [], 10)[0]
                or self.inhibitor.stdout.readline().strip() != 'ready'):
            raise RuntimeError('Temporary sleep inhibitor not ready')
        for interface in before['wifi']:
            command('sudo', '-n', 'iw', 'dev', interface, 'set', 'power_save', 'off')
        if before['clocks']:
            command('sudo', '-n', '/usr/bin/jetson_clocks')

    def restore(self, before):
        errors = []
        calls = []
        if before['clocks']:
            calls.append(('sudo', '-n', '/usr/bin/jetson_clocks', '--restore', str(self.clocks)))
        calls += [('gsettings', 'set', schema, key, value)
                  for schema, keys in before['desktop'].items() for key, value in keys.items()]
        calls += [('sudo', '-n', 'iw', 'dev', interface, 'set', 'power_save', value)
                  for interface, value in before['wifi'].items()]
        for args in calls:
            try:
                command(*args)
            except Exception as error:
                errors.append(str(error))
        if self.inhibitor is not None:
            self.inhibitor.stdin.close()
            try:
                self.inhibitor.wait(timeout=10)
            except subprocess.TimeoutExpired:
                errors.append('Sleep inhibitor did not exit')
            self.inhibitor = None
        if errors:
            raise RuntimeError('; '.join(errors))


def hold(stream, settings, *, timeout=25, ready=None):
    """An exclusive local lease; keep the original journal on restoration errors."""
    root = settings.root
    root.mkdir(parents=True, exist_ok=True, mode=0o700)
    with (root / 'owner.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if settings.journal.exists():
            before = json.loads(settings.journal.read_text())
        else:
            before = settings.capture()
            temporary = root / 'before.tmp'
            temporary.write_text(json.dumps(before))
            temporary.replace(settings.journal)
        try:
            settings.enable(before)
            if ready:
                ready()
            # Raw reads avoid TextIO buffering multiple heartbeats past select().
            while select.select([stream], [], [], timeout)[0]:
                if not os.read(stream.fileno(), 1024):
                    break
        finally:
            settings.restore(before)
            settings.journal.unlink()


def worker():
    bus = Path(f'/run/user/{os.getuid()}/bus')
    if bus.exists():
        os.environ.setdefault('DBUS_SESSION_BUS_ADDRESS', 'unix:path=' + str(bus))
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, lambda *_: (_ for _ in ()).throw(InterruptedError('Power lease ended')))
    root = Path.home() / '.local/state/autolife-dagger-power'
    try:
        hold(sys.stdin, Settings(root), ready=lambda: print('{"ready":true}', flush=True))
    except Exception as error:
        print('POWER: ' + str(error), file=sys.stderr, flush=True)
        return 1
    print('POWER: restored pre-DAgger settings', file=sys.stderr, flush=True)
    return 0


class PowerSession:
    def __init__(self, env):
        self.env = env
        self.children = []
        self.stop = threading.Event()
        self.thread = None

    def start(self):
        if self.env.get('DAGGER_SESSION_POWER', '0') != '1' or self.env.get('DAGGER_PUBLISH') != '1':
            return
        # This deployment uses the existing restricted 340 key, never a shell key.
        if (self.env.get('ROBOT_ID'), self.env.get('DAGGER_SERVER_URL', '').rstrip('/')) != ('340', 'http://192.168.8.224:8783'):
            raise ValueError('Session power is configured only for 340 and its Thor 8783 service')
        local = ['/usr/bin/python3', '-B', str(Path(__file__).resolve()), '--worker']
        remote = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'IdentitiesOnly=yes',
                  '-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=8',
                  '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=3',
                  '-i', str(Path.home() / '.ssh/id_ed25519_collector_s3_checkpoint'),
                  'wayne@192.168.8.224', 'dagger-power-session']
        self.thread = threading.Thread(target=self._heartbeat, daemon=True)
        self.thread.start()
        try:
            for cmd in (local, remote):
                child = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                         text=True, start_new_session=True, env=self.env)
                self.children.append(child)
                if (not select.select([child.stdout], [], [], 35)[0]
                        or json.loads(child.stdout.readline()).get('ready') is not True):
                    raise RuntimeError('DAgger power lease not ready')
        except BaseException:
            self.close()
            raise
        print('INFO  DAgger期间关闭省电，退出后恢复原设置（340 / Thor）', flush=True)

    def _heartbeat(self):
        while not self.stop.wait(5):
            for child in self.children:
                try:
                    child.stdin.write('.\n')
                    child.stdin.flush()
                except (OSError, ValueError):
                    print('WARN  电源会话连接丢失；该主机将恢复原设置，请检查网络', flush=True)
                    return

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=7)
        for child in self.children:
            try:
                child.stdin.close()
            except OSError:
                pass
        for child in self.children:
            try:
                code = child.wait(timeout=45)
                if code:
                    print('WARN  电源恢复未确认，请查看POWER日志', flush=True)
            except subprocess.TimeoutExpired:
                # Do not kill a helper in the middle of restoring clocks/settings.
                print('WARN  电源恢复仍在进行，请查看POWER日志', flush=True)
        self.children.clear()


if __name__ == '__main__':
    if sys.argv[1:] != ['--worker']:
        raise SystemExit('Only --worker is supported')
    raise SystemExit(worker())
