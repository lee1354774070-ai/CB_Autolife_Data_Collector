"""Real web-only instance; private DDS domain, no controller/model/recorder."""
import asyncio
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import aiohttp

async def probe():
    url='https://127.0.0.1:18448'
    async with aiohttp.ClientSession(connector=aiohttp.TCPConnector(ssl=False)) as client:
        async with client.get(url+'/api/config') as response:
            config=await response.json()
            assert config['vr_images']['enabled'] is False, config
            assert config['vr_images']['cameras']==[], config
        async with client.get(url+'/api/camera/frame.jpg?camera=rgbd_head_color') as response:
            assert response.status==404,response.status
        async with client.ws_connect(url+'/ws?client_id=preview-off-test'):
            async with client.get(url+'/api/status') as response:
                status=await response.json()
                assert status['vrConnected'],status
        print('PASS: preview disabled, no camera frames, control WebSocket still connects')

assert os.environ['ROS_DOMAIN_ID']=='213'
assert os.environ['DAGGER_PUBLISH']=='0'
root=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='preview-off-smoke-') as tmp:
    with (Path(tmp)/'web.log').open('w+') as log:
        proc=subprocess.Popen([os.environ['DAGGER_WEB_PY'],'-B',str(root/'dagger/runtime.py'),'web',
             '--ros-args','-p','https_port:=18448','-p','rgbd_camera_enabled:=false'],stdout=log,stderr=subprocess.STDOUT)
        try:
            deadline=time.monotonic()+20
            while True:
                assert proc.poll() is None, 'web exited'
                try:
                    asyncio.run(probe());break
                except aiohttp.ClientConnectorError:
                    if time.monotonic()>deadline:raise
                    time.sleep(.2)
            maps=Path(f'/proc/{proc.pid}/maps').read_text()
            assert 'camera_image_buffer_' not in maps, 'preview still maps camera memory'
            print('PASS: web process has no camera SHM mappings')
        finally:
            if proc.poll() is None:proc.send_signal(signal.SIGINT)
            proc.wait(timeout=15)
            log.seek(0)
            print(log.read()[-700:])
