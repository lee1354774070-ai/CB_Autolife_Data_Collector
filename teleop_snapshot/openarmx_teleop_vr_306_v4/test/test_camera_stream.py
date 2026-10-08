import ast
import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CAMERA = ROOT / 'openarmx_teleop_vr_306_v4' / 'camera_stream.py'
BRIDGE = ROOT / 'openarmx_teleop_vr_306_v4' / 'vr_web_bridge.py'
WEB = ROOT / 'web' / 'vr_app.js'


def test_camera_module_is_importable_without_opening_hardware():
    spec = importlib.util.spec_from_file_location('camera_stream_test', CAMERA)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    camera = module.RgbdColorCamera(maximum_fps=20, jpeg_quality=75)
    status = camera.status()
    assert status['image_key'] == 'rgbd_head_color'
    assert status['online'] is False


def test_bridge_exposes_independent_rgbd_webrtc_route():
    source = BRIDGE.read_text(encoding='utf-8')
    ast.parse(source)
    assert "'/api/camera/status'" in source
    assert "'/api/camera/stream.mjpg'" in source
    assert "'/api/camera/frame.jpg'" in source
    assert "'/api/camera/webrtc/offer'" in source
    assert 'LatestRgbdVideoTrack' in source
    assert "peer.addTrack(LatestRgbdVideoTrack(self._rgbd_camera))" in source
    assert 'RgbdColorCamera' in source
    assert "'default_mode': 'passthrough'" in source
    assert "'toggle_button': 'B'" in source
    assert "'transport': 'webrtc'" in source
    realtime_offer = source[source.index('    async def _negotiate_webrtc_offer('):]
    realtime_offer = realtime_offer[:realtime_offer.index('    def _clear_packet_buffer(')]
    assert 'LatestRgbdVideoTrack' not in realtime_offer
    assert "'camera_video': False" in realtime_offer
    camera_offer = source[source.index('    async def _api_camera_webrtc_offer('):]
    camera_offer = camera_offer[:camera_offer.index('    async def _request_waist_follow(')]
    assert "str(codec.mimeType).lower() == 'video/vp8'" in camera_offer
    assert 'self._route_address_for_peer(request.remote)' in camera_offer


def test_web_uses_b_rising_edge_and_defaults_to_passthrough():
    source = WEB.read_text(encoding='utf-8')
    assert "mode: 'passthrough'" in source
    assert 'function toggleCameraMode()' in source
    assert "handEl.addEventListener('bbuttondown'" in source
    assert 'if (!buttons.b) toggleCameraMode();' in source
    assert 'if (this.rightButtons.b && !previousB) toggleCameraMode();' in source
    assert "setCameraMode('passthrough');" in source
    assert 'panel.setAttribute(\'visible\', state.image.mode === \'rgbd\')' in source
    assert 'async function startLatestJpegStream(id)' in source
    assert "cameraState.config.transport === 'latest-jpeg'" in source
    assert "response.headers.get('X-Frame-Id')" in source
    assert "fetch('/api/camera/webrtc/offer'" in source
    assert "fetch('/api/webrtc/status'" not in source
    assert "fetch('/api/webrtc/offer'" not in source
    assert 'Math.min(30, Number(cameraConfig.fps || 30))' in source
    assert "peer.addTransceiver('video', { direction: 'recvonly' })" in source
    assert 'pumpIndependentVideoFrames(cameraState, peer, generation)' in source
    assert 'video.requestVideoFrameCallback(onFrame)' in source
    assert 'startCameraWatchdog()' in source
    assert "scheduleCameraReconnect(cameraState, 'video frames stalled')" in source
    independent = source[source.index('async function startCameraFeed(id)'):]
    independent = independent[:independent.index('function createAxisPart(')]
    assert 'using MJPEG fallback' not in independent
    assert 'scheduleCameraReconnect(cameraState, `video setup failed:' in independent
    assert "canvas.getContext('2d', { alpha: false, desynchronized: true })" in source
    assert 'updateCameraPanel(id);\n    } catch (error)' not in source
    assert "fetch('/api/camera/client-status'" in source
    assert "page: 'v28'" in source
    assert "'/api/camera/client-status'" in BRIDGE.read_text(encoding='utf-8')


def test_camera_uses_single_latest_raw_slot_and_jpeg_is_on_demand():
    source = CAMERA.read_text(encoding='utf-8')
    assert 'self._bgr = bgr' in source
    assert 'self._jpeg_demand_until' in source
    assert 'jpeg_requested = now <= self._jpeg_demand_until' in source
    assert 'self._jpeg_frame_id = int(frame_id)' in source


def test_control_websocket_uses_one_stable_browser_tab_owner():
    bridge = BRIDGE.read_text(encoding='utf-8')
    web = WEB.read_text(encoding='utf-8')
    assert "request.query.get('client_id', '')" in bridge
    assert 'self._control_client_id != client_id' in bridge
    assert "sessionStorage.getItem(storageKey)" in web
    assert "url.searchParams.set('client_id', state.clientInstanceId)" in web
