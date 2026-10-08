"""Single-port HTTPS/WebSocket bridge for browser WebXR controller packets."""

import asyncio
import json
from pathlib import Path
import queue
import socket
import ssl
import subprocess
import threading
import time

from aiohttp import WSMsgType, web
from ament_index_python.packages import get_package_share_directory
try:
    from aiortc import (
        RTCPeerConnection,
        RTCRtpSender,
        RTCSessionDescription,
        VideoStreamTrack,
    )
    from aioice import ice as aioice_ice
    from av import VideoFrame
except ImportError:  # WebSocket remains a safe fallback during migration.
    RTCPeerConnection = None
    RTCRtpSender = None
    RTCSessionDescription = None
    VideoStreamTrack = None
    VideoFrame = None
    aioice_ice = None
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .qos import latest_sample_qos
from .teleop_core import vr_input_is_fresh
from .camera_stream import RgbdColorCamera


if VideoStreamTrack is not None:
    class LatestRgbdVideoTrack(VideoStreamTrack):
        """30 FPS latest-frame-only RGB-D colour video source."""

        def __init__(self, camera):
            super().__init__()
            self._camera = camera
            self._frame_id = -1

        async def recv(self):
            loop = asyncio.get_running_loop()
            bgr = None
            frame_id = self._frame_id
            while bgr is None:
                bgr, frame_id, _ = await loop.run_in_executor(
                    None, self._camera.raw_snapshot, self._frame_id, 1.0
                )
                if bgr is not None:
                    break
                # The camera producer can take about one second to publish its
                # first frame after service startup.  Do not terminate the RTP
                # track during that normal warm-up window.
                await asyncio.sleep(1.0 / 30.0)
            self._frame_id = int(frame_id)
            pts, time_base = await self.next_timestamp()
            frame = VideoFrame.from_ndarray(bgr, format='bgr24')
            frame.pts = pts
            frame.time_base = time_base
            return frame
else:
    LatestRgbdVideoTrack = None


def vr_enable_precondition_error(teleop_status, maximum_age):
    """Describe why live WebXR controller tracking is not ready to arm."""
    if not isinstance(teleop_status, dict):
        return 'VR 状态不可用，真机保持关闭'
    age = teleop_status.get('vr_age')
    tracked_hands = teleop_status.get('tracked_hands', [])
    if (
        teleop_status.get('vr_input_fresh') is not True
        or not isinstance(tracked_hands, list)
        or not tracked_hands
        or not vr_input_is_fresh(age, maximum_age)
    ):
        if age is None:
            age_text = '尚未收到手柄姿态'
        else:
            try:
                age_text = f'最近手柄姿态距今 {float(age):.2f} 秒'
            except (TypeError, ValueError):
                age_text = '手柄姿态时间无效'
        return (
            f'VR 手柄数据未就绪（{age_text}）。请进入 VR 并唤醒至少一个手柄，'
            '检测到实时姿态后才会使能真机'
        )
    return ''


class VrWebBridge(Node):
    def __init__(self):
        super().__init__('independent_vr_web_bridge_306_v4')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('https_port', 8443)
        self.declare_parameter(
            'certificate_directory',
            '~/.ros/openarmx_teleop_vr_306_v4',
        )
        self.declare_parameter(
            'input_topic', '/openarmx_teleop_vr_306_v4/vr_input'
        )
        self.declare_parameter(
            'status_topic', '/openarmx_teleop_vr_306_v4/teleop_status'
        )
        self.declare_parameter(
            'hardware_enable_service',
            '/openarmx_teleop_vr_306_v4/set_hardware_enabled',
        )
        self.declare_parameter(
            'waist_follow_service',
            '/openarmx_teleop_vr_306_v4/set_waist_follow_enabled',
        )
        self.declare_parameter(
            'head_follow_service',
            '/openarmx_teleop_vr_306_v4/set_head_follow_enabled',
        )
        self.declare_parameter(
            'desktop_mode_service',
            '/openarmx_teleop_vr_306_v4/set_desktop_mode',
        )
        self.declare_parameter('hardware_request_timeout', 3.0)
        self.declare_parameter('hardware_enable_completion_timeout', 65.0)
        self.declare_parameter('teleop_status_timeout', 1.0)
        self.declare_parameter('vr_input_fresh_timeout', 0.80)
        self.declare_parameter('websocket_max_message_bytes', 65536)
        # A sleeping headset drops its browser socket. Keep the already-enabled
        # controller in safe position hold; only the explicit UI disable action
        # or terminal shutdown ends the hardware session.
        self.declare_parameter('disable_hardware_on_last_disconnect', False)
        self.declare_parameter('rgbd_camera_enabled', True)
        self.declare_parameter('rgbd_camera_maximum_fps', 30.0)
        self.declare_parameter('rgbd_camera_jpeg_quality', 68)

        self._publisher = self.create_publisher(
            String,
            str(self.get_parameter('input_topic').value),
            latest_sample_qos(),
        )
        self._hardware_client = self.create_client(
            SetBool, str(self.get_parameter('hardware_enable_service').value)
        )
        self._waist_follow_client = self.create_client(
            SetBool, str(self.get_parameter('waist_follow_service').value)
        )
        self._head_follow_client = self.create_client(
            SetBool, str(self.get_parameter('head_follow_service').value)
        )
        self._desktop_mode_client = self.create_client(
            SetBool, str(self.get_parameter('desktop_mode_service').value)
        )
        self.create_subscription(
            String,
            str(self.get_parameter('status_topic').value),
            self._on_teleop_status,
            10,
        )
        # Pose packets are state samples rather than an event log.  Keeping only
        # the newest packet prevents an overloaded browser or network burst from
        # making the robot replay stale controller motion later.
        self._packets = queue.Queue(maxsize=1)
        self._stop_requested = threading.Event()
        self._connected_clients = 0
        self._control_peer = None
        self._control_client_id = None
        self._control_websocket = None
        self._control_session = 0
        self._control_session_counter = 0
        self._client_lock = threading.Lock()
        self._status_lock = threading.Lock()
        self._latest_teleop_status = None
        self._latest_teleop_status_time = 0.0
        self._camera_client_status = None
        self._camera_client_status_time = 0.0
        self._webrtc_peer = None
        self._webrtc_channel = None
        self._realtime_transport = 'websocket'
        self._transport_packet_counts = {'websocket': 0, 'webrtc': 0}
        self._transport_drop_counts = {'websocket': 0, 'webrtc': 0}
        self._last_realtime_packet = 0.0
        self._last_webrtc_sequence = -1
        self._last_realtime_sequence = -1
        self._last_webrtc_ack_time = 0.0
        self._webrtc_generation = 0
        self._webrtc_offer_lock = None
        self._webrtc_local_address = None
        self._camera_webrtc_peer = None
        self._camera_webrtc_offer_lock = None
        self._camera_webrtc_local_address = None
        self._server_error = None
        self._rgbd_camera = None
        if bool(self.get_parameter('rgbd_camera_enabled').value):
            self._rgbd_camera = RgbdColorCamera(
                maximum_fps=float(
                    self.get_parameter('rgbd_camera_maximum_fps').value
                ),
                jpeg_quality=int(
                    self.get_parameter('rgbd_camera_jpeg_quality').value
                ),
            )
            self._rgbd_camera.start()
        self._web_dir = (
            Path(
                get_package_share_directory(
                    'openarmx_teleop_vr_306_v4'
                )
            )
            / 'web'
        ).resolve()
        self._ssl_context = self._make_ssl_context()
        self._server_thread = threading.Thread(
            target=self._run_server,
            name='independent-vr-single-port-web',
            daemon=True,
        )
        self._server_thread.start()
        self.create_timer(1.0 / 120.0, self._drain_packets)
        self.create_timer(0.20, self._check_server_health)
        self.get_logger().info(
            f'VR网页与WebSocket已共用：https://<306机器人IP>:'
            f'{self.get_parameter("https_port").value}'
        )

    def _make_ssl_context(self):
        certificate_directory = Path(
            str(self.get_parameter('certificate_directory').value)
        ).expanduser()
        certificate_directory.mkdir(parents=True, exist_ok=True)
        certificate = certificate_directory / 'cert.pem'
        private_key = certificate_directory / 'key.pem'
        if not certificate.exists() or not private_key.exists():
            command = [
                'openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes',
                '-days', '3650', '-subj', '/CN=autolife-independent-robot-306-v2',
                '-keyout', str(private_key), '-out', str(certificate),
            ]
            subprocess.run(
                command, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(certfile=certificate, keyfile=private_key)
        return context

    async def _api_config(self, request):
        del request
        return web.json_response(
            {
                'network': {
                    'https_port': int(self.get_parameter('https_port').value),
                    'websocket_path': '/ws',
                    'webrtc_enabled': RTCPeerConnection is not None,
                    'realtime_offer_path': '/api/realtime/offer',
                },
                'vr': {
                    'controller_axes': {'enabled': True},
                    'input_fresh_timeout_seconds': float(
                        self.get_parameter('vr_input_fresh_timeout').value
                    ),
                    'desktop_mode': {
                        'enabled': True,
                        'toggle_gesture': 'both_thumbsticks',
                        'hold_seconds': 1.0,
                    },
                },
                'vr_images': {
                    'enabled': self._rgbd_camera is not None,
                    'opacity': 1.0,
                    'default_mode': 'passthrough',
                    'toggle_button': 'B',
                    'cameras': ([{
                        'id': 'rgbd_head_color',
                        'name': '头部深度相机 RGB',
                        'width': 640,
                        'height': 480,
                        'fps': int(
                            self.get_parameter(
                                'rgbd_camera_maximum_fps'
                            ).value
                        ),
                        # Keep video independent from controller transport.
                        # Control recovery must never tear down live video.
                        'transport': 'webrtc',
                        'frame_url': (
                            '/api/camera/frame.jpg?camera=rgbd_head_color'
                        ),
                        'mjpeg_url': (
                            '/api/camera/stream.mjpg?camera=rgbd_head_color'
                        ),
                        'status_url': (
                            '/api/camera/status?camera=rgbd_head_color'
                        ),
                    }] if self._rgbd_camera is not None else []),
                },
            }
        )

    async def _api_status(self, request):
        del request
        teleop_status, teleop_status_age, teleop_status_fresh = (
            self._teleop_status_snapshot()
        )
        with self._client_lock:
            vr_connected = self._connected_clients > 0
            packet_age = (
                None if self._last_realtime_packet <= 0.0
                else round(time.monotonic() - self._last_realtime_packet, 4)
            )
            transport = self._realtime_transport
            packet_counts = dict(self._transport_packet_counts)
            webrtc_ready = (
                self._webrtc_channel is not None
                and self._webrtc_channel.readyState == 'open'
            )
            xr_pose_fresh = bool(
                packet_age is not None
                and packet_age <= float(
                    self.get_parameter('vr_input_fresh_timeout').value
                )
            )
            camera_client_status = self._camera_client_status
            camera_client_age = (
                None if self._camera_client_status_time <= 0.0
                else round(
                    time.monotonic() - self._camera_client_status_time, 3
                )
            )
        return web.json_response(
            {
                'status': 'running',
                'robot': '306',
                'system': 'openarmx_teleop_vr_306_v4',
                'vrConnected': vr_connected,
                # A connected page is not proof that an immersive WebXR session
                # is still producing head/controller poses (for example after
                # the headset sleeps). Keep both states explicit in the UI.
                'xrPoseFresh': xr_pose_fresh,
                'websocket': 'same-origin:/ws',
                'realtimeTransport': transport,
                'webrtcReady': webrtc_ready,
                'realtimePacketAge': packet_age,
                'realtimePacketCounts': packet_counts,
                'realtimeDropCounts': dict(self._transport_drop_counts),
                'webrtcLocalAddress': self._webrtc_local_address,
                'cameraWebrtcLocalAddress': (
                    self._camera_webrtc_local_address
                ),
                'cameraWebrtcReady': bool(
                    self._camera_webrtc_peer is not None
                    and self._camera_webrtc_peer.connectionState == 'connected'
                ),
                'hardwareControlAvailable': self._hardware_client.service_is_ready(),
                'waistFollowControlAvailable': (
                    self._waist_follow_client.service_is_ready()
                ),
                'headFollowControlAvailable': (
                    self._head_follow_client.service_is_ready()
                ),
                'desktopModeControlAvailable': (
                    self._desktop_mode_client.service_is_ready()
                ),
                'teleopStatusFresh': teleop_status_fresh,
                'teleopStatusAge': teleop_status_age,
                'teleop': teleop_status,
                'rgbdCamera': (
                    None if self._rgbd_camera is None
                    else self._rgbd_camera.status()
                ),
                'rgbdClient': camera_client_status,
                'rgbdClientAge': camera_client_age,
            }
        )

    @staticmethod
    def _requested_camera_id(request):
        return str(request.query.get('camera', 'rgbd_head_color'))

    async def _api_camera_status(self, request):
        camera_id = self._requested_camera_id(request)
        if camera_id != 'rgbd_head_color' or self._rgbd_camera is None:
            raise web.HTTPNotFound(text='camera is unavailable')
        return web.json_response(self._rgbd_camera.status())

    async def _api_camera_client_status(self, request):
        try:
            payload = await request.json()
        except Exception as exc:
            raise web.HTTPBadRequest(text=f'invalid camera status: {exc}')
        if not isinstance(payload, dict):
            raise web.HTTPBadRequest(text='camera status must be an object')
        request_peer = request.remote or 'unknown'
        client_id = str(payload.get('client_id', '')).strip()
        with self._client_lock:
            authorized = bool(
                self._connected_clients == 1
                and self._control_peer == request_peer
                and self._control_client_id == client_id
            )
            if authorized:
                self._camera_client_status = payload
                self._camera_client_status_time = time.monotonic()
        if not authorized:
            return web.json_response(
                {'error': 'camera status is not from the active VR page'},
                status=409,
            )
        return web.json_response({'ok': True})

    async def _api_camera_stream(self, request):
        camera_id = self._requested_camera_id(request)
        if camera_id != 'rgbd_head_color' or self._rgbd_camera is None:
            raise web.HTTPNotFound(text='camera is unavailable')
        response = web.StreamResponse(
            status=200,
            headers={
                'Content-Type': (
                    'multipart/x-mixed-replace; boundary=rgbdframe'
                ),
                'Cache-Control': 'no-store, no-cache, must-revalidate',
                'Pragma': 'no-cache',
                'X-Content-Type-Options': 'nosniff',
            },
        )
        await response.prepare(request)
        frame_id = -1
        loop = asyncio.get_running_loop()
        try:
            while not self._stop_requested.is_set():
                jpeg, next_id, _ = await loop.run_in_executor(
                    None, self._rgbd_camera.snapshot, frame_id, 1.0
                )
                if jpeg is None or next_id <= frame_id:
                    continue
                frame_id = next_id
                header = (
                    b'--rgbdframe\r\nContent-Type: image/jpeg\r\n'
                    + f'Content-Length: {len(jpeg)}\r\n'.encode('ascii')
                    + f'X-Frame-Id: {frame_id}\r\n\r\n'.encode('ascii')
                )
                await response.write(header + jpeg + b'\r\n')
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return response

    async def _api_camera_frame(self, request):
        camera_id = self._requested_camera_id(request)
        if camera_id != 'rgbd_head_color' or self._rgbd_camera is None:
            raise web.HTTPNotFound(text='camera is unavailable')
        try:
            after_frame_id = int(request.query.get('after', '-1'))
        except (TypeError, ValueError):
            raise web.HTTPBadRequest(text='after must be an integer')
        loop = asyncio.get_running_loop()
        jpeg, frame_id, _ = await loop.run_in_executor(
            None, self._rgbd_camera.snapshot, after_frame_id, 1.0
        )
        headers = {
            'Cache-Control': 'no-store, no-cache, must-revalidate',
            'Pragma': 'no-cache',
            'X-Content-Type-Options': 'nosniff',
            'X-Frame-Id': str(frame_id),
        }
        if jpeg is None or frame_id <= after_frame_id:
            return web.Response(status=204, headers=headers)
        return web.Response(
            body=jpeg, content_type='image/jpeg', headers=headers
        )

    async def _close_camera_webrtc_peer(self, expected_peer=None):
        peer = self._camera_webrtc_peer
        if expected_peer is not None and peer is not expected_peer:
            return False
        self._camera_webrtc_peer = None
        if peer is not None:
            await peer.close()
        return peer is not None

    async def _api_camera_webrtc_offer(self, request):
        """Negotiate an independent low-latency camera video connection."""
        if (
                RTCPeerConnection is None
                or LatestRgbdVideoTrack is None
                or self._rgbd_camera is None):
            return web.json_response(
                {'error': 'camera WebRTC is unavailable'},
                status=503,
            )
        try:
            payload = await request.json()
            camera_id = str(payload.get('camera_id', 'rgbd_head_color'))
            if camera_id != 'rgbd_head_color':
                raise ValueError(f'unknown camera: {camera_id}')
            offer = RTCSessionDescription(
                sdp=str(payload['sdp']), type=str(payload['type'])
            )
        except Exception as exc:
            raise web.HTTPBadRequest(text=f'invalid camera WebRTC offer: {exc}')

        if self._camera_webrtc_offer_lock is None:
            self._camera_webrtc_offer_lock = asyncio.Lock()
        async with self._camera_webrtc_offer_lock:
            await self._close_camera_webrtc_peer()
            route_address = self._route_address_for_peer(request.remote)
            self._camera_webrtc_local_address = route_address
            if aioice_ice is not None and route_address:
                aioice_ice.get_host_addresses = (
                    lambda use_ipv4, use_ipv6: (
                        [route_address] if use_ipv4 else []
                    )
                )
            peer = RTCPeerConnection()
            self._camera_webrtc_peer = peer
            sender = peer.addTrack(LatestRgbdVideoTrack(self._rgbd_camera))

            # VP8 is the stable continuous aiortc/Quest path on this robot.
            # The local H.264 encoder previously stopped after about 171
            # frames, so keep it only as a negotiated fallback.
            try:
                codecs = RTCRtpSender.getCapabilities('video').codecs
                preferred = [
                    codec for codec in codecs
                    if str(codec.mimeType).lower() == 'video/vp8'
                ]
                fallback = [
                    codec for codec in codecs
                    if str(codec.mimeType).lower() != 'video/vp8'
                ]
                transceiver = next(
                    item for item in peer.getTransceivers()
                    if item.sender is sender
                )
                if preferred:
                    transceiver.setCodecPreferences(preferred + fallback)
            except Exception as exc:
                self.get_logger().warning(
                    f'相机H.264优先设置失败，将使用浏览器协商编码：{exc}'
                )

            @peer.on('connectionstatechange')
            async def on_connectionstatechange():
                if peer.connectionState in ('failed', 'closed'):
                    if self._camera_webrtc_peer is peer:
                        self._camera_webrtc_peer = None
                    await peer.close()

            try:
                await peer.setRemoteDescription(offer)
                answer = await peer.createAnswer()
                await peer.setLocalDescription(answer)
            except Exception:
                await peer.close()
                if self._camera_webrtc_peer is peer:
                    self._camera_webrtc_peer = None
                raise
            if self._camera_webrtc_peer is not peer:
                await peer.close()
                return web.json_response(
                    {'error': 'camera WebRTC negotiation was superseded'},
                    status=409,
                )
            return web.json_response({
                'sdp': peer.localDescription.sdp,
                'type': peer.localDescription.type,
                'camera_id': camera_id,
            })

    async def _request_waist_follow(self, enabled):
        if not self._waist_follow_client.service_is_ready():
            return False, 'waist-follow service is unavailable'
        ros_request = SetBool.Request()
        ros_request.data = bool(enabled)
        future = self._waist_follow_client.call_async(ros_request)
        timeout = max(
            0.5, float(self.get_parameter('hardware_request_timeout').value)
        )
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout
        while not future.done() and loop.time() < deadline:
            await asyncio.sleep(0.02)
        if not future.done():
            future.cancel()
            return False, 'waist-follow request timed out'
        try:
            result = future.result()
        except Exception as exc:
            return False, str(exc)
        if result is None:
            return False, 'waist-follow service returned no response'
        return bool(result.success), str(result.message)

    async def _api_waist_follow(self, request):
        try:
            payload = await request.json()
        except (json.JSONDecodeError, TypeError):
            raise web.HTTPBadRequest(text='JSON body required')
        enabled = payload.get('enabled') if isinstance(payload, dict) else None
        if not isinstance(enabled, bool):
            raise web.HTTPBadRequest(text='enabled must be true or false')
        success, message = await self._request_waist_follow(enabled)
        return web.json_response(
            {'success': success, 'message': message},
            status=200 if success else 409,
        )

    async def _request_head_follow(self, enabled):
        if not self._head_follow_client.service_is_ready():
            return False, 'head-follow service is unavailable'
        ros_request = SetBool.Request()
        ros_request.data = bool(enabled)
        future = self._head_follow_client.call_async(ros_request)
        timeout = max(
            0.5, float(self.get_parameter('hardware_request_timeout').value)
        )
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout
        while not future.done() and loop.time() < deadline:
            await asyncio.sleep(0.02)
        if not future.done():
            future.cancel()
            return False, 'head-follow request timed out'
        try:
            result = future.result()
        except Exception as exc:
            return False, str(exc)
        if result is None:
            return False, 'head-follow service returned no response'
        return bool(result.success), str(result.message)

    async def _api_head_follow(self, request):
        try:
            payload = await request.json()
        except (json.JSONDecodeError, TypeError):
            raise web.HTTPBadRequest(text='JSON body required')
        enabled = payload.get('enabled') if isinstance(payload, dict) else None
        if not isinstance(enabled, bool):
            raise web.HTTPBadRequest(text='enabled must be true or false')
        success, message = await self._request_head_follow(enabled)
        return web.json_response(
            {'success': success, 'message': message},
            status=200 if success else 409,
        )

    async def _request_desktop_mode(self, enabled):
        if not self._desktop_mode_client.service_is_ready():
            return False, 'desktop-mode service is unavailable'
        ros_request = SetBool.Request()
        ros_request.data = bool(enabled)
        future = self._desktop_mode_client.call_async(ros_request)
        timeout = max(
            0.5, float(self.get_parameter('hardware_request_timeout').value)
        )
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout
        while not future.done() and loop.time() < deadline:
            await asyncio.sleep(0.02)
        if not future.done():
            future.cancel()
            return False, 'desktop-mode request timed out'
        try:
            result = future.result()
        except Exception as exc:
            return False, str(exc)
        if result is None:
            return False, 'desktop-mode service returned no response'
        return bool(result.success), str(result.message)

    async def _api_desktop_mode(self, request):
        try:
            payload = await request.json()
        except (json.JSONDecodeError, TypeError):
            raise web.HTTPBadRequest(text='JSON body required')
        enabled = payload.get('enabled') if isinstance(payload, dict) else None
        if not isinstance(enabled, bool):
            raise web.HTTPBadRequest(text='enabled must be true or false')

        # The mapper transition is first: it immediately drops both clutch
        # anchors and publishes measured-position hold before any IK mode can
        # change.  Desktop mode never permits head/waist assistance.
        success, message = await self._request_desktop_mode(enabled)
        if not success:
            return web.json_response(
                {'success': False, 'message': message}, status=409
            )
        warnings = []
        if enabled:
            for label, operation in (
                ('waist follow', self._request_waist_follow),
                ('head follow', self._request_head_follow),
            ):
                try:
                    follow_success, follow_message = await operation(False)
                except Exception as exc:
                    # Launch shutdown invalidates the rclpy context before the
                    # HTTP request necessarily finishes.  Preserve the safe
                    # mapper hold without leaking an aiohttp traceback.
                    follow_success = False
                    follow_message = f'ROS service context closed: {exc}'
                if not follow_success:
                    warnings.append(f'{label}: {follow_message}')
        if warnings:
            return web.json_response({
                'success': False,
                'mode_changed': True,
                'message': (
                    f'{message}; robot remains held, but assistance shutdown '
                    f'needs attention: {"; ".join(warnings)}'
                ),
            }, status=409)
        return web.json_response({
            'success': True,
            'enabled': enabled,
            'message': message,
        })

    async def _api_hardware_control(self, request):
        try:
            payload = await request.json()
        except (json.JSONDecodeError, TypeError):
            raise web.HTTPBadRequest(text='JSON body required')
        enabled = payload.get('enabled') if isinstance(payload, dict) else None
        if not isinstance(enabled, bool):
            raise web.HTTPBadRequest(text='enabled must be true or false')
        waist_follow_enabled = (
            payload.get('waist_follow_enabled')
            if isinstance(payload, dict)
            else None
        )
        head_follow_enabled = (
            payload.get('head_follow_enabled')
            if isinstance(payload, dict)
            else None
        )
        if waist_follow_enabled is not None and not isinstance(
                waist_follow_enabled, bool):
            raise web.HTTPBadRequest(
                text='waist_follow_enabled must be true or false'
            )
        if head_follow_enabled is not None and not isinstance(
                head_follow_enabled, bool):
            raise web.HTTPBadRequest(
                text='head_follow_enabled must be true or false'
            )
        with self._client_lock:
            vr_connected = self._connected_clients > 0
        if enabled and not vr_connected:
            return web.json_response(
                {
                    'success': False,
                    'message': 'VR data connection is not ready; hardware remains disabled',
                },
                status=409,
            )
        if enabled:
            if waist_follow_enabled is not None:
                success, message = await self._request_waist_follow(
                    waist_follow_enabled
                )
                if not success:
                    return web.json_response(
                        {'success': False, 'message': message}, status=409
                    )
            if head_follow_enabled is not None:
                success, message = await self._request_head_follow(
                    head_follow_enabled
                )
                if not success:
                    return web.json_response(
                        {'success': False, 'message': message}, status=409
                    )
            teleop_status, teleop_status_age, teleop_status_fresh = (
                self._teleop_status_snapshot()
            )
            if not teleop_status_fresh:
                age_text = (
                    'unavailable'
                    if teleop_status_age is None
                    else f'{teleop_status_age:.2f}s old'
                )
                return web.json_response(
                    {
                        'success': False,
                        'message': (
                            'VR mapper status is stale or unavailable '
                            f'({age_text}); hardware remains disabled'
                        ),
                    },
                    status=409,
                )
            precondition_error = vr_enable_precondition_error(
                teleop_status,
                float(self.get_parameter('vr_input_fresh_timeout').value),
            )
            if precondition_error:
                self.get_logger().warning(
                    f'真机遥操使能已拒绝：{precondition_error}'
                )
                return web.json_response(
                    {'success': False, 'message': precondition_error},
                    status=409,
                )
        if not self._hardware_client.service_is_ready():
            return web.json_response(
                {
                    'success': False,
                    'message': 'teleoperation hardware-enable service is unavailable',
                },
                status=503,
            )

        ros_request = SetBool.Request()
        ros_request.data = enabled
        future = self._hardware_client.call_async(ros_request)
        timeout = max(
            0.5, float(self.get_parameter('hardware_request_timeout').value)
        )
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout
        while not future.done() and loop.time() < deadline:
            await asyncio.sleep(0.02)
        if not future.done():
            future.cancel()
            return web.json_response(
                {'success': False, 'message': 'hardware-enable request timed out'},
                status=504,
            )
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Web hardware-control request failed: {exc}')
            return web.json_response(
                {'success': False, 'message': str(exc)}, status=500
            )
        if result.success and enabled:
            ready, message = await self._wait_for_hardware_ready(
                str(result.message)
            )
            if not ready:
                self.get_logger().error(f'真机遥操使能失败：{message}')
                return web.json_response(
                    {'success': False, 'message': message}, status=409
                )
            self.get_logger().info(f'真机遥操使能完成：{message}')
            return web.json_response(
                {'success': True, 'message': message}, status=200
            )
        if not result.success:
            self.get_logger().warning(
                f'真机遥操切换请求被控制器拒绝：{result.message}'
            )
        return web.json_response(
            {'success': bool(result.success), 'message': str(result.message)},
            status=200 if result.success else 409,
        )

    async def _wait_for_hardware_ready(self, initial_message):
        """Wait until the staged controller enable either completes or fails."""
        loop = asyncio.get_running_loop()
        started = loop.time()
        timeout = max(
            1.0,
            float(
                self.get_parameter('hardware_enable_completion_timeout').value
            ),
        )
        deadline = started + timeout
        saw_pending = False
        last_reason = initial_message or '正在等待真机安全使能完成'
        while loop.time() < deadline:
            status, status_age, fresh = self._teleop_status_snapshot()
            # A previous failed attempt may still be cached when the service
            # accepts this attempt. Only inspect status received afterwards.
            if (fresh and status_age is not None
                    and status_age <= loop.time() - started
                    and isinstance(status, dict)):
                backend = status.get('backend')
                if not isinstance(backend, dict):
                    backend = status
                enabled = bool(status.get(
                    'hardware_enabled', backend.get('hardware_enabled', False)
                ))
                ready = bool(status.get(
                    'hardware_ready', backend.get('hardware_ready', False)
                ))
                pending = bool(status.get(
                    'hardware_enable_pending',
                    backend.get('hardware_enable_pending', False),
                ))
                state = str(backend.get('state', status.get('state', ''))).upper()
                quick_reset = status.get('quick_reset')
                if not isinstance(quick_reset, dict):
                    quick_reset = {}
                resetting = bool(
                    quick_reset.get('active', False)
                    or backend.get('quick_reset_active', False)
                    or state in ('RESETTING', 'RESETTING_ARMS')
                )
                last_reason = str(
                    backend.get('reason')
                    or backend.get('detail')
                    or status.get('detail')
                    or last_reason
                )
                if enabled and ready:
                    return True, '真机遥操已安全使能'
                if pending or resetting:
                    saw_pending = True
                elif state in ('FAULT', 'E_STOP'):
                    if saw_pending or loop.time() - started >= 0.75:
                        return False, last_reason
                elif saw_pending:
                    return False, last_reason
                elif loop.time() - started >= 0.75 and state == 'DISARMED':
                    return False, last_reason
            await asyncio.sleep(0.05)
        return False, f'等待真机使能完成超时：{last_reason}'

    def _on_teleop_status(self, message):
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(payload, dict):
            return
        with self._status_lock:
            self._latest_teleop_status = payload
            self._latest_teleop_status_time = time.monotonic()

    def _teleop_status_snapshot(self):
        with self._status_lock:
            status = (
                None
                if self._latest_teleop_status is None
                else dict(self._latest_teleop_status)
            )
            received_at = self._latest_teleop_status_time
        if status is None or received_at <= 0.0:
            return status, None, False
        age = max(0.0, time.monotonic() - received_at)
        timeout = max(
            0.1, float(self.get_parameter('teleop_status_timeout').value)
        )
        return status, round(age, 3), age <= timeout

    def _replace_latest_packet(self, packet):
        try:
            self._packets.get_nowait()
        except queue.Empty:
            pass
        try:
            self._packets.put_nowait(packet)
        except queue.Full:
            # The ROS timer can race this producer between get and put.  In
            # that case the queue already contains an equally new sample.
            pass

    def _accept_realtime_packet(self, raw_packet, transport):
        if isinstance(raw_packet, bytes):
            try:
                raw_packet = raw_packet.decode('utf-8')
            except UnicodeDecodeError:
                return None
        if not isinstance(raw_packet, str):
            return None
        maximum = max(
            1024,
            min(
                int(self.get_parameter('websocket_max_message_bytes').value),
                1024 * 1024,
            ),
        )
        if len(raw_packet.encode('utf-8')) > maximum:
            return None
        try:
            packet = json.loads(raw_packet)
        except json.JSONDecodeError:
            return None
        if not isinstance(packet, dict):
            return None
        if packet.get('type') == 'desktop_keepalive':
            # Keep TCP/NAT state alive without ever treating a timer packet as
            # fresh hand tracking or replacing the newest controller pose.
            return None
        # WebRTC and WebSocket form one latest-frame stream. During transport
        # failover the same sequence can arrive on both; globally reject stale
        # or duplicate samples so fallback can never replay old motion.
        try:
            sequence = int(packet.get('sequence'))
        except (TypeError, ValueError):
            sequence = None
        if sequence is not None:
            with self._client_lock:
                if sequence <= self._last_realtime_sequence:
                    self._transport_drop_counts[transport] += 1
                    return None
                self._last_realtime_sequence = sequence
                if transport == 'webrtc':
                    self._last_webrtc_sequence = sequence
        self._replace_latest_packet(raw_packet)
        with self._client_lock:
            self._realtime_transport = str(transport)
            self._transport_packet_counts[transport] += 1
            self._last_realtime_packet = time.monotonic()
        return sequence

    @staticmethod
    def _route_address_for_peer(peer):
        """Return the IPv4 source address selected by the kernel route."""
        try:
            probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                probe.connect((str(peer), 9))
                return str(probe.getsockname()[0])
            finally:
                probe.close()
        except OSError:
            return None

    async def _close_webrtc_peer(self, expected_peer=None):
        peer = self._webrtc_peer
        if expected_peer is not None and peer is not expected_peer:
            return False
        self._webrtc_peer = None
        self._webrtc_channel = None
        if peer is not None:
            await peer.close()
        return peer is not None

    async def _api_webrtc_offer(self, request):
        if RTCPeerConnection is None:
            return web.json_response(
                {'error': 'aiortc is unavailable; using WebSocket fallback'},
                status=503,
            )
        request_peer = request.remote or 'unknown'
        with self._client_lock:
            authorized = (
                self._connected_clients == 1
                and self._control_peer == request_peer
            )
            owner_session = self._control_session
        if not authorized:
            return web.json_response(
                {'error': 'an active same-origin VR WebSocket is required'},
                status=409,
            )
        try:
            payload = await request.json()
            generation = int(payload['generation'])
            if generation <= 0:
                raise ValueError('generation must be positive')
            offer = RTCSessionDescription(
                sdp=str(payload['sdp']), type=str(payload['type'])
            )
        except Exception as exc:
            raise web.HTTPBadRequest(text=f'invalid WebRTC offer: {exc}')

        # The lock belongs to this aiohttp loop and serializes the complete
        # replace/negotiate/publish transaction.  Without it, two offers can
        # close each other's peer and return an answer for an already dead SCTP
        # association.
        if self._webrtc_offer_lock is None:
            self._webrtc_offer_lock = asyncio.Lock()
        async with self._webrtc_offer_lock:
            return await self._negotiate_webrtc_offer(
                request_peer, owner_session, generation, offer
            )

    def _control_owner_is_current(self, request_peer, owner_session):
        with self._client_lock:
            return bool(
                self._connected_clients == 1
                and self._control_peer == request_peer
                and self._control_session == owner_session
            )

    async def _negotiate_webrtc_offer(
            self, request_peer, owner_session, generation, offer):
        # Re-check ownership after waiting for the negotiation lock.
        if not self._control_owner_is_current(request_peer, owner_session):
            return web.json_response(
                {'error': 'VR WebSocket ownership changed during negotiation'},
                status=409,
            )
        if generation <= self._webrtc_generation:
            return web.json_response(
                {'error': 'stale WebRTC offer generation'}, status=409
            )
        await self._close_webrtc_peer()
        # Closing the previous peer awaits; the controlling WebSocket may have
        # disconnected while that happened. Never create an orphan for a dead
        # ownership session.
        if not self._control_owner_is_current(request_peer, owner_session):
            return web.json_response(
                {'error': 'VR WebSocket disconnected during negotiation'},
                status=409,
            )
        # aioice otherwise advertises every NIC.  On this robot ICE selected
        # 192.168.10.2 even though the headset route is wlo1/192.168.50.138,
        # producing an asymmetric, jittery path.  Advertise only the address
        # the kernel route selects for this headset.
        route_address = self._route_address_for_peer(request_peer)
        self._webrtc_local_address = route_address
        if aioice_ice is not None and route_address:
            aioice_ice.get_host_addresses = (
                lambda use_ipv4, use_ipv6: [route_address] if use_ipv4 else []
            )
        peer = RTCPeerConnection()
        self._webrtc_peer = peer
        self._webrtc_generation = generation

        @peer.on('datachannel')
        def on_datachannel(channel):
            if self._webrtc_peer is not peer:
                channel.close()
                return
            if channel.label != 'teleop':
                channel.close()
                return
            self._webrtc_channel = channel

            @channel.on('open')
            def on_open():
                self.get_logger().info(
                    'WebRTC low-latency teleoperation data channel connected'
                )

            @channel.on('message')
            def on_message(message):
                if (
                    self._webrtc_peer is not peer
                    or self._webrtc_channel is not channel
                    or self._webrtc_generation != generation
                ):
                    return
                accepted_sequence = self._accept_realtime_packet(
                    message, 'webrtc'
                )
                now = time.monotonic()
                if (
                    accepted_sequence is not None
                    and now - self._last_webrtc_ack_time >= 0.10
                    and channel.readyState == 'open'
                ):
                    try:
                        channel.send(json.dumps({
                            'type': 'teleop_ack',
                            'sequence': int(accepted_sequence),
                            'generation': int(generation),
                        }, separators=(',', ':')))
                        self._last_webrtc_ack_time = now
                    except Exception:
                        # The browser watchdog will fall back to WebSocket and
                        # recreate this half-open channel.
                        pass

            @channel.on('close')
            def on_close():
                if self._webrtc_channel is channel:
                    self._webrtc_channel = None
                self.get_logger().warning(
                    'WebRTC teleoperation channel closed; WebSocket fallback remains'
                )

        @peer.on('connectionstatechange')
        async def on_connectionstatechange():
            if peer.connectionState in ('failed', 'closed'):
                if self._webrtc_peer is peer:
                    self._webrtc_peer = None
                    self._webrtc_channel = None
                await peer.close()

        try:
            await peer.setRemoteDescription(offer)
            answer = await peer.createAnswer()
            await peer.setLocalDescription(answer)
        except Exception:
            await peer.close()
            if self._webrtc_peer is peer:
                self._webrtc_peer = None
            raise
        if (
            not self._control_owner_is_current(request_peer, owner_session)
            or self._webrtc_peer is not peer
            or self._webrtc_generation != generation
        ):
            await peer.close()
            if self._webrtc_peer is peer:
                self._webrtc_peer = None
                self._webrtc_channel = None
            return web.json_response(
                {'error': 'WebRTC negotiation was superseded'}, status=409
            )
        return web.json_response({
            'sdp': peer.localDescription.sdp,
            'type': peer.localDescription.type,
            'generation': int(generation),
            'camera_video': False,
        })

    def _clear_packet_buffer(self):
        try:
            self._packets.get_nowait()
        except queue.Empty:
            pass

    def _disable_hardware_after_disconnect(self):
        if not bool(
            self.get_parameter('disable_hardware_on_last_disconnect').value
        ):
            return
        if not self._hardware_client.service_is_ready():
            self.get_logger().warning(
                'VR page disconnected, but hardware-enable service is unavailable'
            )
            return
        request = SetBool.Request()
        request.data = False
        future = self._hardware_client.call_async(request)

        def log_result(completed):
            try:
                result = completed.result()
            except Exception as exc:
                self.get_logger().error(
                    f'Failed to disable hardware after VR disconnect: {exc}'
                )
                return
            if result.success:
                self.get_logger().warning(
                    'Last VR page disconnected; hardware teleoperation was disabled'
                )
            else:
                self.get_logger().error(
                    'Hardware disable after VR disconnect was rejected: '
                    f'{result.message}'
                )

        future.add_done_callback(log_result)

    async def _websocket_handler(self, request):
        maximum_message_bytes = max(
            1024,
            min(
                int(self.get_parameter('websocket_max_message_bytes').value),
                1024 * 1024,
            ),
        )
        websocket = web.WebSocketResponse(
            heartbeat=10.0,
            max_msg_size=maximum_message_bytes,
        )
        peer = request.remote or 'unknown'
        client_id = str(request.query.get('client_id', '')).strip()
        if not client_id or len(client_id) > 128:
            raise web.HTTPBadRequest(text='valid client_id is required')
        replaced_websocket = None
        with self._client_lock:
            if self._connected_clients:
                if (
                        self._control_peer != peer
                        or self._control_client_id != client_id):
                    raise web.HTTPConflict(
                        text='another VR controller is already connected'
                    )
                # A sleeping Quest can leave a half-open socket on the robot.
                # Let the same IP atomically take ownership instead of waiting
                # for the old heartbeat timeout. Different peers remain denied.
                replaced_websocket = self._control_websocket
            # Reserve ownership before the asynchronous WebSocket handshake so
            # two simultaneous requests cannot both become control sources.
            self._connected_clients = 1
            self._control_peer = peer
            self._control_client_id = client_id
            self._control_websocket = websocket
            self._control_session_counter += 1
            owner_session = self._control_session_counter
            self._control_session = owner_session
            self._last_webrtc_sequence = -1
            self._last_realtime_sequence = -1
            self._last_webrtc_ack_time = 0.0
            self._webrtc_generation = 0
            self._camera_client_status = None
            self._camera_client_status_time = 0.0
        try:
            await self._close_webrtc_peer()
            await websocket.prepare(request)
            if replaced_websocket is not None:
                try:
                    await replaced_websocket.close(
                        code=1001,
                        message=(
                            b'replaced by a new session from the same headset'
                        ),
                    )
                except Exception:
                    pass
            self.get_logger().info(f'VR客户端已连接：{peer}')
            async for message in websocket:
                if not self._control_owner_is_current(peer, owner_session):
                    break
                if message.type != WSMsgType.TEXT:
                    continue
                if len(message.data.encode('utf-8')) > maximum_message_bytes:
                    await websocket.close(
                        code=1009,
                        message=b'VR packet exceeds configured size limit',
                    )
                    break
                self._accept_realtime_packet(message.data, 'websocket')
        finally:
            current_owner = False
            with self._client_lock:
                if self._control_session == owner_session:
                    current_owner = True
                    self._connected_clients = 0
                    self._control_peer = None
                    self._control_client_id = None
                    self._control_websocket = None
                    self._control_session = 0
            if current_owner:
                await self._close_webrtc_peer()
                self._clear_packet_buffer()
                with self._client_lock:
                    self._last_webrtc_sequence = -1
                    self._last_realtime_sequence = -1
                    self._last_webrtc_ack_time = 0.0
                    self._webrtc_generation = 0
                    self._camera_client_status = None
                    self._camera_client_status_time = 0.0
                self._disable_hardware_after_disconnect()
            self.get_logger().info(f'VR客户端已断开：{peer}')
        return websocket

    async def _index(self, request):
        del request
        return web.FileResponse(
            self._web_dir / 'index.html',
            headers={'Cache-Control': 'no-store, no-cache, must-revalidate'},
        )

    async def _asset(self, request):
        relative = Path(request.match_info['asset'])
        if relative.is_absolute() or '..' in relative.parts:
            raise web.HTTPForbidden()
        requested = self._web_dir / relative
        if not requested.is_file():
            raise web.HTTPNotFound()
        return web.FileResponse(
            requested,
            headers={'Cache-Control': 'no-store, no-cache, must-revalidate'},
        )

    async def _serve(self):
        application = web.Application(client_max_size=64 * 1024)
        application.router.add_get('/api/config', self._api_config)
        application.router.add_get('/api/status', self._api_status)
        application.router.add_get(
            '/api/camera/status', self._api_camera_status
        )
        application.router.add_post(
            '/api/camera/client-status', self._api_camera_client_status
        )
        application.router.add_get(
            '/api/camera/stream.mjpg', self._api_camera_stream
        )
        application.router.add_get(
            '/api/camera/frame.jpg', self._api_camera_frame
        )
        application.router.add_post(
            '/api/hardware-control', self._api_hardware_control
        )
        application.router.add_post(
            '/api/waist-follow', self._api_waist_follow
        )
        application.router.add_post(
            '/api/head-follow', self._api_head_follow
        )
        application.router.add_post(
            '/api/desktop-mode', self._api_desktop_mode
        )
        application.router.add_post(
            '/api/realtime/offer', self._api_webrtc_offer
        )
        application.router.add_post(
            '/api/camera/webrtc/offer', self._api_camera_webrtc_offer
        )
        application.router.add_get('/ws', self._websocket_handler)
        application.router.add_get('/', self._index)
        application.router.add_get('/{asset:.*}', self._asset)
        runner = web.AppRunner(application, access_log=None)
        await runner.setup()
        site = web.TCPSite(
            runner,
            str(self.get_parameter('host').value),
            int(self.get_parameter('https_port').value),
            ssl_context=self._ssl_context,
        )
        await site.start()
        try:
            while not self._stop_requested.is_set():
                await asyncio.sleep(0.10)
        finally:
            await self._close_camera_webrtc_peer()
            await self._close_webrtc_peer()
            await runner.cleanup()

    def _run_server(self):
        try:
            asyncio.run(self._serve())
        except Exception as exc:
            self._server_error = str(exc)
            self.get_logger().error(f'VR单端口网页服务异常退出：{exc}')

    def _check_server_health(self):
        if self._server_error is None:
            return
        self.get_logger().fatal(
            'VR web bridge cannot continue: ' + self._server_error
        )
        self._server_error = None
        if rclpy.ok():
            rclpy.shutdown()

    def _drain_packets(self):
        try:
            latest = self._packets.get_nowait()
        except queue.Empty:
            return
        self._publisher.publish(String(data=latest))

    def destroy_node(self):
        self._stop_requested.set()
        if hasattr(self, '_server_thread'):
            self._server_thread.join(timeout=3.0)
        if self._rgbd_camera is not None:
            self._rgbd_camera.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VrWebBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
