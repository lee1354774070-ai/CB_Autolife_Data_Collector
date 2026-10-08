"""Adapt original OpenArmX APK ROS topics to the guarded robot-306-v2 mapper."""

import json
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger

from .qos import latest_sample_qos


class OpenArmxUdpInput(Node):
    """Aggregate the bridge's UDP-derived topics without building a backlog."""

    def __init__(self):
        super().__init__('openarmx_udp_input_306_v4')
        self.declare_parameter('output_topic', '/openarmx_teleop_vr_306_v4/vr_input')
        self.declare_parameter('default_control_mode', 'relative')
        self.declare_parameter('grip_threshold', 0.5)
        self.declare_parameter('input_timeout_sec', 0.35)
        self._lock = threading.RLock()
        self._mode = str(self.get_parameter('default_control_mode').value).lower()
        self._poses = {m: {s: None for s in ('left', 'right')}
                       for m in ('relative', 'absolute')}
        self._pose_times = {m: {s: 0.0 for s in ('left', 'right')}
                            for m in ('relative', 'absolute')}
        self._grip = {m: {s: 0.0 for s in ('left', 'right')}
                      for m in ('relative', 'absolute')}
        self._trigger = {m: {s: 0.0 for s in ('left', 'right')}
                         for m in ('relative', 'absolute')}
        self._rate = {'relative': 1.0, 'absolute': 1.0}
        self._buttons = {'A': False, 'B': False, 'X': False, 'Y': False}
        self._publisher = self.create_publisher(
            String, str(self.get_parameter('output_topic').value), latest_sample_qos())
        self._reset_client = self.create_client(
            Trigger, '/openarmx_teleop_vr_306_v4/quick_reset')
        self._subscribe_inputs()
        self.create_timer(0.05, self._watchdog)
        self.get_logger().info(
            'OpenArmX APK/UDP adapter ready (relative and absolute streams)')

    def _subscribe_inputs(self):
        qos = latest_sample_qos()
        for mode in ('relative', 'absolute'):
            for side in ('left', 'right'):
                if mode == 'relative':
                    base = f'pico_{side}_controller'
                    pose_topic = f'{base}/pose'
                else:
                    base = f'vr/{side}'
                    pose_topic = f'{base}/pose_absolute'
                self.create_subscription(
                    PoseStamped, pose_topic,
                    lambda msg, m=mode, s=side: self._on_pose(m, s, msg), qos)
                self.create_subscription(
                    Float32, f'{base}/trigger',
                    lambda msg, m=mode, s=side: self._on_analog(
                        self._trigger, m, s, msg.data), qos)
                self.create_subscription(
                    Float32, f'{base}/grip',
                    lambda msg, m=mode, s=side: self._on_analog(
                        self._grip, m, s, msg.data), qos)
        self.create_subscription(
            Float32, 'pico_left_controller/rate',
            lambda msg: self._on_rate('relative', msg.data), qos)
        self.create_subscription(
            Float32, 'vr/rate',
            lambda msg: self._on_rate('absolute', msg.data), qos)
        self.create_subscription(String, 'vr/control_mode', self._on_mode, qos)
        self.create_subscription(Bool, 'vr/calibrate_done', self._on_calibrate, qos)
        topics = {
            'relative': {
                'A': 'pico_right_controller/button_a',
                'B': 'pico_right_controller/button_b',
                'X': 'pico_left_controller/button_x',
                'Y': 'pico_left_controller/button_y'},
            'absolute': {
                'A': 'vr/right/button_a', 'B': 'vr/right/button_b',
                'X': 'vr/left/button_x', 'Y': 'vr/left/button_y'},
        }
        for mode, values in topics.items():
            for button, topic in values.items():
                self.create_subscription(
                    Bool, topic,
                    lambda msg, m=mode, b=button: self._on_button(
                        m, b, msg.data), qos)

    @staticmethod
    def _pose_dict(message):
        p = message.pose.position
        q = message.pose.orientation
        return {
            'position': {'x': p.x, 'y': p.y, 'z': p.z},
            'quaternion': {'x': q.x, 'y': q.y, 'z': q.z, 'w': q.w}}

    def _on_analog(self, target, mode, side, value):
        with self._lock:
            target[mode][side] = max(0.0, min(1.0, float(value)))

    def _on_rate(self, mode, value):
        with self._lock:
            self._rate[mode] = 0.1 if float(value) <= 0.1 else 1.0

    def _on_mode(self, message):
        mode = str(message.data).strip().lower()
        if mode not in ('relative', 'absolute'):
            return
        with self._lock:
            if mode != self._mode:
                self._publish_release_locked()
                self._mode = mode
                self.get_logger().info(f'APK control mode changed to {mode}')

    def _on_calibrate(self, message):
        if message.data:
            with self._lock:
                self._publish_release_locked()
            self.get_logger().info('APK calibration accepted; clutch anchors cleared')

    def _on_button(self, mode, button, pressed):
        with self._lock:
            if mode == self._mode:
                rising = bool(pressed) and not self._buttons[button]
                self._buttons[button] = bool(pressed)
                self._publish_locked()
                if button == 'B' and rising:
                    self._request_home_reset()

    def _request_home_reset(self):
        """Preserve the upstream B-button home action via the guarded backend."""
        if not self._reset_client.service_is_ready():
            self.get_logger().warning('B/home ignored: guarded reset service is not ready')
            return
        future = self._reset_client.call_async(Trigger.Request())

        def completed(done):
            try:
                result = done.result()
                log = self.get_logger().info if result.success else self.get_logger().warning
                log(f'B/home result: {result.message}')
            except Exception as exc:
                self.get_logger().error(f'B/home request failed: {exc}')

        future.add_done_callback(completed)

    def _on_pose(self, mode, side, message):
        with self._lock:
            self._poses[mode][side] = self._pose_dict(message)
            self._pose_times[mode][side] = time.monotonic()
            if mode == self._mode:
                self._publish_locked()

    def _controller(self, mode, side, now):
        pose = self._poses[mode][side]
        timeout = float(self.get_parameter('input_timeout_sec').value)
        if pose is None or now - self._pose_times[mode][side] > timeout:
            return None
        result = dict(pose)
        result.update({
            'gripActive': self._grip[mode][side] >= float(
                self.get_parameter('grip_threshold').value),
            'trigger': self._trigger[mode][side],
            'rate': self._rate[mode],
            'aButton': int(self._buttons['A']),
            'bButton': int(self._buttons['B']),
            'xButton': int(self._buttons['X']),
            'yButton': int(self._buttons['Y'])})
        return result

    def _publish_locked(self):
        now = time.monotonic()
        payload = {
            'type': 'openarmx_udp_frame', 'source': 'openarmx_apk_udp',
            'mode': self._mode, 'timestamp': now}
        for side in ('left', 'right'):
            controller = self._controller(self._mode, side, now)
            if controller is not None:
                payload[f'{side}Controller'] = controller
        if 'leftController' in payload or 'rightController' in payload:
            self._publisher.publish(String(
                data=json.dumps(payload, separators=(',', ':'))))

    def _publish_release_locked(self):
        for side in ('left', 'right'):
            self._publisher.publish(String(data=json.dumps({
                'type': 'grip_release', 'hand': side, 'gripReleased': True,
                'source': 'openarmx_apk_udp'}, separators=(',', ':'))))

    def _watchdog(self):
        with self._lock:
            now = time.monotonic()
            timeout = float(self.get_parameter('input_timeout_sec').value)
            for side in ('left', 'right'):
                stamp = self._pose_times[self._mode][side]
                if stamp > 0.0 and now - stamp > timeout:
                    self._pose_times[self._mode][side] = 0.0
                    self._publisher.publish(String(data=json.dumps({
                        'type': 'grip_release', 'hand': side,
                        'gripReleased': True,
                        'source': 'openarmx_apk_udp_timeout'},
                        separators=(',', ':'))))


def main(args=None):
    rclpy.init(args=args)
    node = OpenArmxUdpInput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
