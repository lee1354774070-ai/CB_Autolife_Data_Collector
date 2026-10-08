"""OpenArmX APK intent mapper for the independent 306 V4 arm controller.

This node never publishes a vendor hardware command topic.  It converts fresh
APK/UDP controller samples into Cartesian/gripper intents, publishes a liveness
heartbeat, and requests guarded reset through the independent controller.
"""

import json
import math
import threading
import time
import uuid

import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from .gripper_aperture import checked_width, load_width, save_width, width_to_motor
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .desktop_mode import DesktopModeGate, world_to_operator_yaw_rotation
from .qos import latest_sample_qos
from .reset_state import GripRearmState, QuickResetState, ClutchedTrigger
from .teleop_core import (
    ControllerSample,
    HeadMapper,
    HeadSample,
    grip_release_state,
    ik_rejected_sides,
    linear_gripper_position,
    matrix_to_quaternion,
    parse_eef_feedback,
    quaternion_to_matrix,
    vendor_pose_payload,
    webxr_yaw_deg_from_quaternion,
    vr_input_is_fresh,
    vr_tracking_state,
)
from .quest_incremental import QuestIncrementalArmMapper


def eef_target_payload(
        targets, reanchor_sides=None, *, clutch_session=None,
        clutch_sequence=None, active_arms=None):
    """Build a latest-frame target with an authoritative clutch state.

    The old protocol used a one-shot ``release_hold`` message.  Losing that
    single transition allowed the controller to keep chasing its previous IK
    target after Grip had been released.  Every frame now carries the complete
    left/right clutch state, a process-unique session and a monotonic sequence.
    """
    payload = {}
    for side, pose in targets.items():
        payload[f'pos_{side}_in_robot'] = [float(v) for v in pose['position']]
        payload[f'quat_{side}_in_robot'] = [float(v) for v in pose['orientation']]
    if reanchor_sides:
        payload['reanchor_sides'] = [str(side) for side in reanchor_sides]
    if clutch_session is not None:
        payload['clutch_session'] = str(clutch_session)
        payload['clutch_sequence'] = int(clutch_sequence)
        active = set(active_arms or [])
        payload['clutch_state'] = {
            side: side in active for side in ('left', 'right')
        }
    return payload


class IndependentVrMapper(Node):
    def __init__(self):
        super().__init__('independent_vr_mapper_306_v4')
        self._declare_parameters()
        self.declare_parameter('gripper_closed_width_cm', load_width())
        self.add_on_set_parameters_callback(self._validate_gripper_width)
        self._lock = threading.RLock()
        self._dry_run = bool(self.get_parameter('dry_run').value)
        self._current_poses = {}
        self._last_eef_time = 0.0
        self._last_vr_time = 0.0
        self._samples = {}
        self._sample_times = {}
        self._sample_versions = {'left': 0, 'right': 0}
        self._processed_versions = {'left': -1, 'right': -1}
        self._last_targets = {}
        self._grip_rearm = GripRearmState()
        # Keep this compatibility alias because status/tests and older tooling
        # inspect the existing field name.
        self._require_release = self._grip_rearm.waiting
        self._backend_status = {}
        self._hardware_enabled = False
        self._backend_reset_active = False
        self._last_reason = 'waiting for independent controller status'
        self._quick_state = QuickResetState(request_timeout_sec=float(
            self.get_parameter('quick_reset_request_timeout_sec').value
        ))
        self._quick_future = None
        self._hold_reset_state = QuickResetState(
            request_timeout_sec=self._quick_state.request_timeout_sec,
            button_keys=('left_y', 'right_b'),
        )
        self._disable_request_pending = False
        self._gripper_filtered = {'left': None, 'right': None}
        self._gripper_last_sent = {'left': None, 'right': None}
        self._gripper_last_time = {'left': 0.0, 'right': 0.0}
        self._gripper_input_raw = {'left': 0.0, 'right': 0.0}
        self._gripper_input_effective = {'left': 0.0, 'right': 0.0}
        self._gripper_full_close_latched = {'left': False, 'right': False}
        self._gripper_press_time = {'left': None, 'right': None}
        self._gripper_dual_sync_active = False
        self._gripper_clutch = {side: ClutchedTrigger() for side in ('left', 'right')}
        self._release_hold_sent = {'left': False, 'right': False}
        self._grip_false_since = {'left': None, 'right': None}
        self._clutch_session = uuid.uuid4().hex
        self._clutch_sequence = 0
        self._ik_reanchor_pending = {'left': False, 'right': False}
        self._head_sample = None
        self._head_sample_time = 0.0
        self._desktop_gate = DesktopModeGate(
            settle_seconds=self.get_parameter(
                'desktop_mode_settle_seconds').value,
            head_stability_m=self.get_parameter(
                'desktop_mode_head_stability_m').value,
            controller_stability_m=self.get_parameter(
                'desktop_mode_controller_stability_m').value,
            head_stability_deg=self.get_parameter(
                'desktop_mode_head_stability_deg').value,
            head_jump_m=self.get_parameter('desktop_mode_head_jump_m').value,
            head_jump_deg=self.get_parameter(
                'desktop_mode_head_jump_deg').value,
            controller_jump_m=self.get_parameter(
                'desktop_mode_controller_jump_m').value,
        )
        self._desktop_gate.set_enabled(False, self._now())
        self._desktop_entry_yaw_deg = 0.0
        self._desktop_position_rotation = np.eye(3, dtype=float)
        self._wearable_operator_yaw_deg = {'left': 0.0, 'right': 0.0}
        self._wearable_position_rotation = {
            'left': np.eye(3, dtype=float),
            'right': np.eye(3, dtype=float),
        }

        mapper_args = {
            'position_scale': self.get_parameter('position_scale').value,
            'forward_position_scale': self.get_parameter(
                'forward_position_scale'
            ).value,
            'filter_alpha': self.get_parameter('filter_alpha').value,
            'maximum_position_step': self.get_parameter('max_position_step').value,
            'maximum_orientation_step': math.radians(float(
                self.get_parameter('max_orientation_step_deg').value
            )),
            'maximum_anchor_displacement': self.get_parameter(
                'max_anchor_displacement'
            ).value,
            'maximum_forward_displacement': self.get_parameter(
                'max_forward_displacement'
            ).value,
            'wrist_orientation_gain': self.get_parameter(
                'wrist_orientation_gain'
            ).value,
            'wrist_reach_reserve_enabled': self.get_parameter(
                'wrist_reach_reserve_enabled'
            ).value,
            'wrist_reach_reserve_minimum_m': self.get_parameter(
                'wrist_reach_reserve_minimum_m'
            ).value,
            'wrist_reach_reserve_maximum_m': self.get_parameter(
                'wrist_reach_reserve_maximum_m'
            ).value,
            'wrist_reach_reserve_start_angle_deg': self.get_parameter(
                'wrist_reach_reserve_start_angle_deg'
            ).value,
            'wrist_reach_reserve_full_angle_deg': self.get_parameter(
                'wrist_reach_reserve_full_angle_deg'
            ).value,
            'workspace_minimum': self.get_parameter('workspace_minimum').value,
            'workspace_maximum': self.get_parameter('workspace_maximum').value,
            'left_y_minimum': self.get_parameter('left_y_minimum').value,
            'right_y_maximum': self.get_parameter('right_y_maximum').value,
            'forbidden_box_minimum': self.get_parameter(
                'forbidden_box_minimum'
            ).value,
            'forbidden_box_maximum': self.get_parameter(
                'forbidden_box_maximum'
            ).value,
        }
        self._mappers = {
            side: QuestIncrementalArmMapper(side=side, **mapper_args)
            for side in ('left', 'right')
        }
        head_axis_gain = list(self.get_parameter('head_axis_gain').value)
        if not bool(self.get_parameter('head_roll_follow_enabled').value):
            # Keep the neck Roll joint enabled for holding/reset, but remove
            # HMD side-tilt from its live target. Pitch and Yaw are unchanged.
            head_axis_gain[0] = 0.0
        self._head_mapper = HeadMapper(
            filter_alpha=self.get_parameter('head_filter_alpha').value,
            maximum_step_deg=self.get_parameter('head_max_step_deg').value,
            axis_gain=head_axis_gain,
            maximum_offset_deg=self.get_parameter('head_maximum_offset_deg').value,
            joint_minimum_deg=self.get_parameter('head_joint_minimum_deg').value,
            joint_maximum_deg=self.get_parameter('head_joint_maximum_deg').value,
            tracking_mode=self.get_parameter('head_tracking_mode').value,
            neutral_neck_deg=self.get_parameter('head_neutral_joints_deg').value,
            deadband_deg=self.get_parameter('head_deadband_deg').value,
            reference_jump_deg=self.get_parameter(
                'head_reference_jump_deg'
            ).value,
        )

        prefix = '/openarmx_teleop_vr_306_v4'
        self._eef_target_pub = self.create_publisher(
            String, f'{prefix}/eef_target', latest_sample_qos()
        )
        self._gripper_pub = self.create_publisher(
            String, f'{prefix}/gripper_target', latest_sample_qos()
        )
        self._release_pub = self.create_publisher(
            String, f'{prefix}/release_hold', 10
        )
        self._preview_pub = self.create_publisher(
            String, f'{prefix}/vr_preview_target', latest_sample_qos()
        )
        self._joint_target_pub = self.create_publisher(
            String, f'{prefix}/head_target', latest_sample_qos()
        )
        self._status_pub = self.create_publisher(
            String, f'{prefix}/teleop_status', 10
        )
        self._heartbeat_pub = self.create_publisher(
            String, f'{prefix}/teleop_heartbeat', latest_sample_qos()
        )
        self._enable_client = self.create_client(
            SetBool, f'{prefix}/set_hardware_enabled'
        )
        self._reset_client = self.create_client(Trigger, f'{prefix}/quick_reset')
        self._hold_reset_client = self.create_client(Trigger, f'{prefix}/quick_reset_hold_grippers')
        self._desktop_mode_service = self.create_service(
            SetBool, f'{prefix}/set_desktop_mode', self._on_desktop_mode
        )
        self.create_subscription(
            String, f'{prefix}/vr_input', self._on_vr_input,
            latest_sample_qos()
        )
        self.create_subscription(
            String, f'{prefix}/eef_feedback', self._on_eef_feedback,
            latest_sample_qos()
        )
        self.create_subscription(
            String, f'{prefix}/status', self._on_backend_status, 20
        )
        rate = max(10.0, float(self.get_parameter('command_rate').value))
        self.create_timer(1.0 / rate, self._control_tick)
        self.create_timer(0.20, self._publish_status_and_heartbeat)
        self.get_logger().info(
            '306 V4 independent VR mapper ready; no vendor command publishers are created'
        )

    def _declare_parameters(self):
        defaults = {
            'topic_suffix': '0_300',
            'dry_run': True,
            'command_rate': 90.0,
            'vr_timeout': 0.80,
            'hardware_disable_vr_timeout': 5.0,
            'controller_tracking_grace_period': 0.60,
            'grip_release_debounce_sec': 0.06,
            'eef_feedback_timeout': 0.60,
            'disable_on_vr_timeout': False,
            'position_scale': 0.70,
            'forward_position_scale': 0.90,
            'filter_alpha': 0.82,
            'max_position_step': 0.025,
            'max_orientation_step_deg': 8.0,
            'max_anchor_displacement': 0.70,
            'max_forward_displacement': 0.26,
            'wrist_orientation_gain': 1.18,
            'wrist_reach_reserve_enabled': False,
            'wrist_reach_reserve_minimum_m': 0.015,
            'wrist_reach_reserve_maximum_m': 0.070,
            'wrist_reach_reserve_start_angle_deg': 0.0,
            'wrist_reach_reserve_full_angle_deg': 70.0,
            'workspace_minimum': [-0.10, -0.75, 0.25],
            'workspace_maximum': [0.90, 0.75, 1.60],
            'left_y_minimum': -0.22,
            'right_y_maximum': 0.22,
            'forbidden_box_minimum': [-0.10, -0.16, 0.35],
            'forbidden_box_maximum': [0.30, 0.16, 1.25],
            'enable_gripper': True,
            'gripper_open_position': 10.0,
            'gripper_closed_position': 330.0,
            'gripper_trigger_deadzone': 0.05,
            'gripper_left_trigger_scale': 1.0,
            'gripper_right_trigger_scale': 1.0,
            'gripper_full_close_threshold': 0.88,
            'gripper_dual_sync_enabled': True,
            'gripper_dual_sync_activation': 0.15,
            'gripper_dual_sync_window_sec': 0.20,
            'gripper_dual_sync_max_difference': 0.40,
            'gripper_filter_alpha': 0.78,
            'gripper_max_step_per_cycle': 32.0,
            'gripper_command_epsilon': 0.75,
            'gripper_resend_interval': 0.15,
            'quick_reset_enabled': True,
            'quick_reset_hold_seconds': 1.0,
            'quick_reset_input_fresh_sec': 0.30,
            'quick_reset_request_timeout_sec': 2.0,
            # Capture operator-forward explicitly, but map it to calibrated
            # robot neutral rather than inheriting the measured latch offset.
            'head_tracking_mode': 'relative_neutral_quaternion',
            'head_neutral_joints_deg': [0.0, 0.0, 0.0],
            'head_filter_alpha': 0.90,
            'head_max_step_deg': [1.625, 1.625, 1.625],
            'head_axis_gain': [1.25, 1.25, 1.25],
            'head_roll_follow_enabled': True,
            'head_maximum_offset_deg': [18.0, 40.0, 55.0],
            'head_joint_minimum_deg': [-18.0, -40.0, -55.0],
            'head_joint_maximum_deg': [18.0, 25.0, 55.0],
            'head_deadband_deg': [0.20, 0.20, 0.20],
            'head_reference_jump_deg': 35.0,
            'head_input_timeout': 0.60,
            'operator_translation_compensation_enabled': True,
            'operator_yaw_compensation_enabled': True,
            # Desktop dual-arm mode deliberately ignores headset translation.
            # It must see one second of stable HMD and two-controller tracking
            # before a new Grip clutch can be established.
            'desktop_mode_settle_seconds': 1.0,
            'desktop_mode_head_stability_m': 0.015,
            'desktop_mode_controller_stability_m': 0.035,
            'desktop_mode_head_stability_deg': 3.0,
            'desktop_mode_head_jump_m': 0.08,
            'desktop_mode_head_jump_deg': 10.0,
            'desktop_mode_controller_jump_m': 0.18,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    @staticmethod
    def _now():
        return time.monotonic()

    def _publish_release_hold(self, side):
        if not self._hardware_enabled or self._release_hold_sent[side]:
            return
        self._release_pub.publish(String(data=json.dumps(
            {'sides': [side], 'reason': 'vr_grip_released'},
            separators=(',', ':'),
        )))
        self._release_hold_sent[side] = True

    def _release_side(self, side, require_release=True, notify_controller=False):
        if notify_controller:
            self._publish_release_hold(side)
        self._mappers[side].release()
        self._last_targets.pop(side, None)
        self._processed_versions[side] = -1
        self._grip_rearm.set_required(side, require_release)
        self._gripper_clutch[side].release()
        self._gripper_input_raw[side] = 0.0
        if hasattr(self, '_gripper_full_close_latched'):
            self._gripper_full_close_latched[side] = False
        self._gripper_press_time[side] = None
        self._gripper_dual_sync_active = False
        self._grip_false_since[side] = None

    def _release_all(self, require_release=True, notify_controller=False):
        for side in ('left', 'right'):
            self._release_side(side, require_release, notify_controller)

    def _cancel_quick_request_locked(self, reason=None):
        """Invalidate a pending reset request; caller holds ``self._lock``."""
        future = self._quick_future
        self._quick_state.begin_boundary()
        self._hold_reset_state.begin_boundary()
        self._quick_future = None
        if future is not None and not future.done():
            future.cancel()
        if reason:
            self._last_reason = str(reason)

    def _begin_session_boundary_locked(
            self, reason, *, notify_controller=False,
            new_clutch_session=False):
        """Drop all input state that may predate an enable/reset boundary."""
        self._cancel_quick_request_locked()
        self._grip_rearm.begin_boundary()
        self._release_all(
            require_release=True, notify_controller=notify_controller
        )
        self._samples.clear()
        self._sample_times.clear()
        self._sample_versions = {'left': 0, 'right': 0}
        self._processed_versions = {'left': -1, 'right': -1}
        self._last_targets.clear()
        self._ik_reanchor_pending = {'left': False, 'right': False}
        self._grip_false_since = {'left': None, 'right': None}
        self._head_sample = None
        self._head_sample_time = 0.0
        self._head_mapper.release()
        self._wearable_operator_yaw_deg = {'left': 0.0, 'right': 0.0}
        self._wearable_position_rotation = {
            'left': np.eye(3, dtype=float),
            'right': np.eye(3, dtype=float),
        }
        if new_clutch_session:
            self._clutch_session = uuid.uuid4().hex
            self._clutch_sequence = 0
        self._last_reason = str(reason)

    def _on_desktop_mode(self, request, response):
        enabled = bool(request.data)
        with self._lock:
            if self._desktop_gate.enabled == enabled:
                response.success = True
                response.message = (
                    'desktop dual-arm mode is already on'
                    if enabled else 'desktop dual-arm mode is already off'
                )
                return response
            now = self._now()
            entry_yaw_deg = 0.0
            position_rotation = np.eye(3, dtype=float)
            if enabled:
                if (
                    self._head_sample is None
                    or now - self._head_sample_time > float(
                        self.get_parameter('vr_timeout').value
                    )
                ):
                    response.success = False
                    response.message = (
                        'fresh headset orientation is required before desktop mode'
                    )
                    return response
                entry_yaw_deg = float(self._head_sample.rotation[1])
                position_rotation = world_to_operator_yaw_rotation(
                    entry_yaw_deg
                )
            self._begin_session_boundary_locked(
                'desktop mode changed; robot is holding position',
                notify_controller=True,
                new_clutch_session=True,
            )
            self._desktop_entry_yaw_deg = entry_yaw_deg
            self._desktop_position_rotation = position_rotation
            self._desktop_gate.set_enabled(enabled, now)
            response.success = True
            response.message = (
                'desktop dual-arm mode enabled; keep the headset and both '
                'controllers still for one second, then release and press Grip'
                if enabled else
                'wearable mode restored; release and press Grip to re-anchor'
            )
            self._last_reason = response.message
        return response

    def _on_eef_feedback(self, message):
        try:
            poses = parse_eef_feedback(json.loads(message.data))
            if not all(side in poses for side in ('left', 'right')):
                raise ValueError('left or right FK pose is missing')
        except Exception as exc:
            self._last_reason = f'independent FK feedback rejected: {exc}'
            return
        with self._lock:
            self._current_poses = poses
            self._last_eef_time = self._now()

    def _on_backend_status(self, message):
        try:
            status = json.loads(message.data)
            if not isinstance(status, dict):
                return
        except Exception:
            return
        with self._lock:
            previous_enabled = self._hardware_enabled
            previous_reset = self._backend_reset_active
            self._backend_status = status
            self._hardware_enabled = bool(status.get('hardware_enabled', False))
            self._backend_reset_active = bool(status.get('quick_reset_active', False))
            head_follow_enabled = bool(status.get('head_follow_enabled', False))
            rejected_sides = set(ik_rejected_sides(status))
            if self._hardware_enabled != previous_enabled:
                self._begin_session_boundary_locked(
                    'hardware state changed; release both grips before re-clutching',
                    new_clutch_session=True,
                )
            # Apply the reset boundary once on each edge. Releasing on every
            # status frame while reset remained active repeatedly destroyed
            # the operator's re-arm transition.
            if previous_reset != self._backend_reset_active:
                self._begin_session_boundary_locked(
                    'reset state changed; release both grips before re-clutching',
                    new_clutch_session=True,
                )
            if not head_follow_enabled:
                self._head_mapper.release()
            if self._hardware_enabled and not self._backend_reset_active:
                for side in ('left', 'right'):
                    if side not in rejected_sides:
                        self._ik_reanchor_pending[side] = False
                        continue
                    if (
                        not self._ik_reanchor_pending[side]
                        and self._mappers[side].latched
                    ):
                        # Keep Grip held, but discard the accumulated unreachable
                        # Cartesian offset.  The next control tick latches the
                        # current VR hand to measured FK and tells the controller
                        # to rebase its position reference, preventing catch-up.
                        self._mappers[side].release()
                        self._last_targets.pop(side, None)
                        self._processed_versions[side] = -1
                        self._grip_rearm.set_required(side, False)
                        self._release_hold_sent[side] = False
                        self._ik_reanchor_pending[side] = True
                        self._last_reason = (
                            f'{side} IK boundary reached; automatically re-anchoring '
                            'from measured pose'
                        )
            if not self._hardware_enabled:
                self._disable_request_pending = False

    def _on_vr_input(self, message):
        try:
            packet = json.loads(message.data)
            if not isinstance(packet, dict):
                return
        except json.JSONDecodeError:
            return
        now = self._now()
        hand = str(packet.get('hand', '')).lower()
        event_type = packet.get('type')
        if event_type in ('button_press', 'button_release'):
            pressed = bool(packet.get('pressed', event_type == 'button_press'))
            button = str(packet.get('button', '')).upper()
            with self._lock:
                if hand == 'left' and button == 'X':
                    self._quick_state.update_button_event('left_x', pressed)
                elif hand == 'right' and button == 'A':
                    self._quick_state.update_button_event('right_a', pressed)
                elif hand == 'left' and button == 'Y':
                    self._hold_reset_state.update_button_event('left_y', pressed)
                elif hand == 'right' and button == 'B':
                    self._hold_reset_state.update_button_event('right_b', pressed)
                self._last_vr_time = now
            return
        if hand in ('left', 'right') and packet.get('gripReleased'):
            with self._lock:
                # Older cached web pages may still send this reliable event.
                # Treat it as the start of the same bounded debounce window;
                # never let one event tear down and re-anchor an active arm.
                sample = self._samples.get(hand)
                if sample is not None:
                    sample.grip_active = False
                    self._sample_times[hand] = now
                    if self._grip_false_since[hand] is None:
                        self._grip_false_since[hand] = now
                # Re-arm is an input-safety transition, not a motion command.
                # Process it here so a pending X+A request cannot prevent a
                # genuine release from unlocking the next Grip press.
                self._grip_rearm.observe(
                    hand, False, now,
                    self.get_parameter('grip_release_debounce_sec').value,
                )
                self._last_vr_time = now
            return
        if hand in ('left', 'right') and packet.get('triggerReleased'):
            with self._lock:
                sample = self._samples.get(hand)
                if sample is not None:
                    sample.trigger = 0.0
                    # Reliable release is input only. The normal control tick
                    # owns clutch, freshness, reset and enable checks. Never
                    # open a loaded gripper after Grip/session release here.
                self._last_vr_time = now
            return

        parsed = {}
        for side, key in (('left', 'leftController'), ('right', 'rightController')):
            try:
                parsed[side] = ControllerSample.from_packet(packet.get(key))
            except Exception:
                continue
        try:
            head_sample = HeadSample.from_packet(packet.get('head'))
        except Exception:
            head_sample = None
        with self._lock:
            for side, sample in parsed.items():
                self._samples[side] = sample
                self._sample_times[side] = now
                self._sample_versions[side] += 1
                if sample.grip_active:
                    self._grip_false_since[side] = None
                elif self._grip_false_since[side] is None:
                    self._grip_false_since[side] = now
                self._grip_rearm.observe(
                    side, sample.grip_active, now,
                    self.get_parameter('grip_release_debounce_sec').value,
                )
            left_packet = packet.get('leftController') or {}
            right_packet = packet.get('rightController') or {}
            left_tracked = bool(
                isinstance(left_packet.get('position'), dict)
                and isinstance(left_packet.get('quaternion'), dict)
            )
            right_tracked = bool(
                isinstance(right_packet.get('position'), dict)
                and isinstance(right_packet.get('quaternion'), dict)
            )
            self._quick_state.update_full_snapshot(
                bool(left_packet.get('xButton', 0)),
                bool(right_packet.get('aButton', 0)),
                now,
                both_tracked=left_tracked and right_tracked,
            )
            self._hold_reset_state.update_full_snapshot(
                bool(left_packet.get('yButton', 0)),
                bool(right_packet.get('bButton', 0)), now,
                both_tracked=left_tracked and right_tracked,
            )
            if parsed:
                self._last_vr_time = now
            if head_sample is not None:
                self._head_sample = head_sample
                self._head_sample_time = now
            if self._desktop_gate.enabled:
                was_ready = self._desktop_gate.ready
                ready, reason = self._desktop_gate.observe(
                    head_position=(
                        None if head_sample is None else head_sample.position
                    ),
                    head_rotation_deg=(
                        None if head_sample is None else head_sample.rotation
                    ),
                    controllers={
                        side: sample.position for side, sample in parsed.items()
                    },
                    now=now,
                )
                if ready != was_ready:
                    self._begin_session_boundary_locked(
                        (
                            'desktop tracking is stable; release both grips '
                            'before re-clutching'
                            if ready else reason
                        ),
                        notify_controller=True,
                        new_clutch_session=True,
                    )
                else:
                    self._last_reason = reason

    def _check_quick_reset(self, now):
        if self._quick_state.pending:
            return self._check_reset_gesture(now, self._quick_state, self._reset_client)
        if self._hold_reset_state.pending or self._hold_reset_state.both_pressed:
            self._quick_state.consume_chord()
            return self._check_reset_gesture(now, self._hold_reset_state, self._hold_reset_client)
        return self._check_reset_gesture(now, self._quick_state, self._reset_client)

    def _check_reset_gesture(self, now, state, client):
        if not bool(self.get_parameter('quick_reset_enabled').value):
            return False
        expired = state.expire_request(now)
        if expired is not None:
            future = self._quick_future
            self._quick_future = None
            if future is not None and not future.done():
                future.cancel()
            self._last_reason = (
                'quick reset request timed out; release X+A before retrying'
            )
        fresh_limit = max(
            0.05,
            float(self.get_parameter('quick_reset_input_fresh_sec').value),
        )
        gesture = state.gesture_state(
            now,
            fresh_limit,
            self.get_parameter('quick_reset_hold_seconds').value,
        )
        if gesture == 'pending':
            # Suppress Cartesian motion only while a bounded request is in
            # flight. Heartbeat and Grip re-arm continue in their callbacks.
            return True
        if gesture in ('idle', 'holding', 'consumed'):
            return gesture == 'holding'
        backend_dry_run = bool(self._backend_status.get('dry_run', self._dry_run))
        if not self._hardware_enabled and not backend_dry_run:
            self._last_reason = 'quick reset rejected: hardware is not enabled'
            state.consume_chord()
            return True
        if not client.service_is_ready():
            self._last_reason = 'quick reset service is unavailable'
            state.consume_chord()
            return True
        self._release_all(require_release=True)
        future = client.call_async(Trigger.Request())
        generation = state.begin_request(now)
        self._quick_future = future

        def finished(done):
            with self._lock:
                if not state.complete_request(generation):
                    return
                if self._quick_future is done:
                    self._quick_future = None
                try:
                    result = done.result()
                    self._last_reason = result.message
                except Exception as exc:
                    self._last_reason = f'quick reset request failed: {exc}'

        future.add_done_callback(finished)
        return True

    def _calibrated_gripper_trigger(self, side, trigger):
        scale = float(self.get_parameter(
            f'gripper_{side}_trigger_scale'
        ).value)
        value = float(np.clip(float(trigger) * scale, 0.0, 1.0))
        full_close = float(np.clip(
            self.get_parameter('gripper_full_close_threshold').value,
            0.5,
            1.0,
        ))
        # Separate enter/leave thresholds prevent a held trigger around the
        # full-close boundary alternating between 360 and ~310 motor degrees.
        latched = getattr(self, '_gripper_full_close_latched', None)
        if latched is None:
            latched = self._gripper_full_close_latched = {'left': False, 'right': False}
        if value >= full_close:
            latched[side] = True
        elif value <= max(0.0, full_close - 0.08):
            latched[side] = False
        if latched[side]:
            value = 1.0
        return value

    def _synchronise_gripper_triggers(self, triggers, now):
        calibrated = {
            side: self._calibrated_gripper_trigger(side, trigger)
            for side, trigger in triggers.items()
        }
        activation = float(np.clip(
            self.get_parameter('gripper_dual_sync_activation').value,
            0.0,
            1.0,
        ))
        for side in ('left', 'right'):
            if side not in calibrated:
                continue
            previous = self._gripper_input_raw[side]
            current = calibrated[side]
            if current > activation and previous <= activation:
                self._gripper_press_time[side] = now
            elif current <= activation:
                self._gripper_press_time[side] = None
            self._gripper_input_raw[side] = current

        both_present = all(side in calibrated for side in ('left', 'right'))
        both_pressed = both_present and all(
            calibrated[side] > activation for side in ('left', 'right')
        )
        if self._gripper_dual_sync_active and not both_pressed:
            self._gripper_dual_sync_active = False
        if (
            bool(self.get_parameter('gripper_dual_sync_enabled').value)
            and not self._gripper_dual_sync_active
            and both_pressed
            and all(self._gripper_press_time[side] is not None
                    for side in ('left', 'right'))
        ):
            press_delta = abs(
                self._gripper_press_time['left']
                - self._gripper_press_time['right']
            )
            input_delta = abs(calibrated['left'] - calibrated['right'])
            if (
                press_delta <= max(0.0, float(self.get_parameter(
                    'gripper_dual_sync_window_sec').value))
                and input_delta <= max(0.0, float(self.get_parameter(
                    'gripper_dual_sync_max_difference').value))
            ):
                self._gripper_dual_sync_active = True

        if self._gripper_dual_sync_active and both_present:
            shared = max(calibrated['left'], calibrated['right'])
            calibrated['left'] = shared
            calibrated['right'] = shared
        for side, value in calibrated.items():
            self._gripper_input_effective[side] = value
        return calibrated

    def _validate_gripper_width(self, parameters):
        try:
            for parameter in parameters:
                if parameter.name == 'gripper_closed_width_cm':
                    save_width(checked_width(parameter.value))
            return SetParametersResult(successful=True)
        except (ValueError, TypeError, OSError) as error:
            return SetParametersResult(successful=False, reason=str(error))

    def _publish_grippers(self, triggers, now, force=False):
        if not bool(self.get_parameter('enable_gripper').value):
            return
        effective = self._synchronise_gripper_triggers(triggers, now)
        payload = {}
        sent = []
        for side in ('left', 'right'):
            if side not in effective:
                continue
            raw = linear_gripper_position(
                effective[side],
                self.get_parameter('gripper_open_position').value,
                min(float(self.get_parameter('gripper_closed_position').value),
                    width_to_motor(self.get_parameter('gripper_closed_width_cm').value)),
                self.get_parameter('gripper_trigger_deadzone').value,
            )
            previous = self._gripper_filtered[side]
            if previous is None:
                previous = raw
            alpha = float(np.clip(
                self.get_parameter('gripper_filter_alpha').value, 0.0, 1.0
            ))
            candidate = previous + alpha * (raw - previous)
            maximum_step = max(
                0.1,
                float(self.get_parameter('gripper_max_step_per_cycle').value),
            )
            candidate = previous + float(np.clip(
                candidate - previous, -maximum_step, maximum_step
            ))
            self._gripper_filtered[side] = candidate
            last = self._gripper_last_sent[side]
            changed = last is None or abs(candidate - last) >= float(
                self.get_parameter('gripper_command_epsilon').value
            )
            resend = now - self._gripper_last_time[side] >= float(
                self.get_parameter('gripper_resend_interval').value
            )
            if force or changed or resend:
                payload[f'{side}_gripper_target_joints_position'] = [candidate]
                sent.append(side)
        if not payload:
            return
        # Both sides from one WebXR frame are deliberately emitted in one ROS
        # message, so the controller and vendor worker apply them in one tick.
        self._gripper_pub.publish(String(
            data=json.dumps(payload, separators=(',', ':'))
        ))
        for side in sent:
            self._gripper_last_sent[side] = self._gripper_filtered[side]
            self._gripper_last_time[side] = now

    def _request_disable(self, reason):
        if self._disable_request_pending or not self._hardware_enabled:
            return
        self._last_reason = reason
        if not self._enable_client.service_is_ready():
            return
        self._disable_request_pending = True
        future = self._enable_client.call_async(SetBool.Request(data=False))

        def finished(_done):
            self._disable_request_pending = False

        future.add_done_callback(finished)

    def _publish_head_follow_target(self, now):
        """Run HMD-to-neck tracking independently from hand tracking.

        Quest can briefly remove one or both controller input sources while its
        viewer pose remains valid.  The old control flow returned early for the
        arm hold before publishing the neck target, making otherwise healthy
        head tracking stop whenever a hand controller blinked.
        """
        enabled = bool(
            self._hardware_enabled
            and not self._backend_reset_active
            and not self._desktop_gate.enabled
            and self._backend_status.get('follow_authority_allowed', True)
            and self._backend_status.get('head_follow_enabled', False)
        )
        if not enabled:
            self._head_mapper.release()
            return False
        head_timeout = max(
            0.05, float(self.get_parameter('head_input_timeout').value)
        )
        if (
            self._head_sample is None
            or now - self._head_sample_time > head_timeout
        ):
            # A short WebXR/Wi-Fi outage is not a calibration request.  Keep
            # both the HMD reference and the last neck target; when fresh data
            # resumes it continues from the same operator-forward basis.
            return False
        measured_neck = self._backend_status.get('measured_neck_deg')
        if not self._head_mapper.latched:
            if not (
                isinstance(measured_neck, list)
                and len(measured_neck) == 3
                and all(math.isfinite(float(value)) for value in measured_neck)
            ):
                return False
            self._head_mapper.latch(self._head_sample, measured_neck)
        neck_target = self._head_mapper.map(self._head_sample)
        self._joint_target_pub.publish(String(data=json.dumps({
            'neck_target_joints_position': [
                float(value) for value in neck_target
            ],
            'source': 'vr_head_follow',
            'tracking_mode': str(
                self.get_parameter('head_tracking_mode').value
            ),
        }, separators=(',', ':'))))
        return True

    def _control_tick(self):
        with self._lock:
            now = self._now()
            backend_dry_run = bool(self._backend_status.get('dry_run', self._dry_run))
            if self._check_quick_reset(now):
                return
            # Head pose has its own freshness clock and remains useful while a
            # hand controller temporarily disappears. Arm safety handling below
            # may still release/hold both arms without suppressing the neck.
            self._publish_head_follow_target(now)
            eef_age = now - self._last_eef_time
            vr_age = now - self._last_vr_time
            if eef_age > float(self.get_parameter('eef_feedback_timeout').value):
                self._release_all(require_release=True)
                self._last_reason = 'waiting for fresh independent FK feedback'
                return
            tracking_state = vr_tracking_state(
                vr_age,
                self.get_parameter('vr_timeout').value,
                self.get_parameter('hardware_disable_vr_timeout').value,
            )
            if tracking_state != 'active':
                if tracking_state == 'paused':
                    # Hold the robot and drop the old clutch anchor.  If the
                    # controller pose returns, a still-held Grip is re-anchored
                    # at the current robot pose, so recovery cannot jump.
                    self._release_all(
                        require_release=False, notify_controller=True
                    )
                    self._last_reason = (
                        'VR controller tracking paused; holding position and '
                        'waiting for automatic re-anchor'
                    )
                    return
                self._release_all(require_release=True, notify_controller=True)
                self._last_reason = 'VR controller tracking timed out'
                if bool(self.get_parameter('disable_on_vr_timeout').value):
                    self._request_disable(
                        'VR controller tracking lost for too long; '
                        'hardware output disabled'
                    )
                return
            if self._backend_reset_active:
                return

            if self._desktop_gate.enabled and not self._desktop_gate.ready:
                self._release_all(
                    require_release=True, notify_controller=True
                )
                self._last_reason = self._desktop_gate.reason
                return

            targets = {}
            active = []
            reanchored = []
            safety_reasons = []
            grace = float(
                self.get_parameter('controller_tracking_grace_period').value
            )
            body_position = None
            if (
                not self._desktop_gate.enabled
                and
                bool(self.get_parameter(
                    'operator_translation_compensation_enabled').value)
                and self._head_sample is not None
                and self._head_sample.position is not None
                and now - self._head_sample_time <= max(
                    grace,
                    float(self.get_parameter('head_input_timeout').value),
                )
            ):
                body_position = self._head_sample.position
            gripper_triggers = {}
            for side in ('left', 'right'):
                sample = self._samples.get(side)
                mapper = self._mappers[side]
                if sample is None:
                    continue
                if now - self._sample_times.get(side, 0.0) > grace:
                    self._samples.pop(side, None)
                    self._sample_times.pop(side, None)
                    self._release_side(
                        side, require_release=False, notify_controller=True
                    )
                    continue
                if not sample.grip_active:
                    grip_state, started = grip_release_state(
                        False,
                        mapper.latched,
                        self._grip_false_since[side],
                        now,
                        self.get_parameter('grip_release_debounce_sec').value,
                    )
                    self._grip_false_since[side] = started
                    if grip_state == 'pending':
                        # Freeze immediately at the last valid target.  If the
                        # false sample was genuine, release follows within the
                        # bounded debounce time; if it was noise, the existing
                        # clutch anchor survives without a catch-up jump.
                        target = self._last_targets.get(side)
                        if target is not None:
                            targets[side] = target
                            active.append(side)
                        continue
                    self._release_side(
                        side, require_release=False, notify_controller=True
                    )
                    continue
                if self._require_release[side]:
                    continue
                mapping_sample = sample
                mapping_body_position = body_position
                if self._desktop_gate.enabled:
                    # Translation uses the operator-forward basis captured by
                    # the shortcut. Preserve the already-correct controller
                    # quaternion mapping independently.
                    mapping_sample = ControllerSample(
                        position=(
                            self._desktop_position_rotation @ sample.position
                        ),
                        orientation=sample.orientation,
                        grip_active=sample.grip_active,
                        trigger=sample.trigger,
                    )
                    if body_position is not None:
                        mapping_body_position = (
                            self._desktop_position_rotation @ body_position
                        )
                elif bool(self.get_parameter(
                        'operator_yaw_compensation_enabled').value):
                    # Capture the operator-facing basis at each fresh Grip
                    # clutch. X+A/reset clears this basis, so turning the whole
                    # body and then re-clutching makes physical forward map to
                    # robot forward instead of the old room direction.
                    if not mapper.latched:
                        try:
                            if (
                                self._head_sample is None
                                or self._head_sample.quaternion is None
                            ):
                                raise ValueError('fresh HMD quaternion unavailable')
                            operator_yaw = webxr_yaw_deg_from_quaternion(
                                self._head_sample.quaternion
                            )
                            self._wearable_operator_yaw_deg[side] = operator_yaw
                            self._wearable_position_rotation[side] = (
                                world_to_operator_yaw_rotation(operator_yaw)
                            )
                        except ValueError:
                            # A near-vertical HMD has no stable horizontal yaw.
                            # Identity is deterministic and the next explicit
                            # clutch can capture a valid facing direction.
                            self._wearable_operator_yaw_deg[side] = 0.0
                            self._wearable_position_rotation[side] = np.eye(
                                3, dtype=float
                            )
                    operator_rotation = self._wearable_position_rotation[side]
                    mapping_sample = ControllerSample(
                        position=operator_rotation @ sample.position,
                        orientation=matrix_to_quaternion(
                            operator_rotation
                            @ quaternion_to_matrix(sample.orientation)
                        ),
                        grip_active=sample.grip_active,
                        trigger=sample.trigger,
                    )
                    if body_position is not None:
                        mapping_body_position = operator_rotation @ body_position
                if not mapper.latched:
                    mapper.latch(
                        mapping_sample,
                        self._current_poses[side],
                        body_position=mapping_body_position,
                    )
                    self._release_hold_sent[side] = False
                    reanchored.append(side)
                # Quest3 integrates adjacent controller samples exactly once.
                # Re-processing the same WebXR sample would not add motion,
                # but explicitly reusing the last target also prevents HMD
                # compensation from advancing on a mismatched timestamp.
                version = self._sample_versions[side]
                if (
                    version == self._processed_versions[side]
                    and side in self._last_targets
                ):
                    target = self._last_targets[side]
                    reason = ''
                else:
                    target, reason = mapper.map(
                        mapping_sample, body_position=mapping_body_position
                    )
                    self._processed_versions[side] = version
                if target is not None:
                    self._last_targets[side] = target
                if target is None:
                    if reason:
                        safety_reasons.append(f'{side}: {reason}')
                    continue
                targets[side] = target
                active.append(side)
                trigger = self._gripper_clutch[side].observe(sample.trigger)
                if trigger is not None:
                    gripper_triggers[side] = trigger

            if gripper_triggers:
                self._publish_grippers(gripper_triggers, now)

            if self._current_poses:
                preview = vendor_pose_payload(targets, self._current_poses)
                preview.update({
                    'active_arms': active,
                    'hardware_enabled': self._hardware_enabled,
                    'dry_run': backend_dry_run,
                    'safety_reasons': safety_reasons,
                })
                self._preview_pub.publish(
                    String(data=json.dumps(preview, separators=(',', ':')))
                )
            if safety_reasons:
                self._last_reason = '; '.join(safety_reasons)
            elif active:
                self._last_reason = 'active: ' + ', '.join(active)
            else:
                self._last_reason = 'waiting for grip clutch'
            # In real mode, disarmed intent remains local preview only.  In
            # dry-run it is sent to the controller so IK/collision can be tested.
            # Publish the authoritative clutch state even when neither arm has
            # a Cartesian target.  This continuously overwrites a missed
            # release event and invalidates any IK result that was in flight at
            # the release boundary.
            if backend_dry_run or self._hardware_enabled:
                self._clutch_sequence += 1
                self._eef_target_pub.publish(String(
                    data=json.dumps(
                        eef_target_payload(
                            targets,
                            reanchored,
                            clutch_session=self._clutch_session,
                            clutch_sequence=self._clutch_sequence,
                            active_arms=active,
                        ),
                        separators=(',', ':'),
                    )
                ))

    def _publish_status_and_heartbeat(self):
        with self._lock:
            now = self._now()
            vr_timeout = float(self.get_parameter('vr_timeout').value)
            vr_age = None if not self._last_vr_time else (
                now - self._last_vr_time
            )
            tracked_hands = [
                side for side, sample_time in self._sample_times.items()
                if vr_input_is_fresh(now - sample_time, vr_timeout)
            ]
            vr_input_fresh = bool(
                tracked_hands and vr_input_is_fresh(vr_age, vr_timeout)
            )
            active = [side for side, mapper in self._mappers.items() if mapper.latched]
            payload = {
                'state': self._backend_status.get('state', 'waiting_backend'),
                'detail': self._last_reason,
                'dry_run': bool(self._backend_status.get('dry_run', self._dry_run)),
                'hardware_enabled': self._hardware_enabled,
                'hardware_enable_pending': bool(
                    self._backend_status.get('hardware_enable_pending', False)
                ),
                'hardware_ready': bool(
                    self._backend_status.get('hardware_ready', False)
                ),
                'active_arms': active,
                'tracked_hands': tracked_hands,
                'vr_input_fresh': vr_input_fresh,
                'vr_age': None if vr_age is None else round(vr_age, 3),
                'eef_age': None if not self._last_eef_time else round(
                    now - self._last_eef_time, 3
                ),
                'operator_translation_compensation': {
                    'enabled': bool(
                        not self._desktop_gate.enabled
                        and self.get_parameter(
                            'operator_translation_compensation_enabled').value
                    ),
                    'head_position_available': bool(
                        self._head_sample is not None
                        and self._head_sample.position is not None
                    ),
                    'head_sample_age': (
                        None if not self._head_sample_time
                        else round(now - self._head_sample_time, 3)
                    ),
                },
                'operator_yaw_compensation': {
                    'enabled': bool(
                        not self._desktop_gate.enabled
                        and self.get_parameter(
                            'operator_yaw_compensation_enabled').value
                    ),
                    'left_clutch_yaw_deg': round(
                        float(self._wearable_operator_yaw_deg['left']), 2
                    ),
                    'right_clutch_yaw_deg': round(
                        float(self._wearable_operator_yaw_deg['right']), 2
                    ),
                    'recalibrates_after_reset': True,
                },
                'head_follow': {
                    'enabled': bool(
                        self._backend_status.get('head_follow_enabled', False)
                    ),
                    'suppressed_by_desktop_mode': bool(
                        self._desktop_gate.enabled
                    ),
                    'tracking_mode': str(
                        self.get_parameter('head_tracking_mode').value
                    ),
                    'roll_follow_enabled': bool(self.get_parameter(
                        'head_roll_follow_enabled').value),
                    'pose_available': self._head_sample is not None,
                    'pose_fresh': bool(
                        self._head_sample is not None
                        and self._head_sample_time > 0.0
                        and now - self._head_sample_time <= float(
                            self.get_parameter('head_input_timeout').value
                        )
                    ),
                    'pose_age': (
                        None if not self._head_sample_time
                        else round(now - self._head_sample_time, 3)
                    ),
                    'mapper_latched': bool(self._head_mapper.latched),
                    'reference_rebases': int(
                        self._head_mapper.reference_rebases
                    ),
                    'recalibration_required': bool(
                        self._head_mapper.recalibration_required
                    ),
                },
                'desktop_mode': {
                    **self._desktop_gate.status(),
                    'operator_yaw_deg': round(
                        float(self._desktop_entry_yaw_deg), 2
                    ),
                    'shortcut': 'hold both thumbstick buttons for 1.0s',
                    'head_and_waist_follow_suppressed': bool(
                        self._desktop_gate.enabled
                    ),
                },
                'quick_reset': {
                    'gesture': 'hold left X + right A',
                    'triggered': self._quick_state.chord_consumed,
                    'request_pending': self._quick_state.pending,
                    'request_generation': self._quick_state.generation,
                    'active': self._backend_reset_active,
                    'progress': self._backend_status.get('quick_reset_progress', 0.0),
                },
                'grip_rearm_required': {
                    side: bool(self._require_release[side])
                    for side in ('left', 'right')
                },
                'gripper_input': {
                    'raw': {
                        side: round(float(self._gripper_input_raw[side]), 3)
                        for side in ('left', 'right')
                    },
                    'effective': {
                        side: round(
                            float(self._gripper_input_effective[side]), 3
                        ) for side in ('left', 'right')
                    },
                    'command_deg': {
                        side: (
                            None if self._gripper_last_sent[side] is None
                            else round(float(self._gripper_last_sent[side]), 2)
                        ) for side in ('left', 'right')
                    },
                    'dual_sync_active': bool(
                        self._gripper_dual_sync_active
                    ),
                },
                'backend': self._backend_status,
            }
            encoded = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
            self._status_pub.publish(String(data=encoded))
            # Live tracking authorizes a new enable.  When an already-enabled
            # headset sleeps, publish an explicit hold-only liveness heartbeat:
            # the controller remains in its measured position hold, but this
            # packet can never authorize a new hardware enable.
            if vr_input_fresh:
                self._heartbeat_pub.publish(String(data=json.dumps({
                    'stamp_monotonic': now,
                    'vr_age': payload['vr_age'],
                    'vr_input_fresh': True,
                    'mapper_alive': True,
                    'hold_only': False,
                    'tracked_hands': tracked_hands,
                    'eef_age': payload['eef_age'],
                }, separators=(',', ':'))))
            elif self._hardware_enabled:
                self._heartbeat_pub.publish(String(data=json.dumps({
                    'stamp_monotonic': now,
                    'vr_age': payload['vr_age'],
                    'vr_input_fresh': False,
                    'mapper_alive': True,
                    'hold_only': True,
                    'tracked_hands': [],
                    'eef_age': payload['eef_age'],
                }, separators=(',', ':'))))


def main(args=None):
    rclpy.init(args=args)
    node = IndependentVrMapper()
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
