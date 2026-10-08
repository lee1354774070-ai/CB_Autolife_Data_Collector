import glob
import fcntl
import json
import os
import struct
import threading
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .adaptive_position import (
    AdaptivePositionFeedforward,
    DynamicCommandLead,
    JointTargetVelocityEstimator,
    predict_latest_target,
    project_position,
)
from .body_height import (
    BODY_HEIGHT_MAXIMUM_DEG,
    BODY_HEIGHT_JOINT_NAMES,
    MAX_BODY_LOWERING_M,
    MAX_REVERSE_BODY_LOWERING_M,
    coordinated_body_height_targets,
    estimate_body_lowering,
    measured_reverse_squat,
)
from .kinematics import IndependentArmKinematics
from .quest_placo_kinematics import QuestPlacoKinematics
from .latest_sample import LatestSampleMailbox
from .qos import latest_sample_qos, reliable_latest_qos
from .reset_path import (
    conservative_reset_duration,
    continuous_direct_reset,
    feedback_bounded_reset_progress,
)
from .schema import (
    parse_body_motion_feedback,
    parse_eef_target,
    parse_joint_feedback,
)
from .teleop_core import forward_reach_to_waist_pitch, vr_input_is_fresh
from .trajectory_limiter import TrajectoryLimiter, bounded_target_by_feedback
from .velocity_servo import VelocityServo, lookahead_reference


def classify_teleop_heartbeat(payload, maximum_age):
    """Return ``(mode, vr_age, tracked_hands)`` for an authenticated shape.

    ``tracking`` may authorize a new hardware enable. ``hold`` only proves
    that the mapper process is alive after an already-enabled headset sleeps.
    """
    if not isinstance(payload, dict):
        raise ValueError('heartbeat is not an object')
    tracked_hands = payload.get('tracked_hands', [])
    tracking = (
        payload.get('vr_input_fresh') is True
        and isinstance(tracked_hands, list)
        and bool(tracked_hands)
        and all(side in ('left', 'right') for side in tracked_hands)
        and vr_input_is_fresh(payload.get('vr_age'), maximum_age)
    )
    hold_only = (
        payload.get('mapper_alive') is True
        and payload.get('hold_only') is True
        and payload.get('vr_input_fresh') is False
        and isinstance(tracked_hands, list)
        and not tracked_hands
    )
    if tracking:
        return 'tracking', float(payload['vr_age']), list(tracked_hands)
    if hold_only:
        return 'hold', None, []
    raise ValueError('heartbeat is neither live tracking nor safe hold')


ARM_JOINT_NAMES = [
    'Joint_Left_Shoulder_Inner',
    'Joint_Left_Shoulder_Outer',
    'Joint_Left_UpperArm',
    'Joint_Left_Elbow',
    'Joint_Left_Forearm',
    'Joint_Left_Wrist_Upper',
    'Joint_Left_Wrist_Lower',
    'Joint_Right_Shoulder_Inner',
    'Joint_Right_Shoulder_Outer',
    'Joint_Right_UpperArm',
    'Joint_Right_Elbow',
    'Joint_Right_Forearm',
    'Joint_Right_Wrist_Upper',
    'Joint_Right_Wrist_Lower',
]
GRIPPER_JOINT_NAMES = ['Joint_Left_Gripper', 'Joint_Right_Gripper']
WAIST_JOINT_NAMES = ['Joint_Waist_Pitch', 'Joint_Waist_Yaw']
NECK_JOINT_NAMES = ['Joint_Neck_Roll', 'Joint_Neck_Pitch', 'Joint_Neck_Yaw']
TELEOP_JOINT_NAMES = (
    ARM_JOINT_NAMES + GRIPPER_JOINT_NAMES + WAIST_JOINT_NAMES + NECK_JOINT_NAMES
)
# The neck controller has a calibrated standing offset that may sit slightly
# outside the simplified URDF margin.  Neck targets are independently clipped
# against their measured/URDF envelope in _on_head_target, so keep the takeover
# pose guard scoped to the arm and waist joints as before.
TELEOP_LIMIT_JOINT_NAMES = ARM_JOINT_NAMES + WAIST_JOINT_NAMES
RESET_TRACKED_JOINT_NAMES = ARM_JOINT_NAMES + NECK_JOINT_NAMES
ENABLE_TRACKED_JOINT_NAMES = ARM_JOINT_NAMES + WAIST_JOINT_NAMES
# X+A also stands up; Y+B holds height. Both require physical completion.
RESET_REQUIRED_JOINT_NAMES = RESET_TRACKED_JOINT_NAMES + list(BODY_HEIGHT_JOINT_NAMES)


def enable_vendor_pid_loop(shared_memory_glob):
    """Enable the vendor PID loop without invoking its whole-body reset."""
    candidates = []
    for path in glob.glob(str(shared_memory_glob)):
        try:
            stat = os.stat(path)
        except OSError:
            continue
        if stat.st_size == 4:
            candidates.append((stat.st_mtime_ns, path))
    if not candidates:
        raise RuntimeError(
            f'vendor PID-loop shared memory was not found: {shared_memory_glob}'
        )

    _, path = max(candidates)
    with open(path, 'r+b', buffering=0) as stream:
        previous_raw = stream.read(4)
        if len(previous_raw) != 4:
            raise RuntimeError(f'invalid vendor PID-loop state file: {path}')
        previous = struct.unpack('i', previous_raw)[0]
        if previous not in (0, 1):
            raise RuntimeError(
                f'invalid vendor PID-loop state {previous} in {path}'
            )
        stream.seek(0)
        stream.write(struct.pack('i', 1))
        stream.flush()
        os.fsync(stream.fileno())

    with open(path, 'rb') as stream:
        enabled_raw = stream.read(4)
    if len(enabled_raw) != 4 or struct.unpack('i', enabled_raw)[0] != 1:
        raise RuntimeError(f'failed to enable vendor PID loop through {path}')
    return path, bool(previous)


class IndependentArmController(Node):
    def __init__(self):
        super().__init__('independent_arm_controller')
        self._declare_parameters()
        self._lock = threading.RLock()
        self._suffix = str(self.get_parameter('topic_suffix').value)
        self._acquire_instance_lock()
        self._dry_run = bool(self.get_parameter('dry_run').value)
        self._collision_required = bool(
            self.get_parameter('enable_self_collision_check').value
        )
        self._state = 'WAITING_FEEDBACK'
        self._reason = 'waiting for fresh joint feedback'
        self._last_hardware_fault_reason = None
        self._hardware_enabled = False
        self._estop_latched = False
        self._feedback = None
        self._feedback_motion = None
        self._feedback_time = 0.0
        self._target_time = 0.0
        self._target_groups = None
        self._target_source = 'none'
        self._gripper_targets = {'left': None, 'right': None}
        self._gripper_dirty = {'left': False, 'right': False}
        self._reset_gripper_last_publish_time = 0.0
        self._grip_release_held = set()
        self._clutch_session = None
        self._clutch_sequence = -1
        self._last_ik = None
        self._last_tick = time.monotonic()
        self._last_status = 0.0
        self._last_output_guard_check = 0.0
        self._last_command_collisions = []
        self._last_collision_hold_warning = 0.0
        self._limiter = None
        self._enable_pending = False
        self._enable_hold_groups = None
        self._enable_started = 0.0
        self._enable_stage_started = 0.0
        self._enable_stage = None
        self._enable_stable_since = None
        self._enable_last_feedback_groups = None
        self._enable_post_sync_rebased = False
        self._enable_pid_path = None
        self._enable_pid_was_enabled = None
        self._enable_full_reset = False
        self._full_reset_published_at = 0.0
        self._full_reset_stable_since = None
        self._full_reset_max_error_deg = None
        self._full_reset_max_speed_deg_sec = None
        self._last_eef_feedback_publish = 0.0
        self._last_fk_warning = 0.0
        self._reset_active = False
        self._reset_started = 0.0
        self._reset_initial_error = 0.0
        self._reset_progress = 0.0
        self._reset_path_start = None
        self._reset_path_goal = None
        self._reset_path_duration_sec = 0.0
        self._reset_path_elapsed_sec = 0.0
        self._teleop_heartbeat_time = 0.0
        self._teleop_heartbeat_mode = 'missing'
        self._teleop_vr_age = None
        self._teleop_tracked_hands = []
        self._teleop_heartbeat_rejection = 'fresh VR controller data is missing'
        self._command_monitor_started = 0.0
        self._recent_owned_commands = {}
        self._owned_command_sequence = 0
        self._last_external_command_time = 0.0
        self._last_external_command_topic = None
        self._last_external_command_payload = None
        self._external_command_counts = {}
        self._output_control_mode = str(
            self.get_parameter('output_control_mode').value
        ).strip().lower()
        if self._output_control_mode not in ('position', 'velocity'):
            raise ValueError('output_control_mode must be position or velocity')
        self._velocity_servo = None
        self._velocity_active_sides = set()
        self._last_velocity_command = None
        self._last_position_command = None
        self._adaptive_feedforward = AdaptivePositionFeedforward(21, range(4, 18))
        self._adaptive_lookahead = np.zeros(21, dtype=float)
        self._adaptive_velocity_limit = np.zeros(21, dtype=float)
        self._adaptive_acceleration = np.zeros(21, dtype=float)
        self._adaptive_following_error = np.zeros(21, dtype=float)
        self._target_velocity_estimator = JointTargetVelocityEstimator(
            21, range(4, 18)
        )
        self._target_velocity_estimate = np.zeros(21, dtype=float)
        self._dynamic_command_lead = DynamicCommandLead(21, range(4, 18))
        self._dynamic_lead_values = np.zeros(21, dtype=float)
        self._dynamic_following_error = np.zeros(21, dtype=float)
        self._latest_target_mailbox = LatestSampleMailbox()
        self._ik_thread = None
        self._sync_session_acquired = False
        self._sync_session_future = None
        self._authority_heartbeat_required = bool(self.get_parameter('authority_heartbeat_topic').value)
        self._authority_heartbeat_time = 0.0
        self._follow_authority_required = bool(self.get_parameter('follow_authority_topic').value)
        self._follow_authority_mode = ''
        self._follow_authority_time = 0.0
        self._collector_authority_epoch = -1
        self._collector_revoked_epoch = -1
        self._collector_body_origin_epoch = -1
        self._collector_gripper_origin_epoch = -1
        self._collector_session_id = ''
        self._collector_output_pub = self.create_publisher(
            String, '/collector_dagger/controller_output', reliable_latest_qos())
        self._waist_follow_enabled = bool(
            self.get_parameter('allow_waist_in_ik').value
        )
        self._waist_follow_profile = str(
            self.get_parameter('waist_follow_profile').value
        ).strip().lower()
        if self._waist_follow_profile not in ('ik_pitch_yaw', 'forward_pitch_only'):
            raise ValueError(
                'waist_follow_profile must be ik_pitch_yaw or forward_pitch_only'
            )
        self._waist_follow_neutral_pitch_deg = None
        self._waist_follow_locked_yaw_deg = None
        self._waist_follow_anchor_x = {'left': None, 'right': None}
        self._waist_follow_extension_m = 0.0
        self._waist_follow_pitch_target_deg = None
        self._body_height_control_enabled = bool(
            self.get_parameter('body_height_control_enabled').value
        )
        self._body_height_active = False
        self._body_height_watchdog_stopped = False
        self._body_height_lowering_m = None
        self._body_height_command_time = 0.0
        self._body_height_update_time = 0.0
        self._body_height_joint_targets = None
        self._body_height_command_progress = None
        self._body_height_last_direction = 0.0
        self._body_height_command_lead_deg = float(
            self.get_parameter('body_height_hold_command_lead_deg').value
        )
        self._teleop_joint_names = list(TELEOP_JOINT_NAMES)
        if self._body_height_control_enabled:
            self._teleop_joint_names = list(dict.fromkeys(
                self._teleop_joint_names + list(BODY_HEIGHT_JOINT_NAMES)
            ))
        waist_in_ik = (
            self._waist_follow_enabled
            and self._waist_follow_profile == 'ik_pitch_yaw'
        )
        self._head_follow_enabled = bool(
            self.get_parameter('head_follow_enabled').value
        )
        self._head_target = None
        self._head_target_time = 0.0
        self._task_head_target = None
        self._task_head_target_time = 0.0
        self._task_head_request_id = ''
        self._task_head_cancelled_id = ''
        self._task_head_cancel_reason = ''
        self._task_head_goal = None

        share = get_package_share_directory('openarmx_teleop_vr_306_v4')
        urdf = str(self.get_parameter('urdf_path').value)
        srdf = str(self.get_parameter('srdf_path').value)
        if not urdf:
            urdf = f'{share}/urdf/robot_v2_2_simplified.urdf'
        if not srdf:
            srdf = f'{share}/urdf/robot_v2_2.srdf'
        self._kinematics = IndependentArmKinematics(
            urdf,
            srdf,
            allow_waist=waist_in_ik,
            # Keep geometry available for autonomous reset preflight even
            # when ordinary per-frame teleoperation collision checks are off.
            collision_check=True,
            limit_margin_rad=np.deg2rad(
                float(self.get_parameter('joint_limit_margin_deg').value)
            ),
        )
        # V4 replaces only the IK middle layer with the Quest3 Placo QP step.
        # Pinocchio remains a separate outer model for FK, SRDF collision and
        # hard-limit validation, while Placo is refreshed from measured joints
        # for every new target just like Quest3-Teleoperation.
        self._ik_kinematics = QuestPlacoKinematics(
            urdf,
            srdf,
            allow_waist=waist_in_ik,
            collision_check=self._collision_required,
            limit_margin_rad=np.deg2rad(
                float(self.get_parameter('joint_limit_margin_deg').value)
            ),
            dt=1.0 / max(
                10.0, float(self.get_parameter('control_rate_hz').value)
            ),
            frame_weight=float(
                self.get_parameter('quest_placo_frame_weight').value
            ),
            manipulability_weight=float(
                self.get_parameter(
                    'quest_placo_manipulability_weight'
                ).value
            ),
            manipulability_min_weight=float(
                self.get_parameter(
                    'quest_placo_manipulability_min_weight'
                ).value
            ),
            manipulability_fade_start_elbow_deg=float(
                self.get_parameter(
                    'quest_placo_manipulability_fade_start_elbow_deg'
                ).value
            ),
            manipulability_fade_full_elbow_deg=float(
                self.get_parameter(
                    'quest_placo_manipulability_fade_full_elbow_deg'
                ).value
            ),
            kinetic_regularization=float(
                self.get_parameter(
                    'quest_placo_kinetic_regularization'
                ).value
            ),
        )

        input_topic = str(self.get_parameter('input_target_topic').value)
        gripper_topic = str(self.get_parameter('input_gripper_topic').value)
        release_topic = str(self.get_parameter('input_release_topic').value)
        joint_target_topic = str(self.get_parameter('input_joint_target_topic').value)
        body_height_topic = str(self.get_parameter('input_body_height_topic').value)
        head_target_topic = str(self.get_parameter('input_head_target_topic').value)
        preview_topic = str(self.get_parameter('preview_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)
        eef_feedback_topic = str(self.get_parameter('eef_feedback_topic').value)
        heartbeat_topic = str(self.get_parameter('teleop_heartbeat_topic').value)
        feedback_topic = (
            f'/topic_arm_whole_body_and_gripper_current_joints_status_{self._suffix}'
        )
        self._output_topic = (
            f'/topic_arm_whole_body_target_joints_position_{self._suffix}'
        )
        self._velocity_output_topic = (
            f'/topic_arm_whole_body_target_joints_velocity_{self._suffix}'
        )
        self._gripper_output_topic = (
            f'/topic_arm_gripper_target_joints_position_{self._suffix}'
        )
        self._prepare_topic = f'/control_prepare_arms_only_{self._suffix}'
        self._joint_enable_topic = f'/topic_arm_joints_set_enable_state_{self._suffix}'
        self._joint_clear_error_topic = f'/topic_arm_joints_clear_error_{self._suffix}'
        self._full_reset_topic = f'/control_reset_{self._suffix}'
        self._sync_session_service = (
            f'/control_independent_sync_hold_session_{self._suffix}'
            if bool(self.get_parameter('sync_hold_only_enable').value)
            else f'/control_independent_sync_session_{self._suffix}'
        )
        self._sync_session_heartbeat_topic = (
            f'/control_independent_sync_heartbeat_{self._suffix}'
        )
        self._sync_session_command_topic = (
            f'/control_independent_sync_session_command_{self._suffix}'
        )
        self._legacy_eef_topic = (
            f'/topic_arm_move_eef_pose_in_robot_frame_{self._suffix}'
        )
        self._monitored_command_topics = (
            self._output_topic,
            self._velocity_output_topic,
            self._gripper_output_topic,
            self._legacy_eef_topic,
        )
        self._command_monitor_started = time.monotonic()
        self._recent_owned_commands = {
            topic: [] for topic in self._monitored_command_topics
        }
        self._external_command_counts = {
            topic: 0 for topic in self._monitored_command_topics
        }

        self._preview_pub = self.create_publisher(
            String, preview_topic, latest_sample_qos()
        )
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self._eef_feedback_pub = self.create_publisher(
            String, eef_feedback_topic, latest_sample_qos()
        )
        self._joint_pub = None
        self._velocity_pub = None
        self._gripper_pub = None
        self._prepare_pub = None
        self._joint_enable_pub = None
        self._joint_clear_error_pub = None
        self._full_reset_pub = None
        self._sync_session_heartbeat_pub = self.create_publisher(
            String, self._sync_session_heartbeat_topic, reliable_latest_qos()
        )
        self._sync_session_command_pub = self.create_publisher(
            String, self._sync_session_command_topic, reliable_latest_qos()
        )
        self._sync_session_client = self.create_client(
            SetBool, self._sync_session_service
        )
        if not self._dry_run:
            # The vendor motor subscribers require RELIABLE QoS.  Depth one is
            # essential for teleoperation: after a scheduling or Wi-Fi stall,
            # an old joint target must be replaced instead of replayed.
            self._joint_pub = self.create_publisher(
                String, self._output_topic, reliable_latest_qos()
            )
            self._velocity_pub = self.create_publisher(
                String, self._velocity_output_topic, reliable_latest_qos()
            )
            self._gripper_pub = self.create_publisher(
                String, self._gripper_output_topic, reliable_latest_qos()
            )
            self._prepare_pub = self.create_publisher(String, self._prepare_topic, 10)
            self._joint_enable_pub = self.create_publisher(
                String, self._joint_enable_topic, 10
            )
            self._joint_clear_error_pub = self.create_publisher(
                String, self._joint_clear_error_topic, 10
            )
            self._full_reset_pub = self.create_publisher(
                String, self._full_reset_topic, 10
            )

        if self._authority_heartbeat_required:
            self.create_subscription(String, str(self.get_parameter('authority_heartbeat_topic').value),
                                     self._on_authority_heartbeat, reliable_latest_qos())
        if self._follow_authority_required:
            self.create_subscription(
                String, str(self.get_parameter('follow_authority_topic').value),
                self._on_follow_authority, reliable_latest_qos())
        self.create_subscription(
            String, feedback_topic, self._on_feedback, reliable_latest_qos()
        )
        self.create_subscription(
            String, input_topic, self._on_target, latest_sample_qos()
        )
        self.create_subscription(
            String, gripper_topic, self._on_gripper, latest_sample_qos()
        )
        self.create_subscription(
            String, release_topic, self._on_release_hold, reliable_latest_qos()
        )
        self.create_subscription(
            String, joint_target_topic, self._on_joint_target,
            reliable_latest_qos()
        )
        self.create_subscription(
            String, body_height_topic, self._on_body_height_command,
            reliable_latest_qos()
        )
        self.create_subscription(
            String, head_target_topic, self._on_head_target,
            latest_sample_qos()
        )
        self.create_subscription(
            String, '/openarmx_teleop_vr_306_v4/task_head_target',
            self._on_task_head_target, reliable_latest_qos()
        )
        self.create_subscription(
            String, heartbeat_topic, self._on_teleop_heartbeat,
            latest_sample_qos()
        )
        # These read-only subscriptions distinguish idle DDS endpoints from
        # publishers that are actually issuing commands.  They are also
        # created in dry-run mode so arbitration can be checked safely before
        # a later real-hardware launch.
        self.create_subscription(
            String, self._output_topic, self._on_joint_command_observed,
            latest_sample_qos()
        )
        self.create_subscription(
            String, self._velocity_output_topic,
            self._on_velocity_command_observed, latest_sample_qos()
        )
        self.create_subscription(
            String,
            self._gripper_output_topic,
            self._on_gripper_command_observed,
            latest_sample_qos(),
        )
        self.create_subscription(
            String,
            self._legacy_eef_topic,
            self._on_legacy_eef_command_observed,
            latest_sample_qos(),
        )
        self.create_service(
            SetBool,
            '/openarmx_teleop_vr_306_v4/set_hardware_enabled',
            self._on_hardware_enabled,
        )
        self.create_service(
            Trigger,
            '/openarmx_teleop_vr_306_v4/emergency_stop',
            self._on_emergency_stop,
        )
        self.create_service(
            Trigger,
            '/openarmx_teleop_vr_306_v4/clear_emergency_stop',
            self._on_clear_emergency_stop,
        )
        self.create_service(Trigger, '/openarmx_teleop_vr_306_v4/full_body_reset',
                            self._on_full_body_reset)
        self.create_service(
            Trigger,
            '/openarmx_teleop_vr_306_v4/quick_reset',
            self._on_quick_reset,
        )
        self.create_service(
            Trigger, '/openarmx_teleop_vr_306_v4/quick_reset_hold_grippers',
            self._on_hold_gripper_reset,
        )
        self.create_service(
            SetBool,
            '/openarmx_teleop_vr_306_v4/set_waist_follow_enabled',
            self._on_waist_follow_enabled,
        )
        self.create_service(
            SetBool,
            '/openarmx_teleop_vr_306_v4/set_head_follow_enabled',
            self._on_head_follow_enabled,
        )
        rate = float(self.get_parameter('control_rate_hz').value)
        self.create_timer(1.0 / max(rate, 10.0), self._control_tick)
        self.create_timer(0.10, self._publish_sync_session_heartbeat)
        self._ik_thread = threading.Thread(
            target=self._ik_worker_loop,
            name='openarmx-v4-quest-latest-ik',
            daemon=True,
        )
        self._ik_thread.start()
        # DDS graph discovery must not run while holding the 100 Hz control
        # lock. Expired/missing snapshots still fail closed at the same guard.
        self._output_guard_snapshot = (0.0, 'graph check pending', 'graph check pending')
        self._output_guard_stop = threading.Event()
        self._output_guard_thread = threading.Thread(
            target=self._refresh_output_guard, name='collector-dds-guard', daemon=True)
        self._output_guard_thread.start()
        self.get_logger().info(
            f'independent arm controller ready: suffix={self._suffix}, '
            f'dry_run={self._dry_run}, mode={self._output_control_mode}, '
            f'collision={self._kinematics.collision_available}'
        )
        if self._collision_required and not self._kinematics.collision_available:
            self.get_logger().error(
                f'self-collision model unavailable: {self._kinematics.collision_error}'
            )

    def _acquire_instance_lock(self):
        """Prevent two controllers from publishing to the same robot suffix."""
        domain = os.environ.get('ROS_DOMAIN_ID', '0')
        safe_suffix = ''.join(
            character if character.isalnum() else '_'
            for character in self._suffix
        )
        safe_domain = ''.join(
            character if character.isalnum() else '_'
            for character in str(domain)
        )
        path = (
            f'/tmp/openarmx_teleop_vr_306_v4_{safe_suffix}'
            f'_domain_{safe_domain}.lock'
        )
        stream = open(path, 'a+', encoding='utf-8')
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            stream.close()
            raise RuntimeError(
                'another independent arm controller is already running for '
                f'suffix={self._suffix}, ROS_DOMAIN_ID={domain}; '
                'do not launch a second teleop stack'
            ) from exc
        stream.seek(0)
        stream.truncate()
        stream.write(f'{os.getpid()}\n')
        stream.flush()
        self._instance_lock_stream = stream
        self._instance_lock_path = path

    def _declare_parameters(self):
        defaults = {
            'topic_suffix': '0_300',
            'control_rate_hz': 100.0,
            'output_control_mode': 'velocity',
            'dry_run': True,
            'allow_waist_in_ik': False,
            # Waist assistance is deliberately outside the redundant arm QP:
            # free waist pitch/yaw lets the optimizer lean backward to improve
            # manipulability.  The deterministic profile only adds positive
            # pitch after the hands are near full extension and locks yaw.
            'waist_follow_profile': 'forward_pitch_only',
            'waist_forward_assist_start_x_m': 0.28,
            'waist_forward_assist_full_x_m': 0.52,
            'waist_forward_assist_start_extension_m': 0.20,
            'waist_forward_assist_full_extension_m': 0.26,
            'waist_forward_assist_max_lean_deg': 10.0,
            'waist_forward_assist_upright_pitch_deg': 0.0,
            'waist_forward_assist_command_lead_deg': 8.0,
            'body_height_control_enabled': False,
            'body_height_lowering_rate_m_sec': 0.10,
            'body_height_command_timeout_sec': 0.25,
            'body_height_hold_command_lead_deg': 2.0,
            'body_height_motion_command_lead_deg': 6.0,
            'body_height_command_lead_slew_deg_sec': 60.0,
            'authority_heartbeat_topic': '',
            'authority_heartbeat_timeout_sec': 1.0,
            'follow_authority_topic': '',
            'head_follow_enabled': False,
            # Navigation teleoperation can make enable/reset a deliberate
            # head-lock boundary. Standalone teleoperation keeps its legacy
            # behavior unless its launch file explicitly opts in.
            'lock_head_follow_on_enable_reset': False,
            'enable_self_collision_check': False,
            # 306 vendor services keep idle publisher endpoints registered.
            # Endpoint-only rejection is therefore opt-in; message-level
            # active-command arbitration remains enabled by default.
            'reject_competing_publishers': False,
            'enable_active_command_arbitration': True,
            'external_command_quiet_sec': 1.0,
            'own_command_echo_timeout_sec': 1.0,
            'require_hardware_subscriber': True,
            'require_gripper_subscriber': True,
            'require_prepare_subscriber': False,
            # Robot 306 is patched with the same guarded independent SYNC
            # session used on 283.  The vendor side owns the legal
            # ASYNC -> HOME -> SYNC transition and gates official VR input.
            'hardware_session_mode': 'sync',
            'arm_enable_prepare_wait_seconds': 1.5,
            'require_sync_session_service': True,
            'sync_hold_only_enable': False,
            'sync_session_request_timeout_sec': 6.0,
            'require_joint_enable_subscribers': True,
            'output_guard_period_sec': 0.50,
            # Kept for compatibility with the first integrated draft.  The
            # staged parameters below now define the actual enable sequence.
            'hardware_prepare_settle_sec': 3.0,
            'hardware_enable_hold_tolerance_deg': 3.0,
            'hardware_enable_hold_abort_tolerance_deg': 8.0,
            'hardware_enable_stable_delta_deg': 0.20,
            'hardware_enable_stable_seconds': 0.50,
            'hardware_enable_settle_timeout_seconds': 6.0,
            'arm_enable_hold_before_seconds': 0.4,
            'arm_enable_clear_wait_seconds': 1.0,
            'arm_enable_settle_seconds': 1.0,
            'arm_enable_completion_timeout_seconds': 60.0,
            'reset_before_hardware_enable': True,
            'quick_reset_after_hardware_enable': False,
            'full_reset_min_wait_sec': 3.0,
            'full_reset_stable_sec': 1.0,
            'full_reset_position_tolerance_deg': 3.0,
            'full_reset_speed_tolerance_deg_sec': 2.0,
            'vendor_pid_loop_shm_glob': '/dev/shm/pid_loop_state_*',
            'require_teleop_heartbeat': False,
            # Mapper pauses output immediately on stale tracking and requests
            # disable after 5 s.  This slightly longer watchdog is the final
            # fallback if that disable request cannot be delivered.
            'teleop_heartbeat_timeout_sec': 5.5,
            'teleop_vr_input_max_age_sec': 0.80,
            'feedback_timeout_sec': 0.35,
            'target_timeout_sec': 0.15,
            'tracking_error_limit_deg': 22.0,
            # Match the proven old teleop reset path: never let a streamed
            # command run farther ahead of measured hardware than this.
            'arm_max_command_lead_deg': 15.0,
            'waist_max_command_lead_deg': 5.0,
            'neck_max_command_lead_deg': 8.0,
            'quick_reset_max_command_lead_deg': 12.0,
            'quick_reset_waist_max_command_lead_deg': 5.0,
            'joint_limit_margin_deg': 2.0,
            'max_velocity_deg_sec': 180.0,
            'max_acceleration_deg_sec2': 12000.0,
            'velocity_servo_position_gain': 9.0,
            'velocity_servo_feedforward_gain': 0.65,
            'velocity_servo_limit_gain': 8.0,
            'velocity_servo_max_velocity_deg_sec': 110.0,
            'velocity_servo_max_acceleration_deg_sec2': 850.0,
            'velocity_servo_max_jerk_deg_sec3': 4000.0,
            'velocity_servo_goal_rate_filter_tau_sec': 0.04,
            'velocity_servo_collision_horizon_sec': 0.12,
            'hybrid_position_reference_enabled': True,
            'hybrid_position_reference_lookahead_sec': 0.04,
            # Legacy fixed output lookahead is retained as an accepted
            # parameter but adaptive position feed-forward supersedes it.
            'hybrid_position_output_lookahead_sec': 0.08,
            'adaptive_position_feedforward_enabled': True,
            'adaptive_position_min_lookahead_sec': 0.01,
            'adaptive_position_max_lookahead_sec': 0.09,
            'adaptive_position_lookahead_filter_tau_sec': 0.045,
            'adaptive_position_velocity_floor_deg_sec': 5.0,
            'adaptive_position_min_velocity_deg_sec': 90.0,
            'adaptive_position_error_low_deg': 1.0,
            'adaptive_position_error_high_deg': 6.0,
            'adaptive_position_max_acceleration_deg_sec2': 12000.0,
            'direct_position_target_enabled': True,
            'direct_position_max_command_lead_deg': 12.0,
            'dynamic_command_lead_enabled': True,
            'dynamic_command_lead_min_deg': 8.0,
            'dynamic_command_lead_nominal_deg': 12.0,
            'dynamic_command_lead_max_deg': 20.0,
            'dynamic_command_lead_speed_low_deg_sec': 5.0,
            'dynamic_command_lead_speed_high_deg_sec': 70.0,
            'dynamic_command_lead_target_error_low_deg': 2.0,
            'dynamic_command_lead_target_error_high_deg': 24.0,
            'dynamic_command_lead_following_soft_deg': 12.0,
            'dynamic_command_lead_following_hard_deg': 20.0,
            'dynamic_command_lead_filter_tau_sec': 0.08,
            'shoulder_sync_speed_cap_deg_sec': 105.0,
            'ik_target_velocity_filter_tau_sec': 0.04,
            'ik_target_velocity_limit_deg_sec': 180.0,
            'quick_reset_max_velocity_deg_sec': 23.4,
            'quick_reset_max_acceleration_deg_sec2': 65.0,
            'quick_reset_max_jerk_deg_sec3': 325.0,
            'quick_reset_neck_max_velocity_deg_sec': 30.0,
            'quick_reset_neck_max_acceleration_deg_sec2': 83.333,
            'quick_reset_neck_max_jerk_deg_sec3': 416.667,
            'gripper_min_position': 10.0,
            'gripper_max_position': 330.0,
            # The mapper is the single authoritative gripper smoother.  Zero
            # disables this legacy second limiter to avoid staircase chasing.
            'gripper_max_step_per_input': 0.0,
            'ik_max_iterations': 120,
            'quest_placo_frame_weight': 1.0,
            'quest_placo_manipulability_weight': 0.05,
            'quest_placo_manipulability_min_weight': 0.008,
            'quest_placo_manipulability_fade_start_elbow_deg': 50.0,
            'quest_placo_manipulability_fade_full_elbow_deg': 25.0,
            'quest_placo_kinetic_regularization': 1.0e-6,
            'ik_position_tolerance_m': 0.004,
            'ik_orientation_tolerance_rad': 0.035,
            'ik_step_limit_rad': 0.10,
            'ik_damping': 0.025,
            'ik_centering_gain': 0.025,
            'ik_inward_fallback_enabled': True,
            'ik_fallback_max_iterations': 80,
            'ik_fallback_orientation_tolerance_rad': 0.12,
            'ik_fallback_orientation_weight': 0.28,
            'ik_fallback_damping': 0.045,
            'ik_forward_reach_boundary_enabled': True,
            'ik_forward_reach_boundary_position_tolerance_m': 0.012,
            'ik_forward_reach_boundary_max_elbow_deg': 25.0,
            'ik_forward_reach_boundary_max_singular_value': 0.02,
            'input_target_topic': '/openarmx_teleop_vr_306_v4/eef_target',
            'input_gripper_topic': '/openarmx_teleop_vr_306_v4/gripper_target',
            'input_release_topic': '/openarmx_teleop_vr_306_v4/release_hold',
            'input_joint_target_topic': '/openarmx_teleop_vr_306_v4/joint_target',
            'input_body_height_topic': '/openarmx_teleop_vr_306_v4/body_height_command',
            'input_head_target_topic': '/openarmx_teleop_vr_306_v4/head_target',
            'head_target_timeout_sec': 0.60,
            'preview_topic': '/openarmx_teleop_vr_306_v4/joint_target_preview',
            'status_topic': '/openarmx_teleop_vr_306_v4/status',
            'eef_feedback_topic': '/openarmx_teleop_vr_306_v4/eef_feedback',
            'teleop_heartbeat_topic': '/openarmx_teleop_vr_306_v4/teleop_heartbeat',
            'quick_reset_left_arm_joints': [
                0.0, 0.0, 0.0, 110.0, 0.0, 13.0, 0.0,
            ],
            'quick_reset_right_arm_joints': [
                0.0, 0.0, 0.0, -110.0, 0.0, -13.0, 0.0,
            ],
            'quick_reset_waist_joints': [0.0, 0.0],
            # Active vendor/system reset pose on robot 306 (roll, pitch, yaw).
            'quick_reset_neck_joints': [0.0, 0.0, 0.0],
            'quick_reset_open_grippers': True,
            'quick_reset_gripper_open_position': 10.0,
            'quick_reset_tolerance_deg': 1.0,
            'quick_reset_timeout_sec': 20.0,
            # X+A follows one direct quintic sweep.  The complete direct path
            # is validated before motion; an unsafe path is rejected rather
            # than silently adding a visible shoulder detour.
            'quick_reset_path_sample_count': 41,
            'quick_reset_minimum_eef_separation_m': 0.24,
            'quick_reset_minimum_path_duration_sec': 1.0,
            'urdf_path': '',
            'srdf_path': '',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    @staticmethod
    def _group_vector(groups):
        return np.concatenate([
            np.asarray(groups['leg_waist']),
            np.asarray(groups['left_arm']),
            np.asarray(groups['right_arm']),
            np.asarray(groups['neck']),
        ])

    @staticmethod
    def _vector_groups(vector, feedback):
        return {
            'leg_waist': np.asarray(vector[0:4]),
            'left_arm': np.asarray(vector[4:11]),
            'right_arm': np.asarray(vector[11:18]),
            'neck': np.asarray(vector[18:21]),
            'left_gripper': feedback.left_gripper.copy(),
            'right_gripper': feedback.right_gripper.copy(),
        }

    def _quick_reset_limit_vector(self, common_parameter, neck_parameter):
        """Return reset limits with a separate neck-only speed tier."""
        limits = np.full(
            21,
            float(self.get_parameter(common_parameter).value),
            dtype=float,
        )
        limits[18:21] = float(self.get_parameter(neck_parameter).value)
        return limits

    def _make_limiter(self, groups):
        q = self._kinematics.q_from_feedback(groups)
        lower_groups = self._kinematics.groups_from_q_deg(
            self._kinematics.lower, groups
        )
        upper_groups = self._kinematics.groups_from_q_deg(
            self._kinematics.upper, groups
        )
        lower = self._group_vector(lower_groups)
        upper = self._group_vector(upper_groups)
        acceleration_ceiling = float(
            self.get_parameter('max_acceleration_deg_sec2').value
        )
        if bool(self.get_parameter(
                'adaptive_position_feedforward_enabled').value):
            acceleration_ceiling = max(
                acceleration_ceiling,
                float(self.get_parameter(
                    'adaptive_position_max_acceleration_deg_sec2').value),
            )
        limiter = TrajectoryLimiter(
            lower,
            upper,
            float(self.get_parameter('max_velocity_deg_sec').value),
            acceleration_ceiling,
        )
        limiter.reset(self._group_vector(groups))
        return limiter

    def _make_velocity_servo(self, groups):
        lower_groups = self._kinematics.groups_from_q_deg(
            self._kinematics.lower, groups
        )
        upper_groups = self._kinematics.groups_from_q_deg(
            self._kinematics.upper, groups
        )
        servo = VelocityServo(
            self._group_vector(lower_groups),
            self._group_vector(upper_groups),
            float(self.get_parameter(
                'velocity_servo_max_velocity_deg_sec').value),
            float(self.get_parameter(
                'velocity_servo_max_acceleration_deg_sec2').value),
            float(self.get_parameter('velocity_servo_position_gain').value),
            float(self.get_parameter('velocity_servo_feedforward_gain').value),
            float(self.get_parameter('velocity_servo_limit_gain').value),
            float(self.get_parameter(
                'velocity_servo_max_jerk_deg_sec3').value),
            float(self.get_parameter(
                'velocity_servo_goal_rate_filter_tau_sec').value),
        )
        servo.reset(self._group_vector(groups))
        return servo

    def _build_motion_controller_state(self, groups):
        """Construct a fresh motion-history bundle without mutating the node.

        Quick reset uses this as the prepare half of a two-phase state change.
        A bad parameter or invalid measured pose must not partially replace the
        limiter and then leave the old clutch/session mailbox invalidated.
        """
        position = self._group_vector(groups).copy()
        limiter = self._make_limiter(groups)
        velocity_servo = self._make_velocity_servo(groups)

        adaptive_feedforward = AdaptivePositionFeedforward(21, range(4, 18))
        adaptive_lookahead = np.zeros(21, dtype=float)
        adaptive_velocity_limit = np.full(
            21,
            float(self.get_parameter('max_velocity_deg_sec').value),
            dtype=float,
        )
        adaptive_acceleration = np.full(
            21,
            float(self.get_parameter('max_acceleration_deg_sec2').value),
            dtype=float,
        )
        adaptive_following_error = np.zeros(21, dtype=float)

        dynamic_command_lead = DynamicCommandLead(21, range(4, 18))
        dynamic_lead_values = dynamic_command_lead.reset(
            float(self.get_parameter('dynamic_command_lead_nominal_deg').value)
        )
        dynamic_following_error = np.zeros(21, dtype=float)

        target_velocity_estimator = JointTargetVelocityEstimator(
            21, range(4, 18)
        )
        target_velocity_estimate = target_velocity_estimator.reset(
            position, time.monotonic()
        )
        return {
            'limiter': limiter,
            'velocity_servo': velocity_servo,
            'last_velocity_command': np.zeros(21, dtype=float),
            'last_position_command': position,
            'adaptive_feedforward': adaptive_feedforward,
            'adaptive_lookahead': adaptive_lookahead,
            'adaptive_velocity_limit': adaptive_velocity_limit,
            'adaptive_acceleration': adaptive_acceleration,
            'adaptive_following_error': adaptive_following_error,
            'dynamic_command_lead': dynamic_command_lead,
            'dynamic_lead_values': dynamic_lead_values,
            'dynamic_following_error': dynamic_following_error,
            'target_velocity_estimator': target_velocity_estimator,
            'target_velocity_estimate': target_velocity_estimate,
        }

    def _commit_motion_controller_state(self, state):
        """Atomically install a bundle prepared by the method above."""
        self._limiter = state['limiter']
        self._velocity_servo = state['velocity_servo']
        self._last_velocity_command = state['last_velocity_command']
        self._last_position_command = state['last_position_command']
        self._adaptive_feedforward = state['adaptive_feedforward']
        self._adaptive_lookahead = state['adaptive_lookahead']
        self._adaptive_velocity_limit = state['adaptive_velocity_limit']
        self._adaptive_acceleration = state['adaptive_acceleration']
        self._adaptive_following_error = state['adaptive_following_error']
        self._dynamic_command_lead = state['dynamic_command_lead']
        self._dynamic_lead_values = state['dynamic_lead_values']
        self._dynamic_following_error = state['dynamic_following_error']
        self._target_velocity_estimator = state['target_velocity_estimator']
        self._target_velocity_estimate = state['target_velocity_estimate']

    def _reset_motion_controllers(self, groups):
        state = self._build_motion_controller_state(groups)
        self._commit_motion_controller_state(state)

    def _lock_head_follow_for_control_boundary_locked(self):
        """Drop head-follow ownership at a validated enable/reset boundary."""
        self._cancel_task_head_locked('reset')
        if not bool(
            self.get_parameter('lock_head_follow_on_enable_reset').value
        ):
            return
        self._head_follow_enabled = False
        self._head_target = None
        self._head_target_time = 0.0

    def _begin_fresh_teleop_session_locked(self, groups):
        """Drop every motion state belonging to an earlier clutch/reset run."""
        # Prepare first.  If a parameter or measured pose is invalid, preserve
        # the still-running session instead of invalidating its IK mailbox and
        # then failing halfway through the transition.
        motion_state = self._build_motion_controller_state(groups)
        self._latest_target_mailbox.reset()
        self._commit_motion_controller_state(motion_state)
        self._grip_release_held = {'left', 'right'}
        self._clutch_session = None
        self._clutch_sequence = -1
        self._waist_follow_anchor_x = {'left': None, 'right': None}
        self._waist_follow_extension_m = 0.0
        self._velocity_active_sides.clear()
        self._target_groups = {
            key: np.asarray(value, dtype=float).copy()
            for key, value in groups.items()
        }
        self._target_time = 0.0
        self._target_source = 'fresh_session_measured_hold'
        self._last_ik = None
        if self._waist_follow_active() and self._waist_follow_profile == 'forward_pitch_only':
            waist = np.asarray(groups['leg_waist'], dtype=float)
            self._waist_follow_neutral_pitch_deg = float(waist[2])
            self._waist_follow_locked_yaw_deg = float(waist[3])
            self._waist_follow_pitch_target_deg = float(waist[2])

    def _measured_velocity_vector(self):
        """Return fresh vendor joint-speed feedback in controller order."""
        if self._feedback_motion is None:
            return np.zeros(21, dtype=float)
        return np.concatenate([
            np.asarray(self._feedback_motion.speeds['leg_waist'], dtype=float),
            np.asarray(self._feedback_motion.speeds['left_arm'], dtype=float),
            np.asarray(self._feedback_motion.speeds['right_arm'], dtype=float),
            np.asarray(self._feedback_motion.speeds['neck'], dtype=float),
        ])

    def _publish_sync_session_heartbeat(self):
        """Keep the vendor SYNC lease alive even while the web clutch is open."""
        if not self._sync_session_acquired:
            return
        try:
            self._sync_session_heartbeat_pub.publish(String(data=json.dumps({
                'active': True,
                'pid': os.getpid(),
                'monotonic_time': time.monotonic(),
            }, separators=(',', ':'))))
        except Exception as exc:
            self.get_logger().error(f'failed to publish SYNC-session heartbeat: {exc}')

    def _request_sync_session_exit(self):
        """Ask the vendor bridge to hold measured pose and return SYNC to ASYNC."""
        if not self._sync_session_acquired:
            return
        try:
            payload = json.dumps({
                'enable': False,
                'pid': os.getpid(),
                'reason': 'independent teleop controller exiting',
            }, separators=(',', ':'))
            # A short reliable burst makes normal Ctrl+C teardown robust.  The
            # vendor-side heartbeat watchdog remains the crash/SIGKILL fallback.
            for _ in range(3):
                self._sync_session_command_pub.publish(String(data=payload))
        finally:
            self._sync_session_acquired = False
            self._sync_session_future = None

    def _active_velocity_indices(self):
        indices = []
        if 'left' in self._velocity_active_sides:
            indices.extend(range(4, 11))
        if 'right' in self._velocity_active_sides:
            indices.extend(range(11, 18))
        return indices

    @staticmethod
    def _velocity_payload(vector):
        vector = np.asarray(vector, dtype=float)
        return {
            'leg_waist_target_joints_velocity': [float(x) for x in vector[0:4]],
            'left_arm_target_joints_velocity': [float(x) for x in vector[4:11]],
            'right_arm_target_joints_velocity': [float(x) for x in vector[11:18]],
        }

    def _publish_zero_velocity(self):
        """Immediately overwrite any previous SPEED command with all zeros."""
        if self._velocity_servo is not None:
            command = self._velocity_servo.stop()
        else:
            command = np.zeros(21, dtype=float)
        self._publish_velocity_command(command)

    def _publish_velocity_command(self, command):
        command = np.asarray(command, dtype=float)
        self._last_velocity_command = command.copy()
        if self._velocity_pub is not None:
            self._publish_owned_command(
                self._velocity_pub,
                self._velocity_output_topic,
                json.dumps(self._velocity_payload(command), separators=(',', ':')),
            )

    @staticmethod
    def _reset_vector(groups):
        """Vector of joints intentionally moved by quick reset."""
        return np.concatenate([
            np.asarray(groups['left_arm'], dtype=float),
            np.asarray(groups['right_arm'], dtype=float),
            np.asarray(groups['neck'], dtype=float),
        ])

    @staticmethod
    def _maximum_reset_difference(current, reference):
        difference = np.abs(
            IndependentArmController._reset_vector(current)
            - IndependentArmController._reset_vector(reference)
        )
        offset = int(np.argmax(difference))
        return float(difference[offset]), RESET_TRACKED_JOINT_NAMES[offset]

    @staticmethod
    def _enable_vector(groups):
        """Only joints owned by arm takeover may gate arm enable.

        Head following is optional and becomes active only after takeover and
        quick reset.  Neck settling must therefore never reject acquisition of
        the two arms.  Ankle/knee and grippers are likewise outside this guard.
        """
        return np.concatenate([
            np.asarray(groups['left_arm'], dtype=float),
            np.asarray(groups['right_arm'], dtype=float),
            np.asarray(groups['leg_waist'], dtype=float)[2:4],
        ])

    @staticmethod
    def _maximum_enable_difference(current, reference):
        difference = np.abs(
            IndependentArmController._enable_vector(current)
            - IndependentArmController._enable_vector(reference)
        )
        offset = int(np.argmax(difference))
        return float(difference[offset]), ENABLE_TRACKED_JOINT_NAMES[offset]

    @staticmethod
    def _reset_required_vector(groups):
        """Joints that must finish before reset can hand control to VR.

        X+A and post-enable reset both command the configured neck default, so
        all three neck axes are completion gates together with both arms.
        X+A includes the three coordinated height joints. Y+B keeps those
        goals at their measured values, so it does not request standing up.
        This prevents active head-follow from taking over before the physical
        neck has actually reached its reset pose.
        """
        return np.concatenate([
            IndependentArmController._reset_vector(groups),
            np.asarray(groups['leg_waist'], dtype=float)[:3],
        ])

    def _configuration_guard_error(
        self, groups, label, *, limit_joint_names=None
    ):
        """Validate a measured or requested pose without silently clipping it."""
        try:
            q = self._kinematics.q_from_feedback(groups, clip=False)
        except Exception as exc:
            return f'{label} is invalid: {exc}', None
        if limit_joint_names is None:
            indices = np.arange(q.size, dtype=int)
            names_by_index = {
                int(index): name
                for name, index in self._kinematics.joint_q.items()
            }
        else:
            indices = np.asarray([
                self._kinematics.joint_q[name] for name in limit_joint_names
            ], dtype=int)
            names_by_index = {
                int(self._kinematics.joint_q[name]): name
                for name in limit_joint_names
            }
        below = np.maximum(
            self._kinematics.lower[indices] - q[indices], 0.0
        )
        above = np.maximum(
            q[indices] - self._kinematics.upper[indices], 0.0
        )
        per_joint = np.maximum(below, above)
        maximum_offset = int(np.argmax(per_joint))
        violation = float(per_joint[maximum_offset])
        if violation > 1e-6:
            configuration_index = int(indices[maximum_offset])
            joint_name = names_by_index.get(
                configuration_index, f'configuration[{configuration_index}]'
            )
            return (
                f'{label} joint {joint_name} is outside the configured soft '
                f'limits by {np.rad2deg(violation):.3f} deg',
                q,
            )
        if self._collision_required and self._kinematics.collision_available:
            try:
                collision_q = q
                if limit_joint_names is not None:
                    # Feedback-only joints are not commanded by this teleop.
                    # Evaluate them at the closest configuration supported by
                    # the simplified collision model, matching the runtime
                    # command check.  The controlled arms/waist were verified
                    # above and therefore remain at their exact measured pose.
                    collision_q = np.clip(
                        q,
                        self._kinematics.lower,
                        self._kinematics.upper,
                    )
                collisions = self._kinematics.collision_pairs(collision_q)
            except Exception as exc:
                return f'{label} collision check failed: {exc}', q
            if collisions:
                return f'{label} is in self collision: ' + ', '.join(collisions), q
        return '', q

    @staticmethod
    def _kinematics_groups_from_vector(vector):
        vector = np.asarray(vector, dtype=float).reshape(-1)
        if vector.size != 21:
            raise ValueError('joint vector must contain 21 values')
        return {
            'leg_waist': vector[0:4],
            'left_arm': vector[4:11],
            'right_arm': vector[11:18],
            'neck': vector[18:21],
        }

    def _prepare_quick_reset_path(self, measured_groups, goal_groups):
        """Validate the complete direct reset sweep before motion."""
        if not self._kinematics.collision_available:
            return 'quick reset collision model is unavailable; no motion started', None
        start = self._group_vector(measured_groups).astype(float, copy=True)
        goal = self._group_vector(goal_groups).astype(float, copy=True)
        sample_count = max(
            11, int(self.get_parameter('quick_reset_path_sample_count').value)
        )
        required_separation = max(
            0.0,
            float(self.get_parameter(
                'quick_reset_minimum_eef_separation_m').value),
        )
        try:
            start_q = self._kinematics.q_from_feedback(
                self._kinematics_groups_from_vector(start), clip=False
            )
            start_poses = self._kinematics.eef_poses(start_q)
            start_separation = float(np.linalg.norm(
                np.asarray(start_poses['left']['position'], dtype=float)
                - np.asarray(start_poses['right']['position'], dtype=float)
            ))
        except Exception as exc:
            return f'quick reset path initialization failed: {exc}', None
        # The measured starting pose cannot retroactively satisfy a larger
        # requested gap, so guard against making an already-close pair worse.
        separation_floor = max(
            0.0, min(required_separation, start_separation - 0.005)
        )
        for progress in np.linspace(0.0, 1.0, sample_count):
            sample = continuous_direct_reset(start, goal, progress)
            groups = self._kinematics_groups_from_vector(sample)
            guard_error, q = self._configuration_guard_error(
                groups,
                'quick reset direct path',
                limit_joint_names=ARM_JOINT_NAMES + list(BODY_HEIGHT_JOINT_NAMES),
            )
            if guard_error:
                return f'direct quick-reset path is unsafe: {guard_error}', None
            try:
                # Teleoperation may disable per-frame self-collision rejection,
                # but autonomous X+A motion always checks its whole path.
                if self._kinematics.collision_available:
                    collisions = self._kinematics.collision_pairs(q)
                    if collisions:
                        return (
                            'direct quick-reset path is in collision: '
                            + ', '.join(collisions)
                        ), None
                poses = self._kinematics.eef_poses(q)
                separation = float(np.linalg.norm(
                    np.asarray(poses['left']['position'], dtype=float)
                    - np.asarray(poses['right']['position'], dtype=float)
                ))
            except Exception as exc:
                return f'direct quick-reset FK validation failed: {exc}', None
            if separation + 1.0e-9 < separation_floor:
                return (
                    'direct quick-reset path brings the grippers too close '
                    f'({separation:.3f} m < {separation_floor:.3f} m)'
                ), None

        try:
            duration = conservative_reset_duration(
                start,
                goal,
                float(self.get_parameter(
                    'quick_reset_max_velocity_deg_sec').value),
                float(self.get_parameter(
                    'quick_reset_neck_max_velocity_deg_sec').value),
                float(self.get_parameter(
                    'quick_reset_minimum_path_duration_sec').value),
            )
        except Exception as exc:
            return f'quick reset duration calculation failed: {exc}', None
        return '', {
            'start': start,
            'goal': goal,
            'duration_sec': duration,
        }

    def _on_authority_heartbeat(self, message):
        if message.data == 'hg_dagger_alive':
            self._authority_heartbeat_time = time.monotonic()

    def _heartbeat_guard_error(self, now, *, require_fresh=False):
        if getattr(self, '_authority_heartbeat_required', False):
            age = now - self._authority_heartbeat_time
            if self._authority_heartbeat_time <= 0 or not 0 <= age <= float(
                    self.get_parameter('authority_heartbeat_timeout_sec').value):
                return 'HG-DAgger supervisor heartbeat missing or expired'
            return ''
        if not bool(self.get_parameter('require_teleop_heartbeat').value):
            return ''
        timeout = max(
            0.05,
            float(self.get_parameter('teleop_heartbeat_timeout_sec').value),
        )
        if self._teleop_heartbeat_time <= 0.0:
            return self._teleop_heartbeat_rejection
        if now - self._teleop_heartbeat_time > timeout:
            return 'teleoperation mapper heartbeat expired; hardware output stopped'
        if require_fresh and self._teleop_heartbeat_mode != 'tracking':
            return 'fresh VR controller data is required; enter VR and wake a controller'
        return ''

    def _set_hardware_fault(self, reason):
        """Fail closed while leaving the vendor motor layer holding its last target."""
        if self._output_control_mode == 'velocity':
            self._publish_zero_velocity()
        self._velocity_active_sides.clear()
        self._hardware_enabled = False
        self._enable_pending = False
        self._enable_hold_groups = None
        self._enable_stage = None
        self._enable_stable_since = None
        self._enable_last_feedback_groups = None
        self._enable_post_sync_rebased = False
        self._full_reset_stable_since = None
        self._reset_active = False
        self._deactivate_body_height_locked()
        self._gripper_dirty = {'left': False, 'right': False}
        # A stopped controller must not keep a process-local SYNC lease alive.
        # The vendor exit path first holds fresh measured positions and then
        # performs the legal SYNC/HOME -> ASYNC transition.
        self._request_sync_session_exit()
        self._state = 'FAULT'
        self._reason = reason
        self._last_hardware_fault_reason = reason

    def _on_feedback(self, message):
        try:
            payload = json.loads(message.data)
            feedback = parse_joint_feedback(payload)
        except Exception as exc:
            # Keep the last valid sample.  The feedback watchdog will fail
            # closed if malformed packets persist, while one bad packet does
            # not leave state=FAULT with hardware_enabled=true.
            with self._lock:
                self._reason = f'invalid joint feedback ignored: {exc}'
            return
        try:
            feedback_motion = parse_body_motion_feedback(payload)
        except Exception:
            # Basic position feedback remains useful in dry-run mode.  A real
            # full-reset enable will explicitly wait for the richer packet.
            feedback_motion = None
        now = time.monotonic()
        publish_fk = False
        with self._lock:
            self._feedback = feedback
            self._feedback_motion = feedback_motion
            self._feedback_time = now
            if self._limiter is None:
                groups = feedback.as_dict()
                try:
                    self._reset_motion_controllers(groups)
                except Exception as exc:
                    self._state = 'FAULT'
                    self._reason = f'trajectory limiter initialization failed: {exc}'
                else:
                    self._target_groups = {
                        key: value.copy() for key, value in groups.items()
                    }
                    self._gripper_targets = {
                        'left': float(feedback.left_gripper[0]),
                        'right': float(feedback.right_gripper[0]),
                    }
            if self._state == 'WAITING_FEEDBACK' and self._limiter is not None:
                self._state = 'DRY_RUN' if self._dry_run else 'DISARMED'
                self._reason = 'feedback connected; waiting for target'
            publish_fk = now - self._last_eef_feedback_publish >= 0.02

        if publish_fk:
            try:
                # FK must reflect the measured configuration.  Do not clip it
                # to the command soft limits before publishing feedback.
                q = self._kinematics.q_from_feedback(
                    feedback.as_dict(), clip=False
                )
                poses = self._kinematics.eef_poses(q)
                payload = {
                    'pos_left_in_robot': poses['left']['position'],
                    'quat_left_in_robot': poses['left']['orientation'],
                    'pos_right_in_robot': poses['right']['position'],
                    'quat_right_in_robot': poses['right']['orientation'],
                }
                self._eef_feedback_pub.publish(
                    String(data=json.dumps(payload, separators=(',', ':')))
                )
                with self._lock:
                    self._last_eef_feedback_publish = now
            except Exception as exc:
                with self._lock:
                    should_warn = now - self._last_fk_warning >= 1.0
                    if should_warn:
                        self._last_fk_warning = now
                if should_warn:
                    self.get_logger().warning(
                        f'independent FK feedback was not published: {exc}'
                    )

    def _on_target(self, message):
        """Overwrite the single pending Cartesian target without solving IK."""
        now = time.monotonic()
        try:
            envelope = json.loads(message.data)
            if not isinstance(envelope, dict):
                raise ValueError('target envelope must be a JSON object')
        except Exception as exc:
            with self._lock:
                self._reason = f'target rejected: {exc}'
            return
        with self._lock:
            if (not self._follow_authority_allowed(for_arm=True)
                    or not self._collector_target_allowed(envelope, policy_only=False)):
                return  # Reject old clutch state before it mutates the controller.

            if self._state == 'FAULT' and not self._hardware_enabled:
                return
            if self._reset_active:
                self._reason = 'Cartesian target ignored while quick reset is active'
                return
            if self._enable_pending:
                self._reason = 'Cartesian target ignored while hardware enable is pending'
                return
            if self._estop_latched:
                self._reason = 'Cartesian target ignored while emergency stop is latched'
                return
            if self._feedback is None or now - self._feedback_time > float(
                self.get_parameter('feedback_timeout_sec').value):
                self._reason = 'target rejected: joint feedback is stale'
                return
            try:
                self._apply_clutch_envelope_locked(envelope, now)
            except Exception as exc:
                self._reason = f'target rejected: {exc}'
                return
        # The callback deliberately performs no JSON parsing and no IK.  DDS
        # depth one plus this explicit one-slot mailbox means there is nowhere
        # for an old VR pose to queue up.
        self._latest_target_mailbox.put((str(message.data), now))

    def _apply_clutch_envelope_locked(self, payload, now):
        """Apply a complete, sequenced clutch state before any IK is solved."""
        if 'clutch_state' not in payload:
            return
        session = payload.get('clutch_session')
        sequence = payload.get('clutch_sequence')
        state = payload.get('clutch_state')
        if not isinstance(session, str) or not session:
            raise ValueError('clutch_session must be a non-empty string')
        if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
            raise ValueError('clutch_sequence must be a non-negative integer')
        if not isinstance(state, dict) or set(state) != {'left', 'right'}:
            raise ValueError('clutch_state must contain exactly left and right')
        if any(not isinstance(state[side], bool) for side in ('left', 'right')):
            raise ValueError('clutch_state values must be booleans')
        new_session = session != self._clutch_session
        if session != self._clutch_session:
            # A mapper/browser restart begins fail-closed: both arms remain held
            # until the new session supplies a successful re-anchor target.
            self._clutch_session = session
            self._clutch_sequence = -1
            self._grip_release_held.clear()
            self._waist_follow_anchor_x = {'left': None, 'right': None}
            self._waist_follow_extension_m = 0.0
        if sequence <= self._clutch_sequence:
            raise ValueError('stale clutch sequence')
        self._clutch_sequence = sequence
        released = [
            side for side in ('left', 'right')
            if new_session or (
                not state[side] and side not in self._grip_release_held
            )
        ]
        if released:
            self._hold_sides_locked(released, now)

    def _ik_worker_loop(self):
        while True:
            sample = self._latest_target_mailbox.take()
            if sample is None:
                return
            sequence, (message_data, received_at) = sample
            self._process_latest_target(sequence, message_data, received_at)

    def _process_latest_target(self, sequence, message_data, received_at):
        now = time.monotonic()
        with self._lock:
            if (
                (self._state == 'FAULT' and not self._hardware_enabled)
                or self._reset_active
                or self._enable_pending
                or self._estop_latched
            ):
                return
            feedback = self._feedback
            feedback_time = self._feedback_time
        if feedback is None or now - feedback_time > float(
                self.get_parameter('feedback_timeout_sec').value):
            with self._lock:
                self._reason = 'target rejected: joint feedback is stale'
            return
        try:
            payload = json.loads(message_data)
            has_pose = any(
                f'pos_{side}_in_robot' in payload
                or f'quat_{side}_in_robot' in payload
                for side in ('left', 'right')
            )
            if not has_pose:
                return
            targets = parse_eef_target(payload)
            reanchor_sides = payload.get('reanchor_sides', [])
            if not isinstance(reanchor_sides, list) or not all(
                    side in ('left', 'right') for side in reanchor_sides):
                raise ValueError('reanchor_sides must contain only left/right')
            with self._lock:
                clutch_state = payload.get('clutch_state')
                if isinstance(clutch_state, dict):
                    # The mapper re-latches at measured FK before publishing
                    # any active target.  While controller status is travelling
                    # back at 10 Hz, later 90 Hz frames may no longer carry the
                    # one-frame reanchor marker.  Treat a held+active side in
                    # the authoritative clutch session as the same reanchor;
                    # otherwise a newer frame would forever invalidate the
                    # first IK solve before it can release the hold.
                    reanchor_sides = list(dict.fromkeys(
                        list(reanchor_sides) + [
                            side for side in targets
                            if clutch_state.get(side) is True
                            and side in self._grip_release_held
                        ]
                    ))
                # A held side may be solved only as an explicit re-anchor.  It
                # is not released until that IK result has succeeded.
                targets = {
                    side: pose for side, pose in targets.items()
                    if side not in self._grip_release_held
                    or side in reanchor_sides
                }
                waist_follow_enabled = self._waist_follow_active()
                waist_follow_profile = self._waist_follow_profile
                waist_neutral_pitch = self._waist_follow_neutral_pitch_deg
                waist_locked_yaw = self._waist_follow_locked_yaw_deg
                body_height_active = bool(
                    self._body_height_active
                    and self._body_height_joint_targets is not None
                    and now - self._body_height_command_time <= float(
                        self.get_parameter('body_height_command_timeout_sec').value
                    )
                )
                body_height_targets = (
                    None if not body_height_active
                    else np.asarray(self._body_height_joint_targets, dtype=float).copy()
                )
                if (
                    body_height_active
                    and waist_follow_enabled
                    and waist_follow_profile == 'forward_pitch_only'
                ):
                    # The height profile supplies the new neutral torso pitch.
                    # Forward-reach assistance is then layered on top instead
                    # of being disabled for the rest of the chassis session.
                    waist_neutral_pitch = float(body_height_targets[2])
            if not targets:
                return
            groups = feedback.as_dict()
            # Quest3 refreshes the QP state from measured joints every cycle.
            # Do not seed from an earlier accepted command, because that would
            # turn one-step Placo output back into V3's target-continuity IK.
            seed_groups = groups
            waist_pitch_target = None
            waist_yaw_target = None
            if waist_follow_enabled and waist_follow_profile == 'forward_pitch_only':
                # Solve the arms at the current body-height neutral, then layer waist
                # pitch onto the accepted joint groups below. Solving against
                # the already-leaning torso makes IK retract/bend the arms to
                # keep the EEF fixed in base coordinates.
                measured_waist = np.asarray(groups['leg_waist'], dtype=float)
                if waist_neutral_pitch is None:
                    waist_neutral_pitch = float(measured_waist[2])
                if waist_locked_yaw is None:
                    waist_locked_yaw = float(measured_waist[3])
                with self._lock:
                    anchor_x = dict(self._waist_follow_anchor_x)
                for side, pose in targets.items():
                    target_x = float(np.asarray(pose[0], dtype=float)[0])
                    if side in reanchor_sides or anchor_x.get(side) is None:
                        anchor_x[side] = target_x
                forward_extension = max(
                    max(0.0, float(np.asarray(pose[0], dtype=float)[0])
                        - float(anchor_x[side]))
                    for side, pose in targets.items()
                )
                waist_pitch_target = forward_reach_to_waist_pitch(
                    forward_extension,
                    waist_neutral_pitch,
                    self.get_parameter(
                        'waist_forward_assist_start_extension_m').value,
                    self.get_parameter(
                        'waist_forward_assist_full_extension_m').value,
                    min(float(self.get_parameter('waist_forward_assist_max_lean_deg').value),
                        max(0.0, float(np.rad2deg(self._ik_kinematics.upper[
                            self._ik_kinematics.joint_q['Joint_Waist_Pitch']]))
                            - waist_neutral_pitch)),
                    self.get_parameter(
                        'waist_forward_assist_upright_pitch_deg'
                    ).value,
                )
                waist_yaw_target = waist_locked_yaw
                seed_groups = {
                    key: np.asarray(value, dtype=float).copy()
                    for key, value in seed_groups.items()
                }
                seed_groups['leg_waist'][2] = waist_neutral_pitch
                seed_groups['leg_waist'][3] = waist_locked_yaw
                with self._lock:
                    if self._waist_follow_active():
                        self._waist_follow_neutral_pitch_deg = waist_neutral_pitch
                        self._waist_follow_locked_yaw_deg = waist_locked_yaw
                        self._waist_follow_anchor_x = anchor_x
                        self._waist_follow_extension_m = forward_extension
                        self._waist_follow_pitch_target_deg = waist_pitch_target
            seed = self._ik_kinematics.q_from_feedback(seed_groups)
            result = self._ik_kinematics.solve(
                targets,
                seed,
                max_iterations=int(self.get_parameter('ik_max_iterations').value),
                position_tolerance=float(self.get_parameter('ik_position_tolerance_m').value),
                orientation_tolerance=float(self.get_parameter('ik_orientation_tolerance_rad').value),
                step_limit=float(self.get_parameter('ik_step_limit_rad').value),
                damping=float(self.get_parameter('ik_damping').value),
                centering_gain=float(self.get_parameter('ik_centering_gain').value),
            )
            fallback_used = False
            if (
                not result.success
                and result.reason == 'iteration limit reached'
                and bool(self.get_parameter('ik_inward_fallback_enabled').value)
            ):
                # Near the torso a fully constrained wrist orientation can
                # become singular even though its Cartesian position remains
                # reachable.  Continue from the first pass and relax only the
                # orientation objective; hard limits and collision stay on.
                result = self._ik_kinematics.solve(
                    targets,
                    result.q,
                    max_iterations=int(
                        self.get_parameter('ik_fallback_max_iterations').value
                    ),
                    position_tolerance=float(
                        self.get_parameter('ik_position_tolerance_m').value
                    ),
                    orientation_tolerance=float(
                        self.get_parameter(
                            'ik_fallback_orientation_tolerance_rad'
                        ).value
                    ),
                    step_limit=float(self.get_parameter('ik_step_limit_rad').value),
                    damping=float(self.get_parameter('ik_fallback_damping').value),
                    centering_gain=float(
                        self.get_parameter('ik_centering_gain').value
                    ),
                    orientation_weight=float(
                        self.get_parameter('ik_fallback_orientation_weight').value
                    ),
                )
                fallback_used = result.success
            reach_boundary_used = False
            if (
                not result.success
                and bool(self.get_parameter(
                    'ik_forward_reach_boundary_enabled').value)
            ):
                result = self._ik_kinematics.accept_forward_reach_boundary(
                    result,
                    targets,
                    seed,
                    maximum_position_error=float(self.get_parameter(
                        'ik_forward_reach_boundary_position_tolerance_m'
                    ).value),
                    maximum_orientation_error=float(self.get_parameter(
                        'ik_orientation_tolerance_rad').value),
                    maximum_elbow_angle=np.deg2rad(float(self.get_parameter(
                        'ik_forward_reach_boundary_max_elbow_deg').value)),
                    maximum_singular_value=float(self.get_parameter(
                        'ik_forward_reach_boundary_max_singular_value').value),
                )
                reach_boundary_used = result.success
        except Exception as exc:
            with self._lock:
                self._reason = f'target rejected: {exc}'
                if self._hardware_enabled:
                    self._state = 'HOLDING'
            return

        # A newer hand pose may have arrived while Pinocchio was solving.  It
        # is strictly forbidden for this old result to overwrite the new pose.
        if not self._latest_target_mailbox.is_latest(sequence):
            self._latest_target_mailbox.discard_inflight()
            return
        if time.monotonic() - received_at > float(
                self.get_parameter('target_timeout_sec').value):
            self._latest_target_mailbox.discard_inflight()
            with self._lock:
                self._reason = 'stale IK result discarded by latest-frame policy'
            return
        with self._lock:
            if not self._latest_target_mailbox.is_latest(sequence):
                self._latest_target_mailbox.discard_inflight()
                return
            if not self._follow_authority_allowed(for_arm=True):
                return  # EXPERT_READY must accept the first arm IK to complete takeover.
            if not self._collector_target_allowed(payload, policy_only=False):
                return  # A solve from an older authority cannot re-arm this epoch.
            if self._reset_active or self._enable_pending or self._estop_latched:
                self._reason = 'stale IK result discarded after a controller state change'
                return
            self._last_ik = result
            if result.success:
                for side in reanchor_sides:
                    self._grip_release_held.discard(side)
                if reanchor_sides and self._limiter is not None:
                    # The old teleop re-clutches from measured EEF feedback.
                    # Rebase our joint trajectory at the same boundary so a
                    # previous IK goal cannot leak into the new clutch.
                    self._limiter.reset(self._group_vector(groups))
                    if self._velocity_servo is not None:
                        indices = []
                        for side in reanchor_sides:
                            indices.extend(
                                range(4, 11) if side == 'left'
                                else range(11, 18)
                            )
                        self._velocity_servo.rebase_indices(
                            indices, self._group_vector(groups)
                        )
                accepted_groups = self._ik_kinematics.groups_from_q_deg(
                    result.q, groups
                )
                if waist_pitch_target is not None:
                    accepted_groups['leg_waist'][2] = waist_pitch_target
                    accepted_groups['leg_waist'][3] = waist_yaw_target
                if body_height_active:
                    accepted_groups['leg_waist'][0:2] = body_height_targets[0:2]
                    if waist_pitch_target is None:
                        accepted_groups['leg_waist'][2] = body_height_targets[2]
                accepted_vector = self._group_vector(accepted_groups)
                self._target_velocity_estimate = (
                    self._target_velocity_estimator.update(
                        accepted_vector,
                        received_at,
                        filter_tau=float(self.get_parameter(
                            'ik_target_velocity_filter_tau_sec').value),
                        maximum_velocity=float(self.get_parameter(
                            'ik_target_velocity_limit_deg_sec').value),
                    )
                )
                if reanchor_sides:
                    for side in reanchor_sides:
                        indices = list(
                            range(4, 11) if side == 'left' else range(11, 18)
                        )
                        self._target_velocity_estimator.reset_indices(
                            indices, accepted_vector[indices]
                        )
                    self._target_velocity_estimate = (
                        self._target_velocity_estimator.velocity.copy()
                    )
                self._target_groups = accepted_groups
                self._collector_body_origin_epoch = int(payload.get('authority_epoch', -1))
                self._velocity_active_sides = set(targets)
                self._target_time = received_at
                self._target_source = 'cartesian_ik'
                self._reason = (
                    f'IK accepted in {result.solve_time_ms:.1f} ms'
                    + ('; inward position-priority fallback' if fallback_used else '')
                    + ('; proportional forward reach boundary' if reach_boundary_used else '')
                    + ('; re-anchored from measured pose' if reanchor_sides else '')
                )
                if self._hardware_enabled:
                    self._state = 'ARMED'
            else:
                self._reason = f'IK rejected: {result.reason}'
                if self._hardware_enabled:
                    self._state = 'HOLDING'
                    self._velocity_active_sides.clear()
                    self._publish_zero_velocity()

    def _on_release_hold(self, message):
        """Freeze only the arm whose VR grip was released."""
        try:
            payload = json.loads(message.data)
            sides = payload.get('sides') if isinstance(payload, dict) else None
            if not isinstance(sides, list) or not sides:
                raise ValueError('release_hold sides must be a non-empty list')
            if any(side not in ('left', 'right') for side in sides):
                raise ValueError('release_hold sides must contain only left/right')
            sides = list(dict.fromkeys(sides))
        except Exception as exc:
            with self._lock:
                self._reason = f'release hold rejected: {exc}'
            return

        now = time.monotonic()
        with self._lock:
            if self._follow_authority_required:
                release_epoch = payload.get('authority_epoch')
                if (type(release_epoch) is not int
                        or release_epoch < self._collector_authority_epoch):
                    return  # A delayed release cannot interrupt a newer clutch.

            if self._follow_authority_required and self._follow_authority_mode == 'POLICY_ACTIVE':
                # Fence queued policy targets immediately, even if the newer
                # authority-state message is delivered after this hold.
                self._collector_revoked_epoch = self._collector_authority_epoch
            if self._feedback is None or self._limiter is None:
                self._reason = 'release hold rejected: joint feedback is unavailable'
                return
            if now - self._feedback_time > float(
                    self.get_parameter('feedback_timeout_sec').value):
                self._reason = 'release hold rejected: joint feedback is stale'
                return
            if self._reset_active or self._enable_pending or self._estop_latched:
                return
            self._hold_sides_locked(sides, now)

    def _hold_sides_locked(self, sides, now):
        """Freeze released arms at fresh measured joints; caller holds lock."""
        sides = [side for side in sides if side not in self._grip_release_held]
        if not sides:
            return
        measured = self._feedback.as_dict()
        base = self._target_groups or measured
        groups = {
            key: np.asarray(value, dtype=float).copy()
            for key, value in base.items()
        }
        for side in sides:
            self._grip_release_held.add(side)
            self._velocity_active_sides.discard(side)
            groups[f'{side}_arm'] = measured[f'{side}_arm'].copy()
            indices = range(4, 11) if side == 'left' else range(11, 18)
            self._limiter.reset_indices(indices, measured[f'{side}_arm'])
            self._target_velocity_estimator.reset_indices(
                indices, measured[f'{side}_arm']
            )
            self._target_velocity_estimate = (
                self._target_velocity_estimator.velocity.copy()
            )
            if self._velocity_servo is not None:
                self._velocity_servo.rebase_indices(
                    indices, self._group_vector(measured)
                )
        if (
            {'left', 'right'}.issubset(self._grip_release_held)
            and self._waist_follow_active()
            and self._waist_follow_profile == 'forward_pitch_only'
            and self._waist_follow_neutral_pitch_deg is not None
        ):
            # Once both hands release, discard the last forward-assist target.
            # Keep returning to the current height profile's upright baseline
            # instead of freezing the waist at its last partially bent pose.
            neutral_pitch = float(
                self._body_height_joint_targets[2]
                if self._body_height_active and self._body_height_joint_targets is not None
                else self._waist_follow_neutral_pitch_deg
            )
            self._waist_follow_neutral_pitch_deg = neutral_pitch
            groups['leg_waist'][2] = neutral_pitch
            self._waist_follow_pitch_target_deg = neutral_pitch
            self._waist_follow_extension_m = 0.0
            self._waist_follow_anchor_x = {'left': None, 'right': None}
            self._target_velocity_estimator.reset_indices(
                [2], groups['leg_waist'][2:3]
            )
            self._target_velocity_estimate = (
                self._target_velocity_estimator.velocity.copy()
            )
        self._target_groups = groups
        self._target_time = now
        self._target_source = 'grip_release_hold'
        self._reason = 'immediate hold: ' + ', '.join(sides)
        if self._hardware_enabled:
            self._state = 'HOLDING'
            if self._output_control_mode == 'velocity':
                self._publish_velocity_command(self._velocity_servo.velocity)

    def _on_gripper(self, message):
        with self._lock:
            if self._state == 'FAULT' and not self._hardware_enabled:
                return
            if self._reset_active:
                self._reason = 'gripper target ignored while quick reset is active'
                return
            if self._enable_pending:
                self._reason = 'gripper target ignored while hardware enable is pending'
                return
            if self._estop_latched:
                self._reason = 'gripper target ignored while emergency stop is latched'
                return
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError('gripper target must be a JSON object')
            allowed = {}
            minimum = float(self.get_parameter('gripper_min_position').value)
            maximum = float(self.get_parameter('gripper_max_position').value)
            if not np.isfinite(minimum) or not np.isfinite(maximum) or minimum > maximum:
                raise ValueError('configured gripper range is invalid')
            for side in ('left', 'right'):
                key = f'{side}_gripper_target_joints_position'
                if key in payload:
                    values = np.asarray(payload[key], dtype=float).reshape(-1)
                    if values.size != 1 or not np.all(np.isfinite(values)):
                        raise ValueError(f'invalid {key}')
                    requested = float(values[0])
                    if requested < minimum or requested > maximum:
                        raise ValueError(f'{key} is outside [{minimum}, {maximum}]')
                    with self._lock:
                        previous = self._gripper_targets.get(side)
                    if previous is None:
                        previous = requested
                    maximum_step = float(
                        self.get_parameter('gripper_max_step_per_input').value
                    )
                    if maximum_step > 0.0:
                        candidate = previous + float(np.clip(
                            requested - previous,
                            -maximum_step,
                            maximum_step,
                        ))
                    else:
                        candidate = requested
                    allowed[key] = [candidate]
            if not allowed:
                raise ValueError('no gripper target')
            with self._lock:
                if not self._collector_target_allowed(payload, policy_only=False):
                    return
                self._collector_gripper_origin_epoch = int(payload.get('authority_epoch', -1))
                for side in ('left', 'right'):
                    key = f'{side}_gripper_target_joints_position'
                    if key in allowed:
                        self._gripper_targets[side] = float(allowed[key][0])
                        self._gripper_dirty[side] = True
        except Exception as exc:
            with self._lock:
                self._reason = f'gripper target rejected: {exc}'

    def _on_joint_target(self, message):
        """Accept a guarded joint-space target, used by the VR quick reset."""
        fields = {
            'leg_waist_target_joints_position': ('leg_waist', 4),
            'left_arm_target_joints_position': ('left_arm', 7),
            'right_arm_target_joints_position': ('right_arm', 7),
            'neck_target_joints_position': ('neck', 3),
        }
        now = time.monotonic()
        with self._lock:
            if self._state == 'FAULT' and not self._hardware_enabled:
                return
            if self._reset_active:
                self._reason = 'joint target ignored while quick reset is active'
                return
            if self._enable_pending:
                self._reason = 'joint target ignored while hardware enable is pending'
                return
            if self._estop_latched:
                self._reason = 'joint target ignored while emergency stop is latched'
                return
            feedback = self._feedback
            feedback_time = self._feedback_time
            base_groups = self._target_groups
        if feedback is None or now - feedback_time > float(
                self.get_parameter('feedback_timeout_sec').value):
            self._reason = 'joint target rejected: joint feedback is stale'
            return
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError('joint target must be a JSON object')
            with self._lock:
                if not self._collector_target_allowed(payload, policy_only=True):
                    return
            base = feedback.as_dict() if base_groups is None else base_groups
            groups = {
                key: np.asarray(value, dtype=float).copy()
                for key, value in base.items()
            }
            changed = []
            for field, (group, expected) in fields.items():
                if field not in payload:
                    continue
                values = np.asarray(payload[field], dtype=float).reshape(-1)
                if values.size != expected or not np.all(np.isfinite(values)):
                    raise ValueError(f'{field}: expected {expected} finite values')
                groups[group] = values
                changed.append(group)
            if not changed:
                raise ValueError('joint target contains no supported joint group')
            guard_error, q = self._configuration_guard_error(
                groups, 'joint-space target'
            )
            if guard_error:
                raise ValueError(guard_error)
            clamped = self._kinematics.groups_from_q_deg(q, groups)
        except Exception as exc:
            self._reason = f'joint target rejected: {exc}'
            return
        with self._lock:
            # Expensive kinematic checks ran outside the lock. Recheck before
            # committing so a takeover/reset cannot be undone by an old result.
            if (not self._collector_target_allowed(payload, policy_only=True)
                    or self._reset_active or self._enable_pending or self._estop_latched):
                return
            self._target_groups = clamped
            self._collector_body_origin_epoch = int(payload.get('authority_epoch', -1))
            self._target_velocity_estimate = self._target_velocity_estimator.reset(
                self._group_vector(clamped), now
            )
            self._target_time = now
            self._target_source = 'joint_space'
            self._velocity_active_sides = {
                side for side, group in (
                    ('left', 'left_arm'), ('right', 'right_arm')
                ) if group in changed
            }
            self._reason = 'guarded joint-space target accepted: ' + ', '.join(changed)
            if self._hardware_enabled:
                self._state = 'ARMED'

    def _deactivate_body_height_locked(self):
        """Release height ownership without changing the achieved pose."""
        if self._body_height_active and self._target_groups is not None:
            waist = np.asarray(self._target_groups['leg_waist'], dtype=float)
            neutral_pitch = (
                float(self._body_height_joint_targets[2])
                if self._body_height_joint_targets is not None
                else float(waist[2])
            )
            self._waist_follow_neutral_pitch_deg = neutral_pitch
            self._waist_follow_locked_yaw_deg = float(waist[3])
            self._waist_follow_anchor_x = {'left': None, 'right': None}
            self._waist_follow_extension_m = 0.0
            self._waist_follow_pitch_target_deg = neutral_pitch
        self._body_height_active = False
        self._body_height_watchdog_stopped = False
        self._body_height_joint_targets = None
        self._body_height_input_absolute = False
        self._body_height_command_time = 0.0
        self._body_height_update_time = 0.0
        self._body_height_command_progress = None
        self._body_height_last_direction = 0.0
        self._body_height_command_lead_deg = float(
            self.get_parameter('body_height_hold_command_lead_deg').value
        )

    def _body_height_output_target_locked(self, now, feedback):
        """Resolve height ownership at output time, not at an older IK snapshot.

        A lost height stream stops all three joints at measured pose exactly
        once. It must not fall through to the arm watchdog's old waist neutral.
        Fresh input rebases from this hold before advancing again.
        """
        if (not self._body_height_active or self._reset_active
                or self._body_height_joint_targets is None):
            return None
        stale = now - self._body_height_command_time > float(
            self.get_parameter('body_height_command_timeout_sec').value)
        if stale and not self._body_height_watchdog_stopped:
            held = np.asarray(feedback.as_dict()['leg_waist'][:3], dtype=float).copy()
            self._body_height_joint_targets = held
            self._body_height_lowering_m = estimate_body_lowering((*held, 0.0), signed=True)
            self._body_height_command_progress = (
                self._body_height_lowering_m / MAX_BODY_LOWERING_M)
            self._body_height_last_direction = 0.0
            self._body_height_command_lead_deg = float(
                self.get_parameter('body_height_hold_command_lead_deg').value)
            self._body_height_update_time = now
            self._waist_follow_neutral_pitch_deg = float(held[2])
            self._waist_follow_pitch_target_deg = float(held[2])
            self._waist_follow_extension_m = 0.0
            self._waist_follow_anchor_x = {'left': None, 'right': None}
            self._body_height_watchdog_stopped = True
        target = np.asarray(self._body_height_joint_targets, dtype=float).copy()
        if (not stale and self._waist_follow_active()
                and self._waist_follow_profile == 'forward_pitch_only'
                and self._waist_follow_pitch_target_deg is not None
                and self._waist_follow_neutral_pitch_deg is not None):
            target[2] += max(0.0, float(self._waist_follow_pitch_target_deg)
                             - float(self._waist_follow_neutral_pitch_deg))
            target[2] = min(target[2], float(np.rad2deg(self._kinematics.upper[
                self._kinematics.joint_q['Joint_Waist_Pitch']])))
        return target

    def _on_body_height_command(self, message):
        """Apply a watchdog-bounded, coordinated ankle/knee/waist command."""
        if not self._body_height_control_enabled:
            return
        now = time.monotonic()
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError('body-height command must be a JSON object')
            enabled = payload.get('enabled') is True
            direction = float(payload.get('direction', 0.0))
            target_level = payload.get('target_level')
            target_lowering = payload.get('target_lowering_m')
            reverse_squat = payload.get('reverse_squat', False)
            if not isinstance(reverse_squat, bool):
                raise ValueError('reverse_squat must be boolean')
            if target_lowering is not None:
                target_lowering = float(target_lowering)
                maximum = MAX_REVERSE_BODY_LOWERING_M if reverse_squat else MAX_BODY_LOWERING_M
                if not np.isfinite(target_lowering) or not 0.0 <= target_lowering <= maximum:
                    raise ValueError('body-height target_lowering_m is outside the supported range')
                if reverse_squat:
                    target_lowering = -target_lowering
            if target_level is not None:
                target_level = int(target_level)
                if target_level < 1 or target_level > 5:
                    raise ValueError('body-height target_level must be within [1, 5]')
                if target_lowering is None:
                    target_lowering = MAX_BODY_LOWERING_M * (5 - target_level) / 4.0
            if not np.isfinite(direction) or abs(direction) > 1.0:
                raise ValueError('body-height direction must be within [-1, 1]')
        except Exception as exc:
            with self._lock:
                self._reason = f'body-height command rejected: {exc}'
            return

        with self._lock:
            if not enabled:
                if self._feedback is not None and self._body_height_active:
                    # An explicit release has the same no-chasing semantics
                    # as timeout; do not retain a waist target ahead of reality.
                    self._body_height_joint_targets = np.asarray(
                        self._feedback.as_dict()['leg_waist'][:3], dtype=float).copy()
                    if self._target_groups is not None:
                        self._target_groups['leg_waist'][:3] = self._body_height_joint_targets
                self._deactivate_body_height_locked()
                return
            if (
                not self._hardware_enabled
                or self._reset_active
                or self._enable_pending
                or self._estop_latched
            ):
                return
            if self._feedback is None or now - self._feedback_time > float(
                    self.get_parameter('feedback_timeout_sec').value):
                self._reason = 'body-height command rejected: joint feedback is stale'
                return

            feedback_groups = self._feedback.as_dict()
            feedback_height = np.asarray(
                feedback_groups['leg_waist'][0:3], dtype=float
            ).copy()
            waist_assist_delta = 0.0
            if (
                self._waist_follow_active()
                and self._waist_follow_profile == 'forward_pitch_only'
                and self._waist_follow_neutral_pitch_deg is not None
                and self._waist_follow_pitch_target_deg is not None
            ):
                waist_assist_delta = max(
                    0.0,
                    float(self._waist_follow_pitch_target_deg)
                    - float(self._waist_follow_neutral_pitch_deg),
                )
                # Remove only the forward-reach contribution before estimating
                # height progress. Otherwise bending forward would be mistaken
                # for lowering the whole body and would be added a second time.
                feedback_height[2] -= waist_assist_delta
            base = self._target_groups or feedback_groups
            groups = {
                key: np.asarray(value, dtype=float).copy()
                for key, value in base.items()
            }
            entering_manual = (target_lowering is None
                               and getattr(self, '_body_height_input_absolute', False))
            new_manual_press = (target_lowering is None and (
                entering_manual or not self._body_height_active
                or (abs(self._body_height_last_direction) <= 1.e-6 and abs(direction) > 1.e-6)))
            if not self._body_height_active or entering_manual:
                self._body_height_lowering_m = estimate_body_lowering(
                    (*feedback_height, feedback_groups['leg_waist'][3]), signed=True
                )
                self._body_height_command_progress = (
                    float(self._body_height_lowering_m) / MAX_BODY_LOWERING_M
                )
                self._body_height_active = True
                self._body_height_update_time = now
                self._body_height_joint_targets = feedback_height.copy()
            if target_lowering is not None:
                # Every absolute task specifies its branch; missing reverse
                # means normal, never inherit the preceding reverse task.
                self._body_height_reverse_mode = bool(reverse_squat)
            elif new_manual_press:
                self._body_height_reverse_mode = measured_reverse_squat(
                    feedback_height, getattr(self, '_body_height_reverse_mode', False))
            self._body_height_input_absolute = target_lowering is not None
            if target_lowering is None and getattr(self, '_body_height_reverse_mode', False):
                # Latch direction through standing instead of flipping on an
                # ahead-of-feedback reference or waist-assist encoder noise.
                direction = -direction
            if target_lowering is not None:
                target_error = target_lowering - float(self._body_height_lowering_m)
                if abs(target_error) <= 0.002:
                    direction = 0.0
                else:
                    # Slow proportionally in the final 25 mm so an absolute
                    # level settles without overshoot, while preserving the
                    # existing manual direction protocol.
                    direction = max(-1.0, min(1.0, target_error / 0.025))
            dt = max(0.0, min(0.05, now - self._body_height_update_time))
            self._body_height_update_time = now
            hold_lead = max(
                0.1,
                float(self.get_parameter(
                    'body_height_hold_command_lead_deg').value),
            )
            motion_lead = max(
                hold_lead,
                float(self.get_parameter(
                    'body_height_motion_command_lead_deg').value),
            )
            moving = abs(direction) > 1.0e-6
            if not moving and target_lowering is None:
                # A key release is a stop request, not merely a request to stop
                # advancing the distant target. Rebase once at fresh measured
                # joints so the vendor position loop has no old error left to
                # chase, then keep that fixed target until X or Y is pressed.
                if (
                    abs(self._body_height_last_direction) > 1.0e-6
                    or self._body_height_joint_targets is None
                ):
                    targets = np.asarray(
                        feedback_height, dtype=float
                    ).copy()
                    self._body_height_lowering_m = estimate_body_lowering(
                        (*feedback_height, feedback_groups['leg_waist'][3]), signed=True
                    )
                    self._body_height_command_progress = (
                        float(self._body_height_lowering_m)
                        / MAX_BODY_LOWERING_M
                    )
                else:
                    targets = np.asarray(
                        self._body_height_joint_targets, dtype=float
                    ).copy()
                self._body_height_command_lead_deg = hold_lead
            else:
                # An absolute height goal remains owned until the measured
                # body reaches it. Reference convergence (direction == 0)
                # must NOT run the manual key-release/rebase branch above:
                # that freezes lagging joints short of the requested height.
                # Keep the bounded coordinated profile advancing to the same
                # reference while encoders catch up; watchdogs still apply.
                lead_step = max(
                    0.0,
                    float(self.get_parameter(
                        'body_height_command_lead_slew_deg_sec').value),
                ) * dt
                lead_error = motion_lead - self._body_height_command_lead_deg
                self._body_height_command_lead_deg += min(
                    lead_step, max(-lead_step, lead_error)
                )
                rate = max(
                    0.0,
                    float(self.get_parameter(
                        'body_height_lowering_rate_m_sec').value),
                )
                maximum_scale = min(
                    max(
                        0.0,
                        np.rad2deg(self._kinematics.upper[
                            self._kinematics.joint_q[name]
                        ]) / maximum,
                    )
                    for name, maximum in zip(
                        BODY_HEIGHT_JOINT_NAMES, BODY_HEIGHT_MAXIMUM_DEG
                    )
                )
                maximum_lowering = min(
                    MAX_BODY_LOWERING_M,
                    MAX_BODY_LOWERING_M * maximum_scale,
                )
                reverse_scale = min(
                    max(0.0, -np.rad2deg(self._kinematics.lower[
                        self._kinematics.joint_q[name]]) / maximum)
                    for name, maximum in zip(BODY_HEIGHT_JOINT_NAMES, BODY_HEIGHT_MAXIMUM_DEG)
                )
                minimum_lowering = -min(MAX_REVERSE_BODY_LOWERING_M,
                                        MAX_BODY_LOWERING_M * reverse_scale)
                if target_lowering is None:
                    # Manual keys cannot cross standing into the other squat branch.
                    if getattr(self, '_body_height_reverse_mode', False):
                        maximum_lowering = 0.0
                    else:
                        minimum_lowering = 0.0
                self._body_height_lowering_m = min(
                    maximum_lowering,
                    max(
                        minimum_lowering,
                        float(self._body_height_lowering_m)
                        + direction * rate * dt,
                    ),
                )
                targets, self._body_height_command_progress = (
                    coordinated_body_height_targets(
                        self._body_height_lowering_m,
                        feedback_height,
                        self._body_height_command_progress,
                        self._body_height_command_lead_deg,
                        allow_reverse=True,
                    )
                )
                targets = np.asarray(targets, dtype=float)
            self._body_height_last_direction = direction
            groups['leg_waist'][0:3] = targets
            waist_assist_delta = min(waist_assist_delta, max(0.0,
                float(np.rad2deg(self._kinematics.upper[
                    self._kinematics.joint_q['Joint_Waist_Pitch']])) - float(targets[2])))
            if waist_assist_delta > 0.0:
                # Keep the active reach-assist contribution while height owns
                # the baseline. This prevents the 50 Hz height callback from
                # alternately erasing the 90 Hz IK waist target.
                groups['leg_waist'][2] = float(targets[2]) + waist_assist_delta
            guard_error, _q = self._configuration_guard_error(
                groups,
                'body-height target',
                limit_joint_names=BODY_HEIGHT_JOINT_NAMES,
            )
            if guard_error:
                self._reason = f'body-height command rejected: {guard_error}'
                return
            self._body_height_joint_targets = targets
            self._body_height_watchdog_stopped = False
            self._waist_follow_neutral_pitch_deg = float(targets[2])
            self._waist_follow_pitch_target_deg = float(targets[2]) + waist_assist_delta
            self._body_height_command_time = now
            self._target_groups = groups
            self._target_velocity_estimator.reset_indices(
                range(0, 3), targets
            )
            self._target_velocity_estimate = (
                self._target_velocity_estimator.velocity.copy()
            )
            self._target_time = now
            self._target_source = 'body_height'
            if abs(direction) > 1.0e-6:
                self._state = 'ARMED'
                self._reason = (
                    f'body height target: lowering '
                    f'{self._body_height_lowering_m:.3f} m'
                )

    def _cancel_task_head_locked(self, reason='stopped'):
        self._task_head_cancelled_id = getattr(self, '_task_head_request_id', '')
        self._task_head_cancel_reason = reason
        self._task_head_target = None
        self._task_head_target_time = 0.0

    def _on_task_head_target(self, message):
        """Bounded task preset; never enables hardware or HMD tracking."""
        now = time.monotonic()
        try:
            payload = json.loads(message.data)
            request_id = str(payload['request_id'])
            if not request_id or len(request_id) > 100:
                raise ValueError('invalid task head request id')
            enabled = payload.get('enabled') is True
            pitch = float(payload.get('head_pitch_deg', 0))
            if not np.isfinite(pitch) or not 0 <= pitch <= 45:
                raise ValueError('task head pitch must be within 0..45 degrees')
        except Exception as exc:
            with self._lock:
                self._reason = f'task head rejected: {exc}'
            return
        with self._lock:
            if not enabled:
                if request_id == self._task_head_request_id:
                    self._cancel_task_head_locked()
                return
            if request_id == self._task_head_cancelled_id:
                return
            if (self._head_follow_active() or self._reset_active or self._enable_pending
                    or self._estop_latched or not self._hardware_enabled):
                self._task_head_cancelled_id = request_id
                self._task_head_cancel_reason = (
                    'head_tracking' if self._head_follow_active() else
                    'reset' if self._reset_active else
                    'enabling' if self._enable_pending else
                    'estop' if self._estop_latched else 'hardware_disabled')
                self._task_head_target = None
                return
            if self._feedback is None or now - self._feedback_time > float(
                    self.get_parameter('feedback_timeout_sec').value):
                return
            # Config is positive downward; model/vendor pitch is negative.
            # Preserve roll/yaw, which the task did not request changing.
            target = np.asarray(self._feedback.neck, dtype=float).copy()
            idx = self._kinematics.joint_q['Joint_Neck_Pitch']
            lower = float(np.rad2deg(self._kinematics.lower[idx]))
            upper = float(np.rad2deg(self._kinematics.upper[idx]))
            if not np.isfinite(lower) or not np.isfinite(upper) or not lower <= 0 <= upper:
                self._task_head_cancelled_id = request_id
                self._task_head_cancel_reason = 'invalid_neck_limits'
                self._task_head_target = None
                return
            # User range is the nominal URDF range. Keep the existing soft
            # margin (283: 45 degrees nominal minus 2 degrees protection).
            # A valid nominal request at the boundary saturates safely; it is
            # not a takeover/reset failure. Report the exact applied target so
            # the task coordinator verifies encoders against that same value.
            target[1] = float(np.clip(-pitch, lower, upper))
            effective = -float(target[1])
            if request_id != self._task_head_request_id:
                self.get_logger().info(
                    f'task head preset: id={request_id}, requested={pitch:.3f}deg, '
                    f'effective={effective:.3f}deg, safe_max={-lower:.3f}deg')
            self._task_head_goal = {
                'request_id': request_id, 'requested_deg': pitch,
                'effective_deg': effective, 'safe_max_deg': -lower,
            }
            self._task_head_cancel_reason = ''
            self._task_head_target = target
            self._task_head_target_time = now
            self._task_head_request_id = request_id

    def _follow_authority_allowed(self, *, for_arm=False):
        if not getattr(self, '_follow_authority_required', False):
            return True
        modes = ('EXPERT_READY', 'EXPERT_ACTIVE') if for_arm else ('EXPERT_ACTIVE',)
        return (self._follow_authority_mode in modes
                and 0 <= time.monotonic() - self._follow_authority_time <= 0.5)

    def _collector_target_allowed(self, payload, *, policy_only):
        if not self._follow_authority_required:
            return True
        modes = ('POLICY_ACTIVE',) if policy_only else ('POLICY_ACTIVE', 'EXPERT_ACTIVE', 'EXPERT_READY')
        return (self._follow_authority_mode in modes
                and 0 <= time.monotonic() - self._follow_authority_time <= 0.5
                and payload.get('authority_epoch') == self._collector_authority_epoch
                and self._collector_authority_epoch > self._collector_revoked_epoch)

    def _head_follow_active(self):
        return self._head_follow_enabled and self._follow_authority_allowed()

    def _waist_follow_active(self):
        return self._waist_follow_enabled and self._follow_authority_allowed()

    def _on_follow_authority(self, message):
        try:
            payload = json.loads(message.data)
            mode = 'LAUNCHER' if payload.get('launcher_control_active', False) else str(payload['mode'])
        except (ValueError, TypeError, KeyError):
            return
        with self._lock:
            was_active = self._follow_authority_allowed()
            from dagger.controller_handoff import apply_authority
            if not apply_authority(self, payload, mode):
                return
            self._follow_authority_mode = mode
            self._follow_authority_time = time.monotonic()
            active = self._follow_authority_allowed()
            if active != was_active:
                self._head_target = None
                self._head_target_time = 0.0
                self._waist_follow_anchor_x = {'left': None, 'right': None}
                self._waist_follow_extension_m = 0.0
                if self._feedback is not None:
                    waist = self._feedback.leg_waist
                    self._waist_follow_neutral_pitch_deg = float(waist[2])
                    self._waist_follow_locked_yaw_deg = float(waist[3])
                    self._waist_follow_pitch_target_deg = float(waist[2])

    def _on_head_target(self, message):
        """Accept a separately-clocked neck target from live WebXR head pose."""
        now = time.monotonic()
        try:
            payload = json.loads(message.data)
            values = np.asarray(
                payload.get('neck_target_joints_position'), dtype=float
            ).reshape(-1)
            if values.size != 3 or not np.all(np.isfinite(values)):
                raise ValueError('expected three finite neck joint values')
        except Exception as exc:
            with self._lock:
                self._reason = f'head target rejected: {exc}'
            return
        with self._lock:
            if (
                not self._head_follow_active()
                or not self._hardware_enabled
                or self._reset_active
                or self._enable_pending
                or self._estop_latched
            ):
                return
            if self._feedback is None or now - self._feedback_time > float(
                    self.get_parameter('feedback_timeout_sec').value):
                self._reason = 'head target rejected: joint feedback is stale'
                return
            neck_names = ('Joint_Neck_Roll', 'Joint_Neck_Pitch', 'Joint_Neck_Yaw')
            lower = np.asarray([
                np.rad2deg(self._kinematics.lower[self._kinematics.joint_q[name]])
                for name in neck_names
            ])
            upper = np.asarray([
                np.rad2deg(self._kinematics.upper[self._kinematics.joint_q[name]])
                for name in neck_names
            ])
            measured = np.asarray(self._feedback.neck, dtype=float)
            # Preserve an already measured calibration offset without allowing
            # the command to move farther beyond the nominal URDF boundary.
            lower = np.minimum(lower, measured)
            upper = np.maximum(upper, measured)
            self._head_target = np.clip(values, lower, upper)
            self._head_target_time = now


    def _on_teleop_heartbeat(self, message):
        """Accept either live tracking or an explicit mapper-alive hold state.

        A hold heartbeat keeps an already enabled position session alive while
        the headset sleeps.  It can never authorize a new hardware enable.
        """
        try:
            payload = json.loads(message.data)
            maximum_age = float(
                self.get_parameter('teleop_vr_input_max_age_sec').value
            )
            mode, vr_age, tracked_hands = classify_teleop_heartbeat(
                payload, maximum_age
            )
        except (AttributeError, TypeError, ValueError, json.JSONDecodeError) as exc:
            with self._lock:
                self._teleop_heartbeat_rejection = (
                    f'fresh VR controller data is unavailable: {exc}'
                )
            return
        with self._lock:
            self._teleop_heartbeat_time = time.monotonic()
            self._teleop_heartbeat_mode = mode
            self._teleop_vr_age = vr_age
            self._teleop_tracked_hands = list(tracked_hands)
            self._teleop_heartbeat_rejection = ''

    def _own_command_echo_timeout(self):
        try:
            timeout = float(
                self.get_parameter('own_command_echo_timeout_sec').value
            )
        except (TypeError, ValueError):
            return 0.0
        if not np.isfinite(timeout) or timeout <= 0.0:
            return 0.0
        return timeout

    def _prune_owned_commands(self, topic, now):
        entries = self._recent_owned_commands.setdefault(topic, [])
        timeout = self._own_command_echo_timeout()
        if timeout <= 0.0:
            entries.clear()
            return entries
        entries[:] = [
            entry for entry in entries if now - entry[1] <= timeout
        ]
        return entries

    def _publish_owned_command(self, publisher, topic, payload):
        """Publish and remember an exact payload so its local echo is ignored."""
        if publisher is None:
            raise RuntimeError(f'hardware publisher is unavailable for {topic}')
        payload = str(payload)
        now = time.monotonic()
        with self._lock:
            entries = self._prune_owned_commands(topic, now)
            self._owned_command_sequence += 1
            sequence = self._owned_command_sequence
            entries.append((sequence, now, payload))
            # A fixed cap prevents an unhealthy executor from accumulating
            # unbounded payloads.  At 100 Hz this still covers several seconds.
            if len(entries) > 512:
                del entries[:-512]
        try:
            publisher.publish(String(data=payload))
            # Separate audit channel; vendor command JSON remains unchanged.
            # This is proof of publication, never proof of physical completion.
            if topic in (self._output_topic, self._gripper_output_topic):
                gripper = topic == self._gripper_output_topic
                epoch = (self._collector_gripper_origin_epoch if gripper
                         else self._collector_body_origin_epoch)
                self._collector_output_pub.publish(String(data=json.dumps({
                    'gripper': gripper, 'command': json.loads(payload),
                    'timestamp_ns': time.time_ns(), 'origin_authority_epoch': epoch,
                    'session_id': self._collector_session_id,
                }, separators=(',', ':'))))
        except Exception:
            with self._lock:
                entries = self._recent_owned_commands.get(topic, [])
                entries[:] = [entry for entry in entries if entry[0] != sequence]
            raise

    def _on_joint_command_observed(self, message):
        self._on_command_observed(self._output_topic, message)

    def _on_velocity_command_observed(self, message):
        self._on_command_observed(self._velocity_output_topic, message)

    def _on_gripper_command_observed(self, message):
        self._on_command_observed(self._gripper_output_topic, message)

    def _on_legacy_eef_command_observed(self, message):
        # This controller never publishes the vendor Cartesian command topic,
        # so every observed payload is necessarily an external command.
        self._on_command_observed(self._legacy_eef_topic, message)

    def _on_command_observed(self, topic, message):
        """Record an unmatched hardware command and fail closed if active."""
        now = time.monotonic()
        payload = str(message.data)
        fault_reason = ''
        with self._lock:
            entries = self._prune_owned_commands(topic, now)
            if any(entry[2] == payload for entry in entries):
                # ROS subscriptions normally receive this node's own output.
                # Exact recent payloads are compatible with our command even
                # if an idle vendor endpoint happens to repeat the same hold.
                return
            self._last_external_command_time = now
            self._last_external_command_topic = topic
            self._last_external_command_payload = (
                payload if len(payload) <= 512 else payload[:509] + '...'
            )
            self._external_command_counts[topic] = (
                self._external_command_counts.get(topic, 0) + 1
            )
            reset_command_expected = (
                self._enable_pending
                and self._enable_full_reset
                and self._enable_stage in ('wait_full_reset', 'wait_full_reset_quiet')
                and topic in (
                    self._output_topic,
                    self._velocity_output_topic,
                    self._gripper_output_topic,
                )
            )
            if bool(self.get_parameter(
                    'enable_active_command_arbitration').value) and (
                    self._hardware_enabled or self._enable_pending
            ) and not reset_command_expected:
                fault_reason = (
                    f'unmatched external hardware command observed on {topic}; '
                    'controller output stopped'
                )
                self._set_hardware_fault(fault_reason)
        if fault_reason:
            self.get_logger().error(fault_reason)

    def _active_command_guard_error(self, now):
        if not bool(self.get_parameter(
                'enable_active_command_arbitration').value):
            return ''
        try:
            quiet_seconds = float(
                self.get_parameter('external_command_quiet_sec').value
            )
        except (TypeError, ValueError):
            return 'external command quiet-window parameter is invalid'
        if not np.isfinite(quiet_seconds) or quiet_seconds < 0.0:
            return 'external command quiet-window parameter is invalid'
        if self._own_command_echo_timeout() <= 0.0:
            return 'own-command echo timeout parameter is invalid'
        quiet_since = max(
            self._command_monitor_started,
            self._last_external_command_time,
        )
        quiet_age = max(0.0, now - quiet_since)
        if quiet_age < quiet_seconds:
            if self._last_external_command_time >= self._command_monitor_started:
                source = self._last_external_command_topic or 'hardware command topic'
                return (
                    f'external command on {source} has been quiet for only '
                    f'{quiet_age:.2f}s; require {quiet_seconds:.2f}s'
                )
            return (
                f'external command monitor has observed only {quiet_age:.2f}s '
                f'of startup silence; require {quiet_seconds:.2f}s'
            )
        return ''

    def _run_full_reset_enable(self, now, feedback):
        """Run the official full-body reset, then safely claim command output."""
        if self._enable_stage == 'release_sync_before_reset':
            # The vendor gates /control_reset while a previous SYNC lease is
            # active. Await its hold-and-release acknowledgement before reset.
            if not self._sync_session_client.service_is_ready():
                self._set_hardware_fault('vendor SYNC release service is unavailable')
                return True
            try:
                self._sync_session_future = self._sync_session_client.call_async(SetBool.Request(data=False))
            except Exception as exc:
                self._set_hardware_fault(f'cannot prepare system reset: {exc}')
                return True
            self._sync_session_acquired = False
            self._enable_stage = 'wait_sync_release_before_reset'
            self._enable_stage_started = now
            return True

        if self._enable_stage == 'wait_sync_release_before_reset':
            future = self._sync_session_future
            if now - self._enable_stage_started > float(self.get_parameter('sync_session_request_timeout_sec').value):
                self._set_hardware_fault('vendor SYNC release timed out; system reset not sent')
                return True
            if future is None or not future.done():
                self._reason = 'waiting for vendor hold-and-release before system reset'
                return True
            try:
                result = future.result()
                if result is None or not result.success:
                    raise RuntimeError('empty response' if result is None else result.message)
            except Exception as exc:
                self._set_hardware_fault(f'vendor SYNC release failed; system reset not sent: {exc}')
                return True
            self._sync_session_future = None
            self._enable_stage = 'wait_full_reset_subscriber'
            self._enable_stage_started = now
            return True

        if self._enable_stage == 'wait_full_reset_subscriber':
            if self._full_reset_pub is None:
                self._set_hardware_fault('full-body reset publisher is unavailable')
                return True
            if self._full_reset_pub.get_subscription_count() == 0:
                self._reason = 'waiting for the official full-body reset service'
                return True
            try:
                self._full_reset_pub.publish(String(data=''))
            except Exception as exc:
                self._set_hardware_fault(f'failed to request full-body reset: {exc}')
                return True
            self._full_reset_published_at = now
            self._full_reset_stable_since = None
            self._full_reset_max_error_deg = None
            self._full_reset_max_speed_deg_sec = None
            self._enable_stage = 'wait_full_reset'
            self._enable_stage_started = now
            self._reason = 'full-body reset requested; waiting for all joints to settle'
            return True

        if self._enable_stage == 'wait_full_reset':
            minimum_wait = max(
                0.0, float(self.get_parameter('full_reset_min_wait_sec').value)
            )
            elapsed = now - self._full_reset_published_at
            if elapsed < minimum_wait:
                self._reason = (
                    f'full-body reset is running ({elapsed:.1f}/{minimum_wait:.1f}s minimum)'
                )
                return True
            if self._feedback_motion is None:
                self._full_reset_stable_since = None
                self._reason = 'waiting for reset target and joint-speed feedback'
                return True
            try:
                maximum_error, maximum_speed = (
                    self._feedback_motion.maximum_error_and_speed(feedback.as_dict())
                )
            except Exception as exc:
                self._full_reset_stable_since = None
                self._reason = f'waiting for valid reset completion feedback: {exc}'
                return True
            self._full_reset_max_error_deg = maximum_error
            self._full_reset_max_speed_deg_sec = maximum_speed
            position_tolerance = max(
                0.1,
                float(self.get_parameter('full_reset_position_tolerance_deg').value),
            )
            speed_tolerance = max(
                0.1,
                float(self.get_parameter('full_reset_speed_tolerance_deg_sec').value),
            )
            if maximum_error > position_tolerance or maximum_speed > speed_tolerance:
                self._full_reset_stable_since = None
                self._reason = (
                    'full-body reset is moving: '
                    f'target error {maximum_error:.2f}deg, speed {maximum_speed:.2f}deg/s'
                )
                return True
            if self._full_reset_stable_since is None:
                self._full_reset_stable_since = now
            stable_seconds = max(
                0.0, float(self.get_parameter('full_reset_stable_sec').value)
            )
            stable_elapsed = now - self._full_reset_stable_since
            self._reason = (
                'full-body reset position reached; confirming stability '
                f'({stable_elapsed:.1f}/{stable_seconds:.1f}s)'
            )
            if stable_elapsed < stable_seconds:
                return True
            self._enable_stage = 'wait_full_reset_quiet'
            self._enable_stage_started = now
            self._reason = 'full-body reset completed; waiting for vendor command traffic to stop'
            return True

        if self._enable_stage == 'wait_full_reset_quiet':
            if str(self.get_parameter('hardware_session_mode').value).lower() == 'sync':
                # The official reset has already enabled and positioned the
                # whole body. Claim SYNC from this settled pose; never call the
                # legacy prepare callback or run a second reset trajectory.
                arbitration_error = self._active_command_guard_error(now)
                if arbitration_error:
                    self._reason = 'system reset complete; waiting for command ownership: ' + arbitration_error
                    return True
                current = feedback.as_dict()
                pose_error, _ = self._configuration_guard_error(
                    current, 'measured pose after system reset',
                    limit_joint_names=TELEOP_LIMIT_JOINT_NAMES,
                )
                if pose_error:
                    self._set_hardware_fault(pose_error)
                    return True
                try:
                    self._begin_fresh_teleop_session_locked(current)
                    self._enable_hold_groups = {
                        key: np.asarray(value, dtype=float).copy()
                        for key, value in current.items()
                    }
                except Exception as exc:
                    self._set_hardware_fault(f'system reset hold initialization failed: {exc}')
                    return True
                self._enable_stage = 'hold_before_enable'
                self._enable_stage_started = now
                self._reason = 'system reset complete; acquiring SYNC at the settled pose'
                return True
            guard_error = self._hardware_output_guard_error(require_prepare=True)
            self._last_output_guard_check = now
            if guard_error:
                self._set_hardware_fault(guard_error + '; teleop enable cancelled')
                return True
            try:
                self._prepare_pub.publish(String(data=json.dumps({
                    'enable': True,
                    'control_mode': self._output_control_mode,
                }, separators=(',', ':'))))
            except Exception as exc:
                self._set_hardware_fault(
                    f'failed to select {self._output_control_mode} control mode: {exc}'
                )
                return True
            self._enable_stage = 'wait_control_mode_after_reset'
            self._enable_stage_started = now
            self._reason = (
                f'full-body reset complete; selecting '
                f'{self._output_control_mode} arm control mode'
            )
            return True

        if self._enable_stage != 'wait_control_mode_after_reset':
            return False
        if now - self._enable_stage_started < 0.8:
            self._reason = f'waiting for {self._output_control_mode} arm control mode'
            return True

        arbitration_error = self._active_command_guard_error(now)
        if arbitration_error:
            self._reason = (
                'full-body reset completed; waiting to take control safely: '
                + arbitration_error
            )
            return True
        guard_error = self._hardware_output_guard_error(require_prepare=False)
        self._last_output_guard_check = now
        if guard_error:
            self._set_hardware_fault(guard_error + '; teleop enable cancelled')
            return True

        current = feedback.as_dict()
        pose_error, _ = self._configuration_guard_error(
            current,
            'measured pose after full-body reset',
            limit_joint_names=TELEOP_LIMIT_JOINT_NAMES,
        )
        if pose_error:
            self._set_hardware_fault(pose_error + '; teleop enable cancelled')
            return True
        try:
            self._reset_motion_controllers(current)
            self._target_groups = {
                key: np.asarray(value, dtype=float).copy()
                for key, value in current.items()
            }
            self._target_time = 0.0
            self._target_source = 'measured_hold_after_full_reset'
            self._gripper_targets = {
                'left': float(feedback.left_gripper[0]),
                'right': float(feedback.right_gripper[0]),
            }
            self._gripper_dirty = {'left': False, 'right': False}
            self._enable_hold_groups = {
                key: np.asarray(value, dtype=float).copy()
                for key, value in current.items()
            }
            self._publish_enable_hold_pose()
        except Exception as exc:
            self._set_hardware_fault(
                f'failed to take control after full-body reset: {exc}'
            )
            return True

        self._enable_hold_groups = None
        self._enable_pending = False
        self._enable_stage = None
        self._hardware_enabled = True
        self._state = 'ARMED'
        self._reason = 'full-body reset completed; teleop hardware is enabled'
        return False

    def _start_quick_reset_locked(self, now, feedback, *, source='quick_reset', preserve_grippers=False):
        """Start the shared X+A/enable reset trajectory while holding the lock."""
        measured_groups = {
            key: np.asarray(value, dtype=float).copy()
            for key, value in feedback.as_dict().items()
        }
        groups = {
            key: np.asarray(value, dtype=float).copy()
            for key, value in measured_groups.items()
        }
        left = np.asarray(
            self.get_parameter('quick_reset_left_arm_joints').value,
            dtype=float,
        ).reshape(-1)
        right = np.asarray(
            self.get_parameter('quick_reset_right_arm_joints').value,
            dtype=float,
        ).reshape(-1)
        # X+A and the automatic post-enable reset share the same complete
        # vendor reset pose.  In particular, X+A must also return the neck to
        # the configured system default instead of preserving an arbitrary
        # head-follow target from the previous session.
        neck = np.asarray(
            self.get_parameter('quick_reset_neck_joints').value,
            dtype=float,
        ).reshape(-1)
        if left.size != 7 or right.size != 7 or neck.size != 3:
            return 'quick reset configuration has an invalid joint count'
        if not (
            np.all(np.isfinite(left))
            and np.all(np.isfinite(right))
            and np.all(np.isfinite(neck))
        ):
            return 'quick reset configuration contains NaN or Inf'
        groups['left_arm'] = left
        groups['right_arm'] = right
        groups['neck'] = neck
        # Both arm-reset chords preserve the measured body pose, including
        # forward/reverse crouch and torso yaw. Standing is a height command,
        # not a side effect of an arm/head reset.
        guard_error, q = self._configuration_guard_error(
            groups,
            'quick reset target',
            limit_joint_names=ARM_JOINT_NAMES + list(BODY_HEIGHT_JOINT_NAMES),
        )
        if guard_error:
            return guard_error
        clamped = self._kinematics.groups_from_q_deg(q, groups)
        open_grippers = not preserve_grippers and bool(
            self.get_parameter('quick_reset_open_grippers').value
        )
        open_position = None
        if open_grippers:
            open_position = float(
                self.get_parameter('quick_reset_gripper_open_position').value
            )
            minimum = float(self.get_parameter('gripper_min_position').value)
            maximum = float(self.get_parameter('gripper_max_position').value)
            if not np.isfinite(open_position) or not minimum <= open_position <= maximum:
                return 'quick reset gripper-open target is outside the configured range'

        reset_initial_error = max(
            1e-6,
            float(np.max(np.abs(
                self._reset_required_vector(clamped)
                - self._reset_required_vector(measured_groups)
            ))),
        )

        # Phase 1 is now complete: target, gripper bounds, collision/limit
        # guard, and every fresh motion-controller object have been validated.
        # Nothing above this point mutates the active clutch/IK session.
        try:
            motion_state = self._build_motion_controller_state(measured_groups)
        except Exception as exc:
            return f'quick reset controller initialization failed: {exc}'
        path_error, reset_path = self._prepare_quick_reset_path(
            measured_groups, clamped
        )
        if path_error:
            return path_error

        # Phase 2: commit the reset boundary.  From here onward assignments are
        # non-validating and cannot leave a rejected reset half-applied.
        self._latest_target_mailbox.reset()
        self._commit_motion_controller_state(motion_state)
        self._lock_head_follow_for_control_boundary_locked()
        self._grip_release_held = {'left', 'right'}
        self._clutch_session = None
        self._clutch_sequence = -1
        self._waist_follow_anchor_x = {'left': None, 'right': None}
        self._waist_follow_extension_m = 0.0
        self._velocity_active_sides.clear()
        self._last_ik = None
        self._reset_initial_error = reset_initial_error
        self._reset_path_start = reset_path['start']
        self._reset_path_goal = reset_path['goal']
        self._reset_path_duration_sec = reset_path['duration_sec']
        self._reset_path_elapsed_sec = 0.0
        self._target_groups = clamped
        # X+A owns the control boundary now. Release body-height ownership
        # before RESETTING becomes visible. The selected reset path now owns
        # ankle/knee/waist, preventing height-stream or stale IK interference.
        self._deactivate_body_height_locked()
        self._body_height_reverse_mode = measured_reverse_squat(
            measured_groups['leg_waist'],
            previous=bool(getattr(self, '_body_height_reverse_mode', False)),
        )
        self._target_time = now
        self._target_source = str(source)
        if (
            self._waist_follow_active()
            and self._waist_follow_profile == 'forward_pitch_only'
        ):
            # Rebase assistance to the reset goal so the first post-reset frame cannot
            # pull the torso toward an unrelated upright target.
            self._waist_follow_neutral_pitch_deg = float(
                clamped['leg_waist'][2]
            )
            self._waist_follow_locked_yaw_deg = float(clamped['leg_waist'][3])
            self._waist_follow_pitch_target_deg = float(clamped['leg_waist'][2])
        self._velocity_active_sides = {'left', 'right'}
        self._reset_active = True
        self._reset_started = now
        self._reset_progress = 0.0
        self._reset_gripper_last_publish_time = 0.0
        if open_grippers:
            self._gripper_targets = {
                'left': open_position,
                'right': open_position,
            }
            self._gripper_dirty = {'left': True, 'right': True}
        else:
            self._gripper_dirty = {'left': False, 'right': False}
        self._state = 'RESETTING'
        self._reason = (
            'quick reset is following one collision-checked continuous path'
        )
        return ''

    def _on_quick_reset(self, _request, response):
        return self._request_quick_reset(response, preserve_grippers=False)

    def _on_hold_gripper_reset(self, _request, response):
        return self._request_quick_reset(response, preserve_grippers=True)

    def _request_quick_reset(self, response, preserve_grippers=False):
        now = time.monotonic()
        with self._lock:
            feedback = self._feedback
            if self._estop_latched:
                response.success = False
                response.message = 'emergency stop is latched'
                return response
            if self._enable_pending:
                response.success = False
                response.message = 'hardware enable is still pending'
                return response
            if self._reset_active:
                response.success = False
                response.message = 'quick reset is already active'
                return response
            if not self._dry_run and not self._hardware_enabled:
                response.success = False
                response.message = 'hardware must be enabled before quick reset'
                return response
            if feedback is None or now - self._feedback_time > float(
                    self.get_parameter('feedback_timeout_sec').value):
                response.success = False
                response.message = 'fresh joint feedback is required for quick reset'
                return response
            error = self._start_quick_reset_locked(
                now, feedback, preserve_grippers=preserve_grippers,
                source='hold_gripper_reset' if preserve_grippers else 'quick_reset',
            )
            if error:
                response.success = False
                response.message = error
                return response
            message = self._reason
        response.success = True
        response.message = message
        return response

    def _on_waist_follow_enabled(self, request, response):
        """Select the configured waist assistance profile for teleoperation."""
        with self._lock:
            enabled = bool(request.data)
            if self._enable_pending or self._reset_active:
                response.success = False
                response.message = (
                    'waist-follow mode cannot change during enable/reset'
                )
                return response
            if self._hardware_enabled and enabled:
                response.success = False
                response.message = (
                    'enable waist follow before enabling hardware'
                )
                return response
            if self._hardware_enabled and not enabled:
                now = time.monotonic()
                if (
                    self._feedback is None
                    or now - self._feedback_time > float(
                        self.get_parameter('feedback_timeout_sec').value
                    )
                ):
                    response.success = False
                    response.message = (
                        'fresh joint feedback is required to disable live waist follow'
                    )
                    return response
            self._waist_follow_enabled = enabled
            waist_in_ik = enabled and self._waist_follow_profile == 'ik_pitch_yaw'
            self._kinematics.allow_waist = waist_in_ik
            self._ik_kinematics.allow_waist = waist_in_ik
            if enabled and self._waist_follow_profile == 'forward_pitch_only':
                self._waist_follow_neutral_pitch_deg = float(
                    self.get_parameter(
                        'waist_forward_assist_upright_pitch_deg'
                    ).value
                )
                if self._feedback is not None:
                    waist = np.asarray(self._feedback.leg_waist, dtype=float)
                    self._waist_follow_neutral_pitch_deg = float(waist[2])
                    self._waist_follow_locked_yaw_deg = float(waist[3])
                self._waist_follow_anchor_x = {'left': None, 'right': None}
                self._waist_follow_extension_m = 0.0
                self._waist_follow_pitch_target_deg = (
                    self._waist_follow_neutral_pitch_deg
                )
            else:
                self._waist_follow_neutral_pitch_deg = None
                self._waist_follow_locked_yaw_deg = None
                self._waist_follow_anchor_x = {'left': None, 'right': None}
                self._waist_follow_extension_m = 0.0
                self._waist_follow_pitch_target_deg = None
            if self._hardware_enabled and not enabled:
                # A desktop-mode switch first releases both mapper clutches.
                # Rebase again from measured joints here so an IK solve that
                # began with waist DOFs can never commit after they are frozen.
                self._begin_fresh_teleop_session_locked(
                    self._feedback.as_dict()
                )
            self._reason = (
                (
                    'waist follow enabled; forward reach controls pitch only, '
                    'waist yaw is locked'
                    if self._waist_follow_profile == 'forward_pitch_only'
                    else 'waist follow enabled; waist pitch/yaw may assist arm IK'
                )
                if enabled else
                'waist follow disabled; waist is held at measured position'
            )
            response.success = True
            response.message = self._reason
            return response

    def _on_head_follow_enabled(self, request, response):
        """Select whether fresh WebXR head orientation controls the neck."""
        with self._lock:
            enabled = bool(request.data)
            if self._enable_pending or self._reset_active:
                response.success = False
                response.message = (
                    'head-follow mode cannot change during enable/reset'
                )
                return response
            if enabled and self._hardware_enabled:
                now = time.monotonic()
                if self._feedback is None or now - self._feedback_time > float(
                        self.get_parameter('feedback_timeout_sec').value):
                    response.success = False
                    response.message = (
                        'head follow cannot start because joint feedback is stale'
                    )
                    return response
            self._head_follow_enabled = enabled
            self._cancel_task_head_locked('head_mode_changed')
            self._head_target = None
            self._head_target_time = 0.0
            self._reason = (
                'head follow enabled; current headset direction is forward '
                'and the neck will return smoothly to calibrated neutral'
                if enabled
                else 'head follow disabled; neck will remain fixed'
            )
            response.success = True
            response.message = self._reason
            return response

    def _publish_enable_hold_pose(self):
        if self._enable_hold_groups is None:
            raise RuntimeError('arm enable hold pose is unavailable')
        if self._joint_pub is None or self._gripper_pub is None:
            raise RuntimeError('hardware hold publishers are unavailable')
        joint_payload = json.dumps(
            self._command_payload(self._enable_hold_groups), separators=(',', ':')
        )
        gripper_payload = json.dumps({
            'left_gripper_target_joints_position': [
                float(self._enable_hold_groups['left_gripper'][0])
            ],
            'right_gripper_target_joints_position': [
                float(self._enable_hold_groups['right_gripper'][0])
            ],
        }, separators=(',', ':'))
        self._publish_owned_command(
            self._joint_pub, self._output_topic, joint_payload
        )
        self._publish_owned_command(
            self._gripper_pub, self._gripper_output_topic, gripper_payload
        )

    def _run_hardware_enable(self, now, feedback, feedback_age):
        """Advance the fail-closed vendor hardware enable state machine.

        Return True while the caller must stop normal control processing for
        this tick.  Return False only after ARMED has been reached.
        """
        if not self._enable_pending:
            return False

        timeout = max(
            1.0,
            float(self.get_parameter(
                'arm_enable_completion_timeout_seconds'
            ).value),
        )
        if now - self._enable_started > timeout:
            self._set_hardware_fault('arm hardware enable timed out')
            return True
        if feedback is None or feedback_age > float(
                self.get_parameter('feedback_timeout_sec').value):
            self._set_hardware_fault(
                'feedback lost while preparing arm hardware'
            )
            return True
        heartbeat_error = self._heartbeat_guard_error(now, require_fresh=True)
        if heartbeat_error:
            self._set_hardware_fault(
                heartbeat_error + '; reset/teleop enable cancelled'
            )
            return True
        if self._enable_full_reset and (
            self._enable_stage in (
                'release_sync_before_reset',
                'wait_sync_release_before_reset',
                'wait_full_reset_subscriber',
                'wait_full_reset',
                'wait_full_reset_quiet',
                'wait_control_mode_after_reset',
            )
        ):
            return self._run_full_reset_enable(now, feedback)
        arbitration_error = self._active_command_guard_error(now)
        if arbitration_error:
            self._set_hardware_fault(
                arbitration_error + '; arm enable cancelled'
            )
            return True

        guard_period = max(
            0.10, float(self.get_parameter('output_guard_period_sec').value)
        )
        guard_due = now - self._last_output_guard_check >= guard_period
        if guard_due:
            self._last_output_guard_check = now
            guard_error = self._hardware_output_guard_error(require_prepare=False)
            if guard_error:
                self._set_hardware_fault(
                    guard_error + '; arm enable cancelled'
                )
                return True

        try:
            self._publish_enable_hold_pose()
        except Exception as exc:
            self._set_hardware_fault(
                f'failed to publish arm enable hold pose: {exc}'
            )
            return True

        if self._enable_stage == 'hold_before_enable':
            hold_seconds = max(
                0.1,
                float(self.get_parameter(
                    'arm_enable_hold_before_seconds'
                ).value),
            )
            self._reason = 'holding measured pose before enabling arm joints'
            if now - self._enable_stage_started < hold_seconds:
                return True
            guard_error = self._hardware_output_guard_error(require_prepare=False)
            self._last_output_guard_check = now
            if guard_error:
                self._set_hardware_fault(
                    guard_error + '; arm enable cancelled'
                )
                return True
            session_mode = str(
                self.get_parameter('hardware_session_mode').value
            ).strip().lower()
            if session_mode == 'prepare':
                guard_error = self._hardware_output_guard_error(
                    require_prepare=True
                )
                if guard_error:
                    self._set_hardware_fault(
                        guard_error + '; arm enable cancelled'
                    )
                    return True
                try:
                    self._prepare_pub.publish(String(data=json.dumps({
                        'control_mode': 'position',
                        'source': 'openarmx_teleop_vr_306_v4',
                    }, separators=(',', ':'))))
                except Exception as exc:
                    self._set_hardware_fault(
                        f'failed to request vendor arms-only preparation: {exc}'
                    )
                    return True
                self._enable_stage = 'waiting_prepare_session'
                self._enable_stage_started = now
                self._reason = (
                    'vendor is anchoring the 306 arms at the measured pose'
                )
                return True
            if session_mode != 'sync':
                self._set_hardware_fault(
                    f'unsupported hardware_session_mode: {session_mode}'
                )
                return True
            # Always ask the vendor to confirm this enable attempt.  The
            # process-local acquired bit only controls lease heartbeats; it
            # cannot prove that the vendor watchdog/control state still owns a
            # live SYNC lease after a long disabled interval.
            if not self._sync_session_client.service_is_ready():
                self._set_hardware_fault(
                    'vendor SYNC-session service is unavailable; arm enable cancelled'
                )
                return True
            try:
                self._sync_session_future = self._sync_session_client.call_async(
                    SetBool.Request(data=True)
                )
            except Exception as exc:
                self._set_hardware_fault(
                    f'failed to request guarded ASYNC-HOME-SYNC transition: {exc}'
                )
                return True
            self._enable_stage = 'waiting_sync_session'
            self._enable_stage_started = now
            self._reason = (
                'vendor is verifying the guarded SYNC lease for this enable'
            )
            return True

        if self._enable_stage == 'waiting_prepare_session':
            prepare_wait = max(
                0.5,
                float(self.get_parameter(
                    'arm_enable_prepare_wait_seconds'
                ).value),
            )
            if now - self._enable_stage_started < prepare_wait:
                self._reason = (
                    'waiting for the 306 vendor position hold to stabilize'
                )
                return True
            return self._continue_enable_after_vendor_session(
                now, feedback, '306 arms-only preparation'
            )

        if self._enable_stage == 'waiting_sync_session':
            request_timeout = max(
                1.0,
                float(self.get_parameter('sync_session_request_timeout_sec').value),
            )
            if now - self._enable_stage_started > request_timeout:
                self._sync_session_future = None
                self._set_hardware_fault(
                    'guarded ASYNC-HOME-SYNC transition timed out'
                )
                return True
            future = self._sync_session_future
            if future is None:
                self._set_hardware_fault('SYNC-session request state was lost')
                return True
            if not future.done():
                self._reason = (
                    'waiting for measured-pose stability before entering SYNC'
                )
                return True
            try:
                result = future.result()
            except Exception as exc:
                self._sync_session_future = None
                self._set_hardware_fault(f'SYNC-session request failed: {exc}')
                return True
            self._sync_session_future = None
            if result is None or not result.success:
                message = 'empty response' if result is None else result.message
                self._set_hardware_fault(
                    f'vendor rejected guarded SYNC session: {message}'
                )
                return True
            self._sync_session_acquired = True
            self._publish_sync_session_heartbeat()
            # The vendor has now completed the legal ASYNC -> HOME -> SYNC
            # transition and explicitly confirmed that measured joints are
            # stable.  HOME can legitimately settle a few degrees from the
            # snapshot taken before the transition, so that old snapshot is
            # no longer a valid baseline for our subsequent clear/enable
            # verification.  Rebase to the fresh post-SYNC measurement and
            # continue holding exactly that pose.
            return self._continue_enable_after_vendor_session(
                now, feedback, 'guarded SYNC entry'
            )

        if self._enable_stage == 'clearing_errors':
            clear_wait = max(
                0.5,
                float(self.get_parameter(
                    'arm_enable_clear_wait_seconds'
                ).value),
            )
            self._reason = 'arm joint errors cleared; waiting before enable'
            if now - self._enable_stage_started < clear_wait:
                return True
            guard_error = self._hardware_output_guard_error(require_prepare=False)
            self._last_output_guard_check = now
            if guard_error:
                self._set_hardware_fault(
                    guard_error + '; arm enable cancelled'
                )
                return True
            try:
                enable_payload = {
                    joint_name: True for joint_name in self._teleop_joint_names
                }
                self._joint_enable_pub.publish(String(data=json.dumps(
                    enable_payload, separators=(',', ':')
                )))
                pid_path, was_enabled = enable_vendor_pid_loop(
                    self.get_parameter('vendor_pid_loop_shm_glob').value
                )
            except Exception as exc:
                self._set_hardware_fault(
                    f'arm enable aborted: vendor PID loop unavailable: {exc}'
                )
                return True
            self._enable_pid_path = pid_path
            self._enable_pid_was_enabled = was_enabled
            self._enable_stage = 'settling'
            self._enable_stage_started = now
            self._enable_stable_since = None
            self._enable_last_feedback_groups = {
                key: np.asarray(value, dtype=float).copy()
                for key, value in feedback.as_dict().items()
            }
            self._reason = 'arm joints enabled; verifying held pose'
            return True

        if self._enable_stage != 'settling':
            self._set_hardware_fault(
                f'unknown arm enable stage: {self._enable_stage}'
            )
            return True

        settle_seconds = max(
            0.2,
            float(self.get_parameter('arm_enable_settle_seconds').value),
        )
        self._reason = 'arm joints enabled; verifying held pose'
        if now - self._enable_stage_started < settle_seconds:
            return True

        current = feedback.as_dict()
        pose_error, _ = self._configuration_guard_error(
            current,
            'measured pose after hardware preparation',
            limit_joint_names=TELEOP_LIMIT_JOINT_NAMES,
        )
        if pose_error:
            self._set_hardware_fault(pose_error + '; arm enable cancelled')
            return True
        hold_error, hold_joint = self._maximum_enable_difference(
            current, self._enable_hold_groups
        )
        hold_tolerance = max(
            0.1,
            float(self.get_parameter(
                'hardware_enable_hold_tolerance_deg'
            ).value),
        )
        abort_tolerance = max(
            hold_tolerance,
            float(self.get_parameter(
                'hardware_enable_hold_abort_tolerance_deg'
            ).value),
        )
        if hold_error > abort_tolerance:
            self._set_hardware_fault(
                'arm enable aborted: post-SYNC hold changed '
                f'{hold_error:.2f} deg at {hold_joint}'
            )
            return True

        # Compare against the beginning of the stability window, not merely the
        # previous 100 Hz sample.  A neck drifting slowly by several degrees
        # would otherwise look stable one tiny frame at a time.
        last_groups = self._enable_last_feedback_groups
        frame_delta = float('inf')
        frame_joint = hold_joint
        if last_groups is not None:
            frame_delta, frame_joint = self._maximum_enable_difference(
                current, last_groups
            )
        stable_delta = max(
            0.01,
            float(self.get_parameter(
                'hardware_enable_stable_delta_deg'
            ).value),
        )
        stable_seconds = max(
            0.1,
            float(self.get_parameter(
                'hardware_enable_stable_seconds'
            ).value),
        )
        settle_timeout = max(
            settle_seconds + stable_seconds,
            float(self.get_parameter(
                'hardware_enable_settle_timeout_seconds'
            ).value),
        )
        if frame_delta > stable_delta:
            self._enable_stable_since = now
            self._enable_last_feedback_groups = {
                key: np.asarray(value, dtype=float).copy()
                for key, value in current.items()
            }
            self._reason = (
                'arm joints enabled; waiting for post-SYNC pose to settle '
                f'({frame_joint} moved {frame_delta:.2f} deg)'
            )
            if now - self._enable_stage_started > settle_timeout:
                self._set_hardware_fault(
                    'arm enable aborted: post-SYNC pose did not settle; '
                    f'last movement {frame_delta:.2f} deg at {frame_joint}'
                )
            return True
        if self._enable_stable_since is None:
            self._enable_stable_since = now
            self._enable_last_feedback_groups = {
                key: np.asarray(value, dtype=float).copy()
                for key, value in current.items()
            }
            self._reason = 'post-SYNC pose is settling; confirming stability'
            return True
        if now - self._enable_stable_since < stable_seconds:
            self._reason = 'post-SYNC pose is stable; confirming hold'
            return True

        # A mode transition may leave the neck a few degrees from the snapshot
        # even though it has become completely stationary.  Rebase once to that
        # fresh stationary pose instead of failing on a 0.01-degree threshold
        # crossing.  The independent hard abort above remains in force.
        if hold_error > hold_tolerance and not self._enable_post_sync_rebased:
            self._enable_hold_groups = {
                key: np.asarray(value, dtype=float).copy()
                for key, value in current.items()
            }
            self._enable_post_sync_rebased = True
            self._enable_stable_since = None
            self._enable_last_feedback_groups = self._enable_hold_groups
            try:
                self._publish_enable_hold_pose()
            except Exception as exc:
                self._set_hardware_fault(
                    f'failed to publish settled post-SYNC hold pose: {exc}'
                )
                return True
            self._reason = (
                'post-SYNC pose settled; rebased stationary hold at '
                f'{hold_joint} ({hold_error:.2f} deg)'
            )
            return True

        try:
            self._reset_motion_controllers(current)
        except Exception as exc:
            self._set_hardware_fault(
                f'trajectory limiter reinitialization failed: {exc}'
            )
            return True
        self._target_groups = {
            key: np.asarray(value, dtype=float).copy()
            for key, value in current.items()
        }
        self._target_time = 0.0
        self._target_source = 'measured_hold'
        self._gripper_targets = {
            'left': float(feedback.left_gripper[0]),
            'right': float(feedback.right_gripper[0]),
        }
        self._gripper_dirty = {'left': False, 'right': False}
        self._enable_hold_groups = None
        self._enable_pending = False
        self._enable_stage = None
        self._enable_stable_since = None
        self._enable_last_feedback_groups = None
        self._enable_post_sync_rebased = False
        self._hardware_enabled = True
        if (bool(self.get_parameter('quick_reset_after_hardware_enable').value)
                and not self._enable_full_reset):
            error = self._start_quick_reset_locked(
                now, feedback, source='enable_quick_reset'
            )
            if error:
                self._set_hardware_fault(
                    f'automatic post-enable reset rejected: {error}'
                )
                return True
            self._reason = (
                'hardware enabled; moving to the guarded system reset pose'
            )
        else:
            self._state = 'ARMED'
            self._reason = 'arm hardware ready; holding measured pose'
        return False

    def _continue_enable_after_vendor_session(self, now, feedback, label):
        """Rebase to fresh feedback after the vendor-side mode preparation."""
        session_hold = {
            key: np.asarray(value, dtype=float).copy()
            for key, value in feedback.as_dict().items()
        }
        pose_error, _ = self._configuration_guard_error(
            session_hold,
            f'measured pose after {label}',
            limit_joint_names=TELEOP_LIMIT_JOINT_NAMES,
        )
        if pose_error:
            self._set_hardware_fault(pose_error + '; arm enable cancelled')
            return True
        self._enable_hold_groups = session_hold
        self._enable_stable_since = None
        self._enable_last_feedback_groups = session_hold
        self._enable_post_sync_rebased = False
        try:
            self._publish_enable_hold_pose()
        except Exception as exc:
            self._set_hardware_fault(
                f'failed to rebase post-session hold pose: {exc}'
            )
            return True
        if label == 'guarded SYNC entry':
            # Vendor SYNC acquisition already clears/enables joints and turns
            # on the PID loop. Repeating clear/enable can restart motor ramps.
            self._enable_stage = 'settling'
            self._enable_stage_started = now
            self._reason = 'SYNC acquired; verifying the single system-reset pose'
            return True
        try:
            self._joint_clear_error_pub.publish(String(data=json.dumps(
                self._teleop_joint_names, separators=(',', ':')
            )))
        except Exception as exc:
            self._set_hardware_fault(f'arm joint error clear failed: {exc}')
            return True
        self._enable_stage = 'clearing_errors'
        self._enable_stage_started = now
        self._reason = f'{label} complete; joint errors cleared'
        return True

    def _on_full_body_reset(self, _request, response):
        # Reuse the vendor reset state machine and all hardware-enable guards.
        with self._lock:
            if self._hardware_enabled or self._enable_pending or self._reset_active:
                response.success = False
                response.message = 'disable active output before full-body reset'
                return response
        result = self._on_hardware_enabled(
            SetBool.Request(data=True), SetBool.Response(), force_full_reset=True)
        response.success, response.message = result.success, result.message
        return response

    def _on_hardware_enabled(self, request, response, *, force_full_reset=False):
        with self._lock:
            if not request.data:
                if self._output_control_mode == 'velocity':
                    self._publish_zero_velocity()
                elif (
                    self._hardware_enabled
                    and self._feedback is not None
                    and self._joint_pub is not None
                ):
                    # Opening the web clutch must not release or reposition the
                    # arm.  Freeze the newest measured pose, while the vendor
                    # session intentionally remains in SYNC until process exit.
                    current = self._feedback.as_dict()
                    self._publish_owned_command(
                        self._joint_pub,
                        self._output_topic,
                        json.dumps(
                            self._command_payload(current), separators=(',', ':')
                        ),
                    )
                # Release the vendor lease before rebuilding any controller
                # state.  If reconstruction rejects malformed feedback, the
                # real hardware state still cannot be left in a stale SYNC
                # session.
                self._request_sync_session_exit()
                self._velocity_active_sides.clear()
                self._deactivate_body_height_locked()
                if self._feedback is not None:
                    self._begin_fresh_teleop_session_locked(
                        self._feedback.as_dict()
                    )
                self._hardware_enabled = False
                self._enable_pending = False
                self._enable_hold_groups = None
                self._enable_stage = None
                self._sync_session_future = None
                self._full_reset_stable_since = None
                self._reset_active = False
                self._gripper_dirty = {'left': False, 'right': False}
                self._state = (
                    'E_STOP'
                    if self._estop_latched
                    else ('DRY_RUN' if self._dry_run else 'DISARMED')
                )
                self._reason = 'hardware output disabled; current motor state was not de-energized'
                response.success = True
                response.message = self._reason
                return response
            if self._dry_run:
                response.success = False
                response.message = 'launch with dry_run:=false before enabling hardware'
                return response
            if self._estop_latched:
                response.success = False
                response.message = (
                    'emergency stop is latched; call clear_emergency_stop first'
                )
                return response
            if self._hardware_enabled:
                response.success = True
                response.message = 'hardware output is already enabled'
                return response
            if self._enable_pending:
                response.success = True
                response.message = 'hardware enable is already pending'
                return response
            session_mode = str(
                self.get_parameter('hardware_session_mode').value
            ).strip().lower()
            if session_mode not in ('prepare', 'sync'):
                response.success = False
                response.message = (
                    f'unsupported hardware_session_mode: {session_mode}'
                )
                return response
            if (
                session_mode == 'sync'
                and
                bool(self.get_parameter('require_sync_session_service').value)
                and not self._sync_session_client.service_is_ready()
            ):
                response.success = False
                response.message = (
                    'guarded vendor SYNC-session service is unavailable'
                )
                return response
            now = time.monotonic()
            heartbeat_error = self._heartbeat_guard_error(
                now, require_fresh=True
            )
            if heartbeat_error:
                response.success = False
                response.message = heartbeat_error
                return response
            arbitration_error = self._active_command_guard_error(now)
            if arbitration_error:
                response.success = False
                response.message = arbitration_error
                return response
            timeout = float(self.get_parameter('feedback_timeout_sec').value)
            if self._feedback is None or now - self._feedback_time > timeout:
                response.success = False
                response.message = 'fresh joint feedback is required'
                return response
            if self._collision_required and not self._kinematics.collision_available:
                response.success = False
                response.message = f'collision model unavailable: {self._kinematics.collision_error}'
                return response
            reset_before_enable = force_full_reset or bool(
                self.get_parameter('reset_before_hardware_enable').value
            )
            guard_error = self._hardware_output_guard_error(require_prepare=False)
            if guard_error:
                response.success = False
                response.message = guard_error
                return response
            self._last_output_guard_check = now
            current = self._feedback.as_dict()
            if reset_before_enable:
                if self._full_reset_pub is None:
                    response.success = False
                    response.message = 'full-body reset publisher is unavailable'
                    return response
            else:
                pose_error, _ = self._configuration_guard_error(
                    current,
                    'measured pose',
                    limit_joint_names=TELEOP_LIMIT_JOINT_NAMES,
                )
                if pose_error:
                    response.success = False
                    response.message = pose_error + '; hardware enable rejected'
                    return response
                try:
                    self._begin_fresh_teleop_session_locked(current)
                except Exception as exc:
                    response.success = False
                    response.message = f'trajectory limiter initialization failed: {exc}'
                    return response
                self._target_source = 'measured_hold'
                self._target_time = 0.0
                self._gripper_targets = {
                    'left': float(self._feedback.left_gripper[0]),
                    'right': float(self._feedback.right_gripper[0]),
                }
                self._gripper_dirty = {'left': False, 'right': False}
            self._lock_head_follow_for_control_boundary_locked()
            self._hardware_enabled = False
            self._enable_pending = True
            self._velocity_active_sides.clear()
            if self._velocity_servo is not None:
                self._velocity_servo.stop()
            self._last_hardware_fault_reason = None
            self._enable_full_reset = reset_before_enable
            self._enable_hold_groups = None if reset_before_enable else {
                key: np.asarray(value, dtype=float).copy()
                for key, value in current.items()
            }
            self._enable_started = now
            self._enable_stage_started = now
            self._enable_stage = (
                ('release_sync_before_reset' if session_mode == 'sync'
                 else 'wait_full_reset_subscriber')
                if reset_before_enable
                else 'hold_before_enable'
            )
            self._full_reset_published_at = 0.0
            self._full_reset_stable_since = None
            self._full_reset_max_error_deg = None
            self._full_reset_max_speed_deg_sec = None
            self._enable_pid_path = None
            self._enable_pid_was_enabled = None
            self._state = 'ENABLING'
            self._reason = (
                'full-body reset will run before teleop hardware is enabled'
                if reset_before_enable
                else 'holding measured pose before enabling arm joints'
            )
            response.success = True
            response.message = self._reason
            return response

    def _on_emergency_stop(self, _request, response):
        with self._lock:
            if self._output_control_mode == 'velocity':
                self._publish_zero_velocity()
            self._velocity_active_sides.clear()
            self._hardware_enabled = False
            self._enable_pending = False
            self._enable_hold_groups = None
            self._enable_stage = None
            self._full_reset_stable_since = None
            self._reset_active = False
            self._gripper_dirty = {'left': False, 'right': False}
            self._request_sync_session_exit()
            self._estop_latched = True
            self._state = 'E_STOP'
            self._reason = 'software emergency stop latched; no further commands are published'
        response.success = True
        response.message = self._reason
        return response

    def _on_clear_emergency_stop(self, _request, response):
        with self._lock:
            if self._hardware_enabled or self._enable_pending:
                response.success = False
                response.message = (
                    'disable hardware output and cancel pending enable before clearing '
                    'emergency stop'
                )
                return response
            now = time.monotonic()
            timeout = float(self.get_parameter('feedback_timeout_sec').value)
            if self._feedback is None or now - self._feedback_time > timeout:
                response.success = False
                response.message = 'fresh joint feedback is required to clear emergency stop'
                return response
            self._estop_latched = False
            self._state = 'DRY_RUN' if self._dry_run else 'DISARMED'
            self._reason = 'emergency stop cleared; hardware output remains disabled'
            response.success = True
            response.message = self._reason
            return response

    @staticmethod
    def _endpoint_name(endpoint):
        namespace = endpoint.node_namespace.rstrip('/')
        return f'{namespace}/{endpoint.node_name}'

    def _external_publishers(self, topic, has_self_publisher):
        endpoints = self.get_publishers_info_by_topic(topic)
        self_name = f'{self.get_namespace().rstrip("/")}/{self.get_name()}'
        external = []
        self_endpoint_count = 0
        for endpoint in endpoints:
            name = self._endpoint_name(endpoint)
            if has_self_publisher and name == self_name:
                self_endpoint_count += 1
            else:
                external.append(name)
        # ROS 2 permits duplicate node names.  Excluding every endpoint solely
        # by FQN would therefore miss a second controller instance.
        if has_self_publisher and self_endpoint_count > 1:
            external.append(
                f'{self_name} (duplicate publisher endpoints: {self_endpoint_count})'
            )
        return sorted(set(external))

    def _external_subscribers(self, topic, has_self_subscription=False):
        """Return non-controller subscribers for hardware-readiness checks."""
        endpoints = self.get_subscriptions_info_by_topic(topic)
        self_name = f'{self.get_namespace().rstrip("/")}/{self.get_name()}'
        external = []
        for endpoint in endpoints:
            name = self._endpoint_name(endpoint)
            if has_self_subscription and name == self_name:
                continue
            external.append(name)
        return sorted(set(external))

    def _external_joint_publishers(self):
        publishers = self._external_publishers(
            self._output_topic, self._joint_pub is not None
        )
        publishers.extend(self._external_publishers(
            self._velocity_output_topic, self._velocity_pub is not None
        ))
        return sorted(set(publishers))

    def _legacy_eef_publishers(self):
        return self._external_publishers(self._legacy_eef_topic, False)

    def _external_gripper_publishers(self):
        return self._external_publishers(
            self._gripper_output_topic, self._gripper_pub is not None
        )

    def _external_prepare_publishers(self):
        return self._external_publishers(
            self._prepare_topic, self._prepare_pub is not None
        )

    def _external_joint_enable_publishers(self):
        return self._external_publishers(
            self._joint_enable_topic, self._joint_enable_pub is not None
        )

    def _external_joint_clear_error_publishers(self):
        return self._external_publishers(
            self._joint_clear_error_topic,
            self._joint_clear_error_pub is not None,
        )

    def _refresh_output_guard(self):
        while not self._output_guard_stop.is_set():
            try:
                normal = self._query_hardware_output_guard_error(False)
                prepare = self._query_hardware_output_guard_error(True)
                self._output_guard_snapshot = (time.monotonic(), normal, prepare)
            except Exception:
                pass  # Preserve old timestamp: failed reads are not readiness.
            self._output_guard_stop.wait(0.2)

    def _hardware_output_guard_error(self, require_prepare=False):
        checked, normal, prepare = getattr(self, '_output_guard_snapshot', (0., '', ''))
        if time.monotonic() - checked > 0.6:
            return 'DDS output-guard snapshot unavailable or stale'
        return prepare if require_prepare else normal

    def _query_hardware_output_guard_error(self, require_prepare=False):
        active_publisher = (
            self._velocity_pub
            if self._output_control_mode == 'velocity'
            else self._joint_pub
        )
        active_topic = (
            self._velocity_output_topic
            if self._output_control_mode == 'velocity'
            else self._output_topic
        )
        if active_publisher is None or self._joint_pub is None:
            return 'hardware command publisher is unavailable'
        if bool(self.get_parameter('require_hardware_subscriber').value):
            # Do not count our read-only arbitration monitor as the hardware
            # consumer that makes output safe to publish.
            if not self._external_subscribers(
                    active_topic, has_self_subscription=True):
                return 'whole-body joint command subscriber is unavailable'
        if bool(self.get_parameter('require_gripper_subscriber').value):
            if self._gripper_pub is None:
                return 'gripper command publisher is unavailable'
            if not self._external_subscribers(
                    self._gripper_output_topic, has_self_subscription=True):
                return 'gripper command subscriber is unavailable'
        if require_prepare and bool(
                self.get_parameter('require_prepare_subscriber').value):
            if self._prepare_pub is None:
                return 'arm hardware preparation publisher is unavailable'
            if not self._external_subscribers(self._prepare_topic):
                return 'arm hardware preparation subscriber is unavailable'
        if require_prepare and bool(
                self.get_parameter('require_joint_enable_subscribers').value):
            if self._joint_enable_pub is None or not self._external_subscribers(
                    self._joint_enable_topic):
                return 'arm joint enable subscriber is unavailable'
            if self._joint_clear_error_pub is None or not self._external_subscribers(
                    self._joint_clear_error_topic):
                return 'arm joint error-clear subscriber is unavailable'
        if bool(self.get_parameter('reject_competing_publishers').value):
            competing = self._external_joint_publishers()
            if competing:
                return 'competing joint command publisher(s): ' + ', '.join(competing)
            legacy = self._legacy_eef_publishers()
            if legacy:
                return 'legacy Cartesian command publisher(s): ' + ', '.join(legacy)
            gripper = self._external_gripper_publishers()
            if gripper:
                return 'competing gripper command publisher(s): ' + ', '.join(gripper)
            # Do not reject prepare/enable/clear publisher endpoints.  On 306
            # the required vendor service nodes register these endpoints while
            # idle and may also publish as part of the prepare callback.
        return ''

    @staticmethod
    def _command_payload(groups):
        return {
            'leg_waist_target_joints_position': [float(x) for x in groups['leg_waist']],
            'left_arm_target_joints_position': [float(x) for x in groups['left_arm']],
            'right_arm_target_joints_position': [float(x) for x in groups['right_arm']],
            'neck_target_joints_position': [float(x) for x in groups['neck']],
        }

    def _control_tick(self):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        with self._lock:
            feedback = self._feedback
            feedback_age = now - self._feedback_time
            target_age = now - self._target_time if self._target_time else None
            limiter = self._limiter
            target_groups = self._target_groups
            enabled = self._hardware_enabled
            velocity_mode = self._output_control_mode == 'velocity'
            direct_position_mode = (
                not velocity_mode
                and not self._reset_active
                and bool(self.get_parameter(
                    'direct_position_target_enabled').value)
            )
            body_height_output_active = False
            body_height_output_lead = max(
                0.1, float(self._body_height_command_lead_deg)
            )
            waist_assist_output_active = bool(
                self._waist_follow_active()
                and self._waist_follow_profile == 'forward_pitch_only'
                and self._waist_follow_neutral_pitch_deg is not None
                and self._waist_follow_pitch_target_deg is not None
                and float(self._waist_follow_pitch_target_deg)
                > float(self._waist_follow_neutral_pitch_deg) + 1.0e-3
            )
            waist_assist_output_lead = max(
                0.1,
                float(self.get_parameter(
                    'waist_forward_assist_command_lead_deg').value),
            )

            if self._enable_pending:
                if self._run_hardware_enable(now, feedback, feedback_age):
                    self._publish_status(now, feedback_age, target_age)
                    return
                # The state machine has just reached ARMED.  Refresh the local
                # snapshot so this first command cannot use a pre-enable pose.
                limiter = self._limiter
                target_groups = self._target_groups
                target_age = None
                enabled = True
                # Hardware enable may have started the shared automatic quick
                # reset.  Recompute this mode flag after the state transition;
                # otherwise its first cycle could wrongly use the normal
                # direct-position path and bypass the conservative reset
                # velocity/acceleration limiter.
                direct_position_mode = (
                    not velocity_mode
                    and not self._reset_active
                    and bool(self.get_parameter(
                        'direct_position_target_enabled').value)
                )

            if feedback is None or feedback_age > float(
                    self.get_parameter('feedback_timeout_sec').value):
                if enabled or self._reset_active:
                    self._set_hardware_fault(
                        'feedback watchdog expired; hardware publishing stopped'
                    )
                self._publish_status(now, feedback_age, target_age)
                return
            if limiter is None or target_groups is None:
                self._publish_status(now, feedback_age, target_age)
                return

            if enabled:
                heartbeat_error = self._heartbeat_guard_error(now)
                if heartbeat_error:
                    self._set_hardware_fault(
                        heartbeat_error + '; hardware output stopped'
                    )
                    self._publish_status(now, feedback_age, target_age)
                    return
                guard_period = max(
                    0.10, float(self.get_parameter('output_guard_period_sec').value)
                )
                if now - self._last_output_guard_check >= guard_period:
                    self._last_output_guard_check = now
                    guard_error = self._hardware_output_guard_error()
                    if guard_error:
                        self._set_hardware_fault(
                            guard_error + '; hardware publishing stopped'
                        )
                        self._publish_status(now, feedback_age, target_age)
                        return

            current_vector = self._group_vector(feedback.as_dict())
            body_height_output_target = self._body_height_output_target_locked(now, feedback)
            body_height_output_active = body_height_output_target is not None
            body_height_output_lead = max(0.1, float(self._body_height_command_lead_deg))
            desired = self._group_vector(target_groups)
            head_target_fresh = (
                self._head_follow_active()
                and not self._reset_active
                and not self._enable_pending
                and self._head_target is not None
                and now - self._head_target_time <= float(
                    self.get_parameter('head_target_timeout_sec').value
                )
            )
            task_head_fresh = (
                self._follow_authority_allowed()
                and
                not self._head_follow_active() and not self._reset_active
                and not self._enable_pending
                and self._task_head_target is not None
                and now - self._task_head_target_time <= 0.6
            )
            head_command = self._head_target if head_target_fresh else self._task_head_target
            head_target_fresh = head_target_fresh or task_head_fresh
            if self._reset_active:
                # Quick reset owns the neck target.  The former generic
                # "stale HMD -> hold measured neck" fallback overwrote the
                # configured reset pose on every control tick, so the arms and
                # waist reached their goals while the neck never moved and the
                # reset inevitably timed out.
                pass
            elif head_target_fresh:
                desired[18:21] = head_command
            elif (self._follow_authority_required
                  and not self._follow_authority_allowed()
                  and self._target_source == 'joint_space'):
                # Policy owns the neck too; never overwrite it with HMD/hold.
                pass
            else:
                # Do not chase a stale headset sample.  Holding measured neck
                # feedback is independent from the arm target watchdog.
                desired[18:21] = current_vector[18:21]
            if (
                self._reset_active
                and self._reset_path_start is not None
                and self._reset_path_goal is not None
                and self._reset_path_duration_sec > 0.0
            ):
                # Advance from control-cycle time rather than wall time. A
                # stalled process therefore resumes smoothly instead of
                # jumping several degrees on the next command packet.
                previous_progress = self._reset_path_elapsed_sec / self._reset_path_duration_sec
                path_progress = min(
                    1.0,
                    previous_progress + min(max(float(dt), 0.0), 0.05) / self._reset_path_duration_sec,
                )
                path_progress = feedback_bounded_reset_progress(
                    self._reset_path_start, self._reset_path_goal, current_vector,
                    previous_progress, path_progress,
                    float(self.get_parameter('quick_reset_waist_max_command_lead_deg').value),
                )
                self._reset_path_elapsed_sec = path_progress * self._reset_path_duration_sec
                desired = continuous_direct_reset(
                    self._reset_path_start,
                    self._reset_path_goal,
                    path_progress,
                )
            target_stale = (
                not self._reset_active
                and target_age is not None
                and target_age > float(
                    self.get_parameter('target_timeout_sec').value
                )
            )
            if target_stale:
                # Position control used to keep chasing the last interpolated
                # command here.  SPEED control must instead stop in this very
                # packet, otherwise a released/stalled VR client can leave the
                # arm moving for several hundred milliseconds.
                # The arm Cartesian watchdog must stop only the arm stream.
                # X/Y body-height commands arrive on an independent channel;
                # replacing the whole vector with feedback here used to erase
                # waist pitch while ankle and knee continued along the height
                # curve.  Preserve the complete three-joint height target so
                # the chassis stays geometrically coordinated even when the
                # hands are released or a WebXR arm frame is late.
                body_height_watchdog_target = desired[0:3].copy()
                desired = current_vector.copy()
                if body_height_output_active:
                    desired[0:3] = body_height_watchdog_target
                # The arm Cartesian stream and HMD neck stream have independent
                # clocks.  A released hand must stop the arms without stopping
                # a still-fresh, explicitly enabled head-follow stream.
                if head_target_fresh:
                    desired[18:21] = head_command
                if (
                    not body_height_output_active
                    and
                    self._waist_follow_active()
                    and self._waist_follow_profile == 'forward_pitch_only'
                    and self._waist_follow_neutral_pitch_deg is not None
                ):
                    # Arm targets fail closed at measured joints, but the
                    # waist must not be abandoned halfway through returning
                    # upright when the final hand frame becomes stale.
                    neutral_pitch = float(
                        self._waist_follow_neutral_pitch_deg
                    )
                    desired[2] = neutral_pitch
                    self._waist_follow_pitch_target_deg = neutral_pitch
                    self._waist_follow_extension_m = 0.0
                self._target_velocity_estimate.fill(0.0)
                if velocity_mode:
                    self._velocity_active_sides.clear()
                    self._velocity_servo.stop()
                if enabled and self._state == 'ARMED':
                    self._state = 'HOLDING'
                    self._reason = (
                        'body-height watchdog expired; holding ankle, knee and waist together'
                        if body_height_output_active and self._body_height_watchdog_stopped else
                        'arm target watchdog expired; coordinated body-height control remains active'
                        if body_height_output_active else
                        'target watchdog expired; velocity command stopped'
                        if velocity_mode else
                        'target watchdog expired; holding measured position'
                    )
            if body_height_output_active:
                # Height owns ankle/knee/waist even if an in-flight arm IK
                # result or grip release replaced the shared target vector.
                desired[0:3] = body_height_output_target
            # Remember the reset state before this control cycle can mark it
            # complete.  This guarantees that a body already at its reset pose
            # still emits at least one gripper-open command.
            reset_was_active = self._reset_active
            previous_command_vector = (
                limiter.position.copy()
                if self._last_position_command is None
                else self._last_position_command.copy()
            )
            if enabled:
                arm_lead_parameter = (
                    'quick_reset_max_command_lead_deg'
                    if self._reset_active
                    else (
                        'direct_position_max_command_lead_deg'
                        if direct_position_mode
                        else 'arm_max_command_lead_deg'
                    )
                )
                waist_lead_parameter = (
                    'quick_reset_waist_max_command_lead_deg'
                    if self._reset_active
                    else 'waist_max_command_lead_deg'
                )
                maximum_lead = np.full(desired.shape, 1.0e6, dtype=float)
                maximum_lead[2:4] = float(
                    self.get_parameter(waist_lead_parameter).value
                )
                if self._reset_active:
                    maximum_lead[0:3] = float(self.get_parameter(waist_lead_parameter).value)
                if body_height_output_active:
                    # Height control is a single three-joint mechanism. Apply
                    # the same live feedback window to ankle, knee and waist
                    # pitch instead of constraining only waist pitch.
                    maximum_lead[0:3] = body_height_output_lead
                if waist_assist_output_active and not self._reset_active:
                    # Forward reach is a separate waist-pitch contribution.
                    # Do not let the tight stopped-height hold window delay it;
                    # ankle and knee remain on the coordinated height window.
                    maximum_lead[2] = max(
                        maximum_lead[2], waist_assist_output_lead
                    )
                maximum_lead[18:21] = float(
                    self.get_parameter('neck_max_command_lead_deg').value
                )
                if (
                    direct_position_mode
                    and not target_stale
                    and bool(self.get_parameter(
                        'dynamic_command_lead_enabled').value)
                ):
                    (
                        self._dynamic_lead_values,
                        self._dynamic_following_error,
                    ) = self._dynamic_command_lead.update(
                        current_vector,
                        desired,
                        self._measured_velocity_vector(),
                        previous_command_vector,
                        max(float(dt), 1.0e-6),
                        minimum=float(self.get_parameter(
                            'dynamic_command_lead_min_deg').value),
                        nominal=float(self.get_parameter(
                            'dynamic_command_lead_nominal_deg').value),
                        maximum=float(self.get_parameter(
                            'dynamic_command_lead_max_deg').value),
                        speed_low=float(self.get_parameter(
                            'dynamic_command_lead_speed_low_deg_sec').value),
                        speed_high=float(self.get_parameter(
                            'dynamic_command_lead_speed_high_deg_sec').value),
                        target_error_low=float(self.get_parameter(
                            'dynamic_command_lead_target_error_low_deg').value),
                        target_error_high=float(self.get_parameter(
                            'dynamic_command_lead_target_error_high_deg').value),
                        following_error_soft=float(self.get_parameter(
                            'dynamic_command_lead_following_soft_deg').value),
                        following_error_hard=float(self.get_parameter(
                            'dynamic_command_lead_following_hard_deg').value),
                        expansion_filter_tau=float(self.get_parameter(
                            'dynamic_command_lead_filter_tau_sec').value),
                    )
                    maximum_lead[4:18] = self._dynamic_lead_values[4:18]
                else:
                    fixed_arm_lead = float(
                        self.get_parameter(arm_lead_parameter).value
                    )
                    maximum_lead[4:18] = fixed_arm_lead
                    if direct_position_mode:
                        self._dynamic_lead_values = (
                            self._dynamic_command_lead.reset(fixed_arm_lead)
                        )
                        self._dynamic_following_error = np.abs(
                            previous_command_vector - current_vector
                        )
                desired = bounded_target_by_feedback(
                    current_vector,
                    desired,
                    maximum_lead,
                )
            try:
                limiter_kwargs = {}
                if self._reset_active:
                    limiter_kwargs = {
                        'max_velocity': self._quick_reset_limit_vector(
                            'quick_reset_max_velocity_deg_sec',
                            'quick_reset_neck_max_velocity_deg_sec',
                        ),
                        'max_acceleration': self._quick_reset_limit_vector(
                            'quick_reset_max_acceleration_deg_sec2',
                            'quick_reset_neck_max_acceleration_deg_sec2',
                        ),
                    }
                elif (
                    not velocity_mode
                    and bool(self.get_parameter(
                        'adaptive_position_feedforward_enabled'
                    ).value)
                ):
                    previous_output = (
                        previous_command_vector
                        if self._last_position_command is None
                        else self._last_position_command
                    )
                    (
                        self._adaptive_lookahead,
                        self._adaptive_velocity_limit,
                        self._adaptive_acceleration,
                        self._adaptive_following_error,
                    ) = self._adaptive_feedforward.update(
                        current_vector,
                        previous_output,
                        (
                            self._target_velocity_estimate
                            if direct_position_mode else limiter.velocity
                        ),
                        desired,
                        dt,
                        minimum_lookahead=float(self.get_parameter(
                            'adaptive_position_min_lookahead_sec').value),
                        maximum_lookahead=float(self.get_parameter(
                            'adaptive_position_max_lookahead_sec').value),
                        filter_tau=float(self.get_parameter(
                            'adaptive_position_lookahead_filter_tau_sec').value),
                        velocity_floor=float(self.get_parameter(
                            'adaptive_position_velocity_floor_deg_sec').value),
                        minimum_velocity=float(self.get_parameter(
                            'adaptive_position_min_velocity_deg_sec').value),
                        maximum_velocity=float(self.get_parameter(
                            'max_velocity_deg_sec').value),
                        nominal_acceleration=float(self.get_parameter(
                            'max_acceleration_deg_sec2').value),
                        maximum_acceleration=float(self.get_parameter(
                            'adaptive_position_max_acceleration_deg_sec2').value),
                        error_low=float(self.get_parameter(
                            'adaptive_position_error_low_deg').value),
                        error_high=float(self.get_parameter(
                            'adaptive_position_error_high_deg').value),
                    )
                    limiter_kwargs.update({
                        'max_velocity': self._adaptive_velocity_limit,
                        'max_acceleration': self._adaptive_acceleration,
                    })
                if direct_position_mode:
                    lookahead = (
                        self._adaptive_lookahead
                        if bool(self.get_parameter(
                            'adaptive_position_feedforward_enabled').value)
                        else np.zeros_like(desired)
                    )
                    command_vector = predict_latest_target(
                        desired,
                        self._target_velocity_estimate,
                        lookahead,
                        limiter.lower,
                        limiter.upper,
                    )
                    if enabled:
                        command_vector = bounded_target_by_feedback(
                            current_vector,
                            command_vector,
                            maximum_lead,
                        )
                    # Keep reset/collision rollback state aligned, but do not
                    # make normal teleoperation traverse a second trajectory.
                    limiter.reset(command_vector)
                else:
                    command_vector = limiter.step(
                        desired, dt, **limiter_kwargs
                    )
            except Exception as exc:
                reason = f'trajectory limiter failed closed: {exc}'
                if enabled:
                    self._set_hardware_fault(reason)
                else:
                    self._reset_active = False
                    self._state = 'FAULT'
                    self._reason = reason
                self._publish_status(now, feedback_age, target_age)
                return
            if (
                not velocity_mode
                and not direct_position_mode
                and bool(self.get_parameter(
                    'hybrid_position_reference_enabled'
                ).value)
            ):
                adaptive_enabled = bool(self.get_parameter(
                    'adaptive_position_feedforward_enabled').value)
                if adaptive_enabled and not self._reset_active:
                    command_vector = project_position(
                        command_vector,
                        limiter.velocity,
                        desired,
                        self._adaptive_lookahead,
                    )
                else:
                    output_lookahead = (
                        0.0 if self._reset_active else max(
                            0.0,
                            float(self.get_parameter(
                                'hybrid_position_output_lookahead_sec'
                            ).value),
                        )
                    )
                    command_vector = lookahead_reference(
                        command_vector,
                        limiter.velocity,
                        desired,
                        output_lookahead,
                    )
                # Feed-forward is never allowed to bypass the measured command
                # lead guard that protects the heavy 306 arms.
                if enabled:
                    command_vector = bounded_target_by_feedback(
                        current_vector,
                        command_vector,
                        maximum_lead,
                    )
            command_groups = self._vector_groups(command_vector, feedback)
            velocity_command = np.zeros_like(command_vector)
            if velocity_mode:
                try:
                    servo_goal = desired
                    if bool(self.get_parameter(
                            'hybrid_position_reference_enabled').value):
                        lookahead = (
                            0.0 if self._reset_active else max(
                                0.0,
                                float(self.get_parameter(
                                    'hybrid_position_reference_lookahead_sec'
                                ).value),
                            )
                        )
                        servo_goal = lookahead_reference(
                            command_vector,
                            limiter.velocity,
                            desired,
                            lookahead,
                        )
                    servo_kwargs = {}
                    if self._reset_active:
                        servo_kwargs = {
                            'maximum_velocity': self._quick_reset_limit_vector(
                                'quick_reset_max_velocity_deg_sec',
                                'quick_reset_neck_max_velocity_deg_sec',
                            ),
                            'maximum_acceleration': self._quick_reset_limit_vector(
                                'quick_reset_max_acceleration_deg_sec2',
                                'quick_reset_neck_max_acceleration_deg_sec2',
                            ),
                            'maximum_jerk': self._quick_reset_limit_vector(
                                'quick_reset_max_jerk_deg_sec3',
                                'quick_reset_neck_max_jerk_deg_sec3',
                            ),
                        }
                    velocity_command = self._velocity_servo.step(
                        current_vector,
                        servo_goal,
                        dt,
                        self._active_velocity_indices(),
                        **servo_kwargs,
                    )
                except Exception as exc:
                    reason = f'velocity servo failed closed: {exc}'
                    if enabled:
                        self._set_hardware_fault(reason)
                    else:
                        self._reset_active = False
                        self._state = 'FAULT'
                        self._reason = reason
                    self._publish_status(now, feedback_age, target_age)
                    return
            if self._reset_active:
                reset_goal = self._reset_required_vector(target_groups)
                remaining_command = float(np.max(np.abs(
                    reset_goal - self._reset_required_vector(command_groups)
                )))
                self._reset_progress = max(
                    self._reset_progress,
                    min(1.0, 1.0 - remaining_command / self._reset_initial_error),
                )
                remaining_actual = (
                    remaining_command
                    if self._dry_run
                    else float(np.max(np.abs(
                        reset_goal - self._reset_required_vector(feedback.as_dict())
                    )))
                )
                tolerance = max(
                    0.0,
                    float(self.get_parameter('quick_reset_tolerance_deg').value),
                )
                if remaining_command <= tolerance and remaining_actual <= tolerance:
                    self._reset_active = False
                    self._reset_path_start = None
                    self._reset_path_goal = None
                    self._reset_progress = 1.0
                    self._velocity_active_sides.clear()
                    if velocity_mode:
                        velocity_command = self._velocity_servo.stop()
                    self._state = 'ARMED' if enabled else (
                        'DRY_RUN' if self._dry_run else 'DISARMED'
                    )
                    self._reason = (
                        'quick reset completed; release both grips before re-clutching'
                    )
                # Physical height travel is slower than the arm reference.
                # Use the same conservative 2 cm/s budget as task-height
                # control, retaining a finite timeout and feedback completion.
                elif now - self._reset_started > max(
                        0.1,
                        float(self.get_parameter('quick_reset_timeout_sec').value)
                        + MAX_BODY_LOWERING_M / 0.02 * float(np.max(np.abs(
                            (self._reset_path_goal[:3] - self._reset_path_start[:3])
                            / np.asarray(BODY_HEIGHT_MAXIMUM_DEG))))):
                    actual_difference = np.abs(
                        reset_goal - self._reset_required_vector(feedback.as_dict())
                    )
                    command_difference = np.abs(
                        reset_goal - self._reset_required_vector(command_groups)
                    )
                    worst_index = int(np.argmax(actual_difference))
                    worst_joint = RESET_REQUIRED_JOINT_NAMES[worst_index]
                    worst_actual = float(actual_difference[worst_index])
                    worst_command = float(command_difference[worst_index])
                    self._reset_active = False
                    self._reset_path_start = None
                    self._reset_path_goal = None
                    if enabled:
                        self._set_hardware_fault(
                            'quick reset timed out at '
                            f'{worst_joint}: actual error {worst_actual:.2f} deg, '
                            f'command error {worst_command:.2f} deg; '
                            'hardware output stopped'
                        )
                        enabled = False
                    else:
                        self._state = 'DRY_RUN' if self._dry_run else 'DISARMED'
                        self._reason = 'quick reset preview timed out'
            # Publish immediately, including when this same cycle completes
            # the reset, then retry at 5 Hz until completion.  This is robust
            # against a dropped DDS sample without loading the vendor gripper
            # worker with the 100 Hz arm-control rate.
            if reset_was_active and bool(
                    self.get_parameter('quick_reset_open_grippers').value):
                if (
                    self._reset_gripper_last_publish_time <= 0.0
                    or now - self._reset_gripper_last_publish_time >= 0.20
                    or not self._reset_active
                ):
                    self._gripper_dirty = {'left': True, 'right': True}
            if self._collision_required and self._kinematics.collision_available:
                try:
                    collision_groups = command_groups
                    if velocity_mode:
                        horizon = max(
                            0.0,
                            float(self.get_parameter(
                                'velocity_servo_collision_horizon_sec'
                            ).value),
                        )
                        predicted_vector = current_vector + velocity_command * horizon
                        collision_groups = self._vector_groups(
                            predicted_vector, feedback
                        )
                    command_q = self._kinematics.q_from_feedback(collision_groups)
                    self._last_command_collisions = (
                        self._kinematics.collision_pairs(command_q)
                    )
                except Exception as exc:
                    reason = f'command collision check failed closed: {exc}'
                    if enabled:
                        self._set_hardware_fault(reason)
                        enabled = False
                    else:
                        self._reset_active = False
                        self._state = 'FAULT'
                        self._reason = reason
                    self._publish_status(now, feedback_age, target_age)
                    return
                if enabled and self._last_command_collisions:
                    # A Cartesian IK stream can briefly place the next
                    # interpolated sample on a collision boundary.  Never
                    # publish that sample, but do not permanently disarm the
                    # robot for a single rejected frame either.  Rewind the
                    # limiter to the last collision-free command, hold there,
                    # and automatically continue once a safe target arrives.
                    command_vector = limiter.reset(previous_command_vector)
                    command_groups = self._vector_groups(command_vector, feedback)
                    if velocity_mode:
                        velocity_command = self._velocity_servo.stop()
                    self._state = 'HOLDING'
                    self._reason = (
                        'unsafe self-collision frame rejected; holding last safe pose'
                    )
                    if now - self._last_collision_hold_warning >= 1.0:
                        self._last_collision_hold_warning = now
                        self.get_logger().warning(
                            self._reason + ': '
                            + ', '.join(self._last_command_collisions)
                        )
            self._last_position_command = command_vector.copy()
            preview = self._command_payload(command_groups)
            if self._gripper_targets['left'] is not None:
                preview['left_gripper_target_joints_position'] = [
                    float(self._gripper_targets['left'])
                ]
            if self._gripper_targets['right'] is not None:
                preview['right_gripper_target_joints_position'] = [
                    float(self._gripper_targets['right'])
                ]
            preview.update({
                'state': self._state,
                'hardware_enabled': bool(enabled),
                'dry_run': self._dry_run,
                'output_control_mode': self._output_control_mode,
            })
            if velocity_mode:
                preview['joint_velocity_command_deg_sec'] = (
                    self._velocity_payload(velocity_command)
                )
            self._preview_pub.publish(String(data=json.dumps(preview, separators=(',', ':'))))

            current_controlled = self._reset_vector(feedback.as_dict())
            command_controlled = self._reset_vector(command_groups)
            tracking_error = float(np.max(np.abs(
                current_controlled - command_controlled
            )))
            if enabled and tracking_error > float(
                    self.get_parameter('tracking_error_limit_deg').value):
                self._set_hardware_fault(
                    f'tracking error {tracking_error:.1f} deg exceeds limit; output stopped'
                )
                enabled = False
            if enabled and self._joint_pub is not None:
                # Keep publishing the position frame because leg/waist and
                # neck remain in POSITION mode.  The vendor worker ignores
                # arm positions while both arms are configured for SPEED.
                self._publish_owned_command(
                    self._joint_pub,
                    self._output_topic,
                    json.dumps(
                        self._command_payload(command_groups), separators=(',', ':')
                    ),
                )
                if velocity_mode:
                    self._publish_velocity_command(velocity_command)
                gripper_payload = {}
                for side in ('left', 'right'):
                    if self._gripper_dirty[side] and self._gripper_targets[side] is not None:
                        gripper_payload[
                            f'{side}_gripper_target_joints_position'
                        ] = [float(self._gripper_targets[side])]
                if gripper_payload and self._gripper_pub is not None:
                    self._publish_owned_command(
                        self._gripper_pub,
                        self._gripper_output_topic,
                        json.dumps(gripper_payload, separators=(',', ':')),
                    )
                    if reset_was_active:
                        self._reset_gripper_last_publish_time = now
                    for side in ('left', 'right'):
                        key = f'{side}_gripper_target_joints_position'
                        if key in gripper_payload:
                            self._gripper_dirty[side] = False
            self._publish_status(now, feedback_age, target_age, tracking_error)

    def _measured_body_lowering_m(self, signed=False):
        if self._feedback is None:
            return None
        joints = np.asarray(self._feedback.as_dict()['leg_waist'], dtype=float).copy()
        if joints.shape != (4,) or not np.all(np.isfinite(joints)):
            return None
        if (self._waist_follow_active()
                and self._waist_follow_profile == 'forward_pitch_only'
                and self._waist_follow_neutral_pitch_deg is not None
                and self._waist_follow_pitch_target_deg is not None):
            joints[2] -= max(0.0, self._waist_follow_pitch_target_deg
                            - self._waist_follow_neutral_pitch_deg)
        value = estimate_body_lowering(joints, signed=True)
        return value if signed else abs(value)

    def _publish_status(self, now, feedback_age, target_age, tracking_error=None):
        if now - self._last_status < 0.10:
            return
        self._last_status = now
        latest_target_stats = self._latest_target_mailbox.stats()
        arm_slice = slice(4, 18)
        shoulder_indices = np.asarray((4, 5, 11, 12), dtype=int)
        shoulder_command_error = np.zeros(4, dtype=float)
        shoulder_target_error = np.zeros(4, dtype=float)
        if self._feedback is not None:
            measured_vector = self._group_vector(self._feedback.as_dict())
            if self._last_position_command is not None:
                shoulder_command_error = np.abs(
                    self._last_position_command[shoulder_indices]
                    - measured_vector[shoulder_indices]
                )
            if self._target_groups is not None:
                target_vector = self._group_vector(self._target_groups)
                shoulder_target_error = np.abs(
                    target_vector[shoulder_indices]
                    - measured_vector[shoulder_indices]
                )
        payload = {
            'state': self._state,
            'reason': self._reason,
            'detail': self._reason,
            'dry_run': self._dry_run,
            'hardware_enabled': self._hardware_enabled,
            'output_control_mode': self._output_control_mode,
            'direct_position_target_enabled': bool(self.get_parameter(
                'direct_position_target_enabled').value),
            'hardware_enable_pending': self._enable_pending,
            'hardware_enable_stage': self._enable_stage,
            'vendor_sync_session_acquired': self._sync_session_acquired,
            'reset_before_hardware_enable': bool(
                self.get_parameter('reset_before_hardware_enable').value
            ),
            'quick_reset_after_hardware_enable': bool(
                self.get_parameter('quick_reset_after_hardware_enable').value
            ),
            'waist_follow_enabled': self._waist_follow_enabled,
            'waist_follow_profile': self._waist_follow_profile,
            'waist_follow_neutral_pitch_deg': self._waist_follow_neutral_pitch_deg,
            'waist_follow_locked_yaw_deg': self._waist_follow_locked_yaw_deg,
            'waist_follow_anchor_x': dict(self._waist_follow_anchor_x),
            'waist_follow_extension_m': self._waist_follow_extension_m,
            'waist_follow_pitch_target_deg': self._waist_follow_pitch_target_deg,
            'body_height': {
                'enabled': self._body_height_control_enabled,
                'active': self._body_height_active,
                'watchdog_stopped': self._body_height_watchdog_stopped,
                'joint_targets_deg': (
                    None if self._body_height_joint_targets is None
                    else self._body_height_joint_targets.tolist()
                ),
                'lowering_m': (None if self._body_height_lowering_m is None else abs(self._body_height_lowering_m)),
                'measured_lowering_m': self._measured_body_lowering_m(),
                'measured_signed_lowering_m': self._measured_body_lowering_m(signed=True),
                'feedback_time_monotonic': self._feedback_time,
                'feedback_fresh': self._feedback is not None and feedback_age <= float(
                    self.get_parameter('feedback_timeout_sec').value),
                'command_progress': self._body_height_command_progress,
                'command_lead_deg': self._body_height_command_lead_deg,
                'direction': self._body_height_last_direction,
                'command_age': (
                    None if self._body_height_command_time <= 0.0
                    else max(0.0, now - self._body_height_command_time)
                ),
            },
            'head_follow_enabled': self._head_follow_enabled,
            'task_head_cancelled_id': self._task_head_cancelled_id,
            'task_head_cancel_reason': self._task_head_cancel_reason,
            'task_head_goal': self._task_head_goal,
            'clutch_session': self._clutch_session,
            'clutch_sequence': self._clutch_sequence,
            'clutch_held_sides': sorted(self._grip_release_held),
            'measured_neck_deg': (
                [] if self._feedback is None
                else [float(value) for value in self._feedback.neck]
            ),
            'head_target_deg': (
                None if self._head_target is None
                else [float(value) for value in self._head_target]
            ),
            'head_target_age_sec': (
                None if self._head_target_time <= 0.0
                else round(max(0.0, now - self._head_target_time), 4)
            ),
            'full_reset_topic': self._full_reset_topic,
            'full_reset_max_error_deg': self._full_reset_max_error_deg,
            'full_reset_max_speed_deg_sec': self._full_reset_max_speed_deg_sec,
            'hardware_ready': (
                self._hardware_enabled
                and not self._enable_pending
                and not self._reset_active
            ),
            'last_hardware_fault_reason': self._last_hardware_fault_reason,
            'vendor_pid_loop_path': self._enable_pid_path,
            'vendor_pid_loop_was_enabled': self._enable_pid_was_enabled,
            'quick_reset_active': self._reset_active,
            'quick_reset_progress': round(float(self._reset_progress), 3),
            'emergency_stop_latched': self._estop_latched,
            'feedback_age_sec': round(float(feedback_age), 4),
            'target_age_sec': None if target_age is None else round(float(target_age), 4),
            'collision_check_available': self._kinematics.collision_available,
            'tracking_error_deg': tracking_error,
            'shoulder_tracking': {
                'joint_order': [
                    'left_shoulder_inner',
                    'left_shoulder_outer',
                    'right_shoulder_inner',
                    'right_shoulder_outer',
                ],
                'command_following_error_deg': [
                    round(float(value), 3) for value in shoulder_command_error
                ],
                'target_error_deg': [
                    round(float(value), 3) for value in shoulder_target_error
                ],
                'maximum_command_following_error_deg': round(
                    float(np.max(shoulder_command_error)), 3
                ),
                'maximum_target_error_deg': round(
                    float(np.max(shoulder_target_error)), 3
                ),
                # The vendor worker provides the final fixed hard cap.  Our
                # joint-wise dynamic lead below determines how much of that
                # envelope is requested and contracts immediately when the
                # measured following error deteriorates.
                'vendor_sync_speed_cap_deg_sec': float(self.get_parameter(
                    'shoulder_sync_speed_cap_deg_sec').value
                ),
            },
            'latest_frame_policy': latest_target_stats,
            'adaptive_position_feedforward': {
                'enabled': bool(self.get_parameter(
                    'adaptive_position_feedforward_enabled').value),
                'max_lookahead_sec': round(float(np.max(
                    self._adaptive_lookahead[arm_slice]
                )), 4),
                'max_acceleration_deg_sec2': round(float(np.max(
                    self._adaptive_acceleration[arm_slice]
                )), 2),
                'max_velocity_deg_sec': round(float(np.max(
                    self._adaptive_velocity_limit[arm_slice]
                )), 2),
                'max_following_error_deg': round(float(np.max(
                    self._adaptive_following_error[arm_slice]
                )), 3),
                'max_ik_target_velocity_deg_sec': round(float(np.max(np.abs(
                    self._target_velocity_estimate[arm_slice]
                ))), 2),
            },
            'dynamic_command_lead': {
                'enabled': bool(self.get_parameter(
                    'dynamic_command_lead_enabled').value),
                'minimum_arm_lead_deg': round(float(np.min(
                    self._dynamic_lead_values[arm_slice]
                )), 3),
                'maximum_arm_lead_deg': round(float(np.max(
                    self._dynamic_lead_values[arm_slice]
                )), 3),
                'maximum_following_error_deg': round(float(np.max(
                    self._dynamic_following_error[arm_slice]
                )), 3),
            },
            'command_collision_pairs': self._last_command_collisions,
            'competing_joint_publishers': self._external_joint_publishers(),
            'legacy_eef_publishers': self._legacy_eef_publishers(),
            'competing_gripper_publishers': self._external_gripper_publishers(),
            'competing_prepare_publishers': self._external_prepare_publishers(),
            'competing_joint_enable_publishers': (
                self._external_joint_enable_publishers()
            ),
            'competing_joint_clear_error_publishers': (
                self._external_joint_clear_error_publishers()
            ),
            'active_command_arbitration_enabled': bool(
                self.get_parameter('enable_active_command_arbitration').value
            ),
            'external_command_quiet_age_sec': round(float(
                max(
                    0.0,
                    now - max(
                        self._command_monitor_started,
                        self._last_external_command_time,
                    ),
                )
            ), 4),
            'last_external_command_topic': self._last_external_command_topic,
            'last_external_command_age_sec': (
                None if self._last_external_command_time <= 0.0
                else round(float(now - self._last_external_command_time), 4)
            ),
            'last_external_command_payload': self._last_external_command_payload,
            'external_command_counts': dict(self._external_command_counts),
            'target_source': self._target_source,
            'gripper_targets': {
                side: (
                    None
                    if self._gripper_targets[side] is None
                    else float(self._gripper_targets[side])
                )
                for side in ('left', 'right')
            },
            'grip_release_held_sides': sorted(self._grip_release_held),
            'follow_authority_allowed': self._follow_authority_allowed(),
            'teleop_heartbeat_required': bool(
                self.get_parameter('require_teleop_heartbeat').value
            ),
            'teleop_heartbeat_age_sec': (
                None if self._teleop_heartbeat_time == 0.0
                else round(float(now - self._teleop_heartbeat_time), 4)
            ),
            'teleop_heartbeat_mode': self._teleop_heartbeat_mode,
            'teleop_vr_age_sec': self._teleop_vr_age,
            'teleop_tracked_hands': list(self._teleop_tracked_hands),
        }
        if self._last_ik is not None:
            payload['ik'] = {
                'success': self._last_ik.success,
                'reason': self._last_ik.reason,
                'iterations': self._last_ik.iterations,
                'solve_time_ms': round(self._last_ik.solve_time_ms, 3),
                'position_error_m': self._last_ik.position_error_m,
                'orientation_error_rad': self._last_ik.orientation_error_rad,
                'minimum_singular_value': self._last_ik.minimum_singular_value,
                'collision_pairs': self._last_ik.collision_pairs,
            }
        self._status_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))

    def destroy_node(self):
        graph_stop = getattr(self, '_output_guard_stop', None)
        if graph_stop is not None:
            graph_stop.set()
            self._output_guard_thread.join(timeout=0.3)
        mailbox = getattr(self, '_latest_target_mailbox', None)
        if mailbox is not None:
            mailbox.close()
        ik_thread = getattr(self, '_ik_thread', None)
        if ik_thread is not None and ik_thread.is_alive():
            ik_thread.join(timeout=2.0)
        # SPEED mode must never outlive this process's watchdog.  Publish a
        # short burst so DDS teardown cannot leave the last non-zero sample in
        # the vendor motor worker.
        # A disabled/preflight controller must not become a command source
        # merely because it is exiting. Lease release already holds the robot.
        owns_output = (getattr(self, '_hardware_enabled', False)
                       or getattr(self, '_sync_session_acquired', False))
        if getattr(self, '_dry_run', True) or not owns_output:
            return super().destroy_node()
        if getattr(self, '_output_control_mode', 'position') == 'velocity':
            try:
                with self._lock:
                    self._velocity_active_sides.clear()
                    for _ in range(3):
                        self._publish_zero_velocity()
            except Exception as exc:
                self.get_logger().error(f'failed to publish shutdown zero velocity: {exc}')
        else:
            try:
                with self._lock:
                    feedback = getattr(self, '_feedback', None)
                    joint_pub = getattr(self, '_joint_pub', None)
                    if feedback is not None and joint_pub is not None:
                        hold = json.dumps(
                            self._command_payload(feedback.as_dict()),
                            separators=(',', ':'),
                        )
                        for _ in range(3):
                            self._publish_owned_command(
                                joint_pub, self._output_topic, hold
                            )
            except Exception as exc:
                self.get_logger().error(
                    f'failed to publish measured-pose shutdown hold: {exc}'
                )
        try:
            self._request_sync_session_exit()
        except Exception as exc:
            self.get_logger().error(
                f'failed to request guarded SYNC-to-ASYNC shutdown: {exc}'
            )
        return super().destroy_node()


def main(args=None):
    import signal
    from rclpy.signals import SignalHandlerOptions
    # Default ROS SIGINT handling shuts the context before destroy_node can
    # publish the final hold/lease release. Keep it alive until cleanup ends.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stopping = threading.Event()
    previous = {sig: signal.signal(sig, lambda *_: stopping.set())
                for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        node = IndependentArmController()
        try:
            while rclpy.ok() and not stopping.is_set():
                rclpy.spin_once(node, timeout_sec=0.05)
        finally:
            node.destroy_node()
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)


if __name__ == '__main__':
    main()
