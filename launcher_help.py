#!/usr/bin/env python3
"""Launcher environment help, sharing recorder descriptions and defaults.

This module is deliberately standard-library only. Help never sources ROS,
imports LeRobot, creates output directories, or contacts a robot.
"""

from __future__ import annotations

import sys
from recorder_config import build_parser


# Most environment names map directly to argparse options. Only spelling and
# launcher-specific defaults differ; keep those exceptions explicit.
RECORDER_ENV = '''
COLLECT_FPS IMAGE_SOURCE IMAGE_POLL_FPS WITH_HEAD WITH_UPPER_WAIST WITH_WAIST
WITH_DEPTH WITH_EEF_STATE WITH_EEF_ACTION STATE_EEF_TOPIC ACTION_EEF_TOPIC
SUBTASKS_JSON ROBOT_TYPE SYNC_REFERENCE_CAMERA MAX_SYNC_DELTA_SEC
SYNC_IMAGE_BUFFER_SIZE SYNC_SIGNAL_BUFFER_SIZE MIN_CAMERAS MAX_IMAGE_AGE_SEC
MAX_STATE_AGE_SEC MAX_STATE_INTERPOLATION_GAP_SEC MAX_ACTION_HOLD_SEC
MAX_FRAME_INTERVAL_ERROR_RATIO MIN_FREE_DISK_GB MAX_EPISODE_TEMP_GB
MAX_EPISODE_SECONDS IMAGE_WRITER_MAX_PENDING IMAGE_WRITER_MAX_MB
IMAGE_WRITER_THREADS ACTION_MODE COLLECT_VCODEC DEPTH_VCODEC DEPTH_MIN
DEPTH_MAX DEPTH_SHIFT DEPTH_USE_LOG BATCH_ENCODING_SIZE ENCODER_QUEUE_MAXSIZE
ENCODER_THREADS VIDEO_CRF VIDEO_PRESET VIDEO_GOP VIDEO_FILES_SIZE_IN_MB
DATA_FILES_SIZE_IN_MB STREAMING_ENCODING FALLBACK_ACTION_TO_STATE
CAMERA_WARMUP_SEC STATE_READY_TIMEOUT_SEC
'''.split()
SPELLINGS = {'COLLECT_FPS': 'fps', 'COLLECT_VCODEC': 'vcodec',
             'STATE_READY_TIMEOUT_SEC': 'state-warmup-sec'}
DEFAULTS = {'CAMERA_WARMUP_SEC': 10, 'MIN_CAMERAS': 'all selected (3 RGB / 4 RGB+depth)',
            'SUBTASKS_JSON': '[]', 'SYNC_REFERENCE_CAMERA': 'hand_left when active'}
NOTES = {
    'ACTION_MODE': 'keyboard/vr/subtask: status_target. dagger: joint. Add EEF with WITH_EEF_STATE/WITH_EEF_ACTION; the old eef action mode was removed.',
    'IMAGE_POLL_FPS': 'DAgger defaults to 60 Hz; other modes use 120 Hz.',
    'IMAGE_WRITER_THREADS': 'DAgger defaults to 2. Zero is synchronous and may stall capture.',
    'SUBTASKS_JSON': 'In subtask mode: short B marks; last mark saves; long B saves early. Terminal N marks and S saves.',
    'VIDEO_FILES_SIZE_IN_MB': 'Transactional saves create new episode files; this cannot merge episodes.',
    'DATA_FILES_SIZE_IN_MB': 'Transactional saves create new episode files; this cannot merge episodes.',
}

# Launcher/controller settings have no recorder CLI equivalent.
EXTRA = {
    'TASK_NAME': ('mango_pick', 'First positional argument: one directory name, not a path. Reuse it to resume the same dataset.'),
    'TASK_TEXT': ('second argument or task_name', 'Natural-language instruction per episode. Change this while keeping task_name to collect multiple tasks in one dataset.'),
    'OUTPUT_BASE_DIR': ('/home/ubuntu/nas14', 'Dataset parent; output is OUTPUT_BASE_DIR/task_name/dataset, with sibling logs/.'),
    'COLLECTOR_MODE': ('keyboard (or inferred legacy VR_CONTROL/SUBTASKS_JSON)', 'keyboard|vr|subtask|dagger. keyboard/vr/subtask use factory teleoperation and status_target; dagger uses joint plus inspected V4/HG dependencies and Thor. DAGGER_PUBLISH=0 by default; no automatic motion.'),
    'VR_CONTROL': (0, 'Legacy VR button switch. Prefer COLLECTOR_MODE. Explicit contradictory mode settings are rejected.'),
    'VR_START_DELAY_SEC': (3, 'VR start countdown, seconds in [0,30]. B/Y cancels it; lost input cancels it.'),
    'VR_A_LONG_PRESS_SEC': (1, 'Legacy variable name: B hold threshold in subtask mode, seconds in [0.2,10]. Release with GL+GR held to save early.'),
    'VR_SPEECH': (1, '0/1: use the existing robot TTS service. Does not change system volume.'),
    'VR_INPUT_TOPIC': ('/control_topic_<domain>_<robot>', 'Factory VR input. The reviewed V4 input is /openarmx_teleop_vr_306_v4/vr_input.'),
    'VR_RESET_PREFIX': ('/openarmx_teleop_vr_306_v4', 'Guarded reset/status service prefix for compatible V4 only. Factory X reset is NOT integrated; use the factory reset control.'),
    'CONTROL_ACK_TIMEOUT_SEC': (300, 'Maximum wait for a recorder receipt, in seconds. A timeout is an UNKNOWN outcome: inspect logs, do not retry automatically. Normal shutdown waits for finalization and never kills an encoding job at this deadline.'),
    'CAMERA_DETECTION_TIMEOUT_SEC': (15, 'Launcher wait for camera/dataset initialization, seconds; separate from recorder warmup.'),
    'START_HAND_PRODUCER': ('auto (0 if devices busy, otherwise 1)', '0 reuses factory JPEG SHM. 1 starts the fallback hand-camera producer; never compete with the vision service for devices.'),
    'CAMERA_ONLY': (0, '1 selects only head color and permits state-as-action fallback. Link diagnostics only, NOT training data.'),
    'ROBOT_PY': ('/home/ubuntu/miniconda3/envs/robot_env/bin/python', 'Interpreter for camera bridge/producer and VR helper; ROS/OpenCV required.'),
    'LEROBOT_PY': ('/home/ubuntu/miniconda3/envs/lerobot/bin/python', 'Recorder interpreter; ROS, LeRobot 0.6+, PyArrow and video codecs required.'),
    'ROBOT_ID': ('hostname numeric suffix, otherwise 283', 'Robot topic suffix. This selects a robot, not an S2/S3 model.'),
    'ROS_DOMAIN_ID': (0, 'DDS domain and topic suffix. Must match robot services.'),
    'RMW_IMPLEMENTATION': ('rmw_cyclonedds_cpp', 'ROS middleware; must be installed and compatible with the robot services.'),
    'COLLECT_CYCLONEDDS_URI': ('local loopback, multicast disabled', 'Optional collector-specific CycloneDDS XML/path. Takes precedence over CYCLONEDDS_URI, which otherwise overrides the local default.'),
    'CYCLONEDDS_URI': ('local collector default if unset', 'Inherited CycloneDDS XML/path, unless COLLECT_CYCLONEDDS_URI is set.'),
    'OMP_NUM_THREADS': (1, 'Math-library thread cap when unset; DAgger forces 1 to preserve control CPU capacity.'),
    'MKL_NUM_THREADS': (1, 'MKL thread cap; same policy as OMP_NUM_THREADS.'),
    'OPENBLAS_NUM_THREADS': (1, 'OpenBLAS thread cap; same policy as OMP_NUM_THREADS.'),
    'HG_DAGGER_RECORDER_CPUS': ('profile/available-CPU selection', 'DAgger recorder CPU affinity. Explicit lists must pass taskset; do not starve the controller.'),
    'GROOT_REMOTE_TOKEN': ('unset', 'Compatibility token environment variable; if nonempty it overrides the token file. Prefer the file and never commit credentials.'),
    'DAGGER_BACKEND': ('owned', 'owned starts an exclusive compatible controller. attach connects to an existing compatible host and never starts/stops it; Q only detaches. Match robot, task and output path.'),
    'DAGGER_SERVER_URL': ('required', 'Thor GR00T baseline/frame URL, e.g. http://THOR_IP:8777. Preview servers are rejected.'),
    'DAGGER_PUBLISH': (0, '0=no hardware publishing. 1=button-authorized control: A/C starts, Grip takes over, X/R resets when permitted. No automatic start.'),
    'DAGGER_ROBOT_PROFILE': ('s2_v2_2 on robot 300; explicit elsewhere', 's2_v2_2|s3_v3_1. Requires inspected robot description and matching model/action/lease protocol; ROBOT_ID never guesses a model.'),
    'DAGGER_ROBOT_DESCRIPTION_ROOT': ('required for S3', 'S3 SDK description directory containing reviewed urdfs/robot_v3_1.urdf and srdfs/robot_v3_1.srdf. S2 uses inspected V4 files.'),
    'DAGGER_TOKEN_FILE': ('/home/ubuntu/.config/autolife_hg_dagger/groot_server.token', 'Thor authentication token file. Never put a token in a URL or commit it.'),
    'DAGGER_DEPENDENCY_ROOT': ('/home/ubuntu/ros2_ws/src', 'Parent of inspected V4 sources; key files are hash-checked. Do not patch the colleague original.'),
    'DAGGER_TOOLS_ROOT': ('legacy optional Tools root', 'Retained compatibility setting; health checks now use the bundled HTTP client, not a full Tools checkout.'),
    'DAGGER_ROS_SETUP': ('workspace install/setup.bash if present, otherwise system Jazzy', 'Explicit ROS setup path must exist.'),
    'DAGGER_WEB_PORT': (8447, 'Incremental VR HTTPS port. Changing ports does not authorize two controllers.'),
    'DAGGER_ROS_PY': ('/usr/bin/python3', 'Supervisor/bridge ROS interpreter.'),
    'DAGGER_VR_PY': ('/home/ubuntu/ros2_ws/venvs/openarmx_v4_placo/bin/python', 'Reviewed incremental controller interpreter, including Placo.'),
    'DAGGER_WEB_PY': ('/home/ubuntu/ros2_ws/venvs/hg_dagger_web/bin/python', 'HTTPS/WebXR interpreter.'),
}


def parameters():
    actions = build_parser()._option_string_actions
    result = dict(EXTRA)
    for name in RECORDER_ENV:
        option = '--' + SPELLINGS.get(name, name.lower().replace('_', '-'))
        action = actions[option]
        default = DEFAULTS.get(name, action.default)
        if isinstance(default, bool):
            default = int(default)
        description = action.help or ''
        if action.choices:
            description += ' Choices: ' + '|'.join(map(str, action.choices)) + '.'
        if name in NOTES:
            description += ' ' + NOTES[name]
        result[name] = (default, description)
    return result


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    values = parameters()
    requested = [arg.upper() for arg in argv if arg not in ('-h', '--help')]
    if len(requested) > 1 or any(name not in values for name in requested):
        raise SystemExit('Unknown help parameter; run start_lerobot_official_collect.sh --help')
    print('AutoLife Collector | environment parameters\n')
    print('Usage: NAME=value bash start_lerobot_official_collect.sh task_name [task_text]')
    print('Help:  bash start_lerobot_official_collect.sh NAME --help\n')
    for name in requested or values:
        default, description = values[name]
        print(f'{name} (default: {default if default is not None else "unset"})\n  {description}\n')
    if not requested:
        print('Keyboard: Enter=start, S=save, D=discard, N=subtask mark, Q=finalize and exit.')
        print('VR/subtask: hold GL+GR; A=start, B=save/mark, Y=discard. Factory X reset is not integrated.')


if __name__ == '__main__':
    main()
