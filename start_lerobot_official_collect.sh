#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   bash /home/ubuntu/lerobot_data_collector/start_lerobot_official_collect.sh [task_name] [task_text]
#   TASK_TEXT="pick the mango" bash /home/ubuntu/lerobot_data_collector/start_lerobot_official_collect.sh pick_the_mango

show_help() {
    local parameter="${1:-}"
    if [ -n "${parameter}" ]; then
        case "${parameter}" in
            TASK_NAME|task_name)
                echo "TASK_NAME / task_name"
                echo "Usage: bash start_lerobot_official_collect.sh <task_name> [task_text]"
                echo "Directory name, repo-id suffix, and default task text for this collection session."
                echo "Example: bash start_lerobot_official_collect.sh pick_up_bottle"
                ;;
            TASK_TEXT|task_text)
                echo "TASK_TEXT / task_text"
                echo "Usage: TASK_TEXT='pick up the bottle' bash start_lerobot_official_collect.sh pick_up_bottle"
                echo "Natural-language instruction stored in every dataset frame."
                ;;
            COLLECTOR_MODE)
                echo "COLLECTOR_MODE"
                echo "Usage: COLLECTOR_MODE=keyboard|vr|subtask|dagger bash start_lerobot_official_collect.sh task_name"
                echo "keyboard: original terminal controls. vr: single-operator VR buttons. subtask: VR with ordered SUBTASKS_JSON."
                echo "dagger: robot-300 incremental V4 + our Thor GR00T; A=start, GL/GR=takeover, B=save, Y=discard, X=reset."
                echo "Requires DAGGER_SERVER_URL and inspected V4/HG sources. Default DAGGER_PUBLISH=0; set 1 for button-authorized motion."
                echo "Unset: preserve legacy VR_CONTROL/SUBTASKS_JSON selection; default keyboard."
                echo "An explicit mode rejects conflicting legacy flags before creating data or starting processes."
                ;;
            DAGGER_BACKEND|DAGGER_SERVER_URL|DAGGER_PUBLISH|DAGGER_TOKEN_FILE|DAGGER_DEPENDENCY_ROOT|DAGGER_TOOLS_ROOT|DAGGER_ROS_SETUP|DAGGER_WEB_PORT|DAGGER_ROS_PY|DAGGER_VR_PY|DAGGER_WEB_PY)
                echo "${parameter} (COLLECTOR_MODE=dagger only)"
                case "${parameter}" in
                    DAGGER_BACKEND) echo "owned (default): start one exclusive host. attach: connect to an already-compatible host; never starts or stops its controller/recorder/model/web. Q only detaches; host keeps running. Match OUTPUT_BASE_DIR and task_name. Legacy hosts require a one-time compatible-copy migration." ;;
                    DAGGER_SERVER_URL) echo "Required Thor GR00T URL, e.g. http://THOR_IP:8777. No default; baseline/frame protocol only." ;;
                    DAGGER_PUBLISH) echo "0=no hardware publishing (default); 1=A/C and X/R may move the robot. No automatic start." ;;
                    DAGGER_TOKEN_FILE) echo "Token file; default /home/ubuntu/.config/autolife_hg_dagger/groot_server.token. Never put tokens in URLs." ;;
                    DAGGER_DEPENDENCY_ROOT) echo "Inspected V4/HG package parent; default /home/ubuntu/ros2_ws/src. Key sources/configs are hash-checked." ;;
                    DAGGER_TOOLS_ROOT) echo "Our full Autolife_VLA_Tools root for deploy imports; defaults to this repo, or /home/ubuntu/Autolife_VLA_Tools for standalone Collector." ;;
                    DAGGER_ROS_SETUP) echo "ROS workspace environment; default /home/ubuntu/ros2_ws/install/setup.bash." ;;
                    DAGGER_WEB_PORT) echo "VR HTTPS port; default 8447. Changing it does not permit concurrent controllers." ;;
                    DAGGER_ROS_PY) echo "ROS supervisor/bridge interpreter; default /usr/bin/python3." ;;
                    DAGGER_VR_PY) echo "V4 controller interpreter; default /home/ubuntu/ros2_ws/venvs/openarmx_v4_placo/bin/python. Must include Placo." ;;
                    DAGGER_WEB_PY) echo "HTTPS/WebXR interpreter; default /home/ubuntu/ros2_ws/venvs/hg_dagger_web/bin/python." ;;
                esac
                echo "Usage: COLLECTOR_MODE=dagger ${parameter}=<value> bash start_lerobot_official_collect.sh task_name"
                echo "The current adapter supports robot 300, domain 0, 21-D head+upper-waist, three RGB cameras, optional recorded depth."
                ;;
            SUBTASKS_JSON)
                echo "SUBTASKS_JSON"
                echo "Usage: SUBTASKS_JSON='[\"pick\", \"handover\", \"place\"]' VR_CONTROL=1 bash start_lerobot_official_collect.sh task_name"
                echo "Ordered subtask texts. Default: [] (disabled). TASK_TEXT remains the overall episode task."
                echo "Hold GL+GR: A starts; recording short B confirms the current span; last mark saves."
                echo "Hold B for VR_A_LONG_PRESS_SEC then release to save early. Unconfirmed frames have subtask_index=-1."
                echo "Keyboard: N marks, S saves. Use a NEW dataset root when enabling/disabling annotation."
                ;;
            VR_INPUT_TOPIC|VR_RESET_PREFIX)
                echo "${parameter}"
                echo "VR_INPUT_TOPIC: defaults to /control_topic_<domain>_<robot>; V4 uses /openarmx_teleop_vr_306_v4/vr_input."
                echo "VR_RESET_PREFIX: defaults to /openarmx_teleop_vr_306_v4; requires guarded reset/status services, not raw vendor reset."
                echo "Disable old reset chords in the teleoperation copy. X resets only while idle, after both Grips are released; finish with B or Y first."
                exit 0
                ;;
            VR_A_LONG_PRESS_SEC)
                echo "VR_A_LONG_PRESS_SEC"
                echo "Usage: VR_A_LONG_PRESS_SEC=1.2 VR_CONTROL=1 SUBTASKS_JSON='[\"pick\",\"place\"]' bash start_lerobot_official_collect.sh task_name"
                echo "B hold threshold in subtask mode (legacy variable name), seconds. Default: 1.0; finite range [0.2, 10]."
                echo "Action occurs once on release with GL+GR still held; no simultaneous short-press mark."
                ;;
            OUTPUT_BASE_DIR)
                echo "OUTPUT_BASE_DIR"
                echo "Usage: OUTPUT_BASE_DIR=/mnt/nas bash start_lerobot_official_collect.sh task_name"
                echo "Parent directory containing one task directory and dataset subdirectory per task. Default: /home/ubuntu/nas14"
                ;;
            COLLECT_FPS)
                echo "COLLECT_FPS"
                echo "Usage: COLLECT_FPS=30 bash start_lerobot_official_collect.sh task_name"
                echo "Target dataset recording rate in frames per second. Default: 30"
                ;;
            WITH_HEAD)
                echo "WITH_HEAD"
                echo "Usage: WITH_HEAD=1 bash start_lerobot_official_collect.sh task_name"
                echo "Append neck_roll, neck_pitch, and neck_yaw to state/action. Default: 0"
                ;;
            WITH_WAIST)
                echo "WITH_WAIST"
                echo "Usage: WITH_WAIST=1 bash start_lerobot_official_collect.sh task_name"
                echo "Append leg_ankle, leg_knee, waist_pitch, and waist_yaw to state/action. Default: 0"
                ;;
            WITH_UPPER_WAIST)
                echo "WITH_UPPER_WAIST"
                echo "Usage: WITH_UPPER_WAIST=1 bash start_lerobot_official_collect.sh task_name"
                echo "Append only waist_pitch and waist_yaw; leg_ankle and leg_knee are excluded. Cannot be 1 together with WITH_WAIST. Default: 0"
                ;;
            WITH_DEPTH)
                echo "WITH_DEPTH"
                echo "Usage: WITH_DEPTH=1 bash start_lerobot_official_collect.sh task_name"
                echo "Record rgbd_head_depth in addition to the three default RGB cameras. Default: 0"
                ;;
            VR_CONTROL)
                echo "VR_CONTROL"
                echo "Usage: VR_CONTROL=1 bash start_lerobot_official_collect.sh task_name"
                echo "Enable single-operator VR episode control after recorder startup. Default: 0 (keyboard only)."
                echo "Hold GL+GR and tap/release: A=start, B=save, Y=discard, X=guarded reset only. Exit in terminal."
                echo "With SUBTASKS_JSON: recording short B=mark subtask, long B=save early; final mark saves automatically."
                echo "Keyboard controls remain available. Requires factory /control_topic_<domain>_<robot> String messages."
                ;;
            VR_START_DELAY_SEC|VR_SPEECH)
                echo "${parameter}"
                echo "Usage: VR_CONTROL=1 ${parameter}=<value> bash start_lerobot_official_collect.sh task_name"
                echo "VR_START_DELAY_SEC: countdown seconds, 0 through 30, default 3. B/X cancels countdown."
                echo "VR_SPEECH: 1=use existing robot TTS service, 0=silent, default 1. System volume is unchanged."
                echo "Command confirmation timeout uses CONTROL_ACK_TIMEOUT_SEC (default 300 seconds)."
                ;;
            IMAGE_SOURCE)
                echo "IMAGE_SOURCE"
                echo "Usage: IMAGE_SOURCE=shm|ros bash start_lerobot_official_collect.sh task_name"
                echo "Image transport: shm reads shared memory directly; ros starts the SHM-to-ROS bridge. Default: shm"
                ;;
            IMAGE_POLL_FPS)
                echo "IMAGE_POLL_FPS"
                echo "Usage: IMAGE_POLL_FPS=120 bash start_lerobot_official_collect.sh task_name"
                echo "Direct-SHM metadata polling rate. It is not the dataset recording FPS. Default: 120"
                ;;
            SYNC_REFERENCE_CAMERA)
                echo "SYNC_REFERENCE_CAMERA"
                echo "Usage: SYNC_REFERENCE_CAMERA=hand_left bash start_lerobot_official_collect.sh task_name"
                echo "Camera used as the timestamp anchor. It must be one of the active cameras. Default: hand_left when active"
                ;;
            MAX_SYNC_DELTA_SEC)
                echo "MAX_SYNC_DELTA_SEC"
                echo "Usage: MAX_SYNC_DELTA_SEC=0.04 bash start_lerobot_official_collect.sh task_name"
                echo "Maximum allowed absolute timestamp difference from the reference frame. Unit: seconds. Default: 0.03 (30 ms)"
                ;;
            SYNC_IMAGE_BUFFER_SIZE)
                echo "SYNC_IMAGE_BUFFER_SIZE"
                echo "Usage: SYNC_IMAGE_BUFFER_SIZE=16 bash start_lerobot_official_collect.sh task_name"
                echo "Per-camera image FIFO capacity. Overflow invalidates the active episode. Default: 16"
                ;;
            SYNC_SIGNAL_BUFFER_SIZE)
                echo "SYNC_SIGNAL_BUFFER_SIZE"
                echo "Usage: SYNC_SIGNAL_BUFFER_SIZE=64 bash start_lerobot_official_collect.sh task_name"
                echo "Capacity of the state/action timestamp buffers. Default: 64"
                ;;
            MIN_CAMERAS)
                echo "MIN_CAMERAS"
                echo "Usage: MIN_CAMERAS=4 bash start_lerobot_official_collect.sh task_name"
                echo "Minimum number of valid cameras required before recording. Default: all selected cameras (3 RGB, or 4 with depth)"
                ;;
            MAX_IMAGE_AGE_SEC|MAX_STATE_AGE_SEC|MAX_STATE_INTERPOLATION_GAP_SEC|MAX_ACTION_HOLD_SEC)
                cli_parameter="${parameter,,}"
                cli_parameter="${cli_parameter//_/-}"
                case "${parameter}" in
                    MAX_IMAGE_AGE_SEC|MAX_STATE_AGE_SEC) default_value="0.15" ;;
                    MAX_STATE_INTERPOLATION_GAP_SEC) default_value="0.05" ;;
                    MAX_ACTION_HOLD_SEC) default_value="0.5" ;;
                esac
                echo "${parameter}"
                echo "Usage: ${parameter}=${default_value} bash start_lerobot_official_collect.sh task_name"
                echo "Freshness/interpolation limit in seconds; see record_lerobot_official.py --${cli_parameter} --help for the exact rule. Default: ${default_value}"
                ;;
            MAX_FRAME_INTERVAL_ERROR_RATIO)
                echo "MAX_FRAME_INTERVAL_ERROR_RATIO"
                echo "Usage: MAX_FRAME_INTERVAL_ERROR_RATIO=0.45 bash start_lerobot_official_collect.sh task_name"
                echo "Maximum reference-camera interval error as a fraction of one requested frame period. Default: 0.45"
                ;;
            ACTION_MODE)
                echo "ACTION_MODE"
                echo "Usage: ACTION_MODE=status_target|joint|eef bash start_lerobot_official_collect.sh task_name"
                echo "Action source: status_target is recommended; joint reads command topics; eef records end-effector targets. Default: status_target"
                ;;
            COLLECT_VCODEC|DEPTH_VCODEC|DEPTH_MIN|DEPTH_MAX|DEPTH_SHIFT|DEPTH_USE_LOG)
                echo "${parameter}"
                echo "Usage: ${parameter}=<value> bash start_lerobot_official_collect.sh task_name"
                echo "Video/depth encoding option forwarded to the LeRobot recorder. See record_lerobot_official.py --help for units and defaults."
                ;;
            CAMERA_WARMUP_SEC|STATE_READY_TIMEOUT_SEC|CAMERA_DETECTION_TIMEOUT_SEC|CONTROL_ACK_TIMEOUT_SEC)
                echo "${parameter}"
                echo "Usage: ${parameter}=<seconds> bash start_lerobot_official_collect.sh task_name"
                echo "Launcher wait/acknowledgement timeout in seconds. See the corresponding recorder/control --help output for the exact default."
                ;;
            BATCH_ENCODING_SIZE|ENCODER_QUEUE_MAXSIZE|ENCODER_THREADS|VIDEO_FILES_SIZE_IN_MB|DATA_FILES_SIZE_IN_MB|VIDEO_CRF|VIDEO_PRESET|VIDEO_GOP|STREAMING_ENCODING)
                echo "${parameter}"
                echo "Usage: ${parameter}=<value> bash start_lerobot_official_collect.sh task_name"
                echo "Video encoding or file-segmentation option forwarded to the LeRobot recorder. Default: unset unless documented in record_lerobot_official.py --help."
                ;;
            ROBOT_PY|LEROBOT_PY)
                echo "${parameter}"
                echo "Usage: ${parameter}=/path/to/python bash start_lerobot_official_collect.sh task_name"
                echo "Python interpreter used for the robot camera producer or LeRobot recorder. Use this only when environment locations differ from the defaults."
                ;;
            START_HAND_PRODUCER)
                echo "START_HAND_PRODUCER"
                echo "Usage: START_HAND_PRODUCER=0|1 bash start_lerobot_official_collect.sh task_name"
                echo "Whether to start the hand-camera SHM producer. Set 0 when another process already owns the devices. Default: 1 unless devices are busy"
                ;;
            CAMERA_ONLY)
                echo "CAMERA_ONLY"
                echo "Usage: CAMERA_ONLY=1 bash start_lerobot_official_collect.sh task_name"
                echo "Test only rgbd_head_color and use state as action fallback. This is for camera/link tests, not training data. Default: 0"
                ;;
            FALLBACK_ACTION_TO_STATE)
                echo "FALLBACK_ACTION_TO_STATE"
                echo "Usage: FALLBACK_ACTION_TO_STATE=1 bash start_lerobot_official_collect.sh task_name"
                echo "Use measured state as action when action topics are unavailable. Default: 0"
                ;;
            ROS_DOMAIN_ID|ROBOT_ID|RMW_IMPLEMENTATION)
                echo "${parameter}"
                echo "Usage: ${parameter}=<value> bash start_lerobot_official_collect.sh task_name"
                echo "ROS identity/middleware setting used to construct robot topic names. See the robot ROS configuration for valid values."
                ;;
            *)
                echo "Unknown help parameter: ${parameter}" >&2
                echo "Run: bash start_lerobot_official_collect.sh --help" >&2
                return 2
                ;;
        esac
        return 0
    fi

    cat <<'HELP'
Official LeRobot collector launcher

Usage:
  bash start_lerobot_official_collect.sh [task_name] [task_text]
  TASK_TEXT="pick up the bottle" bash start_lerobot_official_collect.sh pick_up_bottle

Show one environment variable:
  bash start_lerobot_official_collect.sh --help MAX_SYNC_DELTA_SEC
  bash start_lerobot_official_collect.sh MAX_SYNC_DELTA_SEC --help

Common parameters:
  COLLECTOR_MODE             keyboard, vr, subtask, dagger. Unset: infer legacy settings.
  DAGGER_BACKEND             owned (default) or attach. Attach requires a compatible host; Q detaches only.
  DAGGER_SERVER_URL          Required by owned dagger mode; attach reuses the host's policy client.
  DAGGER_PUBLISH             0=no hardware publishing (default), 1=button-authorized motion; no auto-start.
  DAGGER_TOKEN_FILE          Thor token path; default ~/.config/autolife_hg_dagger/groot_server.token.
  DAGGER_DEPENDENCY_ROOT     V4/HG source parent; default /home/ubuntu/ros2_ws/src (version checked).
  DAGGER_TOOLS_ROOT          Our full VLA tools root; standalone default /home/ubuntu/Autolife_VLA_Tools.
  DAGGER_ROS_SETUP           Workspace setup.bash; default /home/ubuntu/ros2_ws/install/setup.bash.
  DAGGER_WEB_PORT            Incremental-VR HTTPS port; default 8447. Existing stack must be stopped first.
  DAGGER_ROS_PY              Supervisor/bridge interpreter; default /usr/bin/python3.
  DAGGER_VR_PY               Controller interpreter; default /home/ubuntu/ros2_ws/venvs/openarmx_v4_placo/bin/python.
  DAGGER_WEB_PY              Web interpreter; default /home/ubuntu/ros2_ws/venvs/hg_dagger_web/bin/python.
  TASK_NAME                  Dataset/task directory name. Positional argument 1.
  TASK_TEXT                  Natural-language task instruction. Positional argument 2 or environment variable.
  SUBTASKS_JSON               Ordered subtask JSON array. Default: [] (disabled); keyboard N marks.
  OUTPUT_BASE_DIR            Dataset parent directory. Default: /home/ubuntu/nas14
  COLLECT_FPS                Dataset FPS. Default: 30
  WITH_HEAD                  Add 3 neck joints. Default: 0
  WITH_UPPER_WAIST           Add only waist pitch/yaw. Cannot combine with WITH_WAIST. Default: 0
  WITH_WAIST                 Add 4 leg/waist joints. Default: 0
  WITH_DEPTH                 Add rgbd_head_depth. Default: 0
  VR_CONTROL                 Enable VR episode buttons alongside keyboard controls. Default: 0
  VR_START_DELAY_SEC         VR start countdown, 0 through 30 seconds. Default: 3
  VR_SPEECH                  Use existing robot TTS service; does not change volume. Default: 1
  VR_A_LONG_PRESS_SEC        B hold time for early save (legacy variable name) in subtask mode. Default: 1.0 seconds
  IMAGE_SOURCE               shm or ros. Default: shm
  IMAGE_POLL_FPS             SHM metadata polling rate. Default: 120
  SYNC_REFERENCE_CAMERA      Timestamp anchor camera. Default: hand_left
  MAX_SYNC_DELTA_SEC         Maximum sync error in seconds. Default: 0.03
  SYNC_IMAGE_BUFFER_SIZE     Image FIFO capacity per camera. Default: 16
  SYNC_SIGNAL_BUFFER_SIZE    State/action buffer capacity. Default: 64
  MIN_CAMERAS                Minimum valid cameras. Default: all selected cameras
  ACTION_MODE                status_target, joint, or eef. Default: status_target
  MAX_IMAGE_AGE_SEC          Maximum image age in seconds. Default: 0.15
  MAX_STATE_AGE_SEC          Maximum joint-state age in seconds. Default: 0.15
  MAX_STATE_INTERPOLATION_GAP_SEC  Maximum interpolation span. Default: 0.05
  MAX_ACTION_HOLD_SEC        Maximum held-action age. Default: 0.5
  MAX_FRAME_INTERVAL_ERROR_RATIO  Reference interval error ratio. Default: 0.45
  CAMERA_WARMUP_SEC          Camera initialization timeout. Default: 10 (launcher)
  STATE_READY_TIMEOUT_SEC    First joint-state timeout. Default: 10
  CAMERA_DETECTION_TIMEOUT_SEC  Camera detection timeout. Default: 15
  CONTROL_ACK_TIMEOUT_SEC    Save/discard acknowledgement timeout. Default: 300
  COLLECT_VCODEC             RGB video codec. Default: h264
  DEPTH_VCODEC               Depth video codec. Default: hevc
  DEPTH_MIN/MAX/SHIFT        Depth quantizer range and shift. Defaults: 0.05/10.0/3.5
  DEPTH_USE_LOG              Logarithmic depth quantization. Default: 1
  BATCH_ENCODING_SIZE        Frames per encoder batch. Default: 1
  ENCODER_QUEUE_MAXSIZE      Maximum pending encoder batches. Default: 30
  ENCODER_THREADS            Encoder worker threads. Default: unset
  VIDEO_CRF/PRESET/GOP       Optional RGB encoding controls. Default: unset
  VIDEO_FILES_SIZE_IN_MB     Optional video segment size. Default: unset
  DATA_FILES_SIZE_IN_MB      Optional Parquet segment size. Default: unset
  STREAMING_ENCODING         Encode videos during capture. Default: 0
  START_HAND_PRODUCER        Start hand-camera producer. Default: 1 unless devices are busy
  CAMERA_ONLY                Single-camera link test mode. Default: 0
  FALLBACK_ACTION_TO_STATE   Use state as action for link tests. Default: 0
  ROS_DOMAIN_ID              ROS domain. Default: 0
  ROBOT_ID                   Robot topic suffix. Hostname suffix is used when unset.
  RMW_IMPLEMENTATION         ROS middleware implementation. Default: rmw_cyclonedds_cpp

Advanced timing, encoding, and timeout variables are forwarded to
record_lerobot_official.py. Use its detailed CLI help:
  python record_lerobot_official.py --help
  python record_lerobot_official.py --max-sync-delta-sec --help

Keyboard controls after startup: Enter=start, S=save, D=discard, Q=save and quit.
HELP
}

if [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
    show_help "${2:-}"
    exit 0
fi
if [ "${2:-}" = "--help" ] || [ "${2:-}" = "-h" ]; then
    show_help "${1:-}"
    exit 0
fi

# Keep small telemetry/image operations from creating competing BLAS pools.
# Explicit operator settings win; video/image writer concurrency is unchanged.
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"
export MKL_NUM_THREADS="${MKL_NUM_THREADS:-1}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"

# Reject an ambiguous joint schema before creating directories, sourcing ROS,
# or starting any process.  Both waist modes read the same four-value packet,
# but expose different dataset dimensions and therefore cannot be combined.
WITH_HEAD="${WITH_HEAD:-0}"
WITH_UPPER_WAIST="${WITH_UPPER_WAIST:-0}"
WITH_WAIST="${WITH_WAIST:-0}"
VR_CONTROL="${VR_CONTROL:-}"
VR_SPEECH="${VR_SPEECH:-1}"
if [[ ! "${VR_SPEECH}" =~ ^[01]$ ]]; then
    echo "ERROR: VR_SPEECH must be 0 or 1." >&2
    exit 2
fi
if [ "${WITH_UPPER_WAIST}" = "1" ] && [ "${WITH_WAIST}" = "1" ]; then
    echo "ERROR: WITH_UPPER_WAIST and WITH_WAIST cannot both be 1." >&2
    exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SUBTASKS_JSON="${SUBTASKS_JSON:-[]}"
# Resolve once, before directory creation, ROS setup, or any process startup.
# In particular, DAgger must never accidentally start the factory VR helper.
MODE_CONFIG="$(python3 "${SCRIPT_DIR}/collector_modes.py" --mode "${COLLECTOR_MODE:-}" \
    --vr-control "${VR_CONTROL}" --subtasks-json "${SUBTASKS_JSON}")"
read -r COLLECTOR_MODE VR_CONTROL SUBTASK_COUNT <<< "${MODE_CONFIG}"
if [ "${COLLECTOR_MODE}" = "dagger" ]; then
    exec python3 "${SCRIPT_DIR}/dagger/run.py" "$@"
fi
TASK_NAME="${1:-mango_pick}"
TASK_TEXT="${TASK_TEXT:-${2:-${TASK_NAME}}}"
OUTPUT_BASE_DIR="${OUTPUT_BASE_DIR:-/home/ubuntu/nas14}"
BASE_DIR="${OUTPUT_BASE_DIR}/${TASK_NAME}"
DATASET_ROOT="${BASE_DIR}/dataset"
PIDFILE="${BASE_DIR}/.official_recording_pids"
CURRENT_OUTPUT="${BASE_DIR}/.current_output"
CONTROL_FIFO="${BASE_DIR}/.official_recording_control"
STATUS_FILE="${BASE_DIR}/.official_recording_status.json"
STATE_READY_FILE="${BASE_DIR}/.official_recorder_state_ready.json"
EPISODE_EVENT_FILE="${BASE_DIR}/.official_episode_event.json"
LOG_DIR="${BASE_DIR}/logs"

ROBOT_PY="${ROBOT_PY:-/home/ubuntu/miniconda3/envs/robot_env/bin/python}"
LEROBOT_PY="${LEROBOT_PY:-/home/ubuntu/miniconda3/envs/lerobot/bin/python}"
BRIDGE_SCRIPT="${SCRIPT_DIR}/shm_camera_topic_bridge.py"
RECORDER_SCRIPT="${SCRIPT_DIR}/record_lerobot_official.py"
HAND_PRODUCER_SCRIPT="${SCRIPT_DIR}/hand_camera_producer.py"
CONTROL_SCRIPT="${SCRIPT_DIR}/collector_control.py"
VR_SCRIPT="${SCRIPT_DIR}/vr_collector_control.py"

HOST_ROBOT_ID="$(hostname | sed -n 's/.*-\([0-9][0-9]*\)$/\1/p')"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROBOT_ID="${ROBOT_ID:-${HOST_ROBOT_ID:-283}}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
TOPIC_NODE_ID="${ROS_DOMAIN_ID}_${ROBOT_ID}"
MOTION_LOCK_FILE="${MOTION_LOCK_FILE:-/tmp/lerobot_robot_${TOPIC_NODE_ID}.motion.lock}"

VR_ARGS=("${VR_SCRIPT}" --base-dir "${BASE_DIR}"
    --topic "${VR_INPUT_TOPIC:-/control_topic_${TOPIC_NODE_ID}}" --tts-topic "/topic_tts_${TOPIC_NODE_ID}"
    --reset-prefix "${VR_RESET_PREFIX:-/openarmx_teleop_vr_306_v4}"
    --feedback-topic /collector/feedback
    --motion-lock-file "${MOTION_LOCK_FILE}"
    --start-delay "${VR_START_DELAY_SEC:-3}" --command-timeout "${CONTROL_ACK_TIMEOUT_SEC:-300}"
    --a-long-press-sec "${VR_A_LONG_PRESS_SEC:-1.0}")
if [ "${SUBTASK_COUNT}" -gt 0 ]; then
    VR_ARGS+=(--subtask-mode)
fi
if [ "${VR_SPEECH}" = "0" ]; then
    VR_ARGS+=(--no-speech)
fi
if [ "${VR_CONTROL}" = "1" ]; then
    python3 "${VR_ARGS[@]}" --check-config
fi

mkdir -p "${BASE_DIR}" "${LOG_DIR}"

# Refuse to reuse a session directory while any process from its last PID file
# is still alive. Concurrent writes can corrupt LeRobot parquet/video indices.
if [ -f "${PIDFILE}" ]; then
    while read -r _ pid _; do
        if [ -n "${pid:-}" ] && kill -0 "${pid}" 2>/dev/null; then
            echo "ERROR: collection already running (PID ${pid})."
            echo "Stop it from the active collector terminal with Q or Ctrl+C."
            exit 1
        fi
    done < "${PIDFILE}"
fi

RUN_SEQ=1
# Logs are never overwritten; each restart receives the next numeric prefix.
while [ -e "${LOG_DIR}/${RUN_SEQ}.record_lerobot_official.log" ]; do
    RUN_SEQ=$((RUN_SEQ + 1))
done
LOG_PREFIX="${LOG_DIR}/${RUN_SEQ}"
CLEANUP_DONE=0

send_control() {
    python3 "${CONTROL_SCRIPT}" command --base-dir "${BASE_DIR}" "$1"
}

send_control_and_report() {
    python3 "${CONTROL_SCRIPT}" command \
        --base-dir "${BASE_DIR}" \
        --wait \
        --timeout "${CONTROL_ACK_TIMEOUT_SEC:-300}" \
        "$1"
}

cleanup_session() {
    if [ "${CLEANUP_DONE}" = "1" ]; then
        return
    fi
    CLEANUP_DONE=1

    # Stop our input helper first, so it cannot send new commands during finalization.
    if [ -n "${VR_PID:-}" ] && kill -0 "${VR_PID}" 2>/dev/null; then
        kill -INT "${VR_PID}" 2>/dev/null || true
        for _ in $(seq 1 20); do
            kill -0 "${VR_PID}" 2>/dev/null || break
            sleep 0.1
        done
    fi

    # Give the recorder time to save pending frames and finalize metadata before
    # stopping camera processes in reverse launch order.
    if [ -n "${RECORDER_PID:-}" ] && kill -0 "${RECORDER_PID}" 2>/dev/null; then
        send_control "quit" >/dev/null 2>&1 || kill -INT "${RECORDER_PID}" 2>/dev/null || true
        for _ in $(seq 1 120); do
            kill -0 "${RECORDER_PID}" 2>/dev/null || break
            sleep 0.25
        done
    fi

    if [ -f "${PIDFILE}" ]; then
        tac "${PIDFILE}" | while read -r role pid log; do
            if [ -z "${pid:-}" ]; then
                continue
            fi
            if kill -0 "${pid}" 2>/dev/null; then
                echo "Stopping ${role} ${pid} ..."
                kill -INT "${pid}" 2>/dev/null || true
                for _ in $(seq 1 20); do
                    kill -0 "${pid}" 2>/dev/null || break
                    sleep 0.2
                done
            fi
            if kill -0 "${pid}" 2>/dev/null; then
                echo "Terminating ${role} ${pid} ..."
                kill -TERM "${pid}" 2>/dev/null || true
            fi
        done
    fi

    rm -f "${PIDFILE}" "${CONTROL_FIFO}" "${STATUS_FILE}" "${STATE_READY_FILE}" "${EPISODE_EVENT_FILE}"
    echo ""
    python3 "${CONTROL_SCRIPT}" summary --dataset-root "${DATASET_ROOT}"
    python3 "${CONTROL_SCRIPT}" sync-report --sync-log "${DATASET_ROOT}/sync_log.jsonl"
    echo "=============================================="
}

on_interrupt() {
    echo ""
    echo "Interrupted. Stopping collection ..."
    exit 0
}

trap cleanup_session EXIT
trap on_interrupt INT TERM

echo "=============================================="
echo "  Official LeRobotDataset batch collection"
echo "  collector mode: ${COLLECTOR_MODE}"
if [ "${_COLLECTOR_DAGGER_RECORDING:-0}" = "1" ]; then
    echo "  controller    : DAgger supervisor (recorder-only child; legacy keyboard disabled)"
fi
echo "  task name   : ${TASK_NAME}"
echo "  task text   : ${TASK_TEXT}"
echo "  dataset root: ${DATASET_ROOT}"
echo "  topic id    : ${TOPIC_NODE_ID}"
echo "  reset policy: allowed only while paused after Save/Discard"
echo "  run log seq : ${RUN_SEQ}"
echo "=============================================="

set +u
source /opt/ros/jazzy/setup.bash
set -u
# Preserve paths exported by ROS, then prepend each conda environment's native
# libraries only for the process that needs them.
ROS_LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
ROS_PYTHONPATH="${PYTHONPATH:-}"
ROBOT_ENV_LIB="/home/ubuntu/miniconda3/envs/robot_env/lib"
LEROBOT_ENV_LIB="/home/ubuntu/miniconda3/envs/lerobot/lib"
COLLECT_CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
CAMERA_ONLY="${CAMERA_ONLY:-0}"
COLLECT_FPS="${COLLECT_FPS:-30}"
WITH_DEPTH="${WITH_DEPTH:-0}"
BRIDGE_CAMERAS=(rgbd_head_color hand_left hand_right)
IMAGE_SOURCE="${IMAGE_SOURCE:-shm}"
IMAGE_POLL_FPS="${IMAGE_POLL_FPS:-120}"
RECORDER_CAMERA_ARGS=()
RECORDER_FEATURE_ARGS=()

# Feature switches are intentionally session-level settings.  LeRobot fixes
# feature shapes when a dataset root is created, so changing one of these while
# resuming an existing root will produce a clear recorder startup error.
if [ "${WITH_HEAD}" = "1" ]; then
    RECORDER_FEATURE_ARGS+=(--with-head)
fi
if [ "${WITH_WAIST}" = "1" ]; then
    RECORDER_FEATURE_ARGS+=(--with-waist)
elif [ "${WITH_UPPER_WAIST}" = "1" ]; then
    RECORDER_FEATURE_ARGS+=(--with-upper-waist)
fi
if [ -z "${START_HAND_PRODUCER+x}" ] && { fuser /dev/video12 >/dev/null 2>&1 || fuser /dev/video14 >/dev/null 2>&1; }; then
    START_HAND_PRODUCER=0
    echo "  mode        : hand camera devices busy; reusing existing SHM streams"
fi
if [ "${CAMERA_ONLY}" = "1" ]; then
    START_HAND_PRODUCER=0
    FALLBACK_ACTION_TO_STATE=1
    BRIDGE_CAMERAS=(rgbd_head_color)
    RECORDER_CAMERA_ARGS=(--camera rgbd_head_color)
    echo "  mode        : CAMERA_ONLY=1 (rgbd_head_color only, action=state fallback)"
fi
if [ "${WITH_DEPTH}" = "1" ]; then
    BRIDGE_CAMERAS+=(rgbd_head_depth)
    RECORDER_FEATURE_ARGS+=(--with-depth)
fi
MIN_CAMERAS="${MIN_CAMERAS:-${#BRIDGE_CAMERAS[@]}}"

echo "  joint data  : head=${WITH_HEAD}, upper_waist=${WITH_UPPER_WAIST}, waist=${WITH_WAIST}"
echo "  depth data  : ${WITH_DEPTH}"
echo "  cameras req : ${MIN_CAMERAS}/${#BRIDGE_CAMERAS[@]}"
echo "  image FIFO  : ${SYNC_IMAGE_BUFFER_SIZE:-16} frames/camera"

rm -f "${CONTROL_FIFO}" "${STATUS_FILE}" "${STATE_READY_FILE}" "${EPISODE_EVENT_FILE}"
mkfifo "${CONTROL_FIFO}"
: > "${PIDFILE}"

RECORDER_ARGS=(
    "${RECORDER_SCRIPT}"
    --output-dir "${DATASET_ROOT}"
    --repo-id "local/${TASK_NAME}"
    --task-name "${TASK_TEXT}"
    --subtasks-json "${SUBTASKS_JSON}"
    --motion-lock-file "${MOTION_LOCK_FILE}"
    --fps "${COLLECT_FPS}"
    --image-source "${IMAGE_SOURCE}"
    --image-poll-fps "${IMAGE_POLL_FPS}"
    --state-warmup-sec "${STATE_READY_TIMEOUT_SEC:-10}"
    --camera-warmup-sec "${CAMERA_WARMUP_SEC:-10}"
    --state-ready-file "${STATE_READY_FILE}"
    --episode-event-file "${EPISODE_EVENT_FILE}"
    --min-cameras "${MIN_CAMERAS}"
    --max-sync-delta-sec "${MAX_SYNC_DELTA_SEC:-0.03}"
    --max-image-age-sec "${MAX_IMAGE_AGE_SEC:-0.15}"
    --max-state-age-sec "${MAX_STATE_AGE_SEC:-0.15}"
    --max-state-interpolation-gap-sec "${MAX_STATE_INTERPOLATION_GAP_SEC:-0.05}"
    --max-action-hold-sec "${MAX_ACTION_HOLD_SEC:-0.5}"
    --max-frame-interval-error-ratio "${MAX_FRAME_INTERVAL_ERROR_RATIO:-0.45}"
    --sync-image-buffer-size "${SYNC_IMAGE_BUFFER_SIZE:-16}"
    --sync-signal-buffer-size "${SYNC_SIGNAL_BUFFER_SIZE:-64}"
    --action-mode "${ACTION_MODE:-status_target}"
    --vcodec "${COLLECT_VCODEC:-h264}"
    --depth-vcodec "${DEPTH_VCODEC:-hevc}"
    --depth-min "${DEPTH_MIN:-0.05}"
    --depth-max "${DEPTH_MAX:-10.0}"
    --depth-shift "${DEPTH_SHIFT:-3.5}"
    --batch-encoding-size "${BATCH_ENCODING_SIZE:-1}"
    --encoder-queue-maxsize "${ENCODER_QUEUE_MAXSIZE:-30}"
    --state-topic "/topic_arm_whole_body_and_gripper_current_joints_status_${TOPIC_NODE_ID}"
    --action-arm-topic "/topic_arm_whole_body_target_joints_position_${TOPIC_NODE_ID}"
    --action-gripper-topic "/topic_arm_gripper_target_joints_position_${TOPIC_NODE_ID}"
    --action-eef-topic "/topic_arm_target_robot_eef_pose_${TOPIC_NODE_ID}"
    --action-height-topic "/topic_arm_target_robot_height_z_${TOPIC_NODE_ID}"
    --control-fifo "${CONTROL_FIFO}"
    --status-file "${STATUS_FILE}"
    "${RECORDER_CAMERA_ARGS[@]}"
    "${RECORDER_FEATURE_ARGS[@]}"
)
RECORDER_PREFIX=()
if [ "${_COLLECTOR_DAGGER_RECORDING:-0}" = "1" ]; then
    RECORDER_PREFIX=(bash "${SCRIPT_DIR}/dagger/run_recorder_isolated.sh")
    RECORDER_ARGS+=(--image-writer-threads "${IMAGE_WRITER_THREADS:-2}" --dagger --action-arm-topic /hg_dagger/collector/arm_action
        --action-gripper-topic /hg_dagger/collector/gripper_action)
fi
if [ -n "${SYNC_REFERENCE_CAMERA:-}" ]; then
    RECORDER_ARGS+=(--sync-reference-camera "${SYNC_REFERENCE_CAMERA}")
fi
if [ "${DEPTH_USE_LOG:-1}" = "0" ]; then
    RECORDER_ARGS+=(--no-depth-use-log)
fi
if [ "${FALLBACK_ACTION_TO_STATE:-0}" = "1" ]; then
    RECORDER_ARGS+=(--fallback-action-to-state)
fi
if [ -n "${VIDEO_CRF:-}" ]; then
    RECORDER_ARGS+=(--video-crf "${VIDEO_CRF}")
fi
if [ -n "${VIDEO_PRESET:-}" ]; then
    RECORDER_ARGS+=(--video-preset "${VIDEO_PRESET}")
fi
if [ -n "${VIDEO_GOP:-}" ]; then
    RECORDER_ARGS+=(--video-gop "${VIDEO_GOP}")
fi
if [ -n "${ENCODER_THREADS:-}" ]; then
    RECORDER_ARGS+=(--encoder-threads "${ENCODER_THREADS}")
fi
if [ -n "${VIDEO_FILES_SIZE_IN_MB:-}" ]; then
    RECORDER_ARGS+=(--video-files-size-in-mb "${VIDEO_FILES_SIZE_IN_MB}")
fi
if [ -n "${DATA_FILES_SIZE_IN_MB:-}" ]; then
    RECORDER_ARGS+=(--data-files-size-in-mb "${DATA_FILES_SIZE_IN_MB}")
fi
if [ "${STREAMING_ENCODING:-0}" = "1" ]; then
    RECORDER_ARGS+=(--streaming-encoding)
fi

nohup env CYCLONEDDS_URI="${COLLECT_CYCLONEDDS_URI}" LD_LIBRARY_PATH="${LEROBOT_ENV_LIB}:${ROS_LD_LIBRARY_PATH}" PYTHONPATH="${ROS_PYTHONPATH}" "${RECORDER_PREFIX[@]}" "${LEROBOT_PY}" "${RECORDER_ARGS[@]}" > "${LOG_PREFIX}.record_lerobot_official.log" 2>&1 &
RECORDER_PID=$!
echo "recorder ${RECORDER_PID} ${LOG_PREFIX}.record_lerobot_official.log" >> "${PIDFILE}"
echo "${DATASET_ROOT}" > "${CURRENT_OUTPUT}"

echo "  recorder PID      : ${RECORDER_PID}"
echo "Waiting for the first complete joint-state packet ..."
python3 "${CONTROL_SCRIPT}" wait-ready \
    --path "${STATE_READY_FILE}" \
    --pid "${RECORDER_PID}" \
    --timeout "${STATE_READY_TIMEOUT_SEC:-10}"

# Camera production starts only after the recorder is actively buffering joint
# state. Video frames can then act as anchors for nearest-state lookup without
# losing the first samples to process startup ordering.
if [ "${START_HAND_PRODUCER:-1}" = "1" ]; then
    nohup env CYCLONEDDS_URI="${COLLECT_CYCLONEDDS_URI}" LD_LIBRARY_PATH="${ROBOT_ENV_LIB}:${ROS_LD_LIBRARY_PATH}" "${ROBOT_PY}" "${HAND_PRODUCER_SCRIPT}" > "${LOG_PREFIX}.hand_camera_producer.log" 2>&1 &
    HAND_PID=$!
    echo "hand_producer ${HAND_PID} ${LOG_PREFIX}.hand_camera_producer.log" >> "${PIDFILE}"
    echo "  hand producer PID : ${HAND_PID}"
else
    echo "  hand producer     : skipped (START_HAND_PRODUCER=0)"
fi

if [ "${IMAGE_SOURCE}" = "ros" ]; then
    nohup env CYCLONEDDS_URI="${COLLECT_CYCLONEDDS_URI}" LD_LIBRARY_PATH="${ROBOT_ENV_LIB}:${ROS_LD_LIBRARY_PATH}" PYTHONPATH="${ROS_PYTHONPATH}" "${ROBOT_PY}" "${BRIDGE_SCRIPT}" --cameras "${BRIDGE_CAMERAS[@]}" --fps "${COLLECT_FPS}" > "${LOG_PREFIX}.camera_bridge.log" 2>&1 &
    BRIDGE_PID=$!
    echo "camera_bridge ${BRIDGE_PID} ${LOG_PREFIX}.camera_bridge.log" >> "${PIDFILE}"
    echo "  camera source     : ROS topics via SHM bridge"
    echo "  camera bridge PID : ${BRIDGE_PID}"
else
    echo "  camera source     : direct SHM (poll=${IMAGE_POLL_FPS} Hz)"
fi

echo "  logs              : ${LOG_PREFIX}.*.log"
echo ""
echo "Waiting for camera warmup and dataset initialization ..."
python3 "${CONTROL_SCRIPT}" cameras \
    --dataset-root "${DATASET_ROOT}" \
    --recorder-log "${LOG_PREFIX}.record_lerobot_official.log" \
    --pid "${RECORDER_PID}" \
    --timeout "${CAMERA_DETECTION_TIMEOUT_SEC:-15}"

if [ "${VR_CONTROL}" = "1" ]; then
    # Keep VR output in this terminal and in the session log. Only the helper
    # consumes invalid-episode events in VR mode, avoiding two racing readers.
    env CYCLONEDDS_URI="${COLLECT_CYCLONEDDS_URI}" \
        LD_LIBRARY_PATH="${ROBOT_ENV_LIB}:${ROS_LD_LIBRARY_PATH}" PYTHONPATH="${ROS_PYTHONPATH}" \
        "${ROBOT_PY}" -u "${VR_ARGS[@]}" > >(tee "${LOG_PREFIX}.vr_control.log") 2>&1 &
    VR_PID=$!
    echo "vr_control ${VR_PID} ${LOG_PREFIX}.vr_control.log" >> "${PIDFILE}"
    echo "  VR controls       : hold GL+GR; A=start, B=save, Y=discard, X=reset only; terminal Q=exit"
    if [ "${SUBTASK_COUNT}" -gt 0 ]; then
        echo "  VR annotation     : short B=mark subtask, last mark saves; hold B >= ${VR_A_LONG_PRESS_SEC:-1.0}s then release=save early"
    fi
fi

echo "============================================================"
echo "Interactive controls"
if [ "${_COLLECTOR_DAGGER_RECORDING:-0}" = "1" ]; then
    echo "  Recorder managed by DAgger supervisor; use the outer terminal/VR controls."
else
echo "  Enter   - Start a new episode"
if [ "${SUBTASK_COUNT}" -gt 0 ]; then
    python3 "${SCRIPT_DIR}/subtask_annotations.py" --subtasks-json "${SUBTASKS_JSON}"
    echo "  N / n   - Mark current subtask; the final mark saves the episode"
fi
echo "  S / s   - Save the current episode and pause"
echo "  D / d   - Discard the current episode and pause"
echo "  Q / q   - Save pending data and quit"
fi
echo "============================================================"

while true; do
    if [ "${VR_CONTROL}" = "0" ] && [ "${_COLLECTOR_DAGGER_RECORDING:-0}" != "1" ] && [ -f "${EPISODE_EVENT_FILE}" ]; then
        echo ""
        python3 "${CONTROL_SCRIPT}" episode-event --path "${EPISODE_EVENT_FILE}" || true
    fi

    if ! kill -0 "${RECORDER_PID}" 2>/dev/null; then
        echo ""
        echo "Recorder process exited. Check ${LOG_PREFIX}.record_lerobot_official.log"
        break
    fi

    if [ -n "${VR_PID:-}" ] && ! kill -0 "${VR_PID}" 2>/dev/null; then
        echo "ERROR: VR controller exited; stopping session. Check ${LOG_PREFIX}.vr_control.log" >&2
        exit 1
    fi

    # Detached VR sessions have no stdin. Avoid an EOF busy loop consuming a CPU.
    if [ "${INPUT_CLOSED:-0}" = "1" ]; then
        sleep 0.2
        continue
    fi

    if IFS= read -rsn1 -t 0.2 key; then
        if [ "${_COLLECTOR_DAGGER_RECORDING:-0}" = "1" ]; then
            continue  # Only the authority supervisor may open/close DAgger episodes.
        fi
        case "${key}" in
            "")
                echo ""
                echo "[control] start"
                send_control_and_report "start" || true
                ;;
            [sS])
                echo ""
                echo "[control] save"
                send_control_and_report "save" || true
                ;;
            [nN])
                if [ "${SUBTASK_COUNT}" -gt 0 ]; then
                    echo ""
                    echo "[control] mark_subtask"
                    send_control_and_report "mark_subtask" || true
                fi
                ;;
            [dD])
                echo ""
                echo "[control] discard"
                send_control_and_report "discard" || true
                ;;
            [qQ])
                echo ""
                echo "[control] quit"
                send_control "quit" || true
                break
                ;;
            *)
                ;;
        esac
    else
        read_status=$?
        if [ "${read_status}" = "1" ]; then
            if [ "${VR_CONTROL}" = "0" ]; then
                echo "Input closed. Stopping collection ..."
                break
            fi
            INPUT_CLOSED=1
        fi
    fi
done
