#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   bash /home/ubuntu/lerobot_data_collector/start_lerobot_official_collect.sh [task_name] [task_text]
#   TASK_TEXT="pick the mango" bash /home/ubuntu/lerobot_data_collector/start_lerobot_official_collect.sh pick_the_mango

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" = "--help" || "${1:-}" = "-h" || "${2:-}" = "--help" || "${2:-}" = "-h" ]]; then
    exec python3 "${SCRIPT_DIR}/launcher_help.py" "$@"
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
WITH_EEF_STATE="${WITH_EEF_STATE:-0}"
WITH_EEF_ACTION="${WITH_EEF_ACTION:-0}"
for parameter in WITH_HEAD WITH_WAIST WITH_UPPER_WAIST WITH_EEF_STATE WITH_EEF_ACTION \
    BACKGROUND_SAVE WITH_DEPTH CAMERA_ONLY FALLBACK_ACTION_TO_STATE STREAMING_ENCODING VR_SPEECH DEPTH_USE_LOG START_HAND_PRODUCER; do
    if [[ -v "${parameter}" && ! "${!parameter}" =~ ^[01]$ ]]; then
        echo "ERROR: ${parameter} must be 0 or 1." >&2
        exit 2
    fi
done
# Validate before creating data or starting ROS. Never reinterpret a removed
# action layout as joint targets in an existing dataset.
case "${ACTION_MODE:-status_target}" in
    status_target|joint) ;;
    eef)
        echo "ERROR: ACTION_MODE=eef was removed. Use status_target for regular collection or joint for DAgger; add poses with WITH_EEF_STATE/WITH_EEF_ACTION in a new dataset root." >&2
        exit 2
        ;;
    *)
        echo "ERROR: ACTION_MODE must be status_target (regular collection) or joint (DAgger/verified command topics)." >&2
        exit 2
        ;;
esac
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

SUBTASKS_JSON="${SUBTASKS_JSON:-[]}"
# Resolve once, before directory creation, ROS setup, or any process startup.
# In particular, DAgger must never accidentally start the factory VR helper.
MODE_CONFIG="$(python3 "${SCRIPT_DIR}/collector_modes.py" --mode "${COLLECTOR_MODE:-}" \
    --vr-control "${VR_CONTROL}" --subtasks-json "${SUBTASKS_JSON}")"
read -r COLLECTOR_MODE VR_CONTROL SUBTASK_COUNT <<< "${MODE_CONFIG}"
printf 'AutoLife Collector | mode=%s | %s\n' "${COLLECTOR_MODE}" "${SCRIPT_DIR}"
if [ "${COLLECTOR_MODE}" = "dagger" ]; then
    exec python3 "${SCRIPT_DIR}/dagger/run.py" "$@"
fi
TASK_NAME="${1:-mango_pick}"
if [[ "${TASK_NAME}" = "." || "${TASK_NAME}" = ".." || "${TASK_NAME}" = */* || "${TASK_NAME}" = -* || "${TASK_NAME}" = *$'\n'* ]]; then
    echo "ERROR: task_name must be one directory name, not a path or option." >&2
    exit 2
fi
# Own camera switching for the entire session, including graceful recorder cleanup.
if [[ -f "${HOME}/.config/autolife_collector/camera_session.json" && "${COLLECTOR_CAMERA_SESSION_ACTIVE:-0}" != "1" ]]; then
    exec /usr/bin/python3 "${SCRIPT_DIR}/camera_session.py" run -- bash "${BASH_SOURCE[0]}" "$@"
fi
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
    --a-long-press-sec "${VR_A_LONG_PRESS_SEC:-1.0}"
    --subtask-b-rearm-sec "${VR_SUBTASK_B_REARM_SEC:-2.0}")
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

# Serialize startup as well as recording. The dataset lock is acquired later;
# without this lock two launchers can replace each other's FIFO/PID files.
# Keep the inode; children inherit the lock until they finish if the launcher dies.
exec {SESSION_LOCK_FD}>"${BASE_DIR}/.official_session.lock"
if ! flock -n "${SESSION_LOCK_FD}"; then
    echo "ERROR: another launcher owns this collection directory: ${BASE_DIR}" >&2
    exit 1
fi

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
    # A second signal must not interrupt wait and erase a live writer's PID.
    trap '' INT TERM

    # Stop our input helper first, so it cannot send new commands during finalization.
    if [ -n "${VR_PID:-}" ] && kill -0 "${VR_PID}" 2>/dev/null; then
        kill -INT "${VR_PID}" 2>/dev/null || true
        for _ in $(seq 1 20); do
            kill -0 "${VR_PID}" 2>/dev/null || break
            sleep 0.1
        done
    fi

    # Q/EOF/signal requests graceful finalization. Encoding may take minutes;
    # an acknowledgement timeout is not permission to terminate the writer.
    if [ -n "${RECORDER_PID:-}" ] && kill -0 "${RECORDER_PID}" 2>/dev/null; then
        send_control "quit" >/dev/null 2>&1 || kill -INT "${RECORDER_PID}" 2>/dev/null || true
        echo "Waiting for recorder finalization. Keep this terminal open; encoding can take minutes."
    fi
    if [ -n "${RECORDER_PID:-}" ]; then
        # Startup cancellation can precede the FIFO listener/signal handlers.
        # Quit is idempotent; retry it without killing an encoding writer.
        while kill -0 "${RECORDER_PID}" 2>/dev/null; do
            sleep 2
            kill -0 "${RECORDER_PID}" 2>/dev/null || break
            send_control "quit" >/dev/null 2>&1 || true
        done
        wait "${RECORDER_PID}" || echo "WARNING: recorder exited with an error; inspect ${LOG_PREFIX}.record_lerobot_official.log" >&2
    fi

    if [ -f "${PIDFILE}" ]; then
        tac "${PIDFILE}" | while read -r role pid log; do
            if [ -z "${pid:-}" ] || [ "${role}" = "recorder" ]; then
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
    python3 "${CONTROL_SCRIPT}" summary --dataset-root "${DATASET_ROOT}" || true
    python3 "${CONTROL_SCRIPT}" sync-report --sync-log "${DATASET_ROOT}/sync_log.jsonl" || true
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
# A venv may use a different base interpreter (and OpenSSL) from the old conda layout.
LEROBOT_ENV_LIB="$("${LEROBOT_PY}" -c 'import sys; print(sys.base_prefix + "/lib")')"
DEFAULT_DDS='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
COLLECT_CYCLONEDDS_URI="${COLLECT_CYCLONEDDS_URI:-${CYCLONEDDS_URI:-${DEFAULT_DDS}}}"
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
if [ "${WITH_EEF_STATE}" = "1" ]; then
    RECORDER_FEATURE_ARGS+=(--with-eef-state)
fi
if [ "${WITH_EEF_ACTION}" = "1" ]; then
    RECORDER_FEATURE_ARGS+=(--with-eef-action)
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
ROBOT_ENV_LIB=""
if [[ "${START_HAND_PRODUCER:-1}" = "1" || "${IMAGE_SOURCE}" = "ros" || "${VR_CONTROL}" = "1" ]]; then
    ROBOT_ENV_LIB="$("${ROBOT_PY}" -c 'import sys; print(sys.base_prefix + "/lib")')"
fi

echo "  joint data  : head=${WITH_HEAD}, upper_waist=${WITH_UPPER_WAIST}, waist=${WITH_WAIST}"
echo "  depth data  : ${WITH_DEPTH}"
echo "  EEF data    : state=${WITH_EEF_STATE}, action=${WITH_EEF_ACTION} (appended after joints)"
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
    --min-free-disk-gb "${MIN_FREE_DISK_GB:-2}"
    --max-episode-temp-gb "${MAX_EPISODE_TEMP_GB:-8}"
    --max-episode-seconds "${MAX_EPISODE_SECONDS:-600}"
    --image-writer-max-pending "${IMAGE_WRITER_MAX_PENDING:-64}"
    --image-writer-max-mb "${IMAGE_WRITER_MAX_MB:-256}"
    --image-writer-threads "${IMAGE_WRITER_THREADS:-4}"
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
    --state-eef-topic "${STATE_EEF_TOPIC:-/topic_arm_current_robot_eef_pose_${TOPIC_NODE_ID}}"
    --action-eef-topic "${ACTION_EEF_TOPIC:-/topic_arm_target_robot_eef_pose_${TOPIC_NODE_ID}}"
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
for parameter in VIDEO_CRF VIDEO_PRESET VIDEO_GOP ENCODER_THREADS VIDEO_FILES_SIZE_IN_MB DATA_FILES_SIZE_IN_MB; do
    if [ -n "${!parameter:-}" ]; then
        option="${parameter,,}"
        RECORDER_ARGS+=("--${option//_/-}" "${!parameter}")
    fi
done
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
        echo "  VR anti-repeat    : after acknowledgement, release B continuously for ${VR_SUBTASK_B_REARM_SEC:-2.0}s; retries restart this interval"
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
echo "  F / f   - Seal saved episodes and stay ready (save/discard current episode first)"
echo "  Q / q   - Save pending data and quit"
fi
echo "============================================================"

# A single lightweight monitor prints durable completion, not a new Python
# process on every keyboard poll. It never sends recorder or robot commands.
if [ "${VR_CONTROL}" = "0" ] && [ "${BACKGROUND_SAVE:-1}" = "1" ]; then
    python3 "${SCRIPT_DIR}/background_saves.py" --watch "${BASE_DIR}/.official_background_saves.json" "${RECORDER_PID}" &
    echo "background_feedback $! ${LOG_PREFIX}.record_lerobot_official.log" >> "${PIDFILE}"
fi

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
            [fF])
                send_control_and_report "seal" || true
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
