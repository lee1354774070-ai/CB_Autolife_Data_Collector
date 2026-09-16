# LeRobot Collector

Record synchronized Autolife demonstrations into an official `LeRobotDataset`.
One terminal starts the session and controls episodes.

## Start

```bash
cd /home/ubuntu/lerobot_data_collector
TASK_TEXT="pick up the water bottle" \
bash start_lerobot_official_collect.sh pick_up_water_bottle
```

Data is written to `/home/ubuntu/nas14/<task_name>/dataset` by default. Set
`OUTPUT_BASE_DIR` to use another storage location.

```bash
OUTPUT_BASE_DIR=/mnt/data \
TASK_TEXT="put the water bottle in the box" \
bash start_lerobot_official_collect.sh put_water_bottle_in_box
```

## Controls

| Key | Result |
| --- | --- |
| `Enter` | Start a new episode. |
| `S` | Save the current valid episode and pause. |
| `D` | Discard the current episode and pause. |
| `Q` | Save a valid pending episode, finalize, and exit. |
| `Ctrl+C` | Stop processes and keep saved data. |

When the terminal prints `EPISODE INVALID`, use `D`. Invalid episodes are never
safe to save.

## Single-operator VR control

Use the same launcher; no separate VR shell script is required:

```bash
VR_CONTROL=1 TASK_TEXT="pick up the water bottle" WITH_DEPTH=1 \
bash start_lerobot_official_collect.sh pick_up_water_bottle
```

Hold **both GL+GR**, then tap and release the face button while keeping the
grips held. First release any face buttons already held at startup.

| VR button | Result |
| --- | --- |
| `A` | Three-second countdown, then start a new episode. |
| `B` | Save and pause; announce success only after the recorder confirms. |
| `X` | Discard and pause. |
| Double `Y` within 5 seconds | Save valid pending data and exit, like keyboard `Q`. Keep GL+GR held between taps. |

`B` or `X` cancels an active countdown. Loss of VR messages also cancels the
countdown. Wait for the save/discard result before starting another episode.
An invalid episode is discarded even if you press `B`; it is never announced
as saved. After a command timeout, check logs instead of repeatedly pressing buttons.

`VR_CONTROL=0` (default) keeps keyboard-only operation. Keyboard controls remain
available in VR mode. Optional settings: `VR_START_DELAY_SEC=3` (0 through 30),
`VR_SPEECH=0` to disable Chinese voice announcements. Speech uses the existing
robot TTS service and current system volume. The factory VR input service must
already be running; the collector does not start teleoperation or move joints.

Inspect `logs/<run>.vr_control.log` for VR control receipts and errors.

## Common sessions

```bash
# Base 16-D joints and three RGB cameras.
bash start_lerobot_official_collect.sh rgb_task

# 30 FPS, 23-D joints, three RGB cameras, and head depth.
COLLECT_FPS=30 WITH_HEAD=1 WITH_WAIST=1 WITH_DEPTH=1 \
bash start_lerobot_official_collect.sh hotel_service

# 18-D joints: arms, grippers, and waist pitch/yaw; excludes leg lift joints.
WITH_UPPER_WAIST=1 bash start_lerobot_official_collect.sh upper_waist_task

# Diagnostic only: head-color camera with state-as-action fallback.
CAMERA_ONLY=1 bash start_lerobot_official_collect.sh camera_test
```

## Main configuration

| Variable | Default | Meaning |
| --- | --- | --- |
| `TASK_TEXT` | task name | Language instruction stored with each frame. |
| `COLLECT_FPS` | `30` | Dataset row rate. |
| `VR_CONTROL` | `0` | Enable guarded VR episode controls and voice feedback. |
| `WITH_HEAD` | `0` | Append 3 neck joints. |
| `WITH_UPPER_WAIST` | `0` | Append only `waist_pitch`, `waist_yaw` (2-D). |
| `WITH_WAIST` | `0` | Append ankle, knee, waist pitch/yaw (4-D). |
| `WITH_DEPTH` | `0` | Add `rgbd_head_depth` as uint16 depth video. |
| `ACTION_MODE` | `status_target` | `status_target`, `joint`, or `eef`. |
| `IMAGE_SOURCE` | `shm` | Direct shared-memory input or `ros` topics. |
| `SYNC_REFERENCE_CAMERA` | `hand_left` | Frame timestamp anchor. |
| `MAX_SYNC_DELTA_SEC` | `0.03` | Largest accepted camera timestamp delta. |
| `SYNC_IMAGE_BUFFER_SIZE` | `16` | Per-camera image FIFO capacity. |
| `MIN_CAMERAS` | all selected | Refuse startup when any requested camera is unavailable. |

Use the built-in help for every supported option:

```bash
bash start_lerobot_official_collect.sh --help
bash start_lerobot_official_collect.sh MAX_SYNC_DELTA_SEC --help
python record_lerobot_official.py --with-depth --help
```

## Resume and output

Running the same `task_name` resumes its dataset only when camera features,
FPS, joint schema, depth setting, and action mode match the existing root. Use
a new task name or output directory after changing any of them.

`WITH_UPPER_WAIST=1` and `WITH_WAIST=1` are mutually exclusive.

```text
<task_name>/
├── dataset/    # parquet, metadata, videos, sync_log.jsonl
└── logs/       # one log set per launcher run
```

Multiple video files are expected LeRobot output. Do not concatenate them
before training.

## Requirements

The robot needs ROS2 Jazzy, `robot_env`, `lerobot` (LeRobot 0.6+ for depth),
running robot state/action services, and camera services. See
[INSTRUCTION.md](INSTRUCTION.md) for synchronization and implementation details.
