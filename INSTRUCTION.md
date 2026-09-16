# Collector Design

This document describes implementation constraints. Daily commands belong in
[README.md](README.md).

## Architecture

| Module | Responsibility |
| --- | --- |
| `start_lerobot_official_collect.sh` | Resolves session configuration, owns process lifecycle, and maps keyboard input to IPC commands. |
| `record_lerobot_official.py` | Buffers ROS/SHM signals, synchronizes a dataset row, and owns `LeRobotDataset`. |
| `robot_schema.py` | Owns joint names, canonical policy order, physical q23 conversion, and command parsing. |
| `camera_config.py` | Owns camera names, SHM paths, ROS topics, and shared defaults. |
| `shm_camera.py` | Reads stable metadata/image pairs and decodes BGR or uint16 depth frames. |
| `time_sync.py` | Provides timestamp normalization, FIFO selection, and interpolation primitives. |
| `collector_control.py` | Implements launcher-to-recorder IPC, status reports, and dataset summaries. |
| `vr_collector_control.py` | Optional guarded VR gestures, nonblocking countdown, recorder receipts, and TTS feedback. |

Default direct-SHM runtime:

```text
launcher
├── recorder          LeRobot + ROS2 Python
└── hand producer     robot_env, when hand cameras are enabled
```

`IMAGE_SOURCE=ros` adds `shm_camera_topic_bridge.py` between SHM and the
recorder. Direct SHM is preferred because it avoids image serialization and DDS
queueing.

## Data contract

The policy schema always starts with 16 dimensions:

```text
left arm (7), right arm (7), left gripper (1), right gripper (1)
```

`WITH_HEAD=1` appends neck roll/pitch/yaw. `WITH_UPPER_WAIST=1` appends only
waist pitch/yaw, while `WITH_WAIST=1` appends ankle, knee, waist pitch, and
waist yaw. The two waist modes are mutually exclusive. Joint state and joint
action use this exact order because LeRobot relative actions are index-based.

The ROS controller uses physical q23 order instead:

```text
waist (4), left arm (7), right arm (7), grippers (2), head (3)
```

Only `robot_schema.py` converts between these orders. A policy that does not
enable head or waist leaves those physical joints at their latest measured
positions.

## Image and synchronization path

Each camera exposes:

```text
/dev/shm/camera_metadata_struct_<name>
/dev/shm/camera_image_buffer_<name>
```

Metadata is `"<qiiiii"`: timestamp, width, height, channels, pixel format,
and byte count. A frame is accepted only when metadata before and after copying
the image is identical. RGB is BGR uint8 at the SHM boundary and becomes RGB
HWC for LeRobot. Depth is little-endian uint16 millimetres.

The recorder batch-reads metadata, copies only new frames, and stores each
camera in a bounded FIFO. A saved dataset row is built as follows:

1. Take the oldest mature frame from the reference camera.
2. Select nearest frames from all other cameras within `MAX_SYNC_DELTA_SEC`.
3. Interpolate state at the reference timestamp.
4. Select a causal action at or before that timestamp.
5. Add the complete row to the pending LeRobot episode.

If a required image, state bracket, or causal action is missing or stale, the
current episode is invalidated. FIFO overflow during recording also invalidates
the episode. A reference frame still waiting for peers only increments
`waiting ticks`; it does not mean a source frame was dropped and does not
invalidate the episode by itself.

## Session lifecycle

The launcher starts the recorder before camera producers so state history is
ready when image anchors arrive. `Enter` clears warmup image FIFOs and starts a
new episode. `S` saves it, `D` clears it, and `Q` saves only a valid pending
episode before finalizing the dataset.

IPC is intentionally file-based under the task directory: a named command FIFO,
an atomic status JSON, a readiness JSON, an episode-event JSON, and a PID file.
The files are transient; dataset files and logs are retained.

Resuming a dataset requires the same FPS, camera features, depth mode, joint
schema, and action mode. LeRobot may split videos into multiple files; metadata
indexes them, so they must not be concatenated.

## VR control protocol

`VR_CONTROL=1` starts the helper only after recorder/camera initialization, using
`ROBOT_PY` and the same ROS domain/robot suffix as the recorder. It subscribes to
`/control_topic_<domain>_<robot>` (`std_msgs/String` JSON); `l/r.b[1].p` are grips,
indices 4/5 are X/Y on the left and A/B on the right. It publishes only speech
requests (`status=play`, `text`) to `/topic_tts_<domain>_<robot>`, never motion.
It does not infer ASYNC/HOME/SYNC or alter the factory teleoperation mode.

Gestures require neutral, a single face press, and release with both grips held.
Malformed packets, multiple face buttons, grip loss, and input gaps cancel the
gesture. Start has a configurable countdown and requires recent VR input
(within 0.75 seconds). Countdown cancellation sends no recorder command.

One worker waits for command receipts while ROS callbacks continue processing
button releases. Extra commands during a pending operation are ignored, except
confirmed quit, which waits until that operation returns. Keyboard and VR
clients share `.official_control.lock` (flock held through the acknowledgement)
to prevent overwriting each other's single status file. Busy clients fail fast.
The empty lock file remains on disk to preserve its inode; it is not a running
session indicator. Start now acknowledges rejection as well as success.

Only the helper observes the atomic invalid-episode event in VR mode; the shell
does not delete it. Events and receipts are deduplicated, not inferred from
human-readable logs. Save of invalid data receives a discard acknowledgement
and is announced as discarded. A timeout means unknown outcome, not failure to
execute: never resend automatically. Check the recorder before restarting.

Launcher shutdown stops its own VR helper before recorder finalization. Helper
failure stops the session through normal cleanup. No SDK service is restarted,
no audio mixer is changed, and the colleague's separate HG-DAgger pre-roll
extension is not required or incorporated into ordinary collection.

## Extension rules

- Add cameras in `camera_config.py`, then cover them with configuration and SHM tests.
- Add joints in `robot_schema.py`; never duplicate joint order in the recorder or deployer.
- Add signals as timestamped buffers with an explicit causal or interpolation rule.
- Keep direct-SHM validation and FIFO matching. Replacing them with latest-frame reuse changes the data contract.

## Verification

```bash
PYTHONPATH=. python -m pytest -q tests
python -m py_compile *.py
bash -n start_lerobot_official_collect.sh
# Optional: ROS transport only, isolated topics + fake recorder, no motion/audio.
python tests/vr_ros_smoke.py
```

On the robot, also verify ROS discovery, SHM files, camera source FPS, video
encoder availability, CPU load, disk throughput, and `sync_log.jsonl` before
collecting production demonstrations.
