# LeRobot Collector

Record synchronized Autolife demonstrations into an official `LeRobotDataset`.
One terminal starts the session and controls episodes.

Hand cameras require the vision service's `hand_left_jpeg` / `hand_right_jpeg`
SHM streams. JPEG-only reading is automatic; no new flag is needed. Head color
and optional depth work as before. Missing JPEG does not fall back to hand BGR.

## Select a mode

Use `COLLECTOR_MODE` to select the workflow:

| Value | Current status and purpose |
| --- | --- |
| `keyboard` | Original collection using Enter/S/D/Q in the terminal. |
| `vr` | Single-operator collection with existing GL+GR guards and speech. |
| `subtask` | Single-operator VR subtask annotation; requires nonempty `SUBTASKS_JSON`. |
| `dagger` | Experimental robot-300 V4 incremental control + Thor GR00T correction collection. Local checks passed; live integration not yet accepted. |

When unset, existing `VR_CONTROL` / `SUBTASKS_JSON` commands keep working;
without either setting, the default is `keyboard`. An explicit mode replaces
the need for `VR_CONTROL` and rejects conflicting legacy settings.

## DAgger Correction Collection

Uses **our** `deploy/groot_n1_7` Thor client, not the colleague's Thor client.
Update our Thor server too: `/health` must advertise `controller_submission_receipts: true`.
For a standalone Collector, set `DAGGER_TOOLS_ROOT` to the full `Autolife_VLA_Tools` repo.
The launcher creates a checked teleop runtime copy under `.runtime/`; original sources stay unchanged.

For initial owned-host startup, stop the existing HG-DAgger/V4 control stack;
never run two controllers. Later terminals can attach without restarting it.
This adapter requires the inspected `openarmx_teleop_vr_306_v4` and
`autolife_hg_dagger_MZJ_300` packages on robot 300, without editing those sources.
Currently it supports the 21-D GR00T baseline/frame contract with three RGB
inputs, not a generic PI0.5 or EDVA/SOMA-verifier entrypoint.

```bash
COLLECTOR_MODE=dagger DAGGER_SERVER_URL=http://THOR_IP:8777 \
DAGGER_PUBLISH=0 WITH_HEAD=1 WITH_UPPER_WAIST=1 WITH_DEPTH=1 \
TASK_TEXT="put the towel in the basket" \
bash start_lerobot_official_collect.sh towel_dagger
```

Replace `THOR_IP`; use a new task directory. Omit `WITH_DEPTH=1` for RGB-only
recording. `DAGGER_PUBLISH=0` is a no-hardware-output check, not a live recording
acceptance test. Set it to `1` only with a supervised, clear robot workspace.
Startup itself never begins inference.

Open `https://ROBOT_IP:8447`. VR: Y starts inference+recording; press either GL/GR
and keep holding for immediate expert authority, without waiting for inference or a controller hold acknowledgement.
A saves and X discards, both without reset; B discards the unsaved trial and resets the full
body, including grippers, only after recorder acknowledgement. No automatic
policy handback. A/X act once on press, with no long-press behavior. Terminal: C start, X discard,
A save, D discard, R reset; Q or Ctrl+C exits and
discards unconfirmed frames. A/B only reconcile the original request after an
uncertain save; they never replay it or reset before confirmation.

Append `--check` for read-only preflight. Run `bash start_lerobot_official_collect.sh DAGGER_PUBLISH --help`
for parameters. See [INSTRUCTION.md](INSTRUCTION.md#dagger-integration) for labels and validation boundaries.

### Attach to a running compatible host

An unmodified colleague host cannot be hot-attached. Build a compatible GUI
copy once (the original source is never changed):

```bash
python3 dagger/desktop_copy.py --source /path/to/original/scripts/dagger_launcher.py \
  --tools-root /path/to/Autolife_VLA_Tools --output /path/to/copy/dagger_launcher.py
```

Stop the old task before launching the copy. Launch the copy with the same
`DAGGER_DEPENDENCY_ROOT`, `DAGGER_SERVER_URL`, `DAGGER_TOKEN_FILE` and dataset
settings as the owned launcher; set `DAGGER_PUBLISH=1` only for supervised motion.
Its host remains running without terminal input. Keep this GUI open, then use
the **same task directory** in another terminal:

```bash
COLLECTOR_MODE=dagger DAGGER_BACKEND=attach DAGGER_PUBLISH=1 \
OUTPUT_BASE_DIR=/home/ubuntu/nas14 \
bash start_lerobot_official_collect.sh EXISTING_TASK --check
```

Remove `--check` for controls. C starts, A saves, X/D discards and R resets.
Q or EOF **only detaches**: it does not pause inference, save, reset or stop the
host. Use A/X first when you intend to end the trial. The VR/GUI stays active.
Attachment rejects old hosts, a different task, stale status or duplicate hosts;
it never falls back to launching another controller. An ambiguous command is
not automatically retried. See `DAGGER_BACKEND --help`.

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
COLLECTOR_MODE=vr TASK_TEXT="pick up the water bottle" WITH_DEPTH=1 \
bash start_lerobot_official_collect.sh pick_up_water_bottle
```

Hold **both GL+GR**, then tap and release the face button while keeping the
grips held. First release any face buttons already held at startup.

| VR button | Result |
| --- | --- |
| `Y` | Three-second countdown, then start a new episode. Never double-tap to exit. |
| `A` | Save and pause; announce success only after the recorder confirms. |
| `X` | Discard and pause. |
| `B` | Discard uncommitted data with acknowledgement, then request guarded full-body reset. Release both Grips afterwards. |

`A/X` cancels a countdown; B cancels it and requests reset. Loss of VR messages also cancels the
countdown. Wait for the save/discard result before starting another episode.
An invalid episode is discarded even if you press `A`; it is never announced
as saved. After a command timeout, check logs instead of repeatedly pressing buttons.

`COLLECTOR_MODE=keyboard` keeps keyboard-only operation. Legacy `VR_CONTROL=0/1`
still works when the mode is unset. Keyboard controls remain
available in VR mode. Optional settings: `VR_START_DELAY_SEC=3` (0 through 30),
`VR_SPEECH=0` to disable Chinese voice announcements. Speech uses the existing
robot TTS service and current system volume. The factory VR input service must
already be running. For V4 input set `VR_INPUT_TOPIC=/openarmx_teleop_vr_306_v4/vr_input`.
B causes real motion through the guarded V4 services at
`VR_RESET_PREFIX=/openarmx_teleop_vr_306_v4`; unavailable services block reset.
Disable legacy B/reset chords in the teleoperation mapper so collector owns B exclusively.
No vendor reset-topic fallback exists. Pending/uncertain storage blocks reset;
saved data is retained. Reset leaves the controller disabled; re-enable in the VR page.
Exit in the terminal with Q. See [VR feedback integration](VR_FEEDBACK.md).

Inspect `logs/<run>.vr_control.log` for VR control receipts and errors.

## Continuous subtask annotation

Set the overall task with `TASK_TEXT` and the ordered steps with `SUBTASKS_JSON`.
Use a new dataset directory when first enabling annotations:

```bash
COLLECTOR_MODE=subtask VR_A_LONG_PRESS_SEC=1.0 \
TASK_TEXT="put the towel in the basket" \
SUBTASKS_JSON='["pick up the towel with the right hand","transfer the towel between hands","place the towel with the left hand"]' \
bash start_lerobot_official_collect.sh towel_to_basket_annotated
```

Keep **GL+GR held**; short and long A actions both fire **on release**:

| Operation | Result |
| --- | --- |
| `Y` while idle | Countdown, then start one complete episode. |
| Short `A` while recording | Confirm the current subtask span and advance; recording continues. |
| Short `A` on the final subtask | Confirm the final span and save the entire episode. |
| Hold `A` for at least 1 second, then release | Save early, even with incomplete/missing annotations. |
| Keyboard `N` / `S` | Mark current subtask / save early. |

`X` discards; `B` discards then resets; terminal Q exits. Empty spans are rejected.
Discard resets the plan for the next episode. Intermediate marks do not encode video
or pause recording; receipt file writes run in a background thread. Brief Chinese
speech says "start", "next step", or "saved" only after recorder confirmation.
Early saves announce incomplete annotations. Continue moving without waiting for
audio playback. A final mark or long press first says "saving", not a success claim.
Full step text/progress remains in the terminal; `VR_SPEECH=0`
disables sound. The feedback-enabled VR page provides distinct vibration patterns.

The result remains one episode with one overall task. Parquet adds `subtask_index`:
`0/1/2` identify steps in that episode's plan; an unconfirmed tail is `-1`.
Texts and frame spans are stored in `dataset/annotations/subtasks/episode_000000.json`.
Early saves set `needs_review=true`. Boundaries use accepted frames at recorder
command handling, not a controller hardware timestamp. A missed press cannot be
reconstructed automatically: save early and correct labels later. Synchronization
failures still invalidate/discard the whole episode; long A cannot override this.

In `keyboard`/`vr`, leave `SUBTASKS_JSON` unset (default `[]`) to preserve legacy behavior. Do not change the
annotation switch on an existing root. Ordinary LeRobot training still uses the
overall task; subtask-conditioned sampling requires additional preprocessing.

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
| `COLLECTOR_MODE` | Derived from legacy settings, usually `keyboard` | `keyboard`, `vr`, `subtask`, `dagger`. |
| `TASK_TEXT` | task name | Language instruction stored with each frame. |
| `SUBTASKS_JSON` | `[]` | Ordered subtask texts; an empty array disables annotation. |
| `VR_A_LONG_PRESS_SEC` | `1.0` | A hold threshold for early save in subtask mode; 0.2 through 10 seconds. |
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
FPS, joint schema, depth setting, action mode, and annotation switch match the existing root. Use
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
