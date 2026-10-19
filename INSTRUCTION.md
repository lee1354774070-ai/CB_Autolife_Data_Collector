# Collector Design

This document describes implementation constraints. Daily commands belong in
[README.md](README.md).

## Architecture

| Module | Responsibility |
| --- | --- |
| `start_lerobot_official_collect.sh` | Resolves session configuration, owns process lifecycle, and maps keyboard input to IPC commands. |
| `collector_modes.py` | Resolves exclusive workflows before startup and rejects conflicting flags. |
| `dagger/` | Version-checked V4/HG adapter, exclusive stack launcher, Thor session fencing, VR feedback and asynchronous diagnostic traces. |
| `dagger_labels.py` | Validates causal action origins and builds per-frame DAgger training masks. |
| `record_lerobot_official.py` | Buffers ROS/SHM signals, synchronizes a dataset row, and owns `LeRobotDataset`. |
| `episode_storage.py` | Enforces capture budgets, owns the dataset lock, and publishes finalized episodes with a recovery journal. |
| `dagger/capture_image_writer.py` | Shared bounded asynchronous image writer; propagates worker failures in every mode. |
| `robot_schema.py` | Owns joint names, canonical policy order, physical q23 conversion, and command parsing. |
| `eef_schema.py` | Validates measured/target EEF reports, performs pose interpolation, and declares explicit channel groups. |
| `convert_dataset.py` | Independently projects state/action into a new v3 dataset with consistent per-channel metadata/statistics. |
| `camera_config.py` | Owns camera names, SHM paths, ROS topics, and shared defaults. |
| `shm_camera.py` | Reads stable legacy/SHM2 metadata/image pairs and decodes JPEG, native color or uint16 depth. |
| `time_sync.py` | Provides timestamp normalization, FIFO selection, and interpolation primitives. |
| `collector_control.py` | Implements launcher-to-recorder IPC, status reports, and dataset summaries. |
| `vr_collector_control.py` | Optional guarded VR gestures, nonblocking countdown, recorder receipts, and TTS feedback. |
| `subtask_annotations.py` | Validates ordered plans, tracks spans, and materializes labels/sidecar metadata at save. |

Default direct-SHM runtime:

```text
launcher
├── recorder          LeRobot + ROS2 Python
└── hand producer     robot_env, when hand cameras are enabled
```

`IMAGE_SOURCE=ros` adds `shm_camera_topic_bridge.py` between SHM and the
recorder. Direct SHM is preferred because it avoids image serialization and DDS
queueing.

## S2 / S3 compatibility

Both families use `main`; no S3 branch switch is required. The ROS helper keeps
an explicit `ROS_DOMAIN_ID` (default 0) and resolves the robot ID from
`HG_DAGGER_ROBOT_ID`, then `ROBOT_ID`, then the hostname suffix, then 300.
It restores the caller's nounset setting even if sourcing ROS fails. The vendor
ROS workspace is optional; system ROS is still required. The launcher resolves
library roots from each configured interpreter's `sys.base_prefix`, including
venvs, without changing the recorder's data layout.

Set `ROBOT_TYPE=autolife_s3` for new S3 datasets. The historical default
`autolife_s1` is preserved for compatibility; this is metadata, not a kinematic
conversion or DAgger controller profile. Existing datasets keep their original
label on resume. Optional EEF capture and conversion are shared, but the vendor
pose reports and their semantics must be verified on each robot.

`s3_thor_preview.py` is an optional read-only inference entry point requiring
the tools repository's `deploy.groot_n1_7` runtime with the reviewed phase-gripper
patch (`tools/patch_cb_thor_gripper.py`). It checks that API before loading model
weights; ordinary collection neither imports it nor requires that patch.
Its health response forbids hardware execution and its execution ACK
endpoints reject requests. Joint-sign conversion is not trajectory retargeting;
it does not prove that an S2 policy can control an S3 robot. Historical evidence
and remaining acceptance work are in [S3_TESTING_zh.md](S3_TESTING_zh.md).

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

### Optional EEF capture and projection

`WITH_EEF_STATE` and `WITH_EEF_ACTION` default to zero and independently append
14 pose values after the full joint vector. Each arm contributes XYZ in metres
and quaternion XYZW, left arm first, in the vendor controller's robot frame.
The controller defines the EEF point; no TCP offsets or map transforms are
applied. Grippers remain in the joint block. The flags cannot be combined with
the old 15-D `ACTION_MODE=eef` debugging mode.

State subscribes to `current_robot_eef_pose`; action subscribes to
`target_robot_eef_pose`, with the usual domain/robot suffix. The vendor String
JSON contains `left_eef_pose`/`right_eef_pose`, each with `position[3]` and
`rotation[4]`. Partial/nonfinite payloads and zero quaternions are rejected;
valid quaternions are normalized. This does not start vendor services, publish
commands, call SDK controllers, or substitute FK/zero/measured-state actions.
Robot 345 did not publish these reports during the read-only inspection; joint
feedback alone is not evidence that EEF collection is ready.

EEF state has a separate bounded telemetry buffer. XYZ is interpolated linearly,
rotation uses shortest-arc SLERP, and both use the image anchor and existing
state age/bracket limits. A pending right endpoint retains the image for retry;
missing/stale history invalidates the episode. EEF action holds only a causal
target within `MAX_ACTION_HOLD_SEC`. Timestamp alignment uses the same payload
timestamp/receive-time fallback as joint telemetry, not hardware synchronization.
The optional callbacks do no disk I/O, RPCs, IK, or model work.

`meta/collector_schema.json` describes names, joint/EEF/gripper indices and EEF
semantics. It survives the first empty writer and every episode transaction.
Resume rejects an incompatible recorded layout/source. Legacy joint datasets
without this manifest retain their existing names and remain resumable without
EEF, but conversion refuses to infer EEF indices from array widths.

`convert_dataset.py SOURCE DEST --state joint|eef|all|eef+gripper
--action joint|eef|all|eef+gripper` independently chooses both vectors. Pure EEF
means 14-D; `eef+gripper` means poses followed by two existing gripper channels.
It never adds unrecorded channels or removes rows. Source locking excludes an
active collector. Bounded Arrow batches rewrite vectors and their Hugging Face
schema, while global and per-episode statistics (including quantiles) are
projected exactly; count stays unchanged. The per-action DAgger mask and its
statistics use the same action projection; other labels, tasks, sidecars, timestamps
and v3 shard references are preserved. Videos, including depth, are copied
byte-for-byte. The Linux output directory is atomically published with
RENAME_NOREPLACE only after completion; source and existing destinations are
never overwritten. Logs, IPC and uncommitted workspaces are not training data.

DAgger still controls the original 21-D joint model. Its origin/epoch labels
prove the joint proxy commands, not the independent vendor EEF target report.
The appended EEF action mask is therefore zero, not inferred from joint origin.
EEF expert training needs independently verified targets and explicit labels.
Capture does not add
EEF deployment support: model I/O, IK and relative rotation processing must be
configured separately. Quaternion components must not be subtracted like joints.

## Image and synchronization path

Each camera exposes:

```text
/dev/shm/camera_metadata_struct_<name>
/dev/shm/camera_image_buffer_<name>
```

For hands, the physical `<name>` is `hand_left_jpeg` / `hand_right_jpeg`.
Only these JPEG streams are read and decoded locally to BGR; the old decoded
hand SHM streams are never probed. Missing or stalled JPEG cannot fall back to
BGR. Dataset keys and ROS topics remain `hand_left` / `hand_right`.

Legacy metadata is `"<qiiiii"`: timestamp, width, height, channels, pixel format,
and byte count. SHM2 metadata additionally selects the published buffer slot;
double/ring-buffer support is unchanged. A frame is accepted only when metadata
before and after copying matches. Native head color is unchanged; all color
inputs become RGB HWC for LeRobot. Depth remains little-endian uint16 millimetres.
The optional hand producer forwards original V4L2 JPEG packets without per-frame
decoding/re-encoding and refuses capture backends that cannot supply raw JPEG.
It must not run concurrently with the vision service owning those devices.

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
ready when image anchors arrive. `Enter` clears the reference FIFO and keeps at
most one matching neighbor per other camera (still subject to age/delta checks).
Only a new reference frame can start the dataset rows of a
new episode. `S` saves it, `D` clears it, and `Q` saves only a valid pending
episode before finalizing the dataset.

IPC is intentionally file-based under the task directory: a named command FIFO,
an atomic status JSON, a readiness JSON, an episode-event JSON, and a PID file.
The files are transient; dataset files and logs are retained.

Resuming a dataset requires the same FPS, camera features, depth mode, joint
schema, and action mode. LeRobot may split videos into multiple files; metadata
indexes them, so they must not be concatenated.

## Storage isolation and limits

All modes require LeRobot 0.6+ with `DatasetWriter` and `LeRobotDataset.resume`.
The official dataset is read-only to the active writer: only its metadata is
copied into `.collector-work`. Historical data/video files are not copied or
opened for append. Empty datasets use `create`, since they have no tasks Parquet
to resume. Each episode gets a fresh writer and new data/video file indices.

Publication drains image workers, encodes videos sequentially, closes Parquet
writers, and completes subtask sidecars inside the workspace. `EpisodeStore`
fsyncs staged files/directories, writes a prepared journal, links new output files
without replacing existing ones, preserves old metadata, installs new metadata,
then records the commit. A failure before that commit restores old metadata and
removes only matching newly linked files. Unresolved recovery stops the session.
A completed commit is not undone or reported failed because cleanup or preparation
of the next episode failed. The brief metadata replacement window is not a
concurrent-reader transaction: do not train/edit this root during recording.

Disk checks run on a background thread every 0.5 s. Frame admission uses cached
free bytes minus newly admitted bytes; a snapshot older than 2 s fails closed.
The defaults are 2 GiB minimum free space plus an extra raw-episode reserve,
8 GiB cumulative estimated raw image bytes/overhead per episode, and 600 s per
episode. The first limit reached invalidates the episode. These are estimates,
not a filesystem quota or a promise of 600 s at any resolution. Images consume
most of the budget; other processes can still exhaust space between checks.

The image writer bounds all cameras together to 64 unfinished jobs and 256 MiB
of retained raw input bytes. Worker conversion buffers, camera FIFOs, LeRobot
numeric columns and encoders are additional memory. It never silently drops a
queued frame. DAgger keeps uncompressed temporary PNGs; other modes use light
PNG compression. Both paths propagate disk/codec errors, including delayed worker
errors behind a slow first job. Capture checks are nonblocking; save/discard
explicitly join workers before publishing/deleting files.

An error invalidates the current episode even if only part of its first frame
was submitted. Invalid data cannot be saved; explicit discard closes workers and
removes only its workspace. No automatic save retry occurs. Confirmed invalid
DAgger failures allow Y discard; ambiguous results retain their request ID and
remain blocked. Diagnostic/receipt write failure cannot undo a committed episode
or manufacture a success acknowledgement. RAM invalidation still works when disk
space is exhausted, but file-based notifications can be unavailable.

An advisory lock allows one cooperating collector per dataset. A prepared journal
is rolled back on restart; a committed journal is cleaned without replay. Other
interrupted work is renamed `.collector-interrupted-<timestamp>` for inspection,
not counted as saved, and never deleted automatically. These leftovers still use
disk space. Do not remove journals/workspaces while a collector runs. Storage must
support same-filesystem hard links, rename, flock and fsync; unsupported mounts
fail rather than weaken the protocol. This protects against tested software I/O
failures; it is not a guarantee against hardware failure or all power-loss cases.

`BATCH_ENCODING_SIZE=1` and `STREAMING_ENCODING=0` are required. File-size options
do not merge separate episode files. Historical datasets remain unchanged until
a new episode is successfully published.

## VR control protocol

`COLLECTOR_MODE=vr` or `subtask` enables the following controls. Unset mode
preserves `VR_CONTROL=1`. This is the existing collection protocol, **not an
implementation of DAgger using the new incremental teleoperation**.

`VR_CONTROL=1` starts the helper only after recorder/camera initialization, using
`ROBOT_PY` and the same ROS domain/robot suffix as the recorder. It subscribes to
`/control_topic_<domain>_<robot>` (`std_msgs/String` JSON); `l/r.b[1].p` are grips,
indices 4/5 are X/Y on the left and A/B on the right. It publishes speech
requests to `/topic_tts_<domain>_<robot>` and structured events to `/collector/feedback`.
Factory X reset is currently unavailable and reports that limitation; DAgger X
uses the guarded V4 reset services, never guessed raw joint targets.
It does not infer ASYNC/HOME/SYNC or alter the factory teleoperation mode.

Gestures require neutral, a single face press, and release with both grips held.
Malformed packets, multiple face buttons, grip loss, and input gaps cancel the
gesture. Start has a configurable countdown and requires recent VR input
(within 0.75 seconds). Countdown cancellation sends no recorder command.

One worker waits for command receipts while ROS callbacks continue processing
button releases. Extra commands during a pending operation are ignored, except
terminal quit, which waits until that operation returns. A only starts. Keyboard and VR
clients share `.official_control.lock` (flock held through the acknowledgement)
to prevent overwriting each other's single status file. Busy clients fail fast.
The empty lock file remains on disk to preserve its inode; it is not a running
session indicator. Start now acknowledges rejection as well as success.

The background receipt cache stats files at 10 Hz and parses only replacements
or changed content. ROS callbacks never perform NAS status-file reads. Missing,
malformed or blocked files cannot manufacture a success receipt. The web uses
one SSE connection for feedback; cached UI assets avoid repeated source reads.

Each camera has one SHM source. An unchanged timestamp causes no image read,
decode, or FIFO insertion. No decoded/JPEG source switching is performed;
FIFO and invalidation limits are unchanged.
After dataset creation, only schema cameras are polled; late unselected cameras
cannot waste decoding work or overflow unused queues. OMP/MKL/OpenBLAS default
to one thread unless explicitly configured; recorder OpenCV uses one thread.
LeRobot image/video worker settings and capture FPS remain unchanged.

Only the helper observes the atomic invalid-episode event in VR mode; the shell
does not delete it. Events and receipts are deduplicated, not inferred from
human-readable logs. Save of invalid data receives a discard acknowledgement
and is announced as discarded. A timeout means unknown outcome, not failure to
execute: never resend automatically. Check the recorder before restarting.

Discard joins all image workers and removes only `.collector-work`; partially
submitted first frames are included. An unresolved publication journal blocks
discard and requires recovery before proceeding.

Launcher shutdown stops its own VR helper before recorder finalization. Helper
failure stops the session through normal cleanup. No SDK service is restarted,
no audio mixer is changed, and the colleague's separate HG-DAgger pre-roll
extension is not required or incorporated into ordinary collection.

## DAgger Integration

### Host and attachment ownership

`DAGGER_BACKEND=owned` starts one complete stack; `attach` only subscribes to
fresh control state and calls the compatible host's operator service. The
versioned contract binds host instance, robot, DDS domain and dataset FIFO.
Commands have request IDs and a 30-second validity window. Duplicate requests
are cached; uncertain outcomes are not replayed. This is local coordination,
not an authentication mechanism. Attach Q leaves the host and current task
running. The GUI copy builder preserves the original file, replaces its launch
entry with our persistent `--serve` host, and rejects unreviewed source versions.
A legacy running stack must be stopped once before starting that copy.


`COLLECTOR_MODE=dagger` selects the **new incremental V4 stack**, not factory VR.
The adapter depends on the inspected robot-300 sources under
`/home/ubuntu/ros2_ws/src/openarmx_teleop_vr_306_v4`, plus the bundled `dagger/control/` and `dagger/assets/` directories.
Critical files/configs are SHA-256 checked. Changes require review and new tests;
there is no bypass switch. Existing controllers, unknown command publishers,
or any observed command during the startup window cause preflight rejection.
Only the audited vendor nodes may retain idle endpoints. The vendor's
`target_robot_eef_pose`/`target_robot_height_z` reports are not command inputs;
actual `move_*`, joint and gripper command topics are checked. The guarded
hold-only SYNC service and motor subscribers must also be available. The V4
controller retains continuous command arbitration and heartbeat checks.
Conda-based IK/Web virtualenvs receive their own interpreter's library path;
system ROS processes do not inherit that override.
An enable service acknowledgement is not hardware readiness. The start worker
waits up to 20 seconds for a fresh `ARMED` status with both `hardware_enabled`
and `hardware_ready`, and no pending enable, before granting policy authority.
Cancellation, a fault or timeout stops this start attempt without publishing
policy targets; ROS button callbacks remain independent of the wait.
Do not run the normal Thor robot client
alongside this stack; its policy bridge owns the remote proposal/ACK session.
`runtime_copy.py` creates a content-identified V4 copy in `.runtime/`, applies
`v4_controller.patch` only there, then validates cached-copy hashes on reuse.
The original V4 directory is never patched. Mapper, IK algorithms, limits
and collision checks remain from the inspected version, not replaced with guessed parameters.

Robot ID/domain routing and `DAGGER_ROBOT_PROFILE` are configurable; 300 defaults
to `s2_v2_2`. Other robots need an explicit reviewed model, matching dependencies
and their own hardware acceptance. Configurable routing is not proof of a
deployed S3 controller. The model still uses 21-D arms+grippers+head+upper waist, three RGB
inputs and GR00T `policy_only_baseline`/`policy_only_frame`. Depth can be recorded,
but is not fed to this RGB-only model contract. PI0.5 and causal EDVA/SOMA Outcome
history are not integrated here. The pinned `dagger/control/` snapshot supplies
the DAgger supervisor, incremental gripper/pickup, full-body reset, FIFO and GR00T
capture/21-D mapping/chunk execution. `SOURCE.json` records provenance and changes.
Thin adapters add collection controls, provenance, session/epoch fencing and
controller-submission receipts. Received actions are never clipped or rewritten.

Our Thor server gains `/controller_ack`, limited to policy-only baseline/frame.
It retires the exact digest-bound **submitted target prefix**, not a claim that
the physical robot published/reached every target. It cannot advance verifier
history. Update the Thor server too; preflight requires the advertised capability.
Ordinary execution `/ack` and training behavior are unchanged. Lost HTTP replies
are never replayed; uncertain controller receipts latch failure rather than invent
executed actions. Takeover revokes local output while a slow HTTP request finishes.

A first waits for the recorder's correlated start receipt, then enables policy
authority. Hardware startup may take time: the recorder waits up to 30 s for
fresh action provenance before accepting its first frame. This visible initial
barrier is not a skipped frame inside an episode. After it, ordinary FIFO,
interpolation, freshness and whole-episode invalidation rules remain unchanged.
A new press of either Grip immediately selects expert authority and publishes a
new epoch. No inference result or hold acknowledgement is awaited. The existing
V4 controller atomically replaces old policy goals with measured joints and
invalidates in-flight IK; the existing mapper re-anchors the held Grip against
current pose and tags expert inputs with that epoch. Late policy/expert packets
from an older epoch are rejected, not relabelled. Fault/reset barriers remain.
Actual hardware latency still includes DDS delivery, mapper ticks and IK.
A held button does not repeatedly request takeover. Y only discards the episode.
Human control does not automatically return to policy.

Policy messages must match the active session and authority epoch. HTTP results
from an earlier generation are discarded by the bridge and rejected again at
the supervisor. The V4 controller remains the sole hardware publisher and retains
its joint/configuration, trajectory, watchdog and collision guards. The supervisor
does not impose an additional 8/10-degree target-to-measurement cutoff: tracking
lag is not an adjacent-action jump. Finite 21-D action/state validation, authority
checks and rejection of gripper targets outside [10,360] degrees remain enabled.
This does not bypass the V4 controller's command-lead caps or mechanical limits.
Software stopping cannot replace a physical E-stop.
On exit, the controller keeps ROS alive until its owned hold/lease release is
sent. A disabled controller sends no new motor targets just to shut down.

B revokes output and saves the entire accepted trajectory without reset.
Y discards without reset. X only resets the full body and opens grippers; open
or unresolved data blocks it without changing data. All require a new A to start.
B/Y trigger once per press. Unknown receipts retain the original request ID;
reconcile before reset, never blindly repeat save/discard.

Q/Ctrl+C closes only this launcher's children and discards unconfirmed data.
Recorder invalidation revokes control on the next state update; process death
closes the stack. The default `DAGGER_PUBLISH=0` does not authorize hardware motion.

Custom features are int64 `[1]`, except the per-action mask described below;
they are separate dataset features, not additional state/action channels:

| Field | Meaning |
| --- | --- |
| `dagger.control_source` | 0=policy, 1=human active, 2=hold/transfer. |
| `dagger.is_intervention` | Whether this trial has seen human intervention. Not a loss mask. |
| `dagger.intervention_id` | Episode-local human segment number. |
| `dagger.authority_epoch` | Authority generation for auditing transitions. |
| `dagger.train_mask` | 1 only for human-active frames whose arm AND gripper commands originate at/after this authority boundary and match its epoch. |
| `dagger.action_train_mask` | Same width/order as action: 21 joint channels, or 35 with EEF. Body and gripper origins are masked separately; appended EEF channels stay zero because their origin is unproven. |
| `dagger.anchor_timestamp_ns` | Image-aligned frame anchor on the robot timebase. |
| `dagger.arm_command_timestamp_ns`, `dagger.gripper_command_timestamp_ns` | Original observed command times; republishing a latched gripper target does not make it new expert supervision. |

Use a separate dataset root; schema checks prohibit mixing ordinary, older HG
and current DAgger features. A policy-only saved trajectory has mask=0 throughout.
Saving does not establish task success. Training must explicitly apply the mask
at **each action-chunk timestep**; stock training does not automatically consume
these fields. The legacy scalar mask proves only the original joint commands.
For partial corrections, select `action_train_mask.any(-1)` and apply the
per-dimension mask before reduction, not the scalar mask as a row filter.
No automatic fine-tuning/DAgger iteration is implemented.

FIFO/encoding waits run in workers; X does not wait for either. Diagnostic traces
use a bounded 2048-entry background queue, with drops/errors recorded in the
trial manifest; training labels never rely on this best-effort trace. WebXR
haptics and a 70 ms browser beep provide best-effort feedback after state/receipt
changes, subject to headset and browser audio permission. No robot mixer or
factory TTS settings are changed. This is not hard real-time control.

Recorder-status files are read by a 20 Hz worker; control callbacks read memory
only. A blocked reader older than 1 s revokes output. Trace open/flush/close also
run on one bounded worker. V4 DDS discovery runs at 5 Hz outside its control
lock; a guard snapshot older than 600 ms fails closed at the next guard check. The copied controller
checks authority both before and after kinematic validation and fences pending
policy targets as soon as the authority change arrives. Old clutch and release
packets are rejected before mutating controller state. These changes do not remove watchdogs.

After a successful vendor publish, the copied controller emits a separate
`/collector_dagger/controller_output` receipt with timestamp and target-origin
epoch; vendor JSON is unchanged. Dataset actions use these receipts, not untagged
DDS echoes. Old held targets keep their original epoch and cannot become expert
supervision merely because control mode changed. This still proves publication,
not physical task success. Missing/stale receipts follow the normal invalidation rules.

Inference is chunked and sequential; a chunk boundary can still wait for Thor.
This change removes avoidable control-path I/O stalls, not GPU inference time.
It does not claim RTC overlap, hard deadlines, or zero motion jitter.

Validation: isolated-domain ROS supervisor/FIFO tests with fake hardware, and
real LeRobot 0.6.0 Parquet/video write/resume/read tests using synthetic frames.
Neither validates physical takeover latency, collision behavior, audible/haptic
feedback, or closed-loop task success. Basic operator acceptance was reported on
2026-10-08; release regressions and remaining checks are in `ROBOT300_TESTING_zh.md`.

## Subtask annotation and latency

A nonempty `SUBTASKS_JSON` enables a custom `subtask_index` feature (`int64`,
shape `[1]`). `task` / `task_index` still identify the overall task throughout
the episode. Subtask IDs are **episode-local** plan ordinals, not global task
IDs and not new state/action dimensions. Resolve their texts via each episode's
sidecar before combining labels across different plans.

B uses monotonic hold duration and fires only on release, never both a short
mark and a long save. Grip loss, mixed buttons, malformed input and stream gaps
cancel the gesture. Receipts reset unfinished gestures across state transitions.
The existing FIFO delivers `mark_subtask` to the same recorder thread as
`add_frame`. Its current accepted frame count is the exclusive end of a span;
the next span starts at that end. Empty or excess marks are rejected. The final
mark saves the whole episode, without splitting data or video files per subtask.

Intermediate marks append one boundary and construct a small receipt. They do
not encode video, traverse images, sleep, reset image FIFOs or pause recording.
Receipt file I/O runs on a single background worker, important for NAS roots.
While it is pending, subsequent commands wait but recording timers keep running.
Labels and sidecar files are materialized only when saving. A separate VR worker
polls receipts every 10 ms for the first second, then 100 ms for longer saves.
Completed-command checks run every 20 ms; keyboard receipts/health checks remain
at 100 ms. No feedback playback is awaited.

`Recorder acknowledgement latency` measures local command submission to receipt
handling, **not** camera skew, button-to-speaker delay or haptic delay. This is
not hard real-time: Linux, DDS, NAS and camera scheduling can add jitter. At
30 FPS frame granularity alone is about 33.3 ms, plus synchronization and command
processing effects. Camera frames still waiting in FIFOs are outside the marked
span. Brief TTS cues replace long spoken task texts during continuous work; the
external TTS service may still queue them. Verify audible feedback on hardware.
Factory single-operator haptics are not integrated. The DAgger web UI vibrates
only on takeover; no feedback is sent to motion topics.

Each `annotations/subtasks/episode_<six-digit-index>.json` contains:

| Field | Meaning |
| --- | --- |
| `task`, `episode_index`, `frames`, `fps` | Overall task and episode identity. |
| `subtasks` | Ordered plan; array positions match subtask IDs. |
| `segments` | Confirmed text/ID and episode-local `[start_frame,end_frame)` spans, also in FPS-based seconds. |
| `unannotated_ranges` | Unconfirmed tail, labelled `-1` in Parquet. |
| `annotation_complete`, `needs_review` | Operator annotation completeness, **not physical task success**. |
| `save_reason`, `boundary_basis` | Final-mark/manual/shutdown save reason and boundary convention. |

For 300 frames, marks at 90, 180, 300 label `[0,90)=0`, `[90,180)=1`,
`[180,300)=2`. Saving after only the first two marks leaves `[180,300)=-1`.
An unfinished plan requires review even with no unlabelled tail. A missed press
can incorrectly merge real actions into a confirmed span; the collector cannot
infer this semantic error. Review all boundaries when cleaning such episodes.

Before LeRobot saves, write a `.pending.json` marker inside the private workspace;
rename it after finalization, and publish it with the episode. Failed-save markers
stay private and disappear only on explicit discard. Legacy pending markers in
the official dataset still block resume for manual inspection. Discard publishes
no sidecar. Annotation-incomplete episodes may be saved; invalid episodes cannot.

Changing the annotation switch requires a new dataset root. Plans may change
between annotated episodes because each sidecar stores its own plan. Standard
LeRobot reading preserves the extra numeric feature but does not automatically
use it as language conditioning. Overall-task training is unchanged; subtask-
conditioned training needs explicit text/span sampling and appropriate action-
chunk boundary handling.

## Extension rules

- Add cameras in `camera_config.py`, then cover them with configuration and SHM tests.
- Add joints in `robot_schema.py`; never duplicate joint order in the recorder or deployer.
- Add signals as timestamped buffers with an explicit causal or interpolation rule.
- Keep direct-SHM validation and FIFO matching. Replacing them with latest-frame reuse changes the data contract.

## Verification

```bash
python -B -m unittest discover -s tests -p 'test_*.py'
bash -n start_lerobot_official_collect.sh
# Optional: ROS transport only, isolated topics + fake recorder, no motion/audio.
ROS_DOMAIN_ID=211 python tests/vr_ros_smoke.py
# Optional subtask gestures; still private topics and a fake recorder.
ROS_DOMAIN_ID=211 python tests/vr_ros_smoke.py --subtasks
```

Core regression tests are development-only and never loaded by the launcher.
Historical validation is in `docs/validation/`; opt-in diagnostics are in `tests/remote/`.
The supported live inference probe is `dagger_pinned_readonly.py`: it uses the
current Collector bridge, discards proposals, and never executes them. Run it
only against an idle Thor session. `dagger_rgbd_recording_smoke.py` checks live
cameras with synthetic joint signals; `dagger_thor_receipts.py` checks the
separate Thor source's receipt/gripper contract. These require their documented
environments and are not part of automatic local discovery.
`tools/patch_*` are hash-pinned offline migrations, not startup dependencies;
they preview by default and back up sources only with explicit `--apply`.
SSE tests require aiohttp in the VR web environment; the rest run without it.
Do not infer robot acceptance or minimum CPU usage from these software tests.

On the robot, also verify ROS discovery, SHM files, camera source FPS, video
encoder availability, CPU load, disk throughput, and `sync_log.jsonl` before
collecting production demonstrations.
