# Collector Design

For commands and environment parameters see [README](README.md).
[中文实现说明](INSTRUCTION_zh.md). Dated hardware evidence belongs in `docs/validation/`,
not in the description of current behavior.

## Modules

| Module | Responsibility |
|---|---|
| start_lerobot_official_collect.sh | Mode dispatch, session lock, environment, owned child processes and keyboard |
| collector_modes.py | Side-effect-free mode and legacy-flag validation |
| recorder_config.py / launcher_help.py / cli_help.py | Standard-library argument validation and full/single-parameter help |
| record_lerobot_official.py | Single-threaded ROS sampling, image FIFOs, alignment and episode lifecycle |
| robot_schema.py / eef_schema.py | Joint order, physical mapping, pose validation/interpolation and layout manifest |
| camera_config.py / shm_camera.py / time_sync.py | Camera definitions, consistent SHM snapshots, decoding and time matching |
| episode_storage.py | Storage budget, private workspace, commit journal, rollback and writer lock |
| dagger/capture_image_writer.py | Bounded asynchronous image writer shared by ALL modes |
| collector_control.py | Nonblocking FIFO, correlated receipts, readiness and reports |
| vr_collector_control.py / vr_reset.py | Gestures, countdown, asynchronous confirmation, TTS and guarded compatible-V4 reset |
| subtask_annotations.py | Ordered boundaries, frame labels and annotation sidecars |
| dagger/ / dagger_labels.py | Owned/attach host, authority, Thor bridge, provenance and training masks |
| convert_dataset.py | Independent state/action projection into a new v3 dataset |
| hand_camera_producer.py / shm_camera_topic_bridge.py | Optional fallback JPEG producer and SHM-to-ROS bridge |
| tests/ | Hardware-free regression, isolated ROS and explicitly invoked robot checks |

`dagger/control/` is the inspected control base; `dagger/assets/` contains pinned Web/config resources.
`teleop_snapshot/` is an already-patched review snapshot, not an input for patching again.
SOURCE.json and dependency hashes preserve provenance and must not be deleted as temporary files.
Python caches and .runtime are generated, untracked artifacts; do not remove a live runtime.

## Startup and Environment

Keyboard, VR and subtask modes retain factory teleoperation. DAgger uses an exclusive incremental copy.
The launcher validates modes, switches and task-directory names before startup. A session lock prevents
two launchers replacing each other's FIFO/PID files. A separate dataset writer lock protects data, and
a command-client lock covers the complete request/receipt exchange. These locks are not interchangeable.

The recorder receives complete joint state before the optional camera producer starts. Existing SHM
may be probed earlier, but recording stays paused. Current-process camera initialization, not old
metadata, enables controls. No automatic hardware enable, reset or service restart occurs in ordinary collection.

S2/S3 share main. ROBOT_ID/ROS_DOMAIN_ID route topics; ROBOT_TYPE labels new datasets only.
Existing compatible venv/Conda environments can be reused. DAgger separately requires the inspected
model profile, description, authority interfaces and policy. A robot number is not a model identifier.
DDS precedence is COLLECT_CYCLONEDDS_URI, then CYCLONEDDS_URI, then the local-loopback default.

## Data Contract

Canonical policy order: left arm 7, right arm 7, left/right grippers, optional head 3,
optional upper waist 2 OR full waist/legs 4. Base=16, head+upper-waist=21, full=23.
Joint state/action use the same order; legacy names remain only when physical layout is compatible.
Physical q23 order is waist/legs 4, left arm 7, right arm 7, grippers 2, head 3; robot_schema owns conversion.

| Action source | Meaning |
|---|---|
| status_target | Default for ordinary modes: body targets from status; measured grippers because status has no independent gripper target |
| joint | Required by DAgger: verified controller body/gripper target publication receipts; otherwise only for verified command integrations |

Factory VR movement does not prove the default joint-command topics carry messages.
DAgger forbids state-as-action fallback. The retired 15-D eef action mode is rejected, never reinterpreted.
RGB keys are observation.images.rgbd_head_color/hand_left/hand_right, with optional rgbd_head_depth.
Colors become RGB HWC. Depth enters LeRobot's native depth encoder as uint16 millimetres, with configured quantization.

WITH_EEF_STATE/ACTION independently append 14 values: left XYZ+XYZW, then right XYZ+XYZW.
The joint prefix is unchanged. Inputs are vendor measured/target reports in robot-frame metres and
unit quaternions, with no extra TCP transform or FK. Reject incomplete, nonfinite or zero-quaternion poses.
Measured XYZ is linearly interpolated and orientation uses shortest-arc SLERP; target poses are held causally.
Missing targets never become measured values or zero. The 14 pose values do not include grippers.

meta/collector_schema.json records names, indices, units and sources. Resume verifies FPS/features,
depth, joint/EEF layout, subtask and DAgger schema. Matching vector widths do not establish semantic
calibration, model compatibility, or permission to mix S2/S3 data.

## Image and Time Path

The factory vision process writes SHM; the collector reads it. The optional fallback hand producer
is only for unowned devices. Hands exclusively read hand_left_jpeg/hand_right_jpeg, decode JPEG to
BGR, then convert to RGB. There is no fallback to unreliable direct-BGR hand streams.

The reader recognizes legacy `<qiiiii` and SHM2. SHM2 uses metadata's active published slot, not an
SDK sequential API or a traversal of every historical slot. Metadata must match before/after copying;
a raced snapshot is rejected until a later poll. This does not prove every producer frame was read.
Reference-interval checks enforce the configured continuity tolerance.
Each poll batches metadata reads and image copies before decoding; duplicate timestamps never re-enter FIFOs.

Plausible epoch times are normalized; unknown camera clocks or untrusted telemetry stamps fall back
to receive time. Receive time includes polling/scheduling delay. Software delta is not exposure synchronization.

1. Select the oldest reference frame newer than the previous anchor after its matching wait.
2. Select nearest peer images within sync tolerance and freshness limits.
3. Interpolate bracketed joint/EEF state, checking endpoint ages and span.
4. Hold a fresh action at or before the image time, never a future target.
5. Admit a complete row to the private episode, then consume matched images.

The wait is max(sync tolerance, state interpolation span), normally 50 ms; it is not a 30 ms compute deadline.
At 30 FPS, the default 45% interval tolerance accepts about 18.3-48.3 ms.
Changing dataset FPS does not automatically downsample the camera. Default image FIFO=16 per camera,
signal history=64 per stream. Start clears reference warmup and retains at most one peer boundary neighbor;
normal freshness checks apply, and no pre-start reference row is written.

Waiting for a future state endpoint/reference increments waiting ticks. Stalled/stale/missing streams,
noncausal actions, failed alignment, bad reference intervals or active FIFO overflow invalidate the WHOLE
episode. Rolling warmup buffers while paused is not dropping accepted episode frames.

## Storage and Fault Isolation

All modes use a bounded image writer: 64 unfinished jobs / 256 MiB raw inputs by default.
Overflow fails immediately rather than blocking or silently dropping. DAgger uses uncompressed PNG,
ordinary capture light PNG compression, depth TIFF; background errors reach the recorder.
The queue budget excludes camera FIFOs, encoder memory and other process allocations.

Disk space refreshes in a background thread every 0.5 s; frame callbacks read a cached snapshot and
account for admitted bytes. A snapshot older than 2 s fails closed. Defaults reserve 2 GiB plus one raw
episode estimate for encoding; per-episode limits are 8 GiB and 600 s, whichever is reached first.
These are conservative estimates, not filesystem quotas or a promise to record 600 s at any resolution.

Only .collector-work is writable during an episode. It copies metadata, not old Parquet/video shards.
Save joins writers, encodes, closes writers, completes annotations and flushes sync diagnostics before commit:
- fsync new files/directories and create the commit journal;
- publish new files via no-overwrite hardlinks and install new metadata with a backup;
- mark committed, then clean up; cleanup failure never reverses a confirmed commit.

Pre-commit failure restores old metadata and removes only new links with matching workspace inodes.
An uncertain result stops the session without retry/discard; restart recovers the journal.
Uncommitted leftovers are preserved as collector-interrupted directories, not silently deleted.
The filesystem must support same-filesystem hardlinks, rename, flock and fsync.
Concurrent training/editing during metadata publication is unsupported. This is not a guarantee against
arbitrary power loss or physical disk failure.

Invalidation is latched in memory even if the first row was only partially written. D/Y discards only
the current workspace. Sync logs remain buffered to avoid per-frame flushes; start and pre-commit flush
detect late errors. A failed log prevents further starts until restart, rather than silently collecting
without diagnostics. Diagnostic failure after publication never rolls back already-saved data.

Normal Q/EOF/interruption stops owned VR input, then waits for recorder finalization without a fixed
30-second encoder kill. DAgger has its own outer safety shutdown and does not auto-save unconfirmed data.
Factory services and unrelated processes are not stopped. Starting another episode is always explicit.

## Commands and Subtasks

FIFO messages must be single atomic writes. Ordinary files, symlinks, multiline injection and payloads
larger than PIPE_BUF are rejected. Receipts require the matching request_id; busy clients reject new
commands, and timeout means unknown, never automatic retry. JSON receipts are atomically replaced.
If receipt/event storage also fails, in-memory invalidation still prevents continued capture.

Factory VR requires both Grips and a single face-button release: A=start, B=save, Y=discard.
Factory X reset is NOT integrated. Malformed/stale input, multiple buttons or released Grips cancel
the gesture. Countdown is nonblocking and requires fresh input. A worker waits for recorder receipts;
ROS input continues, and receipt-cache disk reads stay outside callbacks.
TTS uses the factory service; no factory vibration protocol or in-App progress overlay is integrated.

Subtask short B marks; long B release saves early; the last mark saves automatically.
Boundaries are exclusive accepted-frame counts. Intermediate marks only append a boundary and enqueue
a small receipt; no image scan, encoding, pause or FIFO reset occurs. Labels and sidecars materialize on save.
Unconfirmed tails have subtask_index=-1 and needs_review=true. task_index is always the overall task;
subtask_index is local to the episode's plan. Stock LeRobot does not automatically turn it into subtask language.

## DAgger

Owned starts one compatible host; attach reads fresh host state and calls its operation interfaces,
without a second controller. Host/session, robot, domain and FIFO are bound; requests have IDs and
expiry, and unknown operations are never replayed. Attach exit only detaches. Old unmodified hosts
need a one-time migration; the hash-checked runtime copy never modifies the colleague's originals.

Preflight checks actual move/joint/gripper inputs and exclusive services, not target EEF output reports.
An enable ACK is not hardware readiness: require fresh ARMED, enabled, ready, nonpending state.
A waits for the correlated recorder start receipt; a fresh action/provenance barrier may wait up to
30 s BEFORE the first row. It does not relax synchronization inside a recorded episode.

A new Grip press switches authority epoch immediately and revokes local model output without waiting for HTTP.
The sole V4 publisher clears old targets/in-flight IK under its lock and reanchors the incremental mapper.
Late model, human and release messages with an old session/epoch are rejected. Takeover never auto-returns control.
V4 mechanical/collision/trajectory/watchdog protections remain; following error is not an adjacent-action jump.

Only V4 publishes hardware targets and emits controller_output receipts with original time/provenance.
Proxy refreshes cannot relabel held model targets as new expert actions. B revokes then saves; Y discards;
X resets only with no pending data/busy operation. An uncertain save retains its request ID for reconciliation.

| Field | Meaning |
|---|---|
| dagger.control_source | 0=model, 1=active human, 2=hold/handoff |
| dagger.is_intervention / intervention_id | Intervention occurrence and segment number, not loss masks |
| dagger.authority_epoch | Authority generation |
| dagger.train_mask | Frame-level AND: body and grippers must all qualify as expert |
| dagger.action_train_mask | Action-width/order mask; body and grippers verified separately; appended EEF entries always zero |
| dagger.anchor_timestamp_ns | Image anchor |
| dagger.arm_command_timestamp_ns / gripper_command_timestamp_ns | Original command times, never proxy refresh times |

Training must apply masks at every chunk time/dimension. Partial corrections must not be filtered first
by the legacy scalar AND mask. Model-only episodes can be saved but have zero expert mask.
Saving or submitting targets is not evidence of hardware completion or task success.

Thor GR00T baseline/frame must advertise controller_submission_receipts. controller_ack verifies
submitted target prefixes/digests, not motor execution, and does not advance verifier history.
Clients do not rewrite actions. The phase_aware gripper policy is task-specific:
`Pick the laundry bag.` and `Place the laundry bag in the upper compartment of the delivery robot.`.
It latches after measured 120 degrees during pickup and releases on predicted <=60 degrees for placement.
Other tasks require independent policy validation.

Recorder status reads run at 20 Hz off the control path; >1 s stale revokes output. DDS graph cache
updates at 5 Hz; >600 ms stale rejects output. Trace uses a bounded 2048 queue with separate loss/error
accounting; training labels do not depend on diagnostic traces. Only takeover triggers WebXR vibration,
subject to device support. Model chunks remain serial: no RTC or zero-stall guarantee is implied.

## Offline Projection and Verification

Conversion takes the source dataset shared lock, rejects unfinished transactions/symlinks/existing or nested
destinations, and rewrites bounded Parquet batches, Arrow/HF schemas and global/episode statistics.
Count is not sliced; action masks use the same projection. Tasks, annotations, order and video offsets stay
unchanged; video bytes are copied independently. fsync precedes Linux RENAME_NOREPLACE publication.
Only the converter's private failed output is removed. This is selection, not IK/FK or expert relabeling.

```bash
PYTHONPATH=. /path/to/lerobot/python -B -m unittest discover -s tests -p 'test_*.py'
node tests/test_feedback.js
bash -n start_lerobot_official_collect.sh
```

ROS tests belong in isolated domain 211. Real recordings use a dedicated NOT TRAINING DATA directory.
Software regression, real-sensor stationary capture, and physical VR/DAgger motion are distinct evidence.
Schema, pinned assets, storage transactions and authority changes need corresponding regression coverage.
Never remove safety tests or loosen synchronization/resource thresholds merely to obtain a passing run.
