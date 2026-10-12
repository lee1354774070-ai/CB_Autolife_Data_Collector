# 2026-10-12 Storage Protection Validation

## Scope

Based on standalone Collector main `610d345`. Changes cover shared storage,
recorder failure handling, correlated VR/DAgger failure receipts and documentation.
No robot connection, service changes, physical motion or model requests occurred.
Existing joint order, camera keys, takeover authority rules and training masks
were preserved. VLA-repository changes outside the Collector were not included.

## Results

- 220 Python tests passed, with ROS Jazzy, LeRobot 0.6.0 and Python 3.12.
- The opt-in real recorder integration test exercised keyboard, VR, subtask and
  DAgger recording configurations with synthetic frames. Each saved one episode,
  injected an image-write failure, confirmed invalidation and unchanged hashes
  of previously saved data/metadata, discarded, then saved another episode.
- All four used three RGB cameras; VR/DAgger cases included uint16 depth,
  DAgger used the 21-D schema and provenance columns, and subtask used annotations.
  Eight episodes and 28 videos were finalized; every video decoded all three frames.
- A separate real LeRobot save/resume test read frame tensors from both committed
  episodes through `LeRobotDataset`, not only through a video player.
- Fault injection covered full/stale/unavailable storage, duration/byte limits,
  full image job/byte queues, delayed worker errors, a partially written first
  row, publication collisions/failures, journal recovery and post-commit cleanup.
- Isolated DAgger smoke passed: original pending request reconciliation, save/
  discard sequencing, controller snapshot identity (74 files), no hardware calls.
  Slow-storage callback samples: p95 0.649 ms, max 0.737 ms (45 samples); local
  grip callback 0.831 ms. These are one local software run, not robot latency bounds.
- Isolated VR subtask smoke passed A/B/Y/X and fake reset sequencing; no hardware
  calls and no real audio/haptic assertion.
- JavaScript feedback test, shell syntax, single-parameter help and Python syntax
  checks passed. The stored upstream V4 patch retains required blank context
  lines, which Git's generic whitespace check may flag; the patch/runtime identity
  test passed, so those context lines were not stripped.

## Reproduce

From the Collector directory, with ROS and the LeRobot Python available:

```bash
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=211 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST HF_HUB_OFFLINE=1 \
  /path/to/lerobot/bin/python -B -m unittest discover -s tests -p 'test_*.py'
node tests/test_feedback.js
bash -n start_lerobot_official_collect.sh
bash start_lerobot_official_collect.sh MAX_EPISODE_TEMP_GB --help
```

The opt-in recorder integration tests skip outside domain 211 or without ROS.
Production robot tests must use independent data roots and fresh authorization.

## Tradeoffs and Remaining Checks

- One sealed writer per episode means new Parquet/video files, not cross-episode
  append. It requires non-streaming encoding and a batch size of one.
- Metadata is copied at episode preparation; cost grows with dataset metadata.
  Sequential encoding limits save-time CPU peaks but can increase save latency.
- Budget limits are conservative raw-byte estimates, not disk quotas or total
  process RSS limits. A blocked filesystem may still delay worker joins.
- Test the actual robot/NAS filesystem's hard-link, rename, locking and fsync
  behavior, long episodes and competing I/O before production rollout.
- Software-failure recovery does not guarantee survival of failing storage or
  arbitrary power loss. Do not edit/train the dataset concurrently with publication.
- Quarantined interrupted work is retained for inspection and consumes disk until
  an operator removes it after stopping collection.

## Follow-up Cleanup

Starting from VLA `c3fd89c9` / standalone `f67d10f9`, removed the superseded
`dagger_readonly_inference.py` and the one-off `dagger_stack_idle_probe.py` (fixed
robot topics and stale web text). Live inference uses `dagger_pinned_readonly.py`;
the archived full-stack result above remains historical, not a new hardware check.
Removed two helpers with no runtime callers (`read_active_cameras` and
`bracketing_samples`), unused supervisor imports and duplicated timestamp constants.
Thor migration tools share hash checks, syntax validation, preview and backup
handling; their pinned transforms and explicit apply behavior are unchanged.
Local generated Python/test caches were removed, not dataset or runtime workspaces.

All 224 Python tests passed, including preview/no-write, backup and later-source
rejection tests for the shared patch runner. JavaScript feedback, shell syntax,
133 Python files' syntax and isolated DAgger ROS smoke passed again. The smoke
confirmed all 74 teleoperation snapshot files match and made zero hardware calls
or model requests. Compatibility launchers, reviewed snapshots, provenance,
licenses, recovery journals and regression coverage were retained. No robot
deployment or physical validation was performed for this cleanup.
