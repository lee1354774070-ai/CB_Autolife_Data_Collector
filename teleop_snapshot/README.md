# Patched V4 Teleoperation Snapshot

This directory publishes the reviewed V4 source COPY used by the Collector
DAgger adapter, with `dagger/v4_controller.patch` already applied. The colleague's
original source is not modified. This is a source snapshot, not a new launch
entry or a replacement for the robot's installed dependencies.

## Version

- Original publication: Autolife_VLA_Tools `2a3afd48` / standalone Collector `17cda47`.
- DAgger fusion update: 2026-10-08, based on publication `4055b75`; runtime source equality is checked by `tests/dagger_ros_smoke.py`.
- Base: the reviewed robot-300 V4 source pinned in `dagger/dependencies.py`.
- Patch SHA-256: `b08cdabb1b16110cfa5659cc5d5b7c22765b16bcb7ea998253c46c84df960ece`.
- Base controller SHA-256: `099568348c4a92b36200cb8052b7f46eea7bea698ae35caecfe24a862cbb993b`.
- Publication date: 2026-10-07.

Caches, runtime manifests containing local paths, an old JS backup and a nested
duplicate web directory are excluded. Source, configuration, URDF, tests and
upstream attribution are retained. Later robot-side experiments are not included.

The fusion update adds the reviewed DAgger reset-gripper authority check to this
snapshot. The existing incremental teleoperation, smoothing, IK and pose profiles
are unchanged. The DAgger launcher overrides gripper range/speed and disables the
legacy reset gesture; use A start, B save, X reset only, Y discard only.

## Integration

The patched controller imports `dagger.controller_handoff` from the surrounding
Collector. The integrated mapper, authority supervisor and WebXR feedback live
in `dagger/mapper.py`, `dagger/supervisor.py` and `dagger/web_bridge.py`; the
original package launch file alone does NOT enable the Collector workflow.

Continue using `COLLECTOR_MODE=dagger` with the documented launcher. It validates
the original pinned dependencies and creates an immutable runtime copy. Do not
point `DAGGER_DEPENDENCY_ROOT` at this already-patched snapshot: it is not an
unpatched source root; applying the same patch again is invalid.

ROS 2, Placo, the robot SDK/services and our Thor client are still required.
HG control code and assets are now bundled in the Collector. Set `DAGGER_TOOLS_ROOT` to the full VLA Tools checkout when
using the standalone Collector repository. Do not run two hardware controllers.
Publication and static tests do not certify physical motion or current robot
deployment. Upstream package READMEs describe the original teleoperation;
Collector README/INSTRUCTION define the integrated controls and launch behavior.

## Validation Limits

The upstream tests are retained as historical source, not a fully passing
acceptance suite. Local full collection encounters missing ROS/test-path
dependencies and a stale `update_reset_stability` import. A focused run returned
164 passes and two failures: the old authority harness omitted the surrounding
`dagger` package, and the gripper test expects an older reset pose than this
reviewed configuration. No reset positions were changed to satisfy old tests.
Use the Collector's `tests/test_dagger_handoff.py` and isolated ROS smoke tests
for the new handoff contract. These results do not validate physical motion.

## Attribution and Licenses

Keep `NOTICE`, `THIRD_PARTY_LICENSES.md` and source headers with this copy.
The package declares CC BY-NC-SA 4.0; individual third-party components retain
their stated licenses. This publication does not relicense them.
