"""Fail closed when the V4 interfaces and bundled DAgger assets differ from the inspected build.

These are source identities, not a claim of physical validation. There is no
skip-version-check flag: a new robot build must be inspected and tested first.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
import os


HASHES = {
    "openarmx_teleop_vr_306_v4/config/controller.yaml": "5ca8edc6fae5639ebef10ddcb37275906abfe5b80d1dcf9492426aeb6ccb92be",
    "openarmx_teleop_vr_306_v4/config/teleop.yaml": "2c6dd14c14ce656a458f6bea0e84f46114aa571106ec5594fd823394b179629d",
    "openarmx_teleop_vr_306_v4/openarmx_teleop_vr_306_v4/controller_node.py": "099568348c4a92b36200cb8052b7f46eea7bea698ae35caecfe24a862cbb993b",
    "openarmx_teleop_vr_306_v4/openarmx_teleop_vr_306_v4/vr_mapper_node.py": "ccb5dfc38623b143dc9870ac6f909df3e8b50f475d3c979d785b88c8127fd4ee",
    "openarmx_teleop_vr_306_v4/openarmx_teleop_vr_306_v4/vr_web_bridge.py": "6fffdf186ae9c774b2ecb46e9c361a6a6388fa72052cf823eb75ecabad586f53",
}

ASSETS = Path(__file__).with_name("assets")
ASSET_HASHES = {
    "config/hg_dagger.yaml": "2cca5e4a02d2c5a90c7da8d3580dd2f15d2be04e9394430ca75c54782ab58a43",
    "web/index.html": "4471275b1da8565244c0caf9c69ccfb093f3b70b29f5ef7d5c1462cbbc5f21f0",
    "web/styles.css": "f447b2ba2f33c5ee9f781b3063a20fd62f7582947f4b13332b411f8af5fe1582",
    "web/vr_app.js": "39fc7edabd94866d451c7adf24a2cb9b64e8e48eb3128d941d89e9d9f98c3656",
    "web/vr_monitor.html": "0c8392020d4592f1b3c07c0a6d4fb0a7275fcb610bde530cdaa5081902a4e870",
    "web/vr_monitor.js": "79c742d5b5f5720800b8a23cfd9cabf21096c9f9d6be59c7d5580c07f9454f7b"
}
REVIEWED_ALTERNATES = {}

# These vendor nodes register endpoints even when no task is running. Permit
# only their audited idle endpoints, never an active command. In particular,
# target_robot_eef_pose/target_robot_height_z are outgoing vendor telemetry,
# not the move_* command inputs monitored by the V4 controller.
_ARM = "/node_arm_vr_control_service_0_300"
_ACTION = "/node_robot_action_service_0_300"
_VISION = "/vision_service_0_300"
COMMAND_PUBLISHERS = {
    "/topic_arm_whole_body_target_joints_position_0_300": {_ARM, _ACTION, _VISION},
    "/topic_arm_whole_body_target_joints_velocity_0_300": {_ACTION},
    "/topic_arm_gripper_target_joints_position_0_300": {_ACTION},
    "/topic_arm_move_eef_pose_in_robot_frame_0_300": set(),
    "/topic_arm_move_eef_pose_in_vr_frame_0_300": {_ARM},
    "/topic_arm_move_up_down_z_0_300": {_ARM, _ACTION, _VISION},
    "/control_topic_0_300": {_VISION},
    "/control_reset_0_300": set(),
}


def command_conflicts(publishers: dict[str, list[str]], counts: dict[str, int]) -> dict:
    """Fail closed for unknown endpoints or any observed motion/input message.

    This is only startup preflight. The V4 controller must still acquire the
    vendor's guarded SYNC lease, observe its quiet window, and continuously
    arbitrate actual commands while enforcing heartbeats and physical limits.
    """
    unknown = {topic: sorted(set(names) - COMMAND_PUBLISHERS.get(topic, set()))
               for topic, names in publishers.items()}
    return {"unknown_publishers": {t: names for t, names in unknown.items() if names},
            "active_commands": {t: count for t, count in counts.items() if count}}


def interpreter_environment(executable: str, environ: dict[str, str]) -> dict[str, str]:
    """Use the linked Conda interpreter's libraries only in its own subprocess.

    Robot-300's Web/IK virtualenvs link to robot_env Python. Their SSL extension
    requires Conda OpenSSL, whereas system ROS Python must retain system libs.
    This avoids imposing one interpreter's ABI on every node in the stack.
    """
    prefix = Path(executable).resolve().parent.parent
    if not (prefix / "conda-meta").is_dir():
        return {}
    library = str(prefix / "lib")
    inherited = [part for part in environ.get("LD_LIBRARY_PATH", "").split(os.pathsep)
                 if part and part != library]
    return {"LD_LIBRARY_PATH": os.pathsep.join([library, *inherited])}


def validate(root: Path) -> None:
    for directory, manifest in ((ASSETS, ASSET_HASHES), (root, HASHES)):
        for relative, expected in manifest.items():
            path = directory / relative
            allowed = {expected, *REVIEWED_ALTERNATES.get(relative, ())}
            if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() not in allowed:
                raise RuntimeError(f"DAgger dependency changed or missing: {path}; inspect this build before running")


def conflicting_processes(proc: Path = Path("/proc")) -> list[int]:
    """Read command names only; do not print credentials from process arguments."""
    result = []
    signatures = ("hg_dagger_supervisor", "hg_dagger_groot_bridge", "vr_mapper_node",
                  "openarmx_306_v4_mapper", "openarmx_teleop_vr_306_v4.controller_node",
                  "autolife_hg_dagger_mzj_300.vr_web_bridge", "groot_n1_7/robot_deploy.py")
    for path in proc.glob("[0-9]*/cmdline"):
        try:
            args = path.read_bytes().split(b"\0")
            # Command arguments can include shell code. Match whole program
            # basenames/module names, not snippets in an SSH or shell string.
            if any(arg.decode(errors="replace").split("/")[-1] in signatures for arg in args):
                result.append(int(path.parent.name))
        except (OSError, ValueError):
            continue
    return sorted(result)
