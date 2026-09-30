"""Fail closed when the private V4/HG interfaces differ from the inspected build.

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
    "autolife_hg_dagger_MZJ_300/config/hg_dagger.yaml": "2cca5e4a02d2c5a90c7da8d3580dd2f15d2be04e9394430ca75c54782ab58a43",
    "autolife_hg_dagger_MZJ_300/autolife_hg_dagger_mzj_300/core.py": "948c8dd205f091fa3045aa89074b63c42e636c72c65b74f86bf452d32e84325a",
    "autolife_hg_dagger_MZJ_300/autolife_hg_dagger_mzj_300/supervisor_node.py": "28db83d97742a4f89455699315fcfb147d668d1237d7213322b94c15e802321a",
    "autolife_hg_dagger_MZJ_300/autolife_hg_dagger_mzj_300/recorder.py": "df4c53de9b388c54068ad4fed0b2b93d2f0f60faee4e7a10d8e35648f8ae384d",
    "autolife_hg_dagger_MZJ_300/web/vr_app.js": "b70973956d2d89811b44bfa78a159831a86091de226b00d14ea3345bf147901e",
    "openarmx_teleop_vr_306_v4/openarmx_teleop_vr_306_v4/controller_node.py": "099568348c4a92b36200cb8052b7f46eea7bea698ae35caecfe24a862cbb993b",
    "openarmx_teleop_vr_306_v4/openarmx_teleop_vr_306_v4/vr_mapper_node.py": "ccb5dfc38623b143dc9870ac6f909df3e8b50f475d3c979d785b88c8127fd4ee",
    "openarmx_teleop_vr_306_v4/openarmx_teleop_vr_306_v4/vr_web_bridge.py": "6fffdf186ae9c774b2ecb46e9c361a6a6388fa72052cf823eb75ecabad586f53",
}

# Reviewed MZJ revisions: explicit VR start, detailed notices, and verified
# reset gripper opening. Keep original identities; never accept arbitrary edits.
REVIEWED_ALTERNATES = {
    "autolife_hg_dagger_MZJ_300/autolife_hg_dagger_mzj_300/supervisor_node.py": {
        "6987faee7efe058b208a8eb48947a0d4bbefc1a6450e9a9d83f8e93427f96e4d",
    },
    "autolife_hg_dagger_MZJ_300/web/vr_app.js": {
        "a1721c6b7931bfaeba860ebc000d974a1d833657b3b2ebd5cb0db13ee592214d",
        "39fc7edabd94866d451c7adf24a2cb9b64e8e48eb3128d941d89e9d9f98c3656",
    },
}

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
    for relative, expected in HASHES.items():
        path = root / relative
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
