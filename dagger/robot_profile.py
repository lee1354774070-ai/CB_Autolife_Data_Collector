"""Robot identity and model-specific DAgger configuration; no hardware I/O."""
from dataclasses import dataclass
import hashlib
from pathlib import Path
import re
import socket


@dataclass(frozen=True)
class RobotProfile:
    robot_id: str
    domain: str
    model: str

    @property
    def suffix(self):
        return f"{self.domain}_{self.robot_id}"

    @property
    def wire_profile(self):
        return f"autolife_{self.model}"

    @property
    def joint_topic(self):
        return f"/topic_arm_whole_body_and_gripper_current_joints_status_{self.suffix}"

    @property
    def lease_service(self):
        return f"/control_independent_sync_hold_session_{self.suffix}"

    @property
    def lock_file(self):
        return f"/tmp/collector_dagger_{self.suffix}.lock"

    def controller_params(self, runtime, env):
        if self.model == "s2_v2_2":
            return {"urdf_path": str(runtime / "urdf/robot_v2_2_simplified.urdf"),
                    "srdf_path": str(runtime / "urdf/robot_v2_2.srdf")}
        description = Path(env.get("DAGGER_ROBOT_DESCRIPTION_ROOT",
            "/home/ubuntu/miniconda3/envs/robot_env/lib/python3.12/site-packages/"
            "autolife_robot_sdk/descriptions/autolife_s1")).expanduser().resolve()
        return {"urdf_path": str(description / "urdfs/robot_v3_1.urdf"),
                "srdf_path": str(description / "srdfs/robot_v3_1.srdf"),
                # S3 factory standing pose, verified from robot_v3_1.json.
                "quick_reset_left_arm_joints": [20., 0., 0., -110., 0., 0., 0.],
                "quick_reset_right_arm_joints": [20., 0., 0., -110., 0., 0., 0.],
                "quick_reset_waist_joints": [0., 0.],
                "quick_reset_neck_joints": [0., 0., 0.]}

    def validate_description(self, runtime, env):
        params = self.controller_params(runtime, env)
        path = Path(params["urdf_path"])
        expected = {
            "s2_v2_2": "792931382c9e317b833e661c49ab45dbdd64185671b5bb9f00868ee90ef2fecc",
            "s3_v3_1": "f6d4de074adc1afdabc4960b8eda39ce0fe99148dd6c217d253cf1189a236179",
        }[self.model]
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError(f"Unreviewed/missing {self.model} URDF: {path}; inspect calibration before enabling")
        if not Path(params["srdf_path"]).is_file():
            raise ValueError(f"Missing robot SRDF: {params['srdf_path']}")

    def validate_health(self, health):
        if (not health.get("ready") or health.get("policy_type") != "groot"
                or health.get("mode") not in ("policy_only_baseline", "policy_only_frame")
                or not health.get("controller_submission_receipts")
                or health.get("outcome_history_offsets") or health.get("outcome_visual_history_required")
                or health.get("preview_only") or health.get("hardware_execution_allowed") is False):
            raise ValueError("DAgger requires an executable baseline/frame server with controller_submission receipts; preview is inspect/discard only")
        # Older S2 services predate this field. Never apply that fallback to S3.
        actual = health.get("robot_profile", "autolife_s2_v2_2")
        if actual != self.wire_profile:
            raise ValueError(f"Thor robot profile mismatch: expected {self.wire_profile}, got {actual}")


def resolve_robot(env, *, hostname=None):
    match = re.search(r"-(\d+)$", socket.gethostname() if hostname is None else hostname)
    robot_id = env.get("ROBOT_ID", match.group(1) if match else "300")
    domain = env.get("ROS_DOMAIN_ID", "0")
    if not re.fullmatch(r"[1-9][0-9]*", robot_id):
        raise ValueError("ROBOT_ID must be a positive decimal robot number")
    if not re.fullmatch(r"0|[1-9][0-9]*", domain) or int(domain) > 232:
        raise ValueError("ROS_DOMAIN_ID must be an integer in [0, 232]")
    model = env.get("DAGGER_ROBOT_PROFILE")
    if model is None:
        if env.get("ROBOT_TYPE") == "autolife_s3":
            model = "s3_v3_1"
        elif robot_id == "300":
            model = "s2_v2_2"  # Existing 300 commands remain compatible.
        else:
            raise ValueError("Set DAGGER_ROBOT_PROFILE=s2_v2_2 or s3_v3_1; robot number does not identify its model")
    if model not in ("s2_v2_2", "s3_v3_1"):
        raise ValueError(f"Unsupported DAGGER_ROBOT_PROFILE: {model}")
    kind = env.get("ROBOT_TYPE")
    if kind and ((model == "s3_v3_1" and kind != "autolife_s3")
                 or (model == "s2_v2_2" and kind == "autolife_s3")):
        raise ValueError("ROBOT_TYPE disagrees with DAGGER_ROBOT_PROFILE")
    return RobotProfile(robot_id, domain, model)


def recorder_cpus(available, model):
    available = sorted(set(available))
    if not available:
        raise ValueError("Recorder has no available CPUs")
    if model == "s2_v2_2" and set(range(12, 20)).issubset(available):
        return "12-19"  # Retain the measured 300 isolation layout.
    if len(available) < 4:
        raise ValueError("Set HG_DAGGER_RECORDER_CPUS after reviewing this CPU layout")
    # Reserve at least half of the available cores for control and camera I/O.
    return ",".join(map(str, available[-min(4, len(available) // 2):]))
