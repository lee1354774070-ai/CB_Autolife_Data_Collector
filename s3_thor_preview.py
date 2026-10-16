#!/usr/bin/env python3
"""S3 v3.1 -> S2 GR00T baseline preview. Proposals are inspect/discard only.

Run with the existing Thor runtime on PYTHONPATH. No ROS or robot I/O is used.
This converts joint conventions, not link lengths or camera viewpoints.
"""
import argparse
from dataclasses import replace
from http.server import ThreadingHTTPServer
from pathlib import Path

import numpy as np
from robot_schema import BASE_GROUPS, HEAD, UPPER_WAIST, RobotSchema

NAMES = RobotSchema(BASE_GROUPS + (HEAD, UPPER_WAIST)).names
MODE = "policy_only_baseline_s3_preview"
# World-axis signs verified against v2.2/v3.1 URDFs; same map is its inverse.
SIGNS = np.array([1, -1, -1, -1, -1, -1, 1,
                  -1, -1, -1, 1, -1, -1, 1, 1, 1, -1, -1, 1, 1, 1], dtype=np.float32)
# v3.1 URDF radians; gripper motor degrees are separate from URDF finger joints.
LIMITS = np.rad2deg(np.array([
    [-2.932, 2.932], [-0.19198621771937624, 3.14], [-2.9147, 2.9147],
    [-2.618, 0], [-2.9496, 2.9496], [-1.3963, 1.3963], [-1.4486, 1.4486],
    [-2.932, 2.932], [-3.1416, 0.19198621771937624], [-2.914, 2.914],
    [-2.618, 0], [-2.9496, 2.9496], [-1.396, 1.396], [-1.4486, 1.4486],
    [0, 0], [0, 0], [-0.349, 0.349], [-0.5236, 0.7854],
    [-1.05, 1.05], [-1.71042, 2.18166], [-3.14, 3.14],
])).astype(np.float32)
LIMITS[14:16] = [10, 360]
BODY = np.array([i for i in range(21) if i not in (14, 15)])


def validate_body(values, label):
    """Reject out-of-range targets; never clip a prediction into a different pose."""
    bad = (values[..., BODY] < LIMITS[BODY, 0]) | (values[..., BODY] > LIMITS[BODY, 1])
    if np.any(bad):
        flat = values.reshape(-1, 21)
        frame, column = np.argwhere(bad.reshape(-1, len(BODY)))[0]
        index = BODY[column]
        raise ValueError(f"{label}: {NAMES[index]} at frame {frame} is {flat[frame, index]:.3f} deg; "
                         f"S3 range is {LIMITS[index].tolist()}")


def model_state(s3_state):
    state = np.asarray(s3_state, dtype=np.float32)
    if state.shape != (21,) or not np.isfinite(state).all():
        raise ValueError("S3 state must be finite 21-D degrees")
    validate_body(state, "observation")
    return state * SIGNS


class S3CandidateGenerator:
    def __init__(self, generator, contract):
        if tuple(contract.state_feature_names) != NAMES or tuple(contract.action_feature_names) != NAMES:
            raise ValueError("S3 preview requires the exact AutoLife 21-D joint order")
        self.generator = generator
        self.chunk_size = contract.chunk_size

    def generate_candidates(self, observation, task, num_candidates):
        if num_candidates != 1 or observation.causal_history_window is not None:
            raise ValueError("S3 preview supports baseline K=1 without causal history only")
        # The outer session keeps the measured S3 state for phase-aware grippers.
        mapped = replace(observation, state=model_state(observation.state))
        batch = self.generator.generate_candidates(mapped, task, num_candidates)
        physical = np.asarray(batch.physical_actions, dtype=np.float32)
        if physical.shape != (1, self.chunk_size, 21) or not np.isfinite(physical).all():
            raise ValueError("invalid postprocessed baseline action shape or values")
        action = physical * SIGNS
        validate_body(action, "prediction")
        # Session phase-gripper processing and both proposal digests happen next.
        return replace(batch, physical_actions=action)


def preview_engine(contract, baseline_factory):
    from deploy.groot_n1_7.protocol import ProtocolError
    from deploy.groot_n1_7.server import GrootRemoteEngine

    def factory(task):
        session = baseline_factory(task)
        if getattr(session, "_gripper_phase", None) is None:
            raise ValueError("S3 preview requires the existing phase-aware pick/place gripper")
        session.candidate_generator = S3CandidateGenerator(session.candidate_generator, contract)
        return session

    factory._wyli_gripper_mode = "phase_aware"

    class PreviewEngine(GrootRemoteEngine):
        def health(self):
            return {**super().health(), "robot_profile": "autolife_s3_v3_1",
                    "preview_only": True, "hardware_execution_allowed": False}

        def refuse_submission(self, payload):
            raise ProtocolError("S3 preview accepts inspect/discard only; hardware execution is not validated")

        acknowledge = refuse_submission
        controller_ack = refuse_submission
        cancel = refuse_submission

    # The production bridge rejects this mode before enabling any controller.
    return PreviewEngine(contract, factory, mode=MODE)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-dir", type=Path, required=True)
    parser.add_argument("--token-file", type=Path, required=True)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8779)
    args = parser.parse_args()
    from deploy.groot_n1_7.auth import read_token
    from deploy.groot_n1_7.policy_only_session import build_policy_only_session_factory
    from deploy.groot_n1_7.server import make_handler

    token = read_token(None, args.token_file)
    loaded, factory = build_policy_only_session_factory(
        str(args.model_dir), device=args.device, num_candidates=1,
        gripper_phase_aware=True)
    engine = preview_engine(loaded.contract, factory)
    with ThreadingHTTPServer((args.host, args.port), make_handler(engine, token)) as server:
        print(f"S3 preview ready: {args.host}:{args.port}; inspect/discard only", flush=True)
        server.serve_forever()


if __name__ == "__main__":
    main()
