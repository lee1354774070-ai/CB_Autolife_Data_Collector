"""S3 saved-observation -> Thor proposal -> discard. No ROS or hardware imports."""
from pathlib import Path
import argparse
import base64
import json
import time
import urllib.error
import urllib.request

import cv2
import numpy as np
import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from dagger.control.groot_bridge_core import GrootContract, validated_actions
from s3_thor_preview import BODY, MODE, validate_body

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--dataset", type=Path, required=True)
parser.add_argument("--token-file", type=Path, required=True)
parser.add_argument("--output-dir", type=Path, required=True)
parser.add_argument("--url", default="http://192.168.8.224:8779")
args = parser.parse_args()
BASE, DATA, URL, TOKEN_FILE = args.output_dir, args.dataset, args.url.rstrip("/"), args.token_file
BASE.mkdir(parents=True, exist_ok=False)
cv2.setNumThreads(1)
torch.set_num_threads(1)
torch.set_num_interop_threads(1)
token = TOKEN_FILE.read_text().strip()
report = dict(status="RUNNING", source=str(DATA), input_mapping="Thor S3-v3.1 to S2 mapping, then S2 to S3 physical actions before digest",
              hardware_commands=0, execution_acknowledgements=0, trials=[])


def request(endpoint, payload=None):
    if endpoint not in ("/health", "/start", "/discard", "/close"):
        raise ValueError("This probe never sends execution requests")
    body = None if payload is None else json.dumps(payload, allow_nan=False).encode()
    req = urllib.request.Request(URL + endpoint, data=body,
        headers={"Authorization": "Bearer " + token, "Content-Type": "application/json"})
    started = time.monotonic()
    try:
        with urllib.request.urlopen(req, timeout=60) as response:
            value = json.loads(response.read())
    except urllib.error.HTTPError as error:
        raise RuntimeError(f"{endpoint}: HTTP {error.code}: {error.read(1200).decode()}") from None
    return value, round((time.monotonic() - started) * 1000, 2)


def encode_image(image, expected):
    array = np.asarray(image)
    assert array.shape == tuple(expected) and np.isfinite(array).all()
    rgb = np.ascontiguousarray(array.transpose(1, 2, 0))
    if rgb.dtype != np.uint8:
        rgb = np.clip(rgb * (255.0 if float(rgb.max()) <= 1.0 else 1.0), 0, 255).astype(np.uint8)
    ok, encoded = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                               [cv2.IMWRITE_JPEG_QUALITY, 90])
    assert ok
    return base64.b64encode(encoded).decode("ascii")


try:
    health, health_ms = request("/health")
    assert health["ready"] and health["mode"] == MODE
    assert health["robot_io"] is False and not health["controller_submission_receipts"]
    assert health["preview_only"] is True and health["hardware_execution_allowed"] is False
    assert not health["outcome_history_offsets"]
    contract = GrootContract.from_mapping(health["contract"])
    report["health"] = {k: health[k] for k in
        ("ready", "mode", "model_version", "robot_io", "gripper_mode", "controller_submission_receipts", "preview_only")}
    report["health_ms"] = health_ms
    info = json.loads((DATA / "meta/info.json").read_text())
    assert tuple(info["features"]["observation.state"]["names"]) == contract.state_features
    assert info["robot_type"] == "autolife_s3"
    ds = LeRobotDataset("local/s3_preview", root=DATA, video_backend="pyav")
    assert len(ds) == info["total_frames"] and len(ds) > 0
    report["source_frames"] = len(ds)
    report["source_episodes"] = info["total_episodes"]
    for trial, index in enumerate((0, len(ds) // 2, len(ds) - 1), 1):
        row = ds[index]
        state = np.asarray(row["observation.state"], dtype=np.float32)
        assert state.shape == (21,) and np.isfinite(state).all()
        assert row["task"] in ("Pick the laundry bag.",
            "Place the laundry bag in the upper compartment of the delivery robot.")
        observation = dict(sequence_id=trial, state=state.astype(float).tolist(),
            images={key: encode_image(row[key], shape) for key, shape in contract.image_shapes.items()})
        session = None
        proposal = None
        result = dict(frame_index=index, cleanup_complete=False)
        report["trials"].append(result)
        try:
            response, elapsed = request("/start", {"task": row["task"], **observation})
            session = response.get("session_id")
            proposal = response.get("proposal")
            assert session and proposal and response["decision"] == "execute"
            actions = np.asarray(validated_actions(proposal, contract), dtype=np.float32)
            assert actions.shape == (40, 21)
            validate_body(actions, "returned proposal")
            assert np.all((actions[:,14:16] >= 10) & (actions[:,14:16] <= 360))
            result.update(round_trip_ms=elapsed, server_request_ms=response.get("server_request_ms"),
                s3_body_limits_passed=True,
                max_first_body_step_deg=float(np.max(np.abs(actions[0, BODY]-state[BODY]))),
                shape=list(actions.shape), digest_verified=True,
                measured_state=state.astype(float).tolist(),
                first_action=actions[0].astype(float).tolist(),
                gripper_min=actions[:, 14:16].min(axis=0).astype(float).tolist(),
                gripper_max=actions[:, 14:16].max(axis=0).astype(float).tolist(),
                max_first_step_from_state=float(np.max(np.abs(actions[0]-state))),
                max_adjacent_step=float(np.max(np.abs(np.diff(actions, axis=0)))))
            (BASE / f"readonly-proposal-{trial}.json").write_text(
                json.dumps(response, ensure_ascii=False, indent=2, allow_nan=False))
        finally:
            if proposal is not None and session:
                discarded, _ = request("/discard", {"session_id": session,
                    "proposal_id": proposal["proposal_id"]})
                assert discarded["discarded"] is True
                proposal = None
                result["discarded"] = True
            if session:
                closed, _ = request("/close", {"session_id": session})
                assert closed["closed"] is True
                result["cleanup_complete"] = True
        print("READONLY_PROPOSAL_PASS", json.dumps({k: result[k] for k in
            ("frame_index", "round_trip_ms", "shape", "gripper_min", "gripper_max", "cleanup_complete")}), flush=True)
    report["status"] = "PASS_S3_MAPPING_PREVIEW_ONLY"
except Exception as error:
    report.update(status="FAIL", error=f"{type(error).__name__}: {error}")
    raise
finally:
    (BASE / "readonly-inference.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
