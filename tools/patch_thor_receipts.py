#!/usr/bin/env python3
"""Add submission receipts to the inspected DAgger Thor server, preserving legacy ACK/grippers.
Default: print a diff. --apply: back up both sources, then write; never restart a service.
"""
import argparse
import ast
import difflib
import hashlib
from pathlib import Path
import shutil
import time

HASHES = {
    "server.py": "d5407c29a0660b6021af63790574923f44d14015416231d738014657d9759e61",
    "policy_only_session.py": "a08ff7f35180574a07e17c4d60e6f3fe45709e41e98849f1f72d697c3652dc1c",
}
RECEIPT_METHOD = '''
    def finish_controller_submission(self, payload: dict) -> int:
        """Bind exact submitted targets; this is not proof of physical execution."""
        if self._live is None:
            raise ProtocolError("No selected proposal is awaiting a receipt")
        envelope, actions = self._live
        if payload.get("receipt_scope") != "controller_submission":
            raise ProtocolError("Explicit controller_submission scope required")
        for field in ("proposal_id", "context_digest", "executable_chunk_digest"):
            if payload.get(field) != getattr(envelope, field):
                raise ProtocolError(f"Controller receipt {field} mismatch")
        prefix = np.asarray(payload.get("submitted_prefix"), dtype=np.float32)
        if (prefix.ndim != 2 or not 0 < len(prefix) <= len(actions)
                or not np.isfinite(prefix).all()
                or not np.array_equal(prefix, actions[:len(prefix)])
                or payload.get("submitted_prefix_digest") != array_digest(prefix)):
            raise ProtocolError("Controller receipt must bind the exact submitted prefix")
        # Preserve the DAgger command latch for accepted targets only. A discarded
        # prediction must never latch a release. Measured completion is not claimed.
        if self._gripper_phase == "place":
            self._gripper_released |= np.any(
                prefix[:, list(self._gripper_indices)] == 10.0, axis=0)
        if self.gripper_guard:
            self._last_gripper_target = prefix[-1, list(self._gripper_indices)].copy()
        self._live = None
        return len(prefix)

'''
ENGINE_METHOD = '''
    def controller_ack(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        """Retire policy-only submitted targets without an execution ACK."""
        with self.lock:
            if self.mode not in ("policy_only_baseline", "policy_only_frame") or self.history_offsets:
                raise ProtocolError("Controller receipts require policy-only baseline/frame mode")
            if self.session is None or payload.get("session_id") != self.session.session_id:
                raise ProtocolError("session_id does not match active session")
            finish = getattr(self.session, "finish_controller_submission", None)
            if finish is None:
                raise ProtocolError("This policy session does not support controller receipts")
            count = finish(dict(payload))
            return {"session_id": self.session.session_id, "submitted_steps": count,
                    "receipt_scope": "controller_submission", "physical_execution_confirmed": False}

'''

def replace_once(source, old, new):
    if source.count(old) != 1:
        raise ValueError("Unexpected source structure; refusing partial adaptation")
    return source.replace(old, new, 1)

def transform(name, source):
    if name == "server.py":
        source = replace_once(source, '            "robot_io": False,',
            '            "controller_submission_receipts": self.mode in ("policy_only_baseline", "policy_only_frame") and not self.history_offsets,\n            "robot_io": False,')
        source = replace_once(source, "    def cancel(self, payload:", ENGINE_METHOD + "    def cancel(self, payload:")
        source = replace_once(source, '                        "ack": engine.acknowledge,',
            '                        "controller_ack": engine.controller_ack,\n                        "ack": engine.acknowledge,')
        source = replace_once(source, '                "ack",', '                "controller_ack",\n                "ack",')
    else:
        source = replace_once(source, "    def cancel_execution(", RECEIPT_METHOD + "    def cancel_execution(")
    ast.parse(source)
    return source

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("repo", type=Path)
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()
    directory = args.repo / "deploy/groot_n1_7"
    updates = []
    for name, digest in HASHES.items():
        path = directory / name
        raw = path.read_bytes()
        if hashlib.sha256(raw).hexdigest() != digest:
            raise SystemExit(f"Unreviewed source: {path}; inspect before applying")
        source = raw.decode()
        updated = transform(name, source)
        updates.append((path, updated))
        print("".join(difflib.unified_diff(source.splitlines(True), updated.splitlines(True),
                                        fromfile=name, tofile=name)), end="")
    if args.apply:
        backup = args.repo / ".codex_backups" / ("dagger_controller_receipts_" + time.strftime("%Y%m%d_%H%M%S"))
        backup.mkdir(parents=True, exist_ok=False)
        for path, _ in updates:
            shutil.copy2(path, backup / path.name)
        for path, updated in updates:
            path.write_text(updated)
        print(f"BACKUP={backup}")
        print("Sources patched. Running service is unchanged until explicitly restarted.")

if __name__ == "__main__":
    main()
