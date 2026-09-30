#!/usr/bin/env python3
"""Port the validated MZJ phase gripper to the inspected CB Thor source.
Preview by default; --apply backs up sources. Does not restart any service.
Enable with --gripper-phase-aware; omit that flag for continuous actions.
"""
import argparse
import ast
import difflib
import hashlib
from pathlib import Path
import shutil
import time
from patch_mzj_thor_receipts import replace_once

HASHES = {
 "server.py":"507076710ba4ad21669a6796e50e7152562f763a5644c6415d6043b7c9e22cbf",
 "policy_only_session.py":"7b86bb777e1ea41f14e990aba35357c5d14ff100a88bf8ac8509c59d4fe8c1f0",
}
INITIALIZE = '''
        self._gripper_indices = (14, 15)
        self._gripper_phase = None
        if gripper_phase_aware:
            tasks = {
                "Pick the laundry bag.": "pick",
                "Place the laundry bag in the upper compartment of the delivery robot.": "place",
            }
            if task not in tasks:
                raise ValueError("phase-aware gripper requires an exact supported pick/place instruction")
            for names in (contract.state_feature_names, contract.action_feature_names):
                if tuple(names.index(name) for name in ("left_gripper", "right_gripper")) != (14, 15):
                    raise ValueError("phase-aware gripper requires state/action gripper indices 14,15")
            self._gripper_phase = tasks[task]
        self._gripper_held = np.zeros(2, dtype=bool)
        self._gripper_released = np.zeros(2, dtype=bool)
        self._place_initialized = False
'''
PHASE_METHOD = '''
    def _phase_gripper_chunk(self, action: np.ndarray, state: np.ndarray) -> np.ndarray:
        # Physical actions only, before executable digest. Never close early
        # because a later frame predicts closing.
        indices = list(self._gripper_indices)
        measured = np.asarray(state, dtype=np.float32)[indices]
        executable = action.copy()
        executable[:, indices] = np.clip(executable[:, indices], 10.0, 360.0)
        if self._gripper_phase == "pick":
            self._gripper_held |= measured >= 120.0
            for side, index in enumerate(indices):
                if self._gripper_held[side]:
                    executable[:, index] = 360.0
            return executable
        if not self._place_initialized:
            self._gripper_released = measured < 120.0
            self._place_initialized = True
        released = self._gripper_released.copy()
        for step in range(len(executable)):
            released |= action[step, indices] <= 60.0
            executable[step, indices] = np.where(released, 10.0, 360.0)
        return executable

'''
def transform(name, source):
    if name == "server.py":
        source = replace_once(source, '            "controller_submission_receipts": self.mode in ("policy_only_baseline", "policy_only_frame"),',
            '            "controller_submission_receipts": self.mode in ("policy_only_baseline", "policy_only_frame") and not self.history_offsets,\n'
            '            "gripper_mode": getattr(self.session_factory, "_wyli_gripper_mode", "continuous"),')
        source = replace_once(source, '    parser.add_argument("--num-candidates", type=int, default=1)',
            '    parser.add_argument("--gripper-phase-aware", action="store_true", help="Exact laundry pick/place tasks only; omit for continuous grippers")\n'
            '    parser.add_argument("--num-candidates", type=int, default=1)')
        source = replace_once(source, '            str(args.model_dir),\n            device=args.device,\n            num_candidates=args.num_candidates,\n        )',
            '            str(args.model_dir),\n            device=args.device,\n            num_candidates=args.num_candidates,\n'
            '            gripper_phase_aware=args.gripper_phase_aware,\n        )')
    else:
        source = replace_once(source, '        num_candidates: int = 1,\n    ) -> None:',
            '        num_candidates: int = 1,\n        gripper_phase_aware: bool = False,\n    ) -> None:')
        source = replace_once(source, '        self.num_candidates = 1\n', '        self.num_candidates = 1\n' + INITIALIZE)
        source = replace_once(source, '    def infer(self, observation:', PHASE_METHOD + '    def infer(self, observation:')
        source = replace_once(source, '        action = physical[0].copy()', '        scored_action = physical[0].copy()\n'
            '        action = (self._phase_gripper_chunk(scored_action, observation.state)\n'
            '                  if self._gripper_phase is not None else scored_action)')
        source = replace_once(source, '            scored_chunk_digest=array_digest(action),', '            scored_chunk_digest=array_digest(scored_action),')
        source = replace_once(source, '        self._last_ack_sequence = ack.ack_sequence_id\n',
            '        self._last_ack_sequence = ack.ack_sequence_id\n'
            '        if self._gripper_phase == "place":\n'
            '            self._gripper_released |= np.any(expected[:, list(self._gripper_indices)] == 10.0, axis=0)\n')
        source = replace_once(source, '        self._live = None\n        return len(prefix)',
            '        if self._gripper_phase == "place":\n'
            '            self._gripper_released |= np.any(prefix[:, list(self._gripper_indices)] == 10.0, axis=0)\n'
            '        self._live = None\n        return len(prefix)')
        source = replace_once(source, '    num_candidates: int = 1,\n) -> tuple',
            '    num_candidates: int = 1,\n    gripper_phase_aware: bool = False,\n) -> tuple')
        source = replace_once(source, '            num_candidates=num_candidates,\n',
            '            num_candidates=num_candidates,\n            gripper_phase_aware=gripper_phase_aware,\n')
        source = replace_once(source, '    factory._wyli_history_offsets = ()',
            '    factory._wyli_gripper_mode = "phase_aware" if gripper_phase_aware else "continuous"\n'
            '    factory._wyli_history_offsets = ()')
    ast.parse(source)
    return source

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("repo",type=Path)
    parser.add_argument("--apply",action="store_true")
    args=parser.parse_args()
    updates=[]
    for name,digest in HASHES.items():
        path=args.repo/"deploy/groot_n1_7"/name
        raw=path.read_bytes()
        if hashlib.sha256(raw).hexdigest()!=digest:
            raise SystemExit(f"Unreviewed source: {path}; inspect before applying")
        source=raw.decode(); updated=transform(name,source)
        updates.append((path,updated))
        print("".join(difflib.unified_diff(source.splitlines(True),updated.splitlines(True),
                                         fromfile=name,tofile=name)),end="")
    if args.apply:
        backup=args.repo/".codex_backups"/("mzj_phase_gripper_"+time.strftime("%Y%m%d_%H%M%S"))
        backup.mkdir(parents=True,exist_ok=False)
        for path,_ in updates: shutil.copy2(path,backup/path.name)
        for path,updated in updates: path.write_text(updated)
        print(f"BACKUP={backup}")
        print("Sources patched; service unchanged. Enable explicitly with --gripper-phase-aware.")

if __name__=="__main__":
    main()
