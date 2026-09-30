"""Resolve mutually exclusive collector workflows before starting any process.

The legacy VR and annotation settings remain accepted when COLLECTOR_MODE is
unset. An explicit mode must not silently inherit conflicting legacy switches.
DAgger is dispatched to a separate, version-checked incremental teleoperation
adapter; it never falls back to the factory VR helper.
"""

from __future__ import annotations

import argparse

from subtask_annotations import parse_subtasks


MODES = ("keyboard", "vr", "subtask", "dagger")


def resolve_mode(mode: str, vr_control: str, subtasks: tuple[str, ...]) -> tuple[str, bool]:
    if vr_control not in ("", "0", "1"):
        raise ValueError("VR_CONTROL must be 0 or 1")
    if mode and mode not in MODES:
        raise ValueError("COLLECTOR_MODE must be one of: " + ", ".join(MODES))
    if not mode:
        return ("subtask" if subtasks else "vr" if vr_control == "1" else "keyboard"), vr_control == "1"
    expected_vr = mode != "keyboard"
    if vr_control and (vr_control == "1") != expected_vr:
        raise ValueError(f"COLLECTOR_MODE={mode} conflicts with VR_CONTROL={vr_control}; unset VR_CONTROL")
    if mode == "subtask" and not subtasks:
        raise ValueError("COLLECTOR_MODE=subtask requires a nonempty SUBTASKS_JSON plan")
    if mode != "subtask" and subtasks:
        raise ValueError(f"COLLECTOR_MODE={mode} does not use SUBTASKS_JSON; select subtask or unset the plan")
    return mode, expected_vr


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", default="")
    parser.add_argument("--vr-control", default="")
    parser.add_argument("--subtasks-json", default="[]")
    args = parser.parse_args()
    try:
        subtasks = parse_subtasks(args.subtasks_json)
        mode, vr_control = resolve_mode(args.mode, args.vr_control, subtasks)
    except ValueError as exc:
        parser.error(str(exc))
    # Only closed-set tokens and integers reach the launcher's read command.
    print(mode, int(vr_control), len(subtasks))


if __name__ == "__main__":
    main()
