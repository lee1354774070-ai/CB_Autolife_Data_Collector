#!/usr/bin/env python3
"""Opt-in isolated-domain integration test, with NO controller or model process.

Uses the real inspected supervisor/FIFO classes. Hardware enable/reset are
replaced with local spies; candidate publishes are intercepted. The private ROS
domain is mandatory. This tests software sequencing, not robot task execution.
"""

import ast
import json
import numpy as np
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch


def check_controller_copy(root):
    """Execute patched entrypoints, without importing the motor/IK backend."""
    from dagger.runtime_copy import prepare_copy
    with tempfile.TemporaryDirectory(prefix="dagger_controller_copy_") as directory:
        runtime = prepare_copy(root, Path(directory))
        source = runtime / 'openarmx_teleop_vr_306_v4/controller_node.py'
        tree = ast.parse(source.read_text())
        names = {'_on_target', '_on_release_hold', '_on_follow_authority',
                 '_collector_target_allowed', '_follow_authority_allowed', '_on_gripper'}
        methods = [method for cls in tree.body if isinstance(cls, ast.ClassDef)
                   for method in cls.body if isinstance(method, ast.FunctionDef) and method.name in names]
        cls = ast.ClassDef(name='ControllerHooks', bases=[], keywords=[], body=methods, decorator_list=[])
        module = ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[]))
        scope = dict(json=json, time=time, np=np)
        exec(compile(module, str(source), 'exec'), scope)
        node = scope['ControllerHooks']()
        node.__dict__.update(
            _lock=threading.RLock(), _collector_authority_epoch=1,
            _collector_revoked_epoch=-1, _collector_session_id='trial',
            _follow_authority_mode='POLICY_ACTIVE', _follow_authority_required=True,
            _follow_authority_time=time.monotonic(), _latest_target_mailbox=Mock(),
            _feedback=SimpleNamespace(as_dict=lambda: {}, left_gripper=[30.],
                                      right_gripper=[40.], leg_waist=[0., 0., 0., 0.]),
            _feedback_time=time.monotonic(), _reset_active=False, _enable_pending=False,
            _estop_latched=False, _limiter=object(),
            get_parameter=lambda key: SimpleNamespace(value=.5),
            _begin_fresh_teleop_session_locked=Mock(), _hold_sides_locked=Mock(),
            _apply_clutch_envelope_locked=Mock())
        message = lambda value: SimpleNamespace(data=json.dumps(value))
        node._on_follow_authority(message(dict(session_id='trial', authority_epoch=2, mode='EXPERT_ACTIVE')))
        assert node._follow_authority_mode == 'EXPERT_ACTIVE'
        node._begin_fresh_teleop_session_locked.assert_called_once()
        node._latest_target_mailbox.reset.assert_called_once()
        assert not node._collector_target_allowed({'authority_epoch': 1}, policy_only=True)
        assert node._collector_target_allowed({'authority_epoch': 2}, policy_only=False)
        node._on_target(message({'authority_epoch': 1, 'clutch_state': {'left': True, 'right': True}}))
        node._apply_clutch_envelope_locked.assert_not_called()
        node._latest_target_mailbox.put.assert_not_called()
        node._on_release_hold(message({'authority_epoch': 1, 'sides': ['left', 'right']}))
        node._hold_sides_locked.assert_not_called()
        node._on_release_hold(message({'authority_epoch': 2, 'sides': ['left']}))
        node._hold_sides_locked.assert_called_once()
        # Exercise the real native gripper callback with deployed 10..360
        # limits. A fast trigger/phase-aware 360 must not be silently clipped.
        node._state, node._hardware_enabled = 'ARMED', True
        node._gripper_targets = {'left': 10., 'right': 10.}
        node._gripper_dirty = {'left': False, 'right': False}
        node.get_parameter = lambda key: SimpleNamespace(value={
            'gripper_min_position': 10., 'gripper_max_position': 360.,
            'gripper_max_step_per_input': 0., 'feedback_timeout_sec': .5}[key])
        target = dict(authority_epoch=2, left_gripper_target_joints_position=[360.],
                      right_gripper_target_joints_position=[360.])
        node._on_gripper(message(target))
        assert node._gripper_targets == {'left': 360., 'right': 360.}
        node._on_gripper(message({**target, 'left_gripper_target_joints_position': [361.]}))
        assert 'outside [10.0, 360.0]' in node._reason
        assert node._gripper_targets == {'left': 360., 'right': 360.}
        node._feedback_time = time.monotonic()
        reset = dict(session_id='trial', authority_epoch=3, mode='DISARMED', quick_reset={'pending': True})
        node._on_follow_authority(message(reset))
        opening = dict(source='hg_dagger_full_reset', left_gripper_target_joints_position=[10.],
                       right_gripper_target_joints_position=[10.])
        node._on_gripper(message(opening))
        assert node._gripper_targets == {'left': 10., 'right': 10.}
        assert not node._collector_target_allowed(opening, policy_only=True)
        node._on_follow_authority(message({**reset, 'quick_reset': {'pending': False}}))
        assert not node._collector_target_allowed(opening, policy_only=False)
        node._collector_reset_pending = True
        node._follow_authority_time -= 1
        assert not node._collector_target_allowed(opening, policy_only=False)
        print('PATCHED_CONTROLLER_HANDOFF_AND_GRIPPER_PASS range=10..360 reset_gate=true hardware_backend_imported=false')


def main():
    if os.environ.get("ROS_DOMAIN_ID") != "211":
        raise SystemExit("This no-motion test requires isolated ROS_DOMAIN_ID=211")
    root = Path(os.environ["DAGGER_DEPENDENCY_ROOT"])
    sys.path[:0] = [str(Path(__file__).resolve().parents[1]), str(root / "autolife_hg_dagger_MZJ_300"),
                   str(root / "openarmx_teleop_vr_306_v4")]
    from dagger.dependencies import validate
    validate(root)
    check_controller_copy(root)
    import rclpy
    from std_msgs.msg import String
    from dagger.supervisor import CollectorDaggerSupervisor, HgDaggerSupervisor

    commands, enable_calls, reset_calls = [], [], []
    fail_save = threading.Event()
    quit_recorder = threading.Event()
    with tempfile.TemporaryDirectory(prefix="dagger_smoke_") as directory:
        base = Path(directory)
        fifo = base / ".official_recording_control"
        os.mkfifo(fifo)
        fd = os.open(fifo, os.O_RDWR | os.O_NONBLOCK)

        def recorder():
            pending = b""
            while not quit_recorder.wait(.002):
                try:
                    pending += os.read(fd, 4096)
                except BlockingIOError:
                    continue
                while b"\n" in pending:
                    line, pending = pending.split(b"\n", 1)
                    command, request = line.decode().split()
                    commands.append(command)
                    if command == "save":
                        time.sleep(.15)  # Encoding must not block button/control callbacks.
                    temporary = base / "receipt.tmp"
                    temporary.write_text(json.dumps({"request_id": request, "event": command,
                                                     "success": not (command == "save" and fail_save.is_set()),
                                                     "frames": 60, "expert_frames": 25}))
                    temporary.replace(base / ".official_recording_status.json")

        worker = threading.Thread(target=recorder)
        worker.start()
        rclpy.init(args=["--ros-args", "-p", f"trace_root:={base / 'trace'}",
                        "-p", f"rgb_collector_fifo:={fifo}", "-p", f"rgbd_collector_fifo:={fifo}"])
        from dagger.mapper import CollectorVrMapper
        mapper = CollectorVrMapper()
        try:
            mapper._eef_target_pub.publisher = Mock()
            mapper._on_authority(String(data=json.dumps(
                dict(session_id='mapper-test', authority_epoch=2, mode='EXPERT_ACTIVE'))))
            mapper._eef_target_pub.publish(String(data='{"pos_left_in_robot":[0,0,0]}'))
            target = json.loads(mapper._eef_target_pub.publisher.publish.call_args.args[0].data)
            assert target['authority_epoch'] == 2
            assert target['collector_session_id'] == 'mapper-test'
            print('REAL_V4_MAPPER_WRAPPER_PASS hardware_publishers=0')
        finally:
            mapper.destroy_node()
        node = CollectorDaggerSupervisor()
        node._forward_enable = lambda enabled: (enable_calls.append((enabled, list(commands))) or (True, "fake"))
        node._wait_for_controller_ready = lambda since: (True, "fake controller readiness")
        node._selected_joint_pub = Mock()
        node._selected_gripper_pub = Mock()
        reset_patch = patch.object(HgDaggerSupervisor, "_mechanical_reset_worker",
                                  lambda _, button: reset_calls.append((button, list(commands))))
        reset_patch.start()

        def wait_idle():
            deadline = time.monotonic() + 3
            while node._button_worker_active and time.monotonic() < deadline:
                time.sleep(.005)
            assert not node._button_worker_active, "control worker stuck"

        def click(button):
            with node._lock:
                node._on_face_button_click_locked(button, time.monotonic_ns())

        def vr(right=None, **left):
            node._on_vr_input(String(data=json.dumps({"leftController": left,
                "rightController": right if right is not None else {"gripActive": False}})))

        try:
            click("A")
            wait_idle()
            assert commands == ["start"], commands
            assert enable_calls[0] == (True, ["start"]), enable_calls
            assert node._machine.mode.value == "POLICY_ACTIVE"
            # Only controller publication receipts can supply action origins.
            # Untagged vendor echoes and old-session receipts are ignored.
            vendor = dict(left_arm_target_joints_position=[0.] * 7,
                          right_arm_target_joints_position=[0.] * 7,
                          neck_target_joints_position=[0.] * 3,
                          leg_waist_target_joints_position=[0.] * 4)
            node._on_vendor_joint_command(String(data=json.dumps(vendor)))
            assert node._command_origins[False] is None
            stamp = time.time_ns()
            output = dict(command=vendor, gripper=False, timestamp_ns=stamp,
                          origin_authority_epoch=node._machine.authority_epoch, session_id="old")
            node._on_controller_output(String(data=json.dumps(output)))
            assert node._command_origins[False] is None
            output["session_id"] = node._session_id
            node._on_controller_output(String(data=json.dumps(output)))
            assert node._command_origins[False] == (stamp, node._machine.authority_epoch)
            assert node._last_vendor_command_wall_ns == stamp
            vr(gripActive=False)
            assert node._machine.mode.value == "POLICY_ACTIVE"
            good = {"action": [0.] * 14 + [10., 10.] + [0.] * 5, "units": "degrees",
                    "measured_leg_waist": [0.] * 4,
                    "measured_policy_state": [0.] * 14 + [10., 10.] + [0.] * 5,
                    "authority_epoch": node._machine.authority_epoch, "collector_session_id": node._session_id}
            node._on_policy_action(String(data=json.dumps({**good, "collector_session_id": "old"})))
            node._selected_joint_pub.publish.assert_not_called()
            node._on_policy_action(String(data=json.dumps(good)))
            node._selected_joint_pub.publish.assert_called_once()
            node._selected_joint_pub.reset_mock()
            # Tracking error alone is no longer rejected by the adapter. This
            # mocked publisher has no downstream controller or motor interface.
            larger = {**good, "action": [20.] * 14 + [10., 10.] + [20.] * 5}
            node._on_policy_action(String(data=json.dumps(larger)))
            node._selected_joint_pub.publish.assert_called_once()
            assert node._machine.mode.value == "POLICY_ACTIVE"
            node._selected_joint_pub.reset_mock()
            node._selected_release_pub = Mock()
            node._selected_eef_pub = Mock()
            started = time.monotonic()
            vr(gripActive=True)
            takeover_ms = (time.monotonic() - started) * 1000
            assert node._machine.mode.value == "EXPERT_ACTIVE"
            assert takeover_ms < 50, takeover_ms
            node._selected_release_pub.publish.assert_called_once()
            assert node._hold_to_intervene
            node._on_policy_action(String(data=json.dumps(good)))
            node._selected_joint_pub.publish.assert_not_called()
            node._on_controller_status(String(data=json.dumps({"state": "ARMED", "hardware_ready": True})))
            assert node._machine.mode.value == "EXPERT_ACTIVE"
            node._on_controller_status(String(data=json.dumps({"state": "HOLDING", "hardware_ready": True})))
            assert node._machine.mode.value == "EXPERT_ACTIVE", "Hold ACK changed immediate authority"
            epoch = node._machine.authority_epoch
            expert = {"pos_left_in_robot": [0., 0., 0.],
                      "collector_session_id": node._session_id, "authority_epoch": epoch}
            node._on_expert_eef(String(data=json.dumps({**expert, "authority_epoch": epoch - 1})))
            node._selected_eef_pub.publish.assert_not_called()
            node._on_expert_eef(String(data=json.dumps(expert)))
            node._selected_eef_pub.publish.assert_called_once()
            vr(gripActive=True)
            assert node._machine.authority_epoch == epoch, "Held Grip retriggered takeover"
            vr(gripActive=False, yButton=False)
            assert node._machine.mode.value == "EXPERT_ACTIVE"
            click("B")
            time.sleep(.03)
            started = time.monotonic()
            vr(gripActive=False)
            callback_ms = (time.monotonic() - started) * 1000
            assert callback_ms < 50, callback_ms
            wait_idle()
            assert commands == ["start", "save"], commands
            assert not reset_calls, "B unexpectedly moved reset"
            click("A")
            wait_idle()
            vr(gripActive=False, right={"gripActive": True})
            assert node._machine.mode.value == "EXPERT_ACTIVE", "Right Grip did not immediately take over"
            node._on_controller_status(String(data=json.dumps({"state": "HOLDING", "hardware_ready": True})))
            assert node._machine.mode.value == "EXPERT_ACTIVE"
            vr(gripActive=False)
            before = list(commands)
            click("X")
            wait_idle()
            assert commands == before, "X discarded a live episode"
            assert not reset_calls, "X reset during recording"
            click("Y")
            wait_idle()
            assert not reset_calls, "Y unexpectedly reset"
            click("X")
            wait_idle()
            assert commands == ["start", "save", "start", "discard"], commands
            assert reset_calls == [("X", commands)], reset_calls
            assert node._machine.mode.value == "DISARMED"
            # An uncertain save is not replayed and X cannot discard or reset
            # it. Only a receipt for the original request releases the latch.
            click("A")
            wait_idle()
            fail_save.set()
            click("B")
            wait_idle()
            pending = node._pending_collector_result
            assert pending is not None
            sent = list(commands)
            count = len(reset_calls)
            click("B")
            wait_idle()
            click("X")
            wait_idle()
            assert commands == sent, "uncertain command replayed"
            assert len(reset_calls) == count, "reset happened before save confirmation"
            receipt = base / ".official_recording_status.json"
            receipt.write_text(json.dumps({"request_id": pending.request_id, "event": "save",
                                           "success": True, "frames": 60, "expert_frames": 25}))
            click("X")
            wait_idle()
            assert len(reset_calls) == count, "X reconciled storage implicitly"
            click("B")
            wait_idle()
            click("X")
            wait_idle()
            assert commands == sent
            assert len(reset_calls) == count + 1
            # B saves on press, regardless of duration; hold/release cannot
            # send another command or turn a save into a discard.
            fail_save.clear()
            click("A")
            wait_idle()
            sent, count = list(commands), len(reset_calls)
            now = time.monotonic_ns()
            with node._lock:
                node._update_face_button_locked("B", True, now)
                node._update_face_button_locked("B", True, now + 1_200_000_000)
            wait_idle()
            assert commands == sent + ["save"]
            with node._lock:
                node._update_face_button_locked("B", False, now + 1_200_000_001)
            wait_idle()
            assert commands == sent + ["save"]
            assert len(reset_calls) == count
            click("A")
            wait_idle()
            sent = list(commands)
            vr(gripActive=False, yButton=True)
            wait_idle()
            vr(gripActive=False, yButton=True)
            vr(gripActive=False, yButton=False)
            wait_idle()
            assert commands == sent + ["discard"]
            assert node._machine.mode.value == "DISARMED"
            assert len(reset_calls) == count
            # Repeated status publication during a blocked filesystem read is
            # still independent of NAS latency, as are 90 Hz VR callbacks.
            blocked, release = threading.Event(), threading.Event()
            raw_progress = node._collector.collector.progress
            def slow_progress(*args):
                blocked.set()
                release.wait(2)
                return {}
            node._collector.collector.progress = slow_progress
            assert blocked.wait(1)
            timings = []
            try:
                for _ in range(45):
                    before = time.monotonic()
                    vr(gripActive=False)
                    node._publish_state()
                    timings.append((time.monotonic() - before) * 1000)
                    time.sleep(1 / 90)
            finally:
                release.set()
                node._collector.collector.progress = raw_progress
            timings.sort()
            assert timings[-1] < 50, timings[-1]
            print(f"SLOW_STORAGE_CALLBACK_MS p50={timings[22]:.3f} p95={timings[42]:.3f} "
                  f"max={timings[-1]:.3f} samples={len(timings)}")
            print(f"DAGGER_ROS_SMOKE_PASS commands={commands}; grip_to_expert_callback_ms={takeover_ms:.3f}; callback_during_save_ms={callback_ms:.3f}; "
                  "hardware_calls=0; actual_model_requests=0")
        finally:
            node.shutdown_session()
            reset_patch.stop()
            node.destroy_node()
            rclpy.shutdown()
            quit_recorder.set()
            worker.join()
            os.close(fd)


if __name__ == "__main__":
    main()
