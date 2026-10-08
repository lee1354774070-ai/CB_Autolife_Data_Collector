#!/usr/bin/env python3
"""ROS entrypoint for the optional integration; never imported by keyboard mode."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import sys
import threading


def main():
    component = sys.argv.pop(1)
    root = Path(os.environ["DAGGER_DEPENDENCY_ROOT"])
    sys.path[:0] = [os.environ.get("DAGGER_TOOLS_ROOT", str(Path(__file__).resolve().parents[2])),
                   str(Path(__file__).resolve().parents[1]),
                   os.environ.get("DAGGER_V4_RUNTIME_ROOT", str(root / "openarmx_teleop_vr_306_v4"))]
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    if component == "web":
        from dagger.web_bridge import main as web_main
        web_main()
        return
    rclpy.init()
    if component == "check":
        import time
        from deploy.groot_n1_7.robot_client import GrootRemoteClient
        from deploy.groot_n1_7.auth import read_token
        client = GrootRemoteClient(os.environ["DAGGER_SERVER_URL"],
            read_token(os.environ.get("GROOT_REMOTE_TOKEN"), os.environ["DAGGER_TOKEN_FILE"]), timeout_sec=10)
        try:
            health, _ = client.health()
            if (health.get("mode") not in ("policy_only_baseline", "policy_only_frame")
                    or not health.get("controller_submission_receipts") or health.get("outcome_history_offsets")):
                raise SystemExit("Update our Thor baseline/frame server: controller_submission receipts required")
        finally:
            getattr(client, "close_connections", lambda: None)()
        node = rclpy.create_node("collector_dagger_preflight")
        try:
            from collections import Counter
            from std_msgs.msg import String
            from std_srvs.srv import SetBool
            from rclpy.qos import qos_profile_sensor_data
            from dagger.dependencies import COMMAND_PUBLISHERS, command_conflicts
            counts = Counter()
            for topic in COMMAND_PUBLISHERS:
                node.create_subscription(String, topic,
                    lambda message, topic=topic: counts.update([topic]), qos_profile_sensor_data)
            # Discovery plus a command observation window. Endpoint existence
            # alone is not activity, but even one real command rejects startup.
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            publishers = {topic: [info.node_namespace.rstrip('/') + '/' + info.node_name
                                  for info in node.get_publishers_info_by_topic(topic)]
                          for topic in COMMAND_PUBLISHERS}
            conflicts = command_conflicts(publishers, counts)
            if any(conflicts.values()):
                raise SystemExit(f"Command conflict; not starting a second controller: {conflicts}")
            lease = node.create_client(SetBool, "/control_independent_sync_hold_session_0_300")
            if not lease.wait_for_service(timeout_sec=2):
                raise SystemExit("Guarded hold-only SYNC service is unavailable")
            for topic in ("whole_body", "gripper"):
                name = f"/topic_arm_{topic}_target_joints_position_0_300"
                consumers = node.get_subscriptions_info_by_topic(name)
                if not any(info.node_name.startswith("node_mod_motor_") for info in consumers):
                    raise SystemExit(f"Hardware command subscriber unavailable: {name}")
            print("ROS command preflight passed: known idle vendor endpoints, no commands; "
                  "guarded SYNC service and motor subscribers available (read-only).")
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return
    if component == "supervisor":
        from dagger.supervisor import CollectorDaggerSupervisor
        node = CollectorDaggerSupervisor()
    elif component == "bridge":
        from dagger.thor_bridge import CollectorThorBridge
        node = CollectorThorBridge()
    elif component == "mapper":
        from dagger.mapper import CollectorVrMapper
        node = CollectorVrMapper()
    else:
        raise ValueError(f"unknown DAgger component: {component}")
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        while rclpy.ok() and not stopping.wait(0.1):
            pass
    finally:
        if component == "supervisor":
            # Keep the executor spinning while waiting for the disable RPC.
            node.shutdown_session()
        executor.shutdown(timeout_sec=5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=2)


if __name__ == "__main__":
    main()
