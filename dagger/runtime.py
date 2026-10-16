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
        from dagger.robot_profile import resolve_robot
        from dagger.control.groot_policy_bridge import RemoteClient
        from dagger.control.groot_bridge_core import GrootContract
        robot = resolve_robot(os.environ)
        token = os.environ.get("GROOT_REMOTE_TOKEN", "").strip()
        if not token:
            token = Path(os.environ["DAGGER_TOKEN_FILE"]).read_text().strip()
        if not token:
            raise SystemExit("Thor token is empty")
        client = RemoteClient(os.environ["DAGGER_SERVER_URL"], token, 10.)
        health = client.request("/health")
        robot.validate_health(health)
        GrootContract.from_mapping(health.get("contract", {}))
        node = rclpy.create_node("collector_dagger_preflight")
        try:
            from collections import Counter
            from std_msgs.msg import String
            from std_srvs.srv import SetBool
            from rclpy.qos import qos_profile_sensor_data
            from dagger.dependencies import command_publishers, command_conflicts
            topics = command_publishers(robot.suffix)
            counts = Counter()
            for topic in topics:
                node.create_subscription(String, topic,
                    lambda message, topic=topic: counts.update([topic]), qos_profile_sensor_data)
            # Discovery plus a command observation window. Endpoint existence
            # alone is not activity, but even one real command rejects startup.
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            publishers = {topic: [info.node_namespace.rstrip('/') + '/' + info.node_name
                                  for info in node.get_publishers_info_by_topic(topic)]
                          for topic in topics}
            conflicts = command_conflicts(publishers, counts, suffix=robot.suffix)
            if any(conflicts.values()):
                raise SystemExit(f"Command conflict; not starting a second controller: {conflicts}")
            lease = node.create_client(SetBool, robot.lease_service)
            if not lease.wait_for_service(timeout_sec=2):
                raise SystemExit(f"Guarded hold-only SYNC service unavailable: {robot.lease_service}. Install the reviewed ARM lease integration; do not bypass or clear/restart ARM automatically.")
            for topic in ("whole_body", "gripper"):
                name = f"/topic_arm_{topic}_target_joints_position_{robot.suffix}"
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
