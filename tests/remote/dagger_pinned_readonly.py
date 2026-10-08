"""Live camera/feedback -> actual pinned DAgger bridge -> discard; NO execution."""
import json
import threading
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from dagger.thor_bridge import CollectorThorBridge
from dagger.control.groot_bridge_core import validated_actions


def main():
    rclpy.init()
    node = CollectorThorBridge()
    received = []
    node.create_subscription(String, '/hg_dagger/policy_action', lambda msg: received.append(msg), 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stop = threading.Event()
    def spin():
        while not stop.is_set():
            executor.spin_once(timeout_sec=.02)
    worker = threading.Thread(target=spin)
    worker.start()
    try:
        node._connect()
        assert node._health['mode'] == 'policy_only_baseline'
        assert node._health['gripper_mode'] == 'phase_aware'
        assert node._health['controller_submission_receipts']
        print('PINNED_DAGGER_HEALTH_PASS baseline=true gripper=phase_aware', flush=True)
        for seq in range(2):
            deadline = time.monotonic() + 8
            while node._q23 is None and time.monotonic() < deadline:
                time.sleep(.02)
            observation, stamp, state = node._observation()
            response = node._next_response(observation)
            proposal = response.get('proposal')
            assert response['decision'] == 'execute' and proposal is not None
            actions = validated_actions(proposal, node._contract)
            assert len(state) == 21 and all(len(row) == 21 for row in actions)
            assert all(10 <= value <= 360 for row in actions for value in row[14:16])
            print('PINNED_DAGGER_PROPOSAL_PASS', json.dumps(dict(trial=seq+1, shape=[len(actions),21],
                latency_ms=round(response['bridge_round_trip_ms'], 1),
                measured_grippers=state[14:16], first_grippers=actions[0][14:16])), flush=True)
            # No _execute call, no forward ACK: discard the whole proposal.
            node._resolve_proposal(cancel=True)
            node._close_session()
            assert not node._session_id and node._proposal is None
            print('PINNED_DAGGER_DISCARD_CLOSE_PASS', seq+1, flush=True)
        assert not received and node._mode == 'DISARMED'
        print('PINNED_DAGGER_NO_MOTION_PASS policy_messages=0 hardware_publishers=0', flush=True)
    finally:
        try:
            if node._proposal is not None:
                node._resolve_proposal(cancel=True)
            node._close_session()
        finally:
            stop.set(); worker.join(timeout=2)
            executor.shutdown(timeout_sec=2)
            node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
