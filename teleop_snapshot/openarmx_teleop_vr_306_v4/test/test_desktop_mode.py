import numpy as np

from _subject import load_subject


DesktopModeGate = load_subject('desktop_mode').DesktopModeGate
world_to_operator_yaw_rotation = load_subject(
    'desktop_mode'
).world_to_operator_yaw_rotation
VR_TO_ROBOT_ROT = load_subject('teleop_core').VR_TO_ROBOT_ROT


def frame(offset=0.0):
    return {
        'head_position': np.array([0.0 + offset, 1.2, 0.0]),
        'head_rotation_deg': np.array([0.0, 0.0, 0.0]),
        'controllers': {
            'left': np.array([-0.25 + offset, 1.0, -0.3]),
            'right': np.array([0.25 + offset, 1.0, -0.3]),
        },
    }


def observe(gate, values, now):
    return gate.observe(now=now, **values)


def test_mode_requires_a_full_stationary_second():
    gate = DesktopModeGate(settle_seconds=1.0)
    gate.set_enabled(True, 0.0)
    assert observe(gate, frame(), 0.0)[0] is False
    assert observe(gate, frame(0.002), 0.9)[0] is False
    assert observe(gate, frame(0.002), 1.01)[0] is True
    assert gate.status()['ready'] is True


def test_motion_during_settle_restarts_the_timer():
    gate = DesktopModeGate(settle_seconds=1.0)
    gate.set_enabled(True, 0.0)
    observe(gate, frame(), 0.0)
    assert observe(gate, frame(0.05), 0.8)[0] is False
    assert observe(gate, frame(0.05), 1.1)[0] is False
    assert observe(gate, frame(0.05), 1.81)[0] is True


def test_locked_headset_motion_closes_gate():
    gate = DesktopModeGate(settle_seconds=0.1)
    gate.set_enabled(True, 0.0)
    observe(gate, frame(), 0.0)
    assert observe(gate, frame(), 0.11)[0] is True
    moved = frame()
    moved['head_position'] = np.array([0.10, 1.2, 0.0])
    ready, reason = observe(gate, moved, 0.12)
    assert ready is False
    assert 'reference moved' in reason


def test_controller_teleport_closes_gate_but_normal_motion_does_not():
    gate = DesktopModeGate(settle_seconds=0.1)
    gate.set_enabled(True, 0.0)
    observe(gate, frame(), 0.0)
    assert observe(gate, frame(), 0.11)[0] is True
    normal = frame()
    normal['controllers']['left'][0] += 0.10
    assert observe(gate, normal, 0.12)[0] is True
    jumped = frame()
    jumped['controllers']['left'][0] += 0.40
    assert observe(gate, jumped, 0.13)[0] is False


def test_incomplete_tracking_never_opens_gate():
    gate = DesktopModeGate(settle_seconds=0.1)
    gate.set_enabled(True, 0.0)
    values = frame()
    values['controllers'].pop('right')
    ready, reason = observe(gate, values, 1.0)
    assert ready is False
    assert 'incomplete' in reason


def test_operator_yaw_basis_keeps_physical_forward_on_local_forward_axis():
    yaw = 62.0
    operator_forward = np.array([0.0, 0.0, -1.0])
    angle = np.deg2rad(yaw)
    operator_to_world = np.array([
        [np.cos(angle), 0.0, np.sin(angle)],
        [0.0, 1.0, 0.0],
        [-np.sin(angle), 0.0, np.cos(angle)],
    ])
    physical_forward_in_world = operator_to_world @ operator_forward
    recovered = (
        world_to_operator_yaw_rotation(yaw) @ physical_forward_in_world
    )
    assert np.allclose(recovered, operator_forward, atol=1e-9)
    robot_delta = VR_TO_ROBOT_ROT @ recovered
    assert np.allclose(robot_delta, [1.0, 0.0, 0.0], atol=1e-9)
