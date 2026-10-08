import numpy as np
import pytest

pytest.importorskip('placo')

from ._subject import load_subject, source_directory


module = load_subject('quest_placo_kinematics')
QuestPlacoKinematics = module.QuestPlacoKinematics
LEFT_ARM_JOINTS = module.LEFT_ARM_JOINTS
RIGHT_ARM_JOINTS = module.RIGHT_ARM_JOINTS
NECK_JOINTS = module.NECK_JOINTS
LEG_WAIST_JOINTS = module.LEG_WAIST_JOINTS


def test_left_only_qp_cannot_move_waist_neck_or_right_arm():
    source = source_directory().parent
    ik = QuestPlacoKinematics(
        source / 'urdf' / 'robot_v2_2_simplified.urdf',
        source / 'urdf' / 'robot_v2_2.srdf',
        collision_check=False,
        dt=0.01,
    )
    groups = {
        'leg_waist': np.zeros(4),
        'left_arm': np.zeros(7),
        'neck': np.zeros(3),
        'right_arm': np.zeros(7),
    }
    seed = ik.q_from_feedback(groups)
    pose = ik.eef_poses(seed)['left']
    target_position = np.asarray(pose['position'], dtype=float).copy()
    target_position[2] += 0.002
    result = ik.solve({
        'left': (target_position, np.asarray(pose['orientation'], dtype=float))
    }, seed)

    assert result.success, result.reason
    for name in LEG_WAIST_JOINTS + NECK_JOINTS + RIGHT_ARM_JOINTS:
        assert result.q[ik.joint_q[name]] == pytest.approx(
            seed[ik.joint_q[name]], abs=1e-12
        )
    assert any(
        abs(result.q[ik.joint_q[name]] - seed[ik.joint_q[name]]) > 1e-9
        for name in LEFT_ARM_JOINTS
    )


@pytest.mark.parametrize(
    'side,elbow_deg,expected',
    [
        ('left', 110.0, 0.05),
        ('left', 50.0, 0.05),
        ('left', 25.0, 0.008),
        ('left', 0.0, 0.008),
        ('right', -110.0, 0.05),
        ('right', -25.0, 0.008),
    ],
)
def test_manipulability_bias_relaxes_smoothly_near_straight_elbow(
        side, elbow_deg, expected):
    source = source_directory().parent
    ik = QuestPlacoKinematics(
        source / 'urdf' / 'robot_v2_2_simplified.urdf',
        source / 'urdf' / 'robot_v2_2.srdf',
        collision_check=False,
        dt=0.01,
    )
    groups = {
        'leg_waist': np.zeros(4),
        'left_arm': np.zeros(7),
        'neck': np.zeros(3),
        'right_arm': np.zeros(7),
    }
    groups[f'{side}_arm'][3] = elbow_deg
    seed = ik.q_from_feedback(groups)
    assert ik.manipulability_weight_for(side, seed) == pytest.approx(expected)


def test_manipulability_bias_has_no_step_inside_fade_envelope():
    source = source_directory().parent
    ik = QuestPlacoKinematics(
        source / 'urdf' / 'robot_v2_2_simplified.urdf',
        source / 'urdf' / 'robot_v2_2.srdf',
        collision_check=False,
        dt=0.01,
    )
    groups = {
        'leg_waist': np.zeros(4),
        'left_arm': np.zeros(7),
        'neck': np.zeros(3),
        'right_arm': np.zeros(7),
    }
    weights = []
    for elbow_deg in (50.0, 45.0, 40.0, 35.0, 30.0, 25.0):
        groups['left_arm'][3] = elbow_deg
        weights.append(ik.manipulability_weight_for(
            'left', ik.q_from_feedback(groups)
        ))
    assert all(a >= b for a, b in zip(weights, weights[1:]))
    assert weights[0] == pytest.approx(0.05)
    assert weights[-1] == pytest.approx(0.008)
