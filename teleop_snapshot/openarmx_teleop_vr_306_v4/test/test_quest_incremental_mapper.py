from types import SimpleNamespace

import numpy as np

from ._subject import load_subject


module = load_subject('quest_incremental')
QuestIncrementalArmMapper = module.QuestIncrementalArmMapper


def _sample(position, quaternion=(0.0, 0.0, 0.0, 1.0)):
    return SimpleNamespace(
        position=np.asarray(position, dtype=float),
        orientation=np.asarray(quaternion, dtype=float),
    )


def _mapper():
    return QuestIncrementalArmMapper(
        side='left',
        position_scale=0.7,
        forward_position_scale=0.9,
        filter_alpha=0.2,
        maximum_position_step=0.001,
        maximum_orientation_step=np.deg2rad(1.0),
        maximum_anchor_displacement=1.0,
        maximum_forward_displacement=1.0,
        wrist_orientation_gain=1.0,
        wrist_reach_reserve_enabled=False,
        workspace_minimum=[-2.0, -2.0, -2.0],
        workspace_maximum=[2.0, 2.0, 2.0],
        left_y_minimum=-2.0,
        right_y_maximum=2.0,
        forbidden_box_minimum=[5.0, 5.0, 5.0],
        forbidden_box_maximum=[6.0, 6.0, 6.0],
    )


def test_adjacent_frame_deltas_accumulate_once_without_v3_step_filter():
    mapper = _mapper()
    initial = _sample([0.0, 0.0, 0.0])
    robot_pose = {
        'position': [0.4, 0.3, 1.0],
        'orientation': [0.0, 0.0, 0.0, 1.0],
    }
    mapper.latch(initial, robot_pose)
    target0, _ = mapper.map(initial)
    target1, _ = mapper.map(_sample([0.1, 0.0, 0.0]))
    target2, _ = mapper.map(_sample([0.2, 0.0, 0.0]))

    np.testing.assert_allclose(target0['position'], robot_pose['position'])
    # WebXR +X maps to robot -Y; two adjacent 0.1 m steps sum to 0.14 m.
    np.testing.assert_allclose(target1['position'], [0.4, 0.23, 1.0])
    np.testing.assert_allclose(target2['position'], [0.4, 0.16, 1.0])


def test_common_hmd_translation_is_removed_from_increment():
    mapper = _mapper()
    initial = _sample([0.0, 0.0, 0.0])
    robot_pose = {
        'position': [0.4, 0.3, 1.0],
        'orientation': [0.0, 0.0, 0.0, 1.0],
    }
    mapper.latch(initial, robot_pose, body_position=[0.0, 0.0, 0.0])
    target, _ = mapper.map(
        _sample([0.1, 0.0, 0.0]), body_position=[0.1, 0.0, 0.0]
    )
    np.testing.assert_allclose(target['position'], robot_pose['position'])


def test_release_discards_persistent_quest_target():
    mapper = _mapper()
    mapper.latch(
        _sample([0.0, 0.0, 0.0]),
        {'position': [0.4, 0.3, 1.0], 'orientation': [0.0, 0.0, 0.0, 1.0]},
    )
    mapper.map(_sample([0.1, 0.0, 0.0]))
    mapper.release()
    assert not mapper.latched
    assert mapper.target_position is None
    assert mapper.previous_controller_position is None
