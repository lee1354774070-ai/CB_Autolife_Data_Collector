"""Quest3-style incremental end-effector mapping for robot 306.

The transport, clutch state machine and hardware output remain owned by the
306 V4 stack.  This module replaces only the pose-mapping middle layer: each
new WebXR sample contributes a local delta to a persistent Cartesian target,
matching ``realman_increamental_controller.py`` in Quest3-Teleoperation.
"""

import numpy as np

from .teleop_core import (
    ArmMapper,
    VR_TO_ROBOT_ROT,
    matrix_to_quaternion,
    normalized_quaternion,
    quaternion_to_matrix,
)


class QuestIncrementalArmMapper(ArmMapper):
    """Accumulate adjacent controller-pose deltas into one EEF target.

    V3 reconstructed the complete target from a fixed Grip origin on every
    tick.  Quest3 instead integrates the delta between adjacent controller
    samples.  A new Grip clutch always starts from measured robot FK, so no
    target from an earlier clutch can leak into the next one.

    Workspace projection is deliberately retained as an outer 306 safety
    boundary.  No V3 low-pass or per-tick Cartesian interpolation is applied;
    smoothness is produced by Placo's QP step and the unchanged SYNC position
    output layer.
    """

    def release(self):
        super().release()
        self.previous_controller_position = None
        self.previous_controller_orientation = None
        self.previous_body_position = None
        self.target_position = None
        self.target_orientation = None

    def latch(self, sample, current_pose, body_position=None):
        super().latch(sample, current_pose, body_position=body_position)
        self.previous_controller_position = sample.position.copy()
        self.previous_controller_orientation = normalized_quaternion(
            sample.orientation
        )
        self.previous_body_position = self._body_position(body_position)
        self.target_position = self.robot_origin_position.copy()
        self.target_orientation = self.robot_origin_orientation.copy()

    @staticmethod
    def _finite_delta(value, label):
        value = np.asarray(value, dtype=np.float64).reshape(-1)
        if value.size != 3 or not np.all(np.isfinite(value)):
            raise ValueError(f'{label} must contain three finite values')
        return value

    def map(self, sample, body_position=None):
        if not self.latched or self.target_position is None:
            raise RuntimeError('controller origin has not been latched')

        controller_position = self._finite_delta(
            sample.position, 'controller position'
        )
        controller_orientation = normalized_quaternion(sample.orientation)
        delta_vr = (
            controller_position - self.previous_controller_position
        )

        current_body = self._body_position(body_position)
        if current_body is not None and self.previous_body_position is not None:
            # Keep V3's operator-translation compensation, but apply it to the
            # same adjacent-frame delta used by Quest3.
            delta_vr -= current_body - self.previous_body_position

        self.previous_controller_position = controller_position.copy()
        self.previous_controller_orientation, previous_orientation = (
            controller_orientation.copy(),
            self.previous_controller_orientation,
        )
        if current_body is not None:
            self.previous_body_position = current_body.copy()

        delta_robot = VR_TO_ROBOT_ROT @ delta_vr
        delta_robot *= self.position_scale
        delta_robot[0] *= self.forward_position_scale / self.position_scale

        previous_rotation = quaternion_to_matrix(previous_orientation)
        current_rotation = quaternion_to_matrix(controller_orientation)
        delta_rotation_vr = current_rotation @ previous_rotation.T
        delta_rotation_robot = (
            VR_TO_ROBOT_ROT @ delta_rotation_vr @ VR_TO_ROBOT_ROT.T
        )

        # Quest3 composes each new world-space rotation increment onto the
        # persistent desired EEF orientation.
        self.target_position = self.target_position + delta_robot
        self.target_orientation = matrix_to_quaternion(
            delta_rotation_robot @ quaternion_to_matrix(
                self.target_orientation
            )
        )

        reasons = []
        displacement = self.target_position - self.robot_origin_position
        if self.maximum_forward_displacement > 0.0:
            if displacement[0] > self.maximum_forward_displacement:
                displacement[0] = self.maximum_forward_displacement
                reasons.append('incremental target limited at forward reach')
            self.target_position = self.robot_origin_position + displacement

        distance = float(np.linalg.norm(displacement))
        if (
            self.maximum_anchor_displacement > 0.0
            and distance > self.maximum_anchor_displacement
        ):
            self.target_position = self.robot_origin_position + displacement * (
                self.maximum_anchor_displacement / distance
            )
            reasons.append('incremental target limited at clutch radius')

        self.target_position, constraint_reason = self._constrain_position(
            self.target_position
        )
        if constraint_reason:
            reasons.append(constraint_reason)

        # ArmMapper's boundary helper uses this field to select a stable torso
        # face.  It is not used as a low-pass state in V4.
        self.filtered_position = self.target_position.copy()
        self.filtered_orientation = self.target_orientation.copy()
        return {
            'position': self.target_position.copy(),
            'orientation': self.target_orientation.copy(),
        }, '; '.join(reasons)
