import time
from dataclasses import dataclass, field

import numpy as np
import pinocchio as pin


LEG_WAIST_JOINTS = [
    'Joint_Ankle', 'Joint_Knee', 'Joint_Waist_Pitch', 'Joint_Waist_Yaw'
]
LEFT_ARM_JOINTS = [
    'Joint_Left_Shoulder_Inner', 'Joint_Left_Shoulder_Outer',
    'Joint_Left_UpperArm', 'Joint_Left_Elbow', 'Joint_Left_Forearm',
    'Joint_Left_Wrist_Upper', 'Joint_Left_Wrist_Lower',
]
RIGHT_ARM_JOINTS = [
    'Joint_Right_Shoulder_Inner', 'Joint_Right_Shoulder_Outer',
    'Joint_Right_UpperArm', 'Joint_Right_Elbow', 'Joint_Right_Forearm',
    'Joint_Right_Wrist_Upper', 'Joint_Right_Wrist_Lower',
]
NECK_JOINTS = ['Joint_Neck_Roll', 'Joint_Neck_Pitch', 'Joint_Neck_Yaw']
EEF_FRAMES = {
    'left': 'Link_Left_Wrist_Lower_to_Gripper',
    'right': 'Link_Right_Wrist_Lower_to_Gripper',
}


@dataclass
class IkResult:
    success: bool
    q: np.ndarray
    iterations: int
    solve_time_ms: float
    position_error_m: dict = field(default_factory=dict)
    orientation_error_rad: dict = field(default_factory=dict)
    minimum_singular_value: float = 0.0
    collision_pairs: list = field(default_factory=list)
    reason: str = ''


def quaternion_xyzw_to_rotation(quaternion):
    x, y, z, w = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm([x, y, z, w])
    if norm < 1e-9:
        raise ValueError('zero quaternion')
    x, y, z, w = np.asarray([x, y, z, w]) / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class IndependentArmKinematics:
    def __init__(
        self,
        urdf_path,
        srdf_path='',
        allow_waist=False,
        collision_check=True,
        limit_margin_rad=np.deg2rad(2.0),
    ):
        self.urdf_path = str(urdf_path)
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        self.allow_waist = bool(allow_waist)
        self.limit_margin_rad = float(limit_margin_rad)
        self.joint_q = {}
        self.joint_v = {}
        for name in LEG_WAIST_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + NECK_JOINTS:
            joint_id = self.model.getJointId(name)
            if joint_id == 0:
                raise ValueError(f'URDF is missing {name}')
            joint = self.model.joints[joint_id]
            if joint.nq != 1 or joint.nv != 1:
                raise ValueError(f'{name} is not a one-DoF joint')
            self.joint_q[name] = joint.idx_q
            self.joint_v[name] = joint.idx_v
        self.frame_ids = {}
        for side, frame in EEF_FRAMES.items():
            if not self.model.existFrame(frame):
                raise ValueError(f'URDF is missing frame {frame}')
            self.frame_ids[side] = self.model.getFrameId(frame)

        self.lower = self.model.lowerPositionLimit.copy()
        self.upper = self.model.upperPositionLimit.copy()
        finite = np.isfinite(self.lower) & np.isfinite(self.upper)
        self.lower[finite] += self.limit_margin_rad
        self.upper[finite] -= self.limit_margin_rad
        if np.any(self.lower[finite] >= self.upper[finite]):
            raise ValueError('joint limit margin is too large')

        self.geom_model = None
        self.geom_data = None
        self.collision_available = False
        self.collision_error = ''
        if collision_check:
            try:
                self.geom_model = pin.buildGeomFromUrdf(
                    self.model,
                    self.urdf_path,
                    pin.GeometryType.COLLISION,
                    package_dirs=str(__import__('os').path.dirname(self.urdf_path)),
                )
                self.geom_model.addAllCollisionPairs()
                if srdf_path:
                    pin.removeCollisionPairs(self.model, self.geom_model, str(srdf_path))
                self.geom_data = pin.GeometryData(self.geom_model)
                self.collision_available = True
            except Exception as exc:
                self.collision_error = str(exc)

    def _configuration(self, q):
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.size != self.model.nq:
            raise ValueError(
                f'configuration size mismatch: expected {self.model.nq}, got {q.size}'
            )
        if not np.all(np.isfinite(q)):
            raise ValueError('configuration contains NaN or Inf')
        return q

    def q_from_feedback(self, groups_deg, *, clip=True):
        """Build a Pinocchio configuration from degree-valued joint groups.

        IK seeds and command validation normally use the soft-limit-clipped
        configuration.  Measured-pose FK and enable preflight checks pass
        ``clip=False`` so an out-of-limit measurement is not silently moved
        before it is inspected.
        """
        q = pin.neutral(self.model)
        assignments = [
            (LEG_WAIST_JOINTS, groups_deg['leg_waist']),
            (LEFT_ARM_JOINTS, groups_deg['left_arm']),
            (RIGHT_ARM_JOINTS, groups_deg['right_arm']),
            (NECK_JOINTS, groups_deg['neck']),
        ]
        for names, values_deg in assignments:
            if len(values_deg) != len(names):
                raise ValueError(f'joint count mismatch for {names[0]}')
            for name, value_deg in zip(names, values_deg):
                value_deg = float(value_deg)
                if not np.isfinite(value_deg):
                    raise ValueError(f'{name} contains NaN or Inf')
                q[self.joint_q[name]] = np.deg2rad(value_deg)
        q = self._configuration(q)
        return np.clip(q, self.lower, self.upper) if clip else q

    def groups_from_q_deg(self, q, base_groups_deg):
        output = {key: np.asarray(value, dtype=float).copy() for key, value in base_groups_deg.items()}
        for group, names in [('leg_waist', LEG_WAIST_JOINTS), ('left_arm', LEFT_ARM_JOINTS),
                             ('right_arm', RIGHT_ARM_JOINTS), ('neck', NECK_JOINTS)]:
            output[group] = np.asarray([np.rad2deg(q[self.joint_q[name]]) for name in names])
        return output

    def forward_poses(self, q):
        q = self._configuration(q)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return {side: self.data.oMf[frame_id].copy() for side, frame_id in self.frame_ids.items()}

    def _active_velocity_indices(self, target_sides):
        names = []
        if self.allow_waist:
            names.extend(['Joint_Waist_Pitch', 'Joint_Waist_Yaw'])
        if 'left' in target_sides:
            names.extend(LEFT_ARM_JOINTS)
        if 'right' in target_sides:
            names.extend(RIGHT_ARM_JOINTS)
        return [self.joint_v[name] for name in names]

    def _pose_errors(self, q, targets):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        errors = {}
        for side, (position, quaternion) in targets.items():
            desired = pin.SE3(quaternion_xyzw_to_rotation(quaternion), np.asarray(position))
            delta = self.data.oMf[self.frame_ids[side]].actInv(desired)
            errors[side] = pin.log6(delta).vector
        return errors

    def collision_pairs(self, q):
        if not self.collision_available:
            return []
        q = self._configuration(q)
        collided = pin.computeCollisions(
            self.model, self.data, self.geom_model, self.geom_data, q, False
        )
        if not collided:
            return []
        pairs = []
        for index, pair in enumerate(self.geom_model.collisionPairs):
            if self.geom_data.collisionResults[index].isCollision():
                first = self.geom_model.geometryObjects[pair.first].name
                second = self.geom_model.geometryObjects[pair.second].name
                pairs.append(f'{first} <-> {second}')
        return pairs

    def eef_poses(self, q):
        """Return current left/right EEF poses using this package's FK model."""
        placements = self.forward_poses(q)
        poses = {}
        for side, placement in placements.items():
            translation = np.asarray(placement.translation, dtype=float).reshape(-1)
            if translation.size != 3 or not np.all(np.isfinite(translation)):
                raise ValueError(f'{side} FK produced an invalid translation')
            quaternion = np.asarray(
                pin.Quaternion(placement.rotation).coeffs(), dtype=float
            ).reshape(-1)
            norm = float(np.linalg.norm(quaternion))
            if quaternion.size != 4 or not np.isfinite(norm) or norm < 1e-9:
                raise ValueError(f'{side} FK produced an invalid quaternion')
            quaternion = quaternion / norm
            poses[side] = {
                'position': [float(value) for value in translation],
                'orientation': [float(value) for value in quaternion],
            }
        return poses

    def solve(
        self,
        targets,
        q_seed,
        max_iterations=120,
        position_tolerance=0.004,
        orientation_tolerance=0.035,
        step_limit=0.10,
        damping=0.025,
        centering_gain=0.025,
        orientation_weight=0.65,
    ):
        start = time.perf_counter()
        q = np.clip(self._configuration(q_seed).copy(), self.lower, self.upper)
        active = self._active_velocity_indices(targets.keys())
        if not active:
            return IkResult(False, q, 0, 0.0, reason='no active joints')
        last_singular = 0.0
        pos_errors = {}
        rot_errors = {}

        for iteration in range(1, int(max_iterations) + 1):
            errors = self._pose_errors(q, targets)
            pos_errors = {side: float(np.linalg.norm(error[:3])) for side, error in errors.items()}
            rot_errors = {side: float(np.linalg.norm(error[3:])) for side, error in errors.items()}
            if all(value <= position_tolerance for value in pos_errors.values()) and all(
                    value <= orientation_tolerance for value in rot_errors.values()):
                collisions = self.collision_pairs(q)
                elapsed = (time.perf_counter() - start) * 1000.0
                return IkResult(
                    not collisions, q, iteration, elapsed, pos_errors, rot_errors,
                    last_singular, collisions,
                    'self collision' if collisions else 'converged',
                )

            rows = []
            rhs = []
            for side, error in errors.items():
                jacobian = pin.computeFrameJacobian(
                    self.model, self.data, q, self.frame_ids[side], pin.ReferenceFrame.LOCAL
                )[:, active]
                rotation_weight = float(np.clip(orientation_weight, 0.05, 1.0))
                weight = np.diag([
                    1.0, 1.0, 1.0,
                    rotation_weight, rotation_weight, rotation_weight,
                ])
                rows.append(weight @ jacobian)
                rhs.append(weight @ error)
            jacobian = np.vstack(rows)
            error = np.concatenate(rhs)
            singular_values = np.linalg.svd(jacobian, compute_uv=False)
            last_singular = float(singular_values[-1]) if singular_values.size else 0.0
            adaptive = float(damping) + max(0.0, 0.04 - last_singular) * 0.6
            system = jacobian @ jacobian.T + adaptive * adaptive * np.eye(jacobian.shape[0])
            try:
                dq = jacobian.T @ np.linalg.solve(system, error)
            except np.linalg.LinAlgError:
                elapsed = (time.perf_counter() - start) * 1000.0
                return IkResult(False, q, iteration, elapsed, pos_errors, rot_errors,
                                last_singular, reason='singular linear system')

            q_active = np.asarray([q[index] for index in active])
            low_active = np.asarray([self.lower[index] for index in active])
            high_active = np.asarray([self.upper[index] for index in active])
            center = 0.5 * (low_active + high_active)
            span = np.maximum(high_active - low_active, 1e-6)
            center_gradient = (center - q_active) / span
            pseudo = np.linalg.pinv(jacobian, rcond=1e-4)
            nullspace = np.eye(len(active)) - pseudo @ jacobian
            dq += float(centering_gain) * (nullspace @ center_gradient)
            max_component = float(np.max(np.abs(dq)))
            if max_component > step_limit:
                dq *= float(step_limit) / max_component
            velocity = np.zeros(self.model.nv)
            velocity[active] = dq
            q = pin.integrate(self.model, q, velocity)
            q = np.clip(q, self.lower, self.upper)

        elapsed = (time.perf_counter() - start) * 1000.0
        return IkResult(False, q, int(max_iterations), elapsed, pos_errors, rot_errors,
                        last_singular, reason='iteration limit reached')

    def accept_forward_reach_boundary(
        self,
        result,
        targets,
        q_seed,
        maximum_position_error=0.012,
        maximum_orientation_error=0.035,
        maximum_elbow_angle=np.deg2rad(25.0),
        maximum_singular_value=0.02,
    ):
        """Promote only a safe, near-straight forward boundary solution.

        A fully extended arm cannot exactly satisfy a Cartesian target beyond
        its geometric shell.  The iterative solver still returns the closest
        bounded configuration, but the normal millimetre tolerance rejects
        it.  This narrow classifier accepts that closest configuration only
        for forward extension, only near a straight elbow, and only when the
        residual remains centimetre-scale.  Joint limits remain enforced by
        ``solve`` and collision is checked again here.
        """
        if result.success or result.reason != 'iteration limit reached':
            return result
        limits = [
            float(maximum_position_error),
            float(maximum_orientation_error),
            float(maximum_elbow_angle),
            float(maximum_singular_value),
        ]
        if not all(np.isfinite(value) and value > 0.0 for value in limits):
            return result
        if not result.position_error_m or not result.orientation_error_rad:
            return result
        if any(float(value) > limits[0] for value in result.position_error_m.values()):
            return result
        if any(float(value) > limits[1] for value in result.orientation_error_rad.values()):
            return result
        if float(result.minimum_singular_value) > limits[3]:
            return result

        seed_poses = self.forward_poses(q_seed)
        outward_sides = []
        for side, (position, _quaternion) in targets.items():
            desired = np.asarray(position, dtype=float).reshape(-1)
            if desired.size != 3 or not np.all(np.isfinite(desired)):
                return result
            if desired[0] > float(seed_poses[side].translation[0]) + 0.002:
                outward_sides.append(side)
        if not outward_sides:
            return result
        for side in outward_sides:
            elbow = result.q[self.joint_q[f'Joint_{side.capitalize()}_Elbow']]
            if abs(float(elbow)) > limits[2]:
                return result

        collisions = self.collision_pairs(result.q)
        if collisions:
            return result
        return IkResult(
            True,
            result.q.copy(),
            result.iterations,
            result.solve_time_ms,
            dict(result.position_error_m),
            dict(result.orientation_error_rad),
            result.minimum_singular_value,
            [],
            'forward reach boundary',
        )
