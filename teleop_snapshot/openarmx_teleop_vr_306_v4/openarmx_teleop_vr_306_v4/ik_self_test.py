import argparse
import os

import numpy as np
from ament_index_python.packages import get_package_share_directory

from .kinematics import IndependentArmKinematics, LEFT_ARM_JOINTS


def run(urdf_path):
    solver = IndependentArmKinematics(urdf_path, collision_check=False)
    groups = {
        'leg_waist': np.zeros(4),
        'left_arm': np.zeros(7),
        'right_arm': np.zeros(7),
        'neck': np.zeros(3),
        'left_gripper': np.zeros(1),
        'right_gripper': np.zeros(1),
    }
    seed = solver.q_from_feedback(groups)
    goal = seed.copy()
    goal[solver.joint_q[LEFT_ARM_JOINTS[0]]] += 0.08
    goal[solver.joint_q[LEFT_ARM_JOINTS[3]]] += 0.10
    pose = solver.forward_poses(goal)['left']
    quaternion = np.asarray(__import__('pinocchio').Quaternion(pose.rotation).coeffs())
    result = solver.solve(
        {'left': (pose.translation.copy(), quaternion)},
        seed,
        max_iterations=200,
        position_tolerance=0.004,
        orientation_tolerance=0.035,
    )
    print(
        f'success={result.success} iterations={result.iterations} '
        f'time_ms={result.solve_time_ms:.2f} position_error={result.position_error_m} '
        f'orientation_error={result.orientation_error_rad} reason={result.reason}'
    )
    return 0 if result.success else 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--urdf', default='')
    args = parser.parse_args()
    if args.urdf:
        path = args.urdf
    else:
        share = get_package_share_directory('openarmx_teleop_vr_306_v4')
        path = os.path.join(share, 'urdf', 'robot_v2_2_simplified.urdf')
    raise SystemExit(run(path))


if __name__ == '__main__':
    main()
