from glob import glob
from setuptools import find_packages, setup


package_name = 'openarmx_teleop_vr_306_v4'

setup(
    name=package_name,
    version='0.4.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, [
            'package.xml', 'README.md', 'AI_MIGRATION_GUIDE.md', 'NOTICE',
            'THIRD_PARTY_LICENSES.md'
        ]),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/meshes/robot_v2_2', glob('meshes/robot_v2_2/*')),
        ('share/' + package_name + '/web',
         glob('web/*.html') + glob('web/*.css') + glob('web/*.js')),
        ('share/' + package_name + '/web/vendor', glob('web/vendor/*')),
    ],
    install_requires=['setuptools'],
    scripts=[
        'scripts/openarmx_306_v4_arm_controller_robot_env.sh',
        'scripts/setup_v4_placo_env.sh',
    ],
    zip_safe=False,
    maintainer='Autolife Robotics',
    maintainer_email='ubuntu@example.com',
    description='Quest3-middle WebXR arm teleoperation for Autolife robot 306.',
    license='CC-BY-NC-SA-4.0',
    entry_points={
        'console_scripts': [
            'openarmx_306_v4_arm_controller = openarmx_teleop_vr_306_v4.controller_node:main',
            'openarmx_306_v4_mapper = openarmx_teleop_vr_306_v4.vr_mapper_node:main',
            'openarmx_306_v4_udp_input = openarmx_teleop_vr_306_v4.openarmx_udp_input_node:main',
            'openarmx_306_v4_web_bridge = openarmx_teleop_vr_306_v4.vr_web_bridge:main',
            'ik_self_test = openarmx_teleop_vr_306_v4.ik_self_test:main',
        ],
    },
)
