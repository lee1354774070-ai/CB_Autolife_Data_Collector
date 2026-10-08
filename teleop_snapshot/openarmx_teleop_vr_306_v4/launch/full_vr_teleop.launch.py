"""Launch the Quest-middle V4 WebXR teleoperation stack for robot 306."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('openarmx_teleop_vr_306_v4')
    controller_config = os.path.join(share, 'config', 'controller.yaml')
    teleop_config = os.path.join(share, 'config', 'teleop.yaml')
    urdf = os.path.join(share, 'urdf', 'robot_v2_2_simplified.urdf')
    srdf = os.path.join(share, 'urdf', 'robot_v2_2.srdf')

    suffix = LaunchConfiguration('topic_suffix')
    dry_run = LaunchConfiguration('dry_run')
    start_web = LaunchConfiguration('start_web')
    web_host = LaunchConfiguration('web_host')
    web_port = LaunchConfiguration('web_port')
    rgbd_camera_enabled = LaunchConfiguration('rgbd_camera_enabled')
    forward_position_scale = LaunchConfiguration('forward_position_scale')
    waist_follow_profile = LaunchConfiguration('waist_follow_profile')
    waist_forward_assist_max_lean_deg = LaunchConfiguration(
        'waist_forward_assist_max_lean_deg'
    )
    waist_forward_assist_command_lead_deg = LaunchConfiguration(
        'waist_forward_assist_command_lead_deg'
    )
    wrist_reach_reserve_enabled = LaunchConfiguration(
        'wrist_reach_reserve_enabled'
    )
    wrist_reach_reserve_minimum_m = LaunchConfiguration(
        'wrist_reach_reserve_minimum_m'
    )
    wrist_reach_reserve_maximum_m = LaunchConfiguration(
        'wrist_reach_reserve_maximum_m'
    )
    wrist_reach_reserve_start_angle_deg = LaunchConfiguration(
        'wrist_reach_reserve_start_angle_deg'
    )
    wrist_reach_reserve_full_angle_deg = LaunchConfiguration(
        'wrist_reach_reserve_full_angle_deg'
    )
    head_roll_follow_enabled = LaunchConfiguration(
        'head_roll_follow_enabled'
    )
    body_height_control_enabled = LaunchConfiguration(
        'body_height_control_enabled'
    )
    body_height_lowering_rate_m_sec = LaunchConfiguration(
        'body_height_lowering_rate_m_sec'
    )
    quick_reset_max_velocity_deg_sec = LaunchConfiguration(
        'quick_reset_max_velocity_deg_sec'
    )
    quick_reset_max_acceleration_deg_sec2 = LaunchConfiguration(
        'quick_reset_max_acceleration_deg_sec2'
    )
    quick_reset_max_jerk_deg_sec3 = LaunchConfiguration(
        'quick_reset_max_jerk_deg_sec3'
    )
    quick_reset_neck_max_velocity_deg_sec = LaunchConfiguration(
        'quick_reset_neck_max_velocity_deg_sec'
    )
    quick_reset_neck_max_acceleration_deg_sec2 = LaunchConfiguration(
        'quick_reset_neck_max_acceleration_deg_sec2'
    )
    quick_reset_neck_max_jerk_deg_sec3 = LaunchConfiguration(
        'quick_reset_neck_max_jerk_deg_sec3'
    )
    quick_reset_neck_joints = LaunchConfiguration(
        'quick_reset_neck_joints'
    )
    head_neutral_joints_deg = LaunchConfiguration(
        'head_neutral_joints_deg'
    )
    lock_head_follow_on_enable_reset = LaunchConfiguration(
        'lock_head_follow_on_enable_reset'
    )
    robot_env_python = LaunchConfiguration('robot_env_python')
    ros_domain_id = LaunchConfiguration('ros_domain_id')
    rmw_implementation = LaunchConfiguration('rmw_implementation')
    cyclonedds_uri = LaunchConfiguration('cyclonedds_uri')

    controller = Node(
        package='openarmx_teleop_vr_306_v4',
        executable='openarmx_306_v4_arm_controller_robot_env.sh',
        name='independent_arm_controller_306_v4',
        output='screen',
        parameters=[controller_config, {
            'topic_suffix': ParameterValue(suffix, value_type=str),
            'dry_run': ParameterValue(dry_run, value_type=bool),
            'waist_follow_profile': ParameterValue(
                waist_follow_profile, value_type=str
            ),
            'waist_forward_assist_max_lean_deg': ParameterValue(
                waist_forward_assist_max_lean_deg, value_type=float
            ),
            'waist_forward_assist_command_lead_deg': ParameterValue(
                waist_forward_assist_command_lead_deg, value_type=float
            ),
            'enable_self_collision_check': False,
            'body_height_control_enabled': ParameterValue(
                body_height_control_enabled, value_type=bool
            ),
            'body_height_lowering_rate_m_sec': ParameterValue(
                body_height_lowering_rate_m_sec, value_type=float
            ),
            'quick_reset_max_velocity_deg_sec': ParameterValue(
                quick_reset_max_velocity_deg_sec, value_type=float
            ),
            'quick_reset_max_acceleration_deg_sec2': ParameterValue(
                quick_reset_max_acceleration_deg_sec2, value_type=float
            ),
            'quick_reset_max_jerk_deg_sec3': ParameterValue(
                quick_reset_max_jerk_deg_sec3, value_type=float
            ),
            'quick_reset_neck_max_velocity_deg_sec': ParameterValue(
                quick_reset_neck_max_velocity_deg_sec, value_type=float
            ),
            'quick_reset_neck_max_acceleration_deg_sec2': ParameterValue(
                quick_reset_neck_max_acceleration_deg_sec2, value_type=float
            ),
            'quick_reset_neck_max_jerk_deg_sec3': ParameterValue(
                quick_reset_neck_max_jerk_deg_sec3, value_type=float
            ),
            'quick_reset_neck_joints': ParameterValue(
                quick_reset_neck_joints
            ),
            'lock_head_follow_on_enable_reset': ParameterValue(
                lock_head_follow_on_enable_reset, value_type=bool
            ),
            'require_teleop_heartbeat': True,
            'urdf_path': urdf,
            'srdf_path': srdf,
        }],
    )

    mapper = Node(
        package='openarmx_teleop_vr_306_v4',
        executable='openarmx_306_v4_mapper',
        name='independent_vr_mapper_306_v4',
        output='screen',
        parameters=[teleop_config, {
            'topic_suffix': ParameterValue(suffix, value_type=str),
            'dry_run': ParameterValue(dry_run, value_type=bool),
            'forward_position_scale': ParameterValue(
                forward_position_scale, value_type=float
            ),
            'wrist_reach_reserve_enabled': ParameterValue(
                wrist_reach_reserve_enabled, value_type=bool
            ),
            'wrist_reach_reserve_minimum_m': ParameterValue(
                wrist_reach_reserve_minimum_m, value_type=float
            ),
            'wrist_reach_reserve_maximum_m': ParameterValue(
                wrist_reach_reserve_maximum_m, value_type=float
            ),
            'wrist_reach_reserve_start_angle_deg': ParameterValue(
                wrist_reach_reserve_start_angle_deg, value_type=float
            ),
            'wrist_reach_reserve_full_angle_deg': ParameterValue(
                wrist_reach_reserve_full_angle_deg, value_type=float
            ),
            'head_roll_follow_enabled': ParameterValue(
                head_roll_follow_enabled, value_type=bool
            ),
            'head_neutral_joints_deg': ParameterValue(
                head_neutral_joints_deg
            ),
        }],
    )

    web_bridge = ExecuteProcess(
        condition=IfCondition(start_web),
        cmd=[
            robot_env_python,
            '-m', 'openarmx_teleop_vr_306_v4.vr_web_bridge',
            '--ros-args',
            '-p', ['host:=', web_host],
            '-p', ['https_port:=', web_port],
            '-p', ['rgbd_camera_enabled:=', rgbd_camera_enabled],
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('topic_suffix', default_value='0_300'),
        DeclareLaunchArgument(
            'dry_run', default_value='true',
            description='Keep true for WebXR/IK preview without hardware output.'),
        DeclareLaunchArgument('start_web', default_value='true'),
        DeclareLaunchArgument('web_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('web_port', default_value='8446'),
        DeclareLaunchArgument(
            'rgbd_camera_enabled', default_value='true',
            description='Expose the read-only RGB-D colour stream to WebXR.'),
        DeclareLaunchArgument(
            'forward_position_scale', default_value='0.90',
            description=(
                'Operator-forward hand displacement scale. Other axes retain '
                'the position_scale value from teleop.yaml.'
            )),
        DeclareLaunchArgument(
            'waist_follow_profile', default_value='forward_pitch_only',
            description=(
                'Deterministic forward-only waist assistance. The redundant '
                'ik_pitch_yaw profile remains available only as an explicit '
                'diagnostic override.'
            )),
        DeclareLaunchArgument(
            'waist_forward_assist_max_lean_deg', default_value='10.0',
            description='Maximum forward waist pitch assist in degrees.'),
        DeclareLaunchArgument(
            'waist_forward_assist_command_lead_deg', default_value='8.0',
            description=(
                'Feedback-bounded waist-pitch command lead used only while '
                'forward reach assistance is active.'
            )),
        DeclareLaunchArgument(
            'wrist_reach_reserve_enabled', default_value='false',
            description='Dynamically preserve wrist dexterity near full reach.'),
        DeclareLaunchArgument(
            'wrist_reach_reserve_minimum_m', default_value='0.015'),
        DeclareLaunchArgument(
            'wrist_reach_reserve_maximum_m', default_value='0.070'),
        DeclareLaunchArgument(
            'wrist_reach_reserve_start_angle_deg', default_value='0.0'),
        DeclareLaunchArgument(
            'wrist_reach_reserve_full_angle_deg', default_value='70.0'),
        DeclareLaunchArgument(
            'head_roll_follow_enabled', default_value='true',
            description='Allow HMD side tilt to control robot neck Roll.'),
        DeclareLaunchArgument(
            'body_height_control_enabled', default_value='false',
            description='Accept guarded coordinated waist/leg height commands.'),
        DeclareLaunchArgument(
            'body_height_lowering_rate_m_sec', default_value='0.10',
            description='Coordinated body-height travel rate in metres/second.'),
        DeclareLaunchArgument(
            'quick_reset_max_velocity_deg_sec', default_value='23.4'),
        DeclareLaunchArgument(
            'quick_reset_max_acceleration_deg_sec2', default_value='65.0'),
        DeclareLaunchArgument(
            'quick_reset_max_jerk_deg_sec3', default_value='325.0'),
        DeclareLaunchArgument(
            'quick_reset_neck_max_velocity_deg_sec', default_value='30.0'),
        DeclareLaunchArgument(
            'quick_reset_neck_max_acceleration_deg_sec2',
            default_value='83.333'),
        DeclareLaunchArgument(
            'quick_reset_neck_max_jerk_deg_sec3', default_value='416.667'),
        DeclareLaunchArgument(
            'quick_reset_neck_joints', default_value='[0.010928,-29.998649,0.010928]',
            description='Quick-reset neck target [roll, pitch, yaw] in degrees.'),
        DeclareLaunchArgument(
            'head_neutral_joints_deg', default_value='[0.0, 0.0, 0.0]',
            description='Head-follow neutral [roll, pitch, yaw] in degrees.'),
        DeclareLaunchArgument(
            'lock_head_follow_on_enable_reset', default_value='false',
            description='Lock head follow after hardware enable and quick reset.'),
        DeclareLaunchArgument('ros_domain_id', default_value='0'),
        DeclareLaunchArgument(
            'rmw_implementation', default_value='rmw_cyclonedds_cpp'),
        DeclareLaunchArgument(
            'cyclonedds_uri',
            default_value=(
                '<CycloneDDS><Domain><General><Interfaces>'
                '<NetworkInterface name="lo"/>'
                '</Interfaces><AllowMulticast>false</AllowMulticast>'
                '</General><Discovery><ParticipantIndex>auto</ParticipantIndex>'
                '<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>'
                '</Discovery></Domain></CycloneDDS>')),
        DeclareLaunchArgument(
            'robot_env_python',
            default_value='/home/ubuntu/ros2_ws/venvs/openarmx_v4_placo/bin/python'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', ros_domain_id),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', rmw_implementation),
        SetEnvironmentVariable('CYCLONEDDS_URI', cyclonedds_uri),
        RegisterEventHandler(OnProcessExit(
            target_action=controller,
            on_exit=[Shutdown(reason='independent arm controller exited')],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=mapper,
            on_exit=[Shutdown(reason='WebXR mapper exited')],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=web_bridge,
            on_exit=[Shutdown(reason='WebXR bridge exited')],
        )),
        controller,
        mapper,
        web_bridge,
    ])
