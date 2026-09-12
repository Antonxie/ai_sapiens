#!/usr/bin/env python3
# Copyright 2026 ROBOTIS CO., LTD. (adapted for Model_A_E 0624)
#
# Hardware bringup for the 0624wholebodyURDF robot.
# sim_mujoco:=true loads the MuJoCo MujocoSystem (sim2sim);
# sim_mujoco:=false is the real-hardware placeholder (non-Dynamixel harness TBD).

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('sim_mujoco', default_value='true'),
        DeclareLaunchArgument('mujoco_viewer', default_value='false'),
        DeclareLaunchArgument('mujoco_gantry', default_value='false'),
    ]

    # sim2sim 阶段加载带 ros2_control MujocoSystem 的 URDF；
    # $(find pkg) 需展开为绝对路径（MujocoSystem 不解析 $(find)）。
    desc_share = get_package_share_directory('ai_sapiens_description')
    urdf_path = os.path.join(desc_share, 'urdf', 'modelae_0624', 'modelae_0624_mujoco.urdf')
    with open(urdf_path) as f:
        robot_description_content = f.read().replace('$(find ai_sapiens_description)', desc_share)

    controller_manager_config = PathJoinSubstitution([
        FindPackageShare('ai_sapiens_bringup'), 'config',
        'modelae_0624', 'modelae_0624_controllers.yaml',
    ])

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_manager_config],
        output='both',
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 'imu_sensor_broadcaster'],
        output='screen',
    )

    joint_group_impedance_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_impedance_controller'],
        output='screen',
    )

    return LaunchDescription(
        declared_arguments + [
            control_node,
            robot_state_pub_node,
            # 竞态修复: spawner 若在硬件 activate 完成前加载 controller 会 FATAL
            # 失败(broadcaster 保持 unconfigured -> IMU/joint_states 断流 ->
            # sim2real 节点 Damping failsafe 锁存)。延迟 2s 等硬件就绪。
            TimerAction(
                period=2.0,
                actions=[joint_state_broadcaster_spawner],
            ),
            TimerAction(
                period=2.0,
                actions=[joint_group_impedance_controller_spawner],
            ),
        ]
    )