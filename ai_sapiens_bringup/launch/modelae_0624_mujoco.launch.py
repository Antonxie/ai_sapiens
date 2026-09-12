#!/usr/bin/env python3
# Copyright 2026 ROBOTIS CO., LTD. (adapted for Model_A_E 0624)
#
# 0624 MuJoCo sim2sim bringup:
#   1. modelae_0624_hw.launch.py   - ros2_control + MuJoCo MujocoSystem
#   2. ai_sapiens_sim2real.launch.py - ONNX 策略节点 (config/modelae_0624_config.yaml)
#
# Usage:
#   ros2 launch ai_sapiens_bringup modelae_0624_mujoco.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mujoco_viewer', default_value='false'),
        DeclareLaunchArgument('mujoco_gantry', default_value='false'),
        DeclareLaunchArgument('robot', default_value='modelae_0624'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('ai_sapiens_bringup'),
                                      'launch', 'modelae_0624_hw.launch.py'])]),
            launch_arguments={
                'sim_mujoco': 'true',
                'mujoco_viewer': LaunchConfiguration('mujoco_viewer'),
                'mujoco_gantry': LaunchConfiguration('mujoco_gantry'),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('ai_sapiens_sim2real'),
                                      'launch', 'ai_sapiens_sim2real.launch.py'])]),
            launch_arguments={
                'robot': LaunchConfiguration('robot'),
                'cmd_vel_topic': '/cmd_vel',
            }.items(),
        ),
    ])