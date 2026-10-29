#!/usr/bin/env python3
"""Behaviour nodes only — no simulator, no robot bringup.

Useful when pointing the stack at an already-running robot or a bag replay.
For the real TurtleBot 4 use real_robot_launch.py, which also carries the
hardware-tuned parameters.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='cyber_cat', executable='mission_executive',
             name='mission_executive', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='cyber_cat', executable='vision_processor',
             name='vision_processor', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='cyber_cat', executable='chase_controller',
             name='chase_controller', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='cyber_cat', executable='voice_command',
             name='voice_command', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
    ])