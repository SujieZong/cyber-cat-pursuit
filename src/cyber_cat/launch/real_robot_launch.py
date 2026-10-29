"""Launch file for real TurtleBot4 pursuit (no simulator).

Launches vision_processor, mission_executive, and chase_controller.

Usage:
    # Terminal 1 — launch core nodes
    ros2 launch cyber_cat real_robot_launch.py

    # Terminal 2 — command input (type "start" or "stop" + Enter)
    ros2 run cyber_cat voice_command
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = LaunchConfiguration(
        'image_topic',
        default='/turtlebot468/oakd/rgb/preview/image_raw',
    )

    return LaunchDescription([
        # ── Launch arguments ──────────────────────────────────────────
        DeclareLaunchArgument(
            'image_topic',
            default_value='/turtlebot468/oakd/rgb/preview/image_raw',
            description='Camera image topic on the real TurtleBot4',
        ),

        # ── Vision processor ──────────────────────────────────────────
        Node(
            package='cyber_cat',
            executable='vision_processor',
            name='vision_processor',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'image_topic': image_topic,
            }],
        ),

        # ── Mission executive (state machine + light ring) ────────────
        Node(
            package='cyber_cat',
            executable='mission_executive',
            name='mission_executive',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'mission_rate': 10.0,
                'search_timeout': 90.0,
                'detection_window_size': 10,
                'detection_confirm_threshold': 6,     # was 7, slightly looser
                'detection_loss_threshold': 2,
                'target_lost_timeout': 3.0,            # was 5.0, faster back to SEARCH
                'capture_area_threshold': 0.025,       # was 0.01, increased standoff
                'capture_confirm_time': 1.5,           # was 2.0
                'celebrate_timeout': 5.0,
            }],
        ),

        # ── Chase controller ──────────────────────────────────────────
        Node(
            package='cyber_cat',
            executable='chase_controller',
            name='chase_controller',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                # Lower = stop farther from target (larger physical standoff)
                'standoff_forward_area_ratio': 0.018,
                'approach_brake_area_ratio': 0.007,
            }],
        ),
    ])
