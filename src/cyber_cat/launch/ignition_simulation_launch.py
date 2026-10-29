
###############################################################################
# launch/ignition_simulation_launch.py — Ignition Gazebo pursuit arena.
#
# Spawns both robots, bridges the Ignition transport topics ROS 2 cannot see
# natively, then starts the behaviour nodes on an 8 s delay so they do not
# publish into a world that has not finished loading.
# FIXES:
#   - Spawn commands now reference LaunchConfiguration (not hardcoded)
#   - Bridge topics corrected to match SDF plugin namespaces
#   - Added TimerAction for cyber_cat nodes
#   - Crazyflie velocity bridge uses correct model-scoped topic
###############################################################################

#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cyber_cat_dir = get_package_share_directory('cyber_cat')

    world_file      = os.path.join(cyber_cat_dir, 'worlds',
                                   'pursuit_arena_ignition.sdf')
    crazyflie_model = os.path.join(cyber_cat_dir, 'models', 'crazyflie.sdf')
    turtlebot_model = os.path.join(cyber_cat_dir, 'models', 'turtlebot4.sdf')
    params_file     = os.path.join(cyber_cat_dir, 'config', 'simulation_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_turtlebot  = LaunchConfiguration('x_turtlebot')
    y_turtlebot  = LaunchConfiguration('y_turtlebot')
    x_crazyflie  = LaunchConfiguration('x_crazyflie')
    y_crazyflie  = LaunchConfiguration('y_crazyflie')
    z_crazyflie  = LaunchConfiguration('z_crazyflie')

    # --- Launch arguments ---
    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Place robots well inside 10x10 arena (walls at +/-5)
        DeclareLaunchArgument('x_turtlebot',  default_value='0.0'),
        DeclareLaunchArgument('y_turtlebot',  default_value='-1.0'),
        DeclareLaunchArgument('x_crazyflie',  default_value='0.0'),
        DeclareLaunchArgument('y_crazyflie',  default_value='1.0'),
        DeclareLaunchArgument('z_crazyflie',  default_value='0.05'),
    ]

    # --- Ignition Gazebo ---
    ignition_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file],
        output='screen',
    )

    spawn_crazyflie = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_crazyflie',
        arguments=[
            '-world', 'pursuit_arena',
            '-name', 'crazyflie',
            '-file', crazyflie_model,
            '-x', x_crazyflie,
            '-y', y_crazyflie,
            '-z', z_crazyflie,
        ],
        output='screen',
    )

    spawn_turtlebot = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_turtlebot',
        arguments=[
            '-world', 'pursuit_arena',
            '-name', 'turtlebot',
            '-file', turtlebot_model,
            '-x', x_turtlebot,
            '-y', y_turtlebot,
            '-z', '0.08',
        ],
        output='screen',
    )

    # --- Bridges ---
    # FIX: Topic names must match what the SDF plugins actually publish.
    # Ignition scopes topics under /model/<name>/...
    # The '@' syntax: ros_topic@ros_msg_type@ign_msg_type  (bidirectional)
    #                 ros_topic@ros_msg_type[ign_msg_type  (ign→ros only)

    bridge_clock = Node(
        package='ros_ign_bridge', executable='parameter_bridge',
        name='bridge_clock',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen',
    )

    # Crazyflie bridges
    bridge_cf_cmd = Node(
        package='ros_ign_bridge', executable='parameter_bridge',
        name='bridge_cf_cmd_vel',
        arguments=[
            '/model/crazyflie/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'
        ],
        remappings=[('/model/crazyflie/cmd_vel', '/crazyflie/cmd_vel')],
        output='screen',
    )
    bridge_cf_odom = Node(
        package='ros_ign_bridge', executable='parameter_bridge',
        name='bridge_cf_odom',
        arguments=[
            '/model/crazyflie/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'
        ],
        remappings=[('/model/crazyflie/odometry', '/crazyflie/odom')],
        output='screen',
    )
    bridge_cf_cam = Node(
        package='ros_ign_bridge', executable='parameter_bridge',
        name='bridge_cf_camera',
        arguments=[
            '/world/pursuit_arena/model/crazyflie/link/base_link/sensor/camera/image'
            '@sensor_msgs/msg/Image[ignition.msgs.Image'
        ],
        remappings=[
            ('/world/pursuit_arena/model/crazyflie/link/base_link/sensor/camera/image',
             '/crazyflie/camera/image_raw')
        ],
        output='screen',
    )
    bridge_cf_imu = Node(
        package='ros_ign_bridge', executable='parameter_bridge',
        name='bridge_cf_imu',
        arguments=[
            '/world/pursuit_arena/model/crazyflie/link/base_link/sensor/imu/imu'
            '@sensor_msgs/msg/Imu[ignition.msgs.IMU'
        ],
        remappings=[
            ('/world/pursuit_arena/model/crazyflie/link/base_link/sensor/imu/imu',
             '/crazyflie/imu')
        ],
        output='screen',
    )

    # TurtleBot bridges
    bridge_tb_cmd = Node(
        package='ros_ign_bridge', executable='parameter_bridge',
        name='bridge_tb_cmd_vel',
        arguments=[
            '/model/turtlebot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'
        ],
        output='screen',
    )
    bridge_tb_odom = Node(
        package='ros_ign_bridge', executable='parameter_bridge',
        name='bridge_tb_odom',
        arguments=[
            '/model/turtlebot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'
        ],
        remappings=[('/model/turtlebot/odometry', '/turtlebot/odom')],
        output='screen',
    )
    bridge_tb_scan = Node(
        package='ros_ign_bridge', executable='parameter_bridge',
        name='bridge_tb_scan',
        arguments=[
            '/world/pursuit_arena/model/turtlebot/link/base_link/sensor/rplidar/scan'
            '@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'
        ],
        remappings=[
            ('/world/pursuit_arena/model/turtlebot/link/base_link/sensor/rplidar/scan',
             '/turtlebot/scan')
        ],
        output='screen',
    )
    bridge_tb_cam = Node(
        package='ros_ign_bridge', executable='parameter_bridge',
        name='bridge_tb_camera',
        arguments=[
            '/world/pursuit_arena/model/turtlebot/link/base_link/sensor/oakd_camera/image'
            '@sensor_msgs/msg/Image[ignition.msgs.Image'
        ],
        remappings=[
            ('/world/pursuit_arena/model/turtlebot/link/base_link/sensor/oakd_camera/image',
             '/turtlebot/camera/image_raw')
        ],
        output='screen',
    )

    # --- Cyber-Cat nodes (delayed) ---
    cyber_cat_nodes = TimerAction(
        period=8.0,
        actions=[
            Node(package='cyber_cat', executable='mission_executive',
                  name='mission_executive', output='screen',
                  parameters=[params_file, {'use_sim_time': use_sim_time}]),
            Node(package='cyber_cat', executable='vision_processor',
                  name='vision_processor', output='screen',
                  parameters=[params_file, {'use_sim_time': use_sim_time}]),
            Node(package='cyber_cat', executable='chase_controller',
                  name='chase_controller', output='screen',
                  parameters=[params_file, {'use_sim_time': use_sim_time}]),
            Node(package='cyber_cat', executable='voice_command',
                  name='voice_command', output='screen',
                  parameters=[params_file, {'use_sim_time': use_sim_time}]),
        ],
    )

    return LaunchDescription(
        declare_args + [
            ignition_gazebo,
            spawn_crazyflie,
            spawn_turtlebot,
            bridge_clock,
            bridge_cf_cmd, bridge_cf_odom, bridge_cf_cam, bridge_cf_imu,
            bridge_tb_cmd, bridge_tb_odom, bridge_tb_scan, bridge_tb_cam,
            cyber_cat_nodes,
        ]
    )
