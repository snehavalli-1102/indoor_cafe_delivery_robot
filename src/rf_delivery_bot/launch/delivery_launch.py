#!/usr/bin/env python3
"""
delivery_launch.py — Production Launch File for ROS 2 Humble
Launches:
  1. Gazebo Fortress with indoor_delivery.sdf
  2. ros_gz_bridge (LaserScan, Ultrasonic Range, CmdVel)
  3. rssi_filter node with sim_mode capability
  4. rssi_navigator node with safety and gradient tracking parameters
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    try:
        pkg_share = get_package_share_directory('rf_delivery_bot')
        world_file = os.path.join(pkg_share, 'worlds', 'indoor_delivery.sdf')
    except Exception:
        # Fallback to relative local directory if package not installed
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        world_file = os.path.join(base_dir, 'worlds', 'indoor_delivery.sdf')

    # Arguments
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='true',
        description='Generate synthetic sinusoidal RSSI beacon signals for mapless testing.'
    )

    filter_type_arg = DeclareLaunchArgument(
        'filter_type',
        default_value='ema',
        description='Filter algorithm: ema or moving_avg'
    )

    # 1. Gazebo Simulator
    start_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        name='gazebo_fortress'
    )

    # 2. Gazebo <-> ROS 2 Bridge
    # Bridges scan, ultrasonic range, and cmd_vel
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        output='screen',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ultrasonic/range@sensor_msgs/msg/Range[gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ]
    )

    # 3. RSSI Filter Node
    rssi_filter_node = Node(
        package='rf_delivery_bot',
        executable='rssi_filter',
        name='rssi_filter',
        output='screen',
        parameters=[{
            'sim_mode': LaunchConfiguration('sim_mode'),
            'filter_type': LaunchConfiguration('filter_type'),
            'ema_alpha': 0.30,
            'window_size': 10
        }]
    )

    # 4. RSSI Navigator Node
    rssi_navigator_node = Node(
        package='rf_delivery_bot',
        executable='rssi_navigator',
        name='rssi_navigator',
        output='screen',
        parameters=[{
            'obstacle_threshold': 0.50,
            'forward_speed': 0.22,
            'search_angular_speed': 0.40,
            'target_arrival_rssi': -36.0,
            'trap_count_threshold': 3
        }]
    )

    # Delayed start of navigation nodes (3.0s after Gazebo starts)
    delayed_nodes = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='[Launch] Core physics engine stabilized. Activating Navigation Stack...'),
            rssi_filter_node,
            rssi_navigator_node,
        ]
    )

    return LaunchDescription([
        sim_mode_arg,
        filter_type_arg,
        LogInfo(msg='[Launch] Initializing Mapless RF Café Delivery Robot Simulation...'),
        start_gazebo,
        bridge_node,
        delayed_nodes
    ])
