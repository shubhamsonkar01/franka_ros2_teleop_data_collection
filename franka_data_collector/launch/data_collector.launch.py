#!/usr/bin/env python3
# Launch file for the franka_data_collector node
# Usage:
#   ros2 launch franka_data_collector data_collector.launch.py
#   ros2 launch franka_data_collector data_collector.launch.py follower_namespace:=franka_teleop/follower output_dir:=/tmp/franka_data

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'follower_namespace',
            default_value='franka_teleop/follower',
            description=(
                'ROS 2 namespace of the follower robot. '
                'E.g. "franka_teleop/follower" or "fr3_duo/left/follower". '
                'Matches the namespace in your fr3_teleop_config.yaml pair namespace + /follower'
            ),
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value='/tmp/franka_data',
            description='Directory to save collected data (CSV and/or NPY files)',
        ),
        DeclareLaunchArgument(
            'save_csv',
            default_value='true',
            description='Save data as CSV file',
        ),
        DeclareLaunchArgument(
            'save_npy',
            default_value='true',
            description='Save data as NPY file (NumPy binary, faster to load)',
        ),
        DeclareLaunchArgument(
            'buffer_size',
            default_value='100000',
            description='Number of samples to buffer before auto-flushing to disk',
        ),
        DeclareLaunchArgument(
            'auto_save_interval_sec',
            default_value='30.0',
            description='Interval in seconds between automatic saves (0 = disable)',
        ),
        DeclareLaunchArgument(
            'use_franka_robot_state',
            default_value='true',
            description=(
                'Subscribe to FrankaRobotState for EE pose, Cartesian velocity, external torques. '
                'Requires franka_msgs to be built. Set false if not available.'
            ),
        ),
        Node(
            package='franka_data_collector',
            executable='data_collector_node',
            name='franka_data_collector',
            output='screen',
            parameters=[{
                'follower_namespace': LaunchConfiguration('follower_namespace'),
                'output_dir': LaunchConfiguration('output_dir'),
                'save_csv': LaunchConfiguration('save_csv'),
                'save_npy': LaunchConfiguration('save_npy'),
                'buffer_size': LaunchConfiguration('buffer_size'),
                'auto_save_interval_sec': LaunchConfiguration('auto_save_interval_sec'),
                'use_franka_robot_state': LaunchConfiguration('use_franka_robot_state'),
            }],
        ),
    ])
