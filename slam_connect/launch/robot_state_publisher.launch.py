#!/usr/bin/env python3
"""
robot_state_publisher.launch.py
---------------------------------
Publishes the TF tree of the robot from its URDF description (via Xacro).
Used in simulation with FAST-LIO2.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # --- Launch arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    # --- Path to your drone URDF (Xacro) ---
    urdf_path = PathJoinSubstitution([
        FindPackageShare('slam_connect'),
        'description', 'urdf', 'drone.urdf.xacro'
    ])

    # --- Robot State Publisher Node ---
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_path]),
                value_type=str
            ),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        rsp_node
    ])
