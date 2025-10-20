#!/usr/bin/env python3
"""
fast_lio2_only.launch.py
---------------------------------
Runs the FAST-LIO2 LiDAR–IMU Odometry and Mapping node only.
This version does not start PX4 or Gazebo — ideal for dataset replay
or testing SLAM parameters independently.
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # -------------------------------------------------------------------------
    # --- CONFIGURATION FILES ---
    # -------------------------------------------------------------------------
    # Load the FAST-LIO2 parameter YAML from slam_connect
    fastlio_params = PathJoinSubstitution([
        FindPackageShare('slam_connect'),
        'config', 'slam_params.yaml'
    ])

    # -------------------------------------------------------------------------
    # --- FAST-LIO2 NODE ---
    # -------------------------------------------------------------------------
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio2',
        output='screen',
        emulate_tty=True,
        parameters=[fastlio_params, {'use_sim_time': True}],
        remappings=[
            ('/Odometry', '/odom_lio'),
            # ('/Laser_map', '/cloud_map'),  # optional
        ],
    )

    # -------------------------------------------------------------------------
    # --- TF STATIC BROADCASTER ---
    # -------------------------------------------------------------------------
    # Provides a fixed transform between map and camera_init (needed by FAST-LIO)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # -------------------------------------------------------------------------
    # --- RVIZ2 FOR VISUALIZATION (OPTIONAL) ---
    # -------------------------------------------------------------------------
    # If you want this standalone FAST-LIO2 launch to also open RViz,
    # just uncomment the lines below.
    #
    # rviz_config = PathJoinSubstitution([
    #     FindPackageShare('slam_connect'),
    #     'config', 'my_minimal_display.rviz'
    # ])
    #
    # rviz_node = TimerAction(
    #     period=2.0,
    #     actions=[Node(
    #         package='rviz2',
    #         executable='rviz2',
    #         name='rviz2',
    #         output='screen',
    #         parameters=[{'use_sim_time': True}],
    #         arguments=['-d', rviz_config],
    #     )]
    # )

    # -------------------------------------------------------------------------
    # --- RETURN COMPLETE LAUNCH DESCRIPTION ---
    # -------------------------------------------------------------------------
    return LaunchDescription([
        static_tf,      # Publish map → camera_init
        fast_lio_node,  # Run FAST-LIO2 core algorithm
        # rviz_node,    # (Uncomment if you want RViz for standalone runs)
    ])
