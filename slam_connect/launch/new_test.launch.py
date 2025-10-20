#!/usr/bin/env python3
"""
full_sim_slam.launch.py
---------------------------------
Full UAV simulation:
PX4 SITL + Gazebo Harmonic + FAST-LIO2 + slam_diagnostic tools + MAVROS + RViz2

🧭 PURPOSE:
This launch file starts a complete simulation pipeline for a drone in Gazebo,
connected to PX4 , and running LiDAR–IMU SLAM (FAST-LIO2)
with diagnostic tools and visualization (RViz2).
"""

# -------------------------------------------------------------------------
# --- IMPORTS ---
# -------------------------------------------------------------------------
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # -------------------------------------------------------------------------
    # --- PATHS AND ENVIRONMENT SETUP ---
    # -------------------------------------------------------------------------
    pkg_share = FindPackageShare('slam_connect')

    px4_dir = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                           '../../../../../src/PX4-Autopilot'))
    match_models_path = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                     '../../../../../src/match_models'))

    px4_gz_models = f"{px4_dir}/Tools/simulation/gz/models"
    px4_gz_worlds = f"{px4_dir}/Tools/simulation/gz/worlds"
    px4_gz_plugins = f"{px4_dir}/build/px4_sitl_default/src/modules/simulation/gz_plugins"
    px4_gz_server_config = f"{px4_dir}/src/modules/simulation/gz_bridge/server.config"

    selected_world = f"{px4_gz_worlds}/forest.sdf"

    PX4_HOME_LAT = "52.42449457140792"
    PX4_HOME_LON = "9.620245153463955"
    PX4_HOME_ALT = "20.0"

    prev_res = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = f"{(prev_res + ':') if prev_res else ''}{px4_gz_worlds}:{px4_gz_models}:{match_models_path}"
    prev_plugins = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    gz_plugin_path = f"{(prev_plugins + ':') if prev_plugins else ''}{px4_gz_plugins}"
    prev_config = os.environ.get('GZ_SIM_SERVER_CONFIG_PATH', '')
    gz_server_config_path = f"{(prev_config + ':') if prev_config else ''}{px4_gz_server_config}"

    print(f"PX4 Directory: {px4_dir}")
    print(f"GZ Resource Path: {gz_resource_path}")
    print(f"GZ Plugin Path: {gz_plugin_path}")
    print(f"GZ Server Config Path: {gz_server_config_path}")

    # -------------------------------------------------------------------------
    # --- 1) GAZEBO SERVER ---
    # -------------------------------------------------------------------------
    gazebo_server = ExecuteProcess(
        name="gazebo_server",
        cmd=["bash", "-c", f"gz sim --verbose=1 -r -s {selected_world}"],
        additional_env={
            "PX4_GZ_MODELS": px4_gz_models,
            "PX4_GZ_WORLDS": px4_gz_worlds,
            "PX4_GZ_PLUGINS": px4_gz_plugins,
            "PX4_GZ_SERVER_CONFIG": px4_gz_server_config,
            "GZ_SIM_RESOURCE_PATH": gz_resource_path,
            "GZ_SIM_SYSTEM_PLUGIN_PATH": gz_plugin_path,
            "GZ_SIM_SERVER_CONFIG_PATH": gz_server_config_path,
        },
        output="screen"
    )

    # -------------------------------------------------------------------------
    # --- 2) ROS–GAZEBO BRIDGES ---
    # -------------------------------------------------------------------------
    bridge_tf_clock = TimerAction(
        period=1.0,
        actions=[Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge_tf_clock',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
                '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )]
    )

    bridge_sensors = TimerAction(
        period=4.0,
        actions=[Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge_sensors',
            arguments=[
                '/gz_scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                '/livox/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            ],
            remappings=[('/gz_scan', '/scan')],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )]
    )

    # -------------------------------------------------------------------------
    # --- 3) ROBOT STATE PUBLISHER ---
    # -------------------------------------------------------------------------
    rsp_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'robot_state_publisher.launch.py'])
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # -------------------------------------------------------------------------
    # --- 4) STATIC TF FIXES (correct chain for FAST-LIO2) ---
    # -------------------------------------------------------------------------
    # ✅ Correct TF hierarchy: map → camera_init → base_link → lidar_link

    static_tf_map_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
        output='screen'
    )

    static_tf_camera_body = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'base_link'],
        output='screen'
    )

    static_tf_body_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'lidar_link'],
        output='screen'
    )

    # -------------------------------------------------------------------------
    # --- 5) PX4 SITL ---
    # -------------------------------------------------------------------------
    px4_sitl = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            name="px4_sitl",
            cmd=["./build/px4_sitl_default/bin/px4"],
            cwd=px4_dir,
            shell=True,
            output="screen",
            additional_env={
                "PX4_SYS_AUTOSTART": "4001",
                "PX4_SIM_MODEL": "match_drohne",
                "PX4_SIMULATOR": "GZ",
                "VERBOSE": "1",
                "PX4_GZ_STANDALONE": "false",
                "PX4_HOME_LAT": PX4_HOME_LAT,
                "PX4_HOME_LON": PX4_HOME_LON,
                "PX4_HOME_ALT": PX4_HOME_ALT,
                "PX4_GZ_WORLD": "forest",
                "PX4_GZ_WORLDS": match_models_path,
                "GZ_SIM_RESOURCE_PATH": gz_resource_path,
            }
        )]
    )

    # -------------------------------------------------------------------------
    # --- 6) SLAM DIAGNOSTIC NODES ---
    # -------------------------------------------------------------------------
    slam_fixers = TimerAction(
        period=5.0,
        actions=[Node(
            package='slam_diagnostic',
            executable='slam_fixers_node',
            name='slam_fixers_node',
            parameters=[{'use_sim_time': True}],
            output='screen'
        )]
    )

    odom_bridge = TimerAction(
        period=5.5,
        actions=[Node(
            package='slam_diagnostic',
            executable='odom_bridge_node',
            name='odom_bridge_node',
            parameters=[{'use_sim_time': True}],
            output='screen'
        )]
    )

    # -------------------------------------------------------------------------
    # --- 6.5) LIVOX ROS DRIVER 2 ---
    # -------------------------------------------------------------------------
    livox_params = PathJoinSubstitution([
        FindPackageShare('slam_connect'),
        'config', 'livox_params.yaml'
    ])
    livox_driver = TimerAction(
        period=5.8,
        actions=[Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_ros_driver2_node',
            parameters=[livox_params],
            output='screen'
        )]
    )

    # -------------------------------------------------------------------------
    # --- 7) FAST-LIO2 ---
    # -------------------------------------------------------------------------
    fast_lio2 = TimerAction(
        period=6.5,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('slam_connect'),
                    'launch', 'fast_lio2_only.launch.py'
                ])
            )
        )]
    )

    # -------------------------------------------------------------------------
    # --- 8) GAZEBO CLIENT (GUI) ---
    # -------------------------------------------------------------------------
    gazebo_client = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            name="gazebo_client",
            cmd=["gz", "sim", "-g"],
            output="screen"
        )]
    )

    # -------------------------------------------------------------------------
    # --- 9) MAVROS (AFTER PX4 IS READY) ---
    # -------------------------------------------------------------------------
    mavros_node = TimerAction(
        period=10.0,
        actions=[ExecuteProcess(
            cmd=["ros2", "launch", "mavros", "px4.launch", "fcu_url:=udp://:14540@"],
            output="screen"
        )]
    )

    # -------------------------------------------------------------------------
    # --- 10) RVIZ2 ---
    # -------------------------------------------------------------------------
    rviz_node = TimerAction(
        period=7.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': True}],
            arguments=['-d',
                       PathJoinSubstitution([FindPackageShare('slam_connect'),
                                             'config', 'my_minimal_display.rviz'])],
            output='screen'
        )]
    )

    # -------------------------------------------------------------------------
    # --- RETURN FULL LAUNCH DESCRIPTION ---
    # -------------------------------------------------------------------------
    return LaunchDescription([
        gazebo_server,
        rsp_node,
        static_tf_map_camera,
        static_tf_camera_body,
        static_tf_body_lidar,
        bridge_tf_clock,
        gazebo_client,
        px4_sitl,
        bridge_sensors,
        slam_fixers,
        odom_bridge,
        livox_driver,
        fast_lio2,
        rviz_node,
        mavros_node,
    ])
