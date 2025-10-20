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
# These Python libraries are used to create a launch description in ROS 2.
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
    # Find the shared path for your ROS 2 package `slam_connect`
    pkg_share = FindPackageShare('slam_connect')

    # Define where the PX4-Autopilot source directory is located
    px4_dir = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                           '../../../../../src/PX4-Autopilot'))

    # Path to your drone models (match_drohne, etc.)
    match_models_path = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                     '../../../../../src/match_models'))

    # Define PX4 Gazebo resource directories
    px4_gz_models = f"{px4_dir}/Tools/simulation/gz/models"
    px4_gz_worlds = f"{px4_dir}/Tools/simulation/gz/worlds"
    px4_gz_plugins = f"{px4_dir}/build/px4_sitl_default/src/modules/simulation/gz_plugins"
    px4_gz_server_config = f"{px4_dir}/src/modules/simulation/gz_bridge/server.config"

    # Choose which Gazebo world to load
    selected_world = f"{px4_gz_worlds}/forest.sdf"

    # Define PX4 "home" GPS coordinates (where the drone spawns)
    PX4_HOME_LAT = "52.42449457140792"
    PX4_HOME_LON = "9.620245153463955"
    PX4_HOME_ALT = "20.0"

    # -------------------------------------------------------------
    # GAZEBO ENVIRONMENT VARIABLES
    # These tell Gazebo where to find models, worlds, and plugins
    # -------------------------------------------------------------
    prev_res = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = f"{(prev_res + ':') if prev_res else ''}{px4_gz_worlds}:{px4_gz_models}:{match_models_path}"

    prev_plugins = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    gz_plugin_path = f"{(prev_plugins + ':') if prev_plugins else ''}{px4_gz_plugins}"

    prev_config = os.environ.get('GZ_SIM_SERVER_CONFIG_PATH', '')
    gz_server_config_path = f"{(prev_config + ':') if prev_config else ''}{px4_gz_server_config}"

    # Print some info to verify paths (useful for debugging)
    print(f"PX4 Directory: {px4_dir}")
    print(f"GZ Resource Path: {gz_resource_path}")
    print(f"GZ Plugin Path: {gz_plugin_path}")
    print(f"GZ Server Config Path: {gz_server_config_path}")

    # -------------------------------------------------------------------------
    # --- 1) GAZEBO SERVER ---
    # -------------------------------------------------------------------------
    # Starts Gazebo (gz sim) in headless/server mode and loads the world file
    gazebo_server = ExecuteProcess(
        name="gazebo_server",
        cmd=[
            "bash", "-c",
            f"gz sim --verbose=1 -r -s {selected_world}"
        ],
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
    # The bridges connect Gazebo topics to ROS 2 topics.
    # Example: Gazebo /imu/data → ROS 2 /imu/data (same message format)

    # (A) Bridge for /clock and /tf (transforms)
    bridge_tf_clock = TimerAction(
        period=1.0,  # wait 1 second before starting
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

    # (B) Bridge for sensor data: LiDAR, IMU, odometry, etc.
    bridge_sensors = TimerAction(
        period=4.0,  # wait 4 seconds until Gazebo is ready
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
            remappings=[('/gz_scan', '/scan')],  # rename topic to /scan for SLAM
            parameters=[{'use_sim_time': True}],
            output='screen'
        )]
    )

    # -------------------------------------------------------------------------
    # --- 3) ROBOT STATE PUBLISHER ---
    # -------------------------------------------------------------------------
    # Publishes the robot’s URDF (joint transforms) to TF tree
    # It’s defined in a separate launch file
    rsp_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'robot_state_publisher.launch.py'])
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # -------------------------------------------------------------------------
    # --- 4) STATIC TF: map → camera_init ---
    # -------------------------------------------------------------------------
    # Publishes a fixed transform between two coordinate frames.
    # Here, map and camera_init are aligned.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )


        # -------------------------------------------------------------------------
    # ---  STATIC TF FIXES (NED↔ENU alignment and map↔odom connection) ---
    # -------------------------------------------------------------------------

    # 1️ Connect PX4's "odom" (NED world) to SLAM "map" (ENU world)
    #    180° rotation about X flips Z-down → Z-up.
    static_tf_odom_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_map_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],

        output='log'
    )

    # 2️ Ensure lidar sits correctly above base_link (10 cm up)
    static_tf_base_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'lidar_link'],
        output='log'
    )

    # -------------------------------------------------------------------------
    # --- 5) PX4 SITL (Software-In-The-Loop) ---
    # -------------------------------------------------------------------------
    # Launches the PX4 flight controller in simulation mode.
    px4_sitl = TimerAction(
        period=3.0,  # start after Gazebo initializes
        actions=[ExecuteProcess(
            name="px4_sitl",
            cmd=["./build/px4_sitl_default/bin/px4"],
            cwd=px4_dir,
            shell=True,
            output="screen",
            additional_env={
                "PX4_SYS_AUTOSTART": "4001",  # X500 base model
                "PX4_SIM_MODEL": "match_drohne",  # your custom drone
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
    # Custom debugging tools to fix TFs and bridges before SLAM starts.

    # Node 1: Fixes IMU/LiDAR frame alignment
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

    # Node 2: Bridges odometry frames (odom → base_link)
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
    # --- 6.5) LIVOX ROS DRIVER 2 (LiDAR + IMU SOURCE) ---
    # -------------------------------------------------------------------------
    # Starts the Livox driver to publish /livox/imu and /livox/lidar topics.
    # Uses parameters from slam_connect/config/livox_params.yaml.
    livox_params = PathJoinSubstitution([
        FindPackageShare('slam_connect'),
        'config', 'livox_params.yaml'
    ])

    livox_driver = TimerAction(
        period=5.8,  # start just before FAST-LIO2
        actions=[Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_ros_driver2_node',
            parameters=[livox_params],
            output='screen'
        )]
    )


    # -------------------------------------------------------------------------
    # --- 7) FAST-LIO2 SLAM NODE ---
    # -------------------------------------------------------------------------
    # Includes the separate launch file that starts the FAST-LIO2 SLAM system.
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
    # Opens the Gazebo graphical interface after the server starts.
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
    # MAVROS is the ROS 2 interface to PX4 (it bridges FCU <-> ROS topics).
    mavros_node = TimerAction(
        period=10.0,  # start after PX4 SITL is running
        actions=[ExecuteProcess(
            cmd=[
                "ros2", "launch", "mavros", "px4.launch",
                "fcu_url:=udp://:14540@"  # communication link to PX4
            ],
            output="screen"
        )]
    )

    # -------------------------------------------------------------------------
    # --- 10) RVIZ2 VISUALIZATION ---
    # -------------------------------------------------------------------------
    # Launch RViz with preconfigured visualization settings (LiDAR, odometry, etc.)
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
    # The list below defines the exact order in which each component starts.
    # TimerAction ensures that everything starts in a logical sequence.
    return LaunchDescription([
        gazebo_server,   # Start Gazebo physics world
        rsp_node,        # Publish robot model
        static_tf,       # Define static transforms
        static_tf_odom_map,  #  Connect map↔odom (fixes TF tree)
        static_tf_base_lidar,
        bridge_tf_clock, # Sync TF and clock
        gazebo_client,   # Open Gazebo GUI
        px4_sitl,        # Start PX4 flight controller
        bridge_sensors,  # Bridge Gazebo sensors to ROS
        slam_fixers,     # Fix IMU/LiDAR frames
        odom_bridge,     # Bridge odometry frames
        livox_driver,    # Livox LiDAR + IMU source
        fast_lio2,       # Start SLAM
        rviz_node,       # Open visualization
        mavros_node,     # Connect PX4 <-> ROS via MAVROS
    ])
