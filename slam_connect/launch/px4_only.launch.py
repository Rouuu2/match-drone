#!/usr/bin/env python3
"""
px4_only.launch.py
---------------------------------
Runs PX4 SITL with Gazebo Harmonic and MAVROS.
Use this to test drone spawning, PX4 parameters, and QGroundControl link
without running SLAM or any extra nodes.
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # --- Paths / config ---
    px4_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../../src/PX4-Autopilot'))
    match_models_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../../src/match_models'))
    px4_gz_models = f"{px4_dir}/Tools/simulation/gz/models"
    px4_gz_worlds = f"{px4_dir}/Tools/simulation/gz/worlds"
    px4_gz_plugins = f"{px4_dir}/build/px4_sitl_default/src/modules/simulation/gz_plugins"
    px4_gz_server_config = f"{px4_dir}/src/modules/simulation/gz_bridge/server.config"
    selected_world = f"{px4_dir}/Tools/simulation/gz/worlds/forest.sdf"

    PX4_HOME_LAT = "52.42449457140792"
    PX4_HOME_LON = "9.620245153463955"
    PX4_HOME_ALT = "20.0"

    prev_res = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = f"{(prev_res + ':') if prev_res else ''}{px4_gz_worlds}:{px4_gz_models}:{match_models_path}"
    prev_plugins = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    gz_plugin_path = f"{(prev_plugins + ':') if prev_plugins else ''}{px4_gz_plugins}"

    # 1) Gazebo server - start immediately (no drone in SDF!)
    gazebo_server = ExecuteProcess(
        name="gazebo_server",
        cmd=["bash", "-c", f"gz sim --verbose=1 {selected_world}"],
        additional_env={
            "PX4_GZ_MODELS": px4_gz_models,
            "PX4_GZ_WORLDS": px4_gz_worlds,
            "PX4_GZ_PLUGINS": px4_gz_plugins,
            "PX4_GZ_SERVER_CONFIG": px4_gz_server_config,
            "GZ_SIM_RESOURCE_PATH": gz_resource_path,
            "GZ_SIM_SYSTEM_PLUGIN_PATH": gz_plugin_path,
        },
        output="screen"
    )

    # 2) Optional GUI
    gazebo_client = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            name="gazebo_client",
            cmd=["gz", "sim", "-g"],
            output="screen"
        )]
    )

    # 3) PX4 SITL - after Gazebo is up, spawns drone automatically!
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

    enable_broadcast = ExecuteProcess(
        cmd=["bash", "-c", "sleep 5 && px4-params set MAV_0_BROADCAST 1"],
        shell=True,
        output="screen",
    )

    # 4) robot_state_publisher for TF (not for spawning!)
    rsp_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('slam_connect'), 'launch', 'robot_state_publisher.launch.py'])
        )
    )

    # 5) MAVROS (after PX4 SITL is up)
    mavros_node = TimerAction(
        period=12.0,
        actions=[ExecuteProcess(
            cmd=[
                "ros2", "launch", "mavros", "px4.launch",
                "fcu_url:=udp://:14540@"
            ],
            output="screen"
        )]
    )

    return LaunchDescription([
        gazebo_server,
        gazebo_client,   # Remove if you want headless only
        px4_sitl,
        enable_broadcast,
        rsp_node,
        mavros_node,
    ])