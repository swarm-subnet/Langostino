#!/usr/bin/env python3
"""
Launch file for FC Full Loop Simulator

This launches the complete closed-loop simulation including fc_adapter_node:
1. fc_full_loop_simulator_node - Simulates drone + INAV + sensors
2. ai_adapter_node - Processes sensors → 131-D observation
3. ai_flight_node - PPO inference → actions
4. fc_adapter_node - PID control → RC commands → back to simulator

This tests the REAL control loop including PID interactions and ALT HOLD mode.

Usage:
    ros2 launch swarm_ai_integration fc_loop_sim_launch.py

To disable ALT HOLD simulation:
    ros2 launch swarm_ai_integration fc_loop_sim_launch.py enable_althold:=false

To test with different PID gains:
    ros2 launch swarm_ai_integration fc_loop_sim_launch.py kp_z:=20.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Package paths
    pkg_share = FindPackageShare('swarm_ai_integration')
    config_file = PathJoinSubstitution([pkg_share, 'config', 'swarm_params.yaml'])

    # Launch arguments
    enable_althold_arg = DeclareLaunchArgument(
        'enable_althold',
        default_value='true',
        description='Enable ALT HOLD mode simulation (true/false)'
    )

    kp_z_arg = DeclareLaunchArgument(
        'kp_z',
        default_value='100.0',
        description='Z-axis PID proportional gain'
    )

    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='5.0',
        description='Goal X position in ENU (meters)'
    )

    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='5.0',
        description='Goal Y position in ENU (meters)'
    )

    goal_z_arg = DeclareLaunchArgument(
        'goal_z',
        default_value='3.0',
        description='Goal Z position in ENU (meters)'
    )

    # Launch configurations
    enable_althold = LaunchConfiguration('enable_althold')
    kp_z = LaunchConfiguration('kp_z')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_z = LaunchConfiguration('goal_z')

    # 1. FC Full Loop Simulator (replaces fc_comms)
    fc_simulator_node = Node(
        package='swarm_ai_integration',
        executable='fc_full_loop_simulator_node.py',
        name='fc_full_loop_simulator',
        output='screen',
        parameters=[{
            'physics_rate': 100.0,
            'gps_publish_rate': 1.5,
            'imu_publish_rate': 50.0,
            'sensor_publish_rate': 10.0,
            'lidar_publish_rate': 100.0,
            'start_lat': 37.2486500,
            'start_lon': -5.3724274,
            'start_alt': 182.0,
            'relative_start_enu': [0.0, 0.0, 3.0],
            'goal_relative_enu': [goal_x, goal_y, goal_z],
            'enable_althold_simulation': enable_althold,
            'althold_kp': 50.0,
            'max_climb_rate': 3.0,
            'motor_time_constant': 0.1,
            'drag_coefficient': 0.5,
            'mass_kg': 1.5,
            'gps_noise_std': 2.0,
            'baro_drift_rate': 0.01,
            'velocity_noise_std': 0.1,
            'lidar_min_range': 0.05,
            'lidar_max_range': 20.0,
            'enable_logging': True,
        }]
    )

    # 2. AI Adapter Node (sensor fusion)
    ai_adapter_node = Node(
        package='swarm_ai_integration',
        executable='ai_adapter_node.py',
        name='ai_adapter_node',
        output='screen',
        parameters=[config_file]
    )

    # 3. AI Flight Node (PPO inference)
    ai_flight_node = Node(
        package='swarm_ai_integration',
        executable='ai_flight_node.py',
        name='ai_flight_node',
        output='screen',
        parameters=[config_file]
    )

    # 4. FC Adapter Node (PID velocity control)
    fc_adapter_node = Node(
        package='swarm_ai_integration',
        executable='fc_adapter_node.py',
        name='fc_adapter_node',
        output='screen',
        parameters=[
            config_file,
            {
                'kp_z': kp_z,  # Allow override via launch arg
                'prearm_enabled': False,  # Skip pre-arm in simulation
                'startup_delay_sec': 2.0,  # Shorter delay
            }
        ]
    )

    return LaunchDescription([
        # Arguments
        enable_althold_arg,
        kp_z_arg,
        goal_x_arg,
        goal_y_arg,
        goal_z_arg,

        # Nodes
        fc_simulator_node,
        ai_adapter_node,
        ai_flight_node,
        fc_adapter_node,
    ])
