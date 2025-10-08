#!/usr/bin/env python3
"""
Launch file for AI Adapter Simulator Node

This launch file starts the simulator node for testing AI models without hardware.

Usage:
    ros2 launch swarm_ai_integration simulator_launch.py

    # With custom goal
    ros2 launch swarm_ai_integration simulator_launch.py goal_x:=10.0 goal_y:=10.0 goal_z:=3.0

    # With custom step size
    ros2 launch swarm_ai_integration simulator_launch.py step_size:=0.20
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for simulator node."""

    # Declare launch arguments
    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='5.0',
        description='Goal position X (East) in meters'
    )

    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='5.0',
        description='Goal position Y (North) in meters'
    )

    goal_z_arg = DeclareLaunchArgument(
        'goal_z',
        default_value='3.0',
        description='Goal position Z (Up) in meters'
    )

    step_size_arg = DeclareLaunchArgument(
        'step_size',
        default_value='0.15',
        description='Movement step size in meters (15cm default)'
    )

    telemetry_rate_arg = DeclareLaunchArgument(
        'telemetry_rate',
        default_value='30.0',
        description='Observation publishing rate in Hz'
    )

    physics_rate_arg = DeclareLaunchArgument(
        'physics_rate',
        default_value='10.0',
        description='Physics update rate in Hz'
    )

    # Simulator node
    simulator_node = Node(
        package='swarm_ai_integration',
        executable='ai_adapter_simulator_node.py',
        name='ai_adapter_simulator',
        output='screen',
        parameters=[{
            'goal_relative_enu': [
                LaunchConfiguration('goal_x'),
                LaunchConfiguration('goal_y'),
                LaunchConfiguration('goal_z')
            ],
            'meters_per_step': LaunchConfiguration('step_size'),
            'telemetry_rate': LaunchConfiguration('telemetry_rate'),
            'physics_rate': LaunchConfiguration('physics_rate'),
            'relative_start_enu': [0.0, 0.0, 3.0],
        }]
    )

    return LaunchDescription([
        goal_x_arg,
        goal_y_arg,
        goal_z_arg,
        step_size_arg,
        telemetry_rate_arg,
        physics_rate_arg,
        simulator_node
    ])
