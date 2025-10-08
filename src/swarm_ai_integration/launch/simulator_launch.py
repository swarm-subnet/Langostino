#!/usr/bin/env python3
"""
Launch file for AI Adapter Simulator Node

This launch file starts the simulator node for testing AI models without hardware.

Usage:
    ros2 launch swarm_ai_integration simulator_launch.py

    # With custom goal (MUST be in quotes and comma-separated)
    ros2 launch swarm_ai_integration simulator_launch.py goal_position:='[10.0,10.0,3.0]'

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
    goal_position_arg = DeclareLaunchArgument(
        'goal_position',
        default_value='[5.0, 5.0, 3.0]',
        description='Goal position [E, N, U] in meters'
    )

    start_position_arg = DeclareLaunchArgument(
        'start_position',
        default_value='[0.0, 0.0, 3.0]',
        description='Start position [E, N, U] in meters'
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
            'meters_per_step': LaunchConfiguration('step_size'),
            'telemetry_rate': LaunchConfiguration('telemetry_rate'),
            'physics_rate': LaunchConfiguration('physics_rate'),
        }],
        # Use remapping or command line args for arrays since they don't work well in parameters
        arguments=['--ros-args',
                   '-p', 'goal_relative_enu:=' + LaunchConfiguration('goal_position'),
                   '-p', 'relative_start_enu:=' + LaunchConfiguration('start_position')]
    )

    return LaunchDescription([
        goal_position_arg,
        start_position_arg,
        step_size_arg,
        telemetry_rate_arg,
        physics_rate_arg,
        simulator_node
    ])
