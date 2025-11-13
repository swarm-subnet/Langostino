#!/usr/bin/env python3
"""
Launch file for Flight Test with Black Box Recording

This launches:
1. flight_test_node - Executes hardcoded flight test sequence
2. black_box_recorder_node - Records all RC commands and telemetry
3. (Optional) fc_comms_node - If you want live FC communication

The flight test will execute the following sequence:
- 3 second startup delay
- Arm + setup flight modes (Angle mode + Alt Hold)
- Throttle up to 1520 for 2 seconds
- Roll right to 1520 for 2 seconds, then center
- Roll left to 1480 for 2 seconds, then center
- Throttle down to 1480 for 2 seconds
- Disarm and complete

All commands are logged to ~/swarm-ros/flight-logs/

Usage:
    ros2 launch swarm_ai_integration flight_test_launch.py

To change phase duration:
    ros2 launch swarm_ai_integration flight_test_launch.py phase_duration:=3.0

To change startup delay:
    ros2 launch swarm_ai_integration flight_test_launch.py startup_delay:=5.0

To include FC communications (for real drone):
    ros2 launch swarm_ai_integration flight_test_launch.py use_fc_comms:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package paths
    pkg_share = FindPackageShare('swarm_ai_integration')
    config_file = PathJoinSubstitution([pkg_share, 'config', 'swarm_params.yaml'])

    # Launch arguments
    phase_duration_arg = DeclareLaunchArgument(
        'phase_duration',
        default_value='2.0',
        description='Duration of each flight action in seconds'
    )

    arm_duration_arg = DeclareLaunchArgument(
        'arm_duration',
        default_value='10.0',
        description='Duration for ARM phase in seconds (minimum 5-10 seconds recommended)'
    )

    startup_delay_arg = DeclareLaunchArgument(
        'startup_delay',
        default_value='3.0',
        description='Delay before starting flight sequence in seconds'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='40.0',
        description='RC command publish rate in Hz'
    )

    use_fc_comms_arg = DeclareLaunchArgument(
        'use_fc_comms',
        default_value='false',
        description='Launch FC communications node for real drone (true/false)'
    )

    # Flight Test Node
    flight_test_node = Node(
        package='swarm_ai_integration',
        executable='flight_test_node.py',
        name='flight_test_node',
        output='screen',
        parameters=[{
            'phase_duration_sec': LaunchConfiguration('phase_duration'),
            'arm_duration_sec': LaunchConfiguration('arm_duration'),
            'startup_delay_sec': LaunchConfiguration('startup_delay'),
            'publish_rate_hz': LaunchConfiguration('publish_rate'),
        }],
        emulate_tty=True,
    )

    # Black Box Recorder Node
    black_box_node = Node(
        package='swarm_ai_integration',
        executable='black_box_recorder_node.py',
        name='black_box_recorder_node',
        output='screen',
        parameters=[config_file],
        emulate_tty=True,
    )

    # FC Communications Node (optional - for real drone)
    fc_comms_node = Node(
        package='swarm_ai_integration',
        executable='fc_comms_node.py',
        name='fc_comms_node',
        output='screen',
        parameters=[config_file],
        condition=IfCondition(LaunchConfiguration('use_fc_comms')),
        emulate_tty=True,
    )

    return LaunchDescription([
        # Arguments
        phase_duration_arg,
        arm_duration_arg,
        startup_delay_arg,
        publish_rate_arg,
        use_fc_comms_arg,

        # Nodes
        flight_test_node,
        black_box_node,
        fc_comms_node,
    ])
