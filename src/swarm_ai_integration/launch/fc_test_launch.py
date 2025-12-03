#!/usr/bin/env python3
"""
Launch file for FC Adapter Testing

This launch file starts the necessary nodes for testing flight controller interface:
- FC Comms Node: MSP communication with INAV flight controller
- FC Adapter Node: Translates /ai/action to RC override commands (joystick mode)
- Test Motion Sim Node: Generates test motion commands

Usage:
    ros2 launch swarm_ai_integration fc_test_launch.py
    ros2 launch swarm_ai_integration fc_test_launch.py serial_port:=/dev/ttyUSB0
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Load swarm_params.yaml to get default values
    package_dir = get_package_share_directory('swarm_ai_integration')
    params_file = os.path.join(package_dir, 'config', 'swarm_params.yaml')

    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)

    # Extract default values from YAML
    fc_comms_params = params['fc_comms_node']['ros__parameters']
    fc_adapter_params = params['fc_adapter_node']['ros__parameters']

    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value=str(fc_comms_params['serial_port']),
        description='Serial port for flight controller communication'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value=str(fc_comms_params['baud_rate']),
        description='Baud rate for serial communication'
    )

    control_rate_arg = DeclareLaunchArgument(
        'control_rate_hz',
        default_value=str(fc_adapter_params.get('control_rate_hz', 40.0)),
        description='Control loop frequency (Hz)'
    )

    update_rate_arg = DeclareLaunchArgument(
        'test_update_rate_hz',
        default_value='2.0',
        description='Test sequence update rate (Hz)'
    )

    # FC Communications Node - MSP interface to INAV
    fc_comms_node = Node(
        package='swarm_ai_integration',
        executable='fc_comms_node.py',
        name='fc_comms_node',
        output='screen',
        parameters=[
            params_file,  # Load all parameters from YAML
            {
                # Override with launch arguments
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate')
            }
        ]
    )

    # FC Adapter Node - Joystick Mode (translates /ai/action to RC)
    fc_adapter_node = Node(
        package='swarm_ai_integration',
        executable='fc_adapter_node.py',
        name='fc_adapter_node',
        output='screen',
        parameters=[
            params_file,  # Load all parameters from YAML
            {
                # Override with launch arguments
                'control_rate_hz': LaunchConfiguration('control_rate_hz')
            }
        ]
    )

    # Test Motion Simulator Node - Generates test commands
    test_motion_sim_node = Node(
        package='swarm_ai_integration',
        executable='test_motion_sim_node.py',
        name='test_motion_sim_node',
        output='screen',
        parameters=[{
            'update_rate_hz': LaunchConfiguration('test_update_rate_hz'),
            'test_sequence_enabled': True,
            'hold_duration_sec': 5.0
        }]
    )

    # Log info about the launch
    launch_info = LogInfo(
        msg=[
            'Starting FC Adapter Test System\n',
            '  Serial Port: ', LaunchConfiguration('serial_port'), '\n',
            '  Baud Rate: ', LaunchConfiguration('baud_rate'), '\n',
            '  Control Rate: ', LaunchConfiguration('control_rate_hz'), ' Hz\n',
            '  Test Update Rate: ', LaunchConfiguration('test_update_rate_hz'), ' Hz\n'
        ]
    )

    return LaunchDescription([
        # Launch arguments
        serial_port_arg,
        baud_rate_arg,
        control_rate_arg,
        update_rate_arg,

        # Launch info
        launch_info,

        # Nodes
        fc_comms_node,
        fc_adapter_node,
        test_motion_sim_node,
    ])
