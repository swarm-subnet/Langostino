#!/usr/bin/env python3
"""
Launch file for Swarm AI Integration system

This launch file starts all the necessary nodes for real-world AI flight control:
- AI Adapter Node: Converts sensor data to 131-D observation array
- AI Flight Node: Executes the AI model and generates control commands
- Safety Monitor Node: Monitors system safety and triggers failsafe

Usage:
    ros2 launch swarm_ai_integration swarm_ai_launch.py model_path:=/path/to/model.zip
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Load swarm_params.yaml to get default values
    package_dir = get_package_share_directory('swarm_ai_integration')
    params_file = os.path.join(package_dir, 'config', 'swarm_params.yaml')

    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)

    # Extract default values from YAML
    ai_adapter_params = params['ai_adapter_node']['ros__parameters']
    ai_flight_params = params['ai_flight_node']['ros__parameters']
    safety_params = params['safety_monitor_node']['ros__parameters']
    fc_comms_params = params['fc_comms_node']['ros__parameters']
    fc_adapter_params = params['fc_adapter_node']['ros__parameters']
    down_lidar_params = params['down_lidar/lidar_reader_down']['ros__parameters']
    blackbox_params = params['black_box_recorder_node']['ros__parameters']

    # Declare launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=str(ai_flight_params['model_path']),
        description='Path to the trained AI model file (.zip)'
    )

    device_arg = DeclareLaunchArgument(
        'device',
        default_value=str(ai_flight_params['device']),
        description='Device to run AI model on (cpu or cuda)'
    )

    enable_safety_arg = DeclareLaunchArgument(
        'enable_safety',
        default_value=str(ai_flight_params['safety_enabled']).lower(),
        description='Enable safety monitoring'
    )

    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value=str(fc_adapter_params['max_velocity']),
        description='Maximum allowed velocity (m/s)'
    )

    max_altitude_arg = DeclareLaunchArgument(
        'max_altitude',
        default_value=str(safety_params['max_altitude']),
        description='Maximum allowed altitude (m)'
    )

    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value=str(ai_adapter_params['debug_mode']).lower(),
        description='Enable debug logging'
    )

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

    # front_lidar_i2c_bus_arg = DeclareLaunchArgument(
    #     'front_lidar_i2c_bus',
    #     default_value='1',
    #     description='I2C bus number for front LiDAR sensor'
    # )

    # front_lidar_i2c_address_arg = DeclareLaunchArgument(
    #     'front_lidar_i2c_address',
    #     default_value='8',
    #     description='I2C address for front LiDAR sensor (decimal)'
    # )

    down_lidar_i2c_bus_arg = DeclareLaunchArgument(
        'down_lidar_i2c_bus',
        default_value=str(down_lidar_params['i2c_bus']),
        description='I2C bus number for down-facing LiDAR sensor'
    )

    down_lidar_i2c_address_arg = DeclareLaunchArgument(
        'down_lidar_i2c_address',
        default_value=str(down_lidar_params['i2c_address']),
        description='I2C address for down-facing LiDAR sensor (decimal)'
    )

    # Black Box Recorder arguments
    log_directory_arg = DeclareLaunchArgument(
        'log_directory',
        default_value=str(blackbox_params['log_directory']),
        description='Directory for black box log files'
    )

    enable_blackbox_arg = DeclareLaunchArgument(
        'enable_blackbox',
        default_value='true',  # Not in YAML, keeping as default
        description='Enable black box flight data recorder'
    )

    # LiDAR Reader Nodes - Front and Down sensors
    # front_lidar_node = Node(
    #     package='swarm_ai_integration',
    #     executable='lidar_reader_node.py',
    #     name='lidar_reader_front',
    #     namespace='front_lidar',
    #     output='screen',
    #     parameters=[{
    #         'i2c_bus': LaunchConfiguration('front_lidar_i2c_bus'),
    #         'i2c_address': LaunchConfiguration('front_lidar_i2c_address'),
    #         'distance_register': 0x24,
    #         'publish_rate': 100.0,
    #         'frame_id': 'front_lidar_link',
    #         'sensor_position': 'front',
    #         'max_range': 50.0,
    #         'min_range': 0.05,
    #         'field_of_view': 0.035,
    #         'enable_filtering': True,
    #         'filter_window_size': 5
    #     }],
    #     remappings=[
    #         ('lidar_distance', 'lidar_distance_front'),
    #         ('lidar_raw', 'lidar_raw_front'),
    #         ('lidar_status', 'lidar_status_front'),
    #         ('lidar_point', 'lidar_point_front'),
    #         ('lidar_healthy', 'lidar_healthy_front')
    #     ]
    # )

    down_lidar_node = Node(
        package='swarm_ai_integration',
        executable='lidar_reader_node.py',
        name='lidar_reader_down',
        namespace='down_lidar',
        output='screen',
        parameters=[
            params_file,  # Load all parameters from YAML
            {
                # Override with launch arguments
                'i2c_bus': LaunchConfiguration('down_lidar_i2c_bus'),
                'i2c_address': LaunchConfiguration('down_lidar_i2c_address')
            }
        ],
        remappings=[
            ('lidar_distance', 'lidar_distance_down'),
            ('lidar_raw', 'lidar_raw_down'),
            ('lidar_status', 'lidar_status_down'),
            ('lidar_point', 'lidar_point_down'),
            ('lidar_healthy', 'lidar_healthy_down')
        ]
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
        ],
        remappings=[
            ('/fc/imu_raw', '/imu/data'),
            ('/fc/gps_fix', '/gps/fix'),
            ('/fc/status', '/fc/status'),
            ('/fc/battery', '/mavros/battery'),
            ('/fc/motor_rpm', '/fc/motor_rpm')
        ]
    )

    # AI Adapter Node - converts sensor data to observation array
    ai_adapter_node = Node(
        package='swarm_ai_integration',
        executable='ai_adapter_node.py',
        name='ai_adapter_node',
        output='screen',
        parameters=[
            params_file,  # Load all parameters from YAML
            {
                # Override with launch arguments
                'debug_mode': LaunchConfiguration('debug_mode')
            }
        ],
        remappings=[
            ('/lidar_scan', '/lidar/scan'),
            ('/imu/data', '/imu/data'),
            ('/gps/fix', '/gps/fix'),
            ('/fc/state', '/mavros/local_position/velocity_local'),
            ('/fc/rpm', '/fc/motor_rpm'),
            ('/goal_pose', '/move_base_simple/goal'),
            # ('/front_lidar/lidar_distance_front', '/front_lidar/lidar_distance_front'),
            ('/down_lidar/lidar_distance_down', '/down_lidar/lidar_distance_down')
        ]
    )

    # AI Flight Node - executes the AI model (runs in Python venv via wrapper)
    ai_flight_node = Node(
        package='swarm_ai_integration',
        executable='ai_flight_node_wrapper.sh',
        name='ai_flight_node',
        output='screen',
        parameters=[
            params_file,  # Load all parameters from YAML
            {
                # Override with launch arguments
                'model_path': LaunchConfiguration('model_path'),
                'device': LaunchConfiguration('device'),
                'max_velocity': LaunchConfiguration('max_velocity'),
                'safety_enabled': LaunchConfiguration('enable_safety')
            }
        ]
    )

    # FC Adapter Node - VEL to MSP command translation
    fc_adapter_node = Node(
        package='swarm_ai_integration',
        executable='fc_adapter_node.py',
        name='fc_adapter_node',
        output='screen',
        parameters=[
            params_file,  # Load all parameters from YAML
            {
                # Override with launch arguments
                'max_velocity': LaunchConfiguration('max_velocity'),
                'safety_checks_enabled': LaunchConfiguration('enable_safety')
            }
        ]
    )

    # Safety Monitor Node - monitors system safety
    safety_monitor_node = Node(
        package='swarm_ai_integration',
        executable='safety_monitor_node.py',
        name='safety_monitor_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_safety')),
        parameters=[
            params_file,  # Load all parameters from YAML
            {
                # Override with launch arguments
                'max_altitude': LaunchConfiguration('max_altitude'),
                'max_velocity': LaunchConfiguration('max_velocity')
            }
        ],
        remappings=[
            ('/drone/pose', '/mavros/local_position/pose'),
            ('/drone/velocity', '/mavros/local_position/velocity_local'),
            ('/battery_state', '/mavros/battery')
        ]
    )

    # Black Box Recorder Node - logs all flight data
    black_box_recorder_node = Node(
        package='swarm_ai_integration',
        executable='black_box_recorder_node.py',
        name='black_box_recorder',
        output='screen',
        parameters=[
            params_file,  # Load all parameters from YAML
            {
                # Override with launch arguments
                'log_directory': LaunchConfiguration('log_directory')
            }
        ],
        condition=IfCondition(LaunchConfiguration('enable_blackbox'))
    )

    # Log info about the launch
    launch_info = LogInfo(
        msg=[
            'Starting Swarm AI Integration with model: ',
            LaunchConfiguration('model_path')
        ]
    )

    return LaunchDescription([
        # Launch arguments
        model_path_arg,
        device_arg,
        enable_safety_arg,
        max_velocity_arg,
        max_altitude_arg,
        debug_mode_arg,
        serial_port_arg,
        baud_rate_arg,
        # front_lidar_i2c_bus_arg,
        # front_lidar_i2c_address_arg,
        down_lidar_i2c_bus_arg,
        down_lidar_i2c_address_arg,
        log_directory_arg,
        enable_blackbox_arg,

        # Launch info
        launch_info,

        # Nodes
        #front_lidar_node,
        down_lidar_node,
        fc_comms_node,
        ai_adapter_node,
        ai_flight_node,
        fc_adapter_node,
        safety_monitor_node,
        black_box_recorder_node,
    ])