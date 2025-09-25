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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # Declare launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='Path to the trained AI model file (.zip)'
    )

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cpu',
        description='Device to run AI model on (cpu or cuda)'
    )

    enable_safety_arg = DeclareLaunchArgument(
        'enable_safety',
        default_value='true',
        description='Enable safety monitoring'
    )

    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='3.0',
        description='Maximum allowed velocity (m/s)'
    )

    max_altitude_arg = DeclareLaunchArgument(
        'max_altitude',
        default_value='50.0',
        description='Maximum allowed altitude (m)'
    )

    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='Enable debug logging'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for flight controller communication'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )

    front_lidar_port_arg = DeclareLaunchArgument(
        'front_lidar_port',
        default_value='/dev/ttyAMA0',
        description='Serial port for front LiDAR sensor'
    )

    down_lidar_port_arg = DeclareLaunchArgument(
        'down_lidar_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for down-facing LiDAR sensor'
    )

    lidar_baud_rate_arg = DeclareLaunchArgument(
        'lidar_baud_rate',
        default_value='921600',
        description='Baud rate for LiDAR sensors'
    )

    # Black Box Recorder arguments
    log_directory_arg = DeclareLaunchArgument(
        'log_directory',
        default_value='/var/log/swarm_blackbox',
        description='Directory for black box log files'
    )

    enable_blackbox_arg = DeclareLaunchArgument(
        'enable_blackbox',
        default_value='true',
        description='Enable black box flight data recorder'
    )

    # AI Adapter Node - converts sensor data to observation array
    ai_adapter_node = Node(
        package='swarm_ai_integration',
        executable='ai_adapter_node.py',
        name='ai_adapter_node',
        output='screen',
        parameters=[{
            'max_ray_distance': 10.0,
            'update_rate': 30.0,
            'debug_mode': LaunchConfiguration('debug_mode')
        }],
        remappings=[
            ('/lidar_scan', '/lidar/scan'),
            ('/imu/data', '/imu/data'),
            ('/gps/fix', '/gps/fix'),
            ('/fc/state', '/mavros/local_position/velocity_local'),
            ('/fc/rpm', '/fc/motor_rpm'),
            ('/goal_pose', '/move_base_simple/goal'),
            ('/front_lidar/lidar_distance_front', '/front_lidar/lidar_distance_front'),
            ('/down_lidar/lidar_distance_down', '/down_lidar/lidar_distance_down')
        ]
    )

    # AI Flight Node - executes the AI model
    ai_flight_node = Node(
        package='swarm_ai_integration',
        executable='ai_flight_node.py',
        name='ai_flight_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'device': LaunchConfiguration('device'),
            'prediction_timeout': 0.1,
            'max_velocity': LaunchConfiguration('max_velocity'),
            'safety_enabled': LaunchConfiguration('enable_safety')
        }],
        remappings=[
            ('/ai/action', '/cmd_vel')
        ]
    )

    # Safety Monitor Node - monitors system safety
    safety_monitor_node = Node(
        package='swarm_ai_integration',
        executable='safety_monitor_node.py',
        name='safety_monitor_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_safety')),
        parameters=[{
            'max_altitude': LaunchConfiguration('max_altitude'),
            'min_altitude': 0.5,
            'max_velocity': LaunchConfiguration('max_velocity'),
            'min_battery_voltage': 14.0,
            'max_distance_from_home': 100.0,
            'obstacle_danger_distance': 1.0,
            'communication_timeout': 2.0
        }],
        remappings=[
            ('/drone/pose', '/mavros/local_position/pose'),
            ('/drone/velocity', '/mavros/local_position/velocity_local'),
            ('/battery_state', '/mavros/battery')
        ]
    )

    # FC Communications Node - MSP interface to INAV
    fc_comms_node = Node(
        package='swarm_ai_integration',
        executable='fc_comms_node.py',
        name='fc_comms_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'timeout': 1.0,
            'reconnect_interval': 5.0,
            'telemetry_rate': 10.0,
            'heartbeat_rate': 1.0
        }],
        remappings=[
            ('/fc/imu_raw', '/imu/data'),
            ('/fc/gps_fix', '/gps/fix'),
            ('/fc/attitude', '/fc/attitude'),
            ('/fc/status', '/fc/status'),
            ('/fc/battery', '/mavros/battery'),
            ('/fc/motor_rpm', '/fc/motor_rpm')
        ]
    )

    # FC Adapter Node - VEL to MSP command translation
    fc_adapter_node = Node(
        package='swarm_ai_integration',
        executable='fc_adapter_node.py',
        name='fc_adapter_node',
        output='screen',
        parameters=[{
            'max_velocity': LaunchConfiguration('max_velocity'),
            'max_yaw_rate': 180.0,
            'max_acceleration': 3.0,
            'command_timeout': 1.0,
            'rate_limit_enabled': True,
            'smooth_commands': True,
            'safety_checks_enabled': LaunchConfiguration('enable_safety'),
            'auto_arm': False,
            'failsafe_mode': 'hover'
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/safety/override', '/safety/override'),
            ('/fc/attitude', '/fc/attitude'),
            ('/manual_control', '/joy'),
            ('/fc/rc_override', '/fc/rc_override'),
            ('/fc/msp_command', '/fc/msp_command')
        ]
    )

    # LiDAR Reader Nodes - Front and Down sensors
    front_lidar_node = Node(
        package='swarm_ai_integration',
        executable='lidar_reader_node.py',
        name='lidar_reader_front',
        namespace='front_lidar',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('front_lidar_port'),
            'baud_rate': LaunchConfiguration('lidar_baud_rate'),
            'publish_rate': 100.0,
            'frame_id': 'front_lidar_link',
            'sensor_position': 'front',
            'max_range': 50.0,
            'min_range': 0.05,
            'field_of_view': 0.035,
            'enable_filtering': True,
            'filter_window_size': 5
        }],
        remappings=[
            ('lidar_distance', 'lidar_distance_front'),
            ('lidar_raw', 'lidar_raw_front'),
            ('lidar_status', 'lidar_status_front'),
            ('lidar_point', 'lidar_point_front'),
            ('lidar_healthy', 'lidar_healthy_front')
        ]
    )

    down_lidar_node = Node(
        package='swarm_ai_integration',
        executable='lidar_reader_node.py',
        name='lidar_reader_down',
        namespace='down_lidar',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('down_lidar_port'),
            'baud_rate': LaunchConfiguration('lidar_baud_rate'),
            'publish_rate': 100.0,
            'frame_id': 'down_lidar_link',
            'sensor_position': 'down',
            'max_range': 50.0,
            'min_range': 0.05,
            'field_of_view': 0.035,
            'enable_filtering': True,
            'filter_window_size': 5
        }],
        remappings=[
            ('lidar_distance', 'lidar_distance_down'),
            ('lidar_raw', 'lidar_raw_down'),
            ('lidar_status', 'lidar_status_down'),
            ('lidar_point', 'lidar_point_down'),
            ('lidar_healthy', 'lidar_healthy_down')
        ]
    )

    # Black Box Recorder Node - logs all flight data
    black_box_recorder_node = Node(
        package='swarm_ai_integration',
        executable='black_box_recorder_node.py',
        name='black_box_recorder',
        output='screen',
        parameters=[{
            'log_directory': LaunchConfiguration('log_directory'),
            'max_file_size_mb': 100,
            'max_files_per_session': 20,
            'compress_old_files': True,
            'log_level': 'INFO',
            'buffer_size': 1000,
            'flush_interval': 5.0
        }],
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
        front_lidar_port_arg,
        down_lidar_port_arg,
        lidar_baud_rate_arg,
        log_directory_arg,
        enable_blackbox_arg,

        # Launch info
        launch_info,

        # Nodes
        ai_adapter_node,
        ai_flight_node,
        safety_monitor_node,
        fc_comms_node,
        fc_adapter_node,
        front_lidar_node,
        down_lidar_node,
        black_box_recorder_node,
    ])