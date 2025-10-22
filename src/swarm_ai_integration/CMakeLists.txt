cmake_minimum_required(VERSION 3.8)
project(swarm_ai_integration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  swarm_ai_integration/ai_adapter_node.py
  swarm_ai_integration/ai_adapter_simulator_node.py
  swarm_ai_integration/ai_adapter_simulator_multiple_node.py
  swarm_ai_integration/ai_flight_node.py
  swarm_ai_integration/safety_monitor_node.py
  swarm_ai_integration/fc_comms_node.py
  swarm_ai_integration/fc_adapter_node.py
  swarm_ai_integration/lidar_reader_node.py
  swarm_ai_integration/black_box_recorder_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()