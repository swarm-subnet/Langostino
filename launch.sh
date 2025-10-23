echo "-------------------------------"
echo "Starting ROS2 launch for Swarm AI Integration"
echo "-------------------------------"

pm2 start "ros2 launch swarm_ai_integration swarm_ai_launch.py" --name swarm-ros-launched

echo "-------------------------------"
echo "ROS2 launch command has been executed."
echo "-------------------------------"
