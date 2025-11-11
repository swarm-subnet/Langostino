### List of commands to run the whole system with launch or each node with pm2

```
pm2 start "ros2 launch swarm_ai_integration swarm_ai_launch.py" --name swarm_ai
```
  
```
pm2 start "ros2 run swarm_ai_integration lidar_reader_node.py" --name lidar_reader
```

```
pm2 start "ros2 run swarm_ai_integration fc_comms_node.py" --name fc_comms 
```
  
```
pm2 start "ros2 run swarm_ai_integration ai_adapter_node.py" --name ai_adapter 
```
  
```
pm2 start "ros2 run swarm_ai_integration ai_flight_node.py --ros-args -p model_path:=/home/pi/swarm-ros/model/UID_3.zip" --name ai_flight
```
 
```
pm2 start "ros2 run swarm_ai_integration fc_adapter_node.py" --name fc_adapter
```
  
```
pm2 start "ros2 run swarm_ai_integration black_box_recorder_node.py --ros-args -p log_directory:=~/swarm-ros/flight-logs" --name black_box
```