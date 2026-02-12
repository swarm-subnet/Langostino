Here are the exact step‑by‑step instructions (1 by 1) to run Langostino autonomously, including the INAV waypoint and GPS check.


Previous:

0. TEST GPS
1. Test manual flight (modes)

0) INAV setup (via USB cable)

Connect the flight controller to your laptop with USB.
Open INAV Configurator.
Connect to the FC.
Go to Mission Control / Mission Planner.
Create at least 1 waypoint (the system reads WP #1).
Save/Upload the mission to the FC.
Disconnect USB.
1) Power and GPS

Power the drone (battery connected).
Wait for GPS lock.
On the Pi, check satellites:
ros2 topic echo /fc/gps_satellites
Make sure it shows 5 or more satellites (default requirement).
2) Start LiDAR

pm2 start "ros2 run swarm_ai_integration lidar_reader_node.py" --name lidar_reader
What it does: reads LiDAR over I2C (addr 0x08) and publishes /lidar_distance.

3) Start FC communications (MSP)

pm2 start "ros2 run swarm_ai_integration fc_comms_node.py" --name fc_comms
What it does: connects to /dev/ttyAMA0, publishes FC telemetry, and sends RC overrides.

4) Start AI adapter

pm2 start "ros2 run swarm_ai_integration ai_adapter_node.py" --name ai_adapter
What it does: builds the 131‑D AI observation from telemetry + LiDAR.

5) Start AI flight model

pm2 start "ros2 run swarm_ai_integration ai_flight_node.py --ros-args -p model_path:=/home/pi/swarm-ros/model/UID_3.zip" --name ai_flight
What it does: runs inference at 10 Hz and publishes /ai/action.

6) Start FC adapter (AI → RC)

pm2 start "ros2 run swarm_ai_integration fc_adapter_node.py" --name fc_adapter
What it does: converts AI actions to RC overrides and arms/rises/yaw‑aligns.

7) Start black box recorder

pm2 start "ros2 run swarm_ai_integration black_box_recorder_node.py --ros-args -p log_directory:=~/flight-logs" --name black_box
What it does: logs all key topics to JSONL for analysis.

8) Start safety monitor

pm2 start "ros2 run swarm_ai_integration safety_monitor_node.py" --name safety_monitor
What it does: monitors attitude/altitude/geofence and can trigger /safety/override.

