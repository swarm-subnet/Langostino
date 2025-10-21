# Black Box Flight Data Recorder

The Black Box Recorder Node provides comprehensive flight data logging for the Swarm AI system, similar to aircraft black boxes. It records all critical system data including sensor inputs, AI model outputs, flight controller communications, and safety system status.

## Features

- **Comprehensive Logging**: Records all sensor data, AI decisions, and flight controller communications
- **Persistent Storage**: Log files survive system crashes and restarts
- **Automatic Rotation**: Creates new files when size limits are reached
- **Compression**: Automatically compresses old log files to save space
- **Real-time Performance**: Buffered logging doesn't impact flight performance
- **JSON Format**: Easy to parse and analyze with standard tools

## Data Logged

### Sensor Data
- LiDAR distance measurements and point clouds
- IMU data (acceleration, gyroscope, orientation)
- GPS coordinates and fix status
- Battery voltage, current, and status

### AI System
- 131-dimensional observation arrays fed to the AI model
- AI-generated control actions (velocity commands)
- Model status and diagnostics
- Processing timestamps and performance metrics

### Flight Controller Communications
- MSP commands sent to flight controller
- RC channel overrides
- Motor RPM data
- Flight controller telemetry
- Connection status

### Safety System
- Safety override triggers and reasons
- Emergency landing commands
- Failsafe actions
- System health monitoring

## Usage

### Basic Usage
The black box recorder is automatically started when using the main launch file:

```bash
ros2 launch swarm_ai_integration swarm_ai_launch.py model_path:=/path/to/model.zip
```

### Standalone Usage
To run only the black box recorder:

```bash
ros2 run swarm_ai_integration black_box_recorder_node.py
```

### Custom Configuration
```bash
ros2 run swarm_ai_integration black_box_recorder_node.py --ros-args \
    -p log_directory:=/custom/log/path \
    -p max_file_size_mb:=50 \
    -p compress_old_files:=true
```

### Disable Black Box (if needed)
```bash
ros2 launch swarm_ai_integration swarm_ai_launch.py \
    model_path:=/path/to/model.zip \
    enable_blackbox:=false
```

## Configuration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `log_directory` | `/var/log/swarm_blackbox` | Directory for log files |
| `max_file_size_mb` | `100` | Maximum file size before rotation (MB) |
| `max_files_per_session` | `20` | Maximum files per flight session |
| `compress_old_files` | `true` | Compress rotated files with gzip |
| `log_level` | `INFO` | Logging verbosity (DEBUG, INFO, WARN, ERROR) |
| `buffer_size` | `1000` | Messages to buffer before writing |
| `flush_interval` | `5.0` | Seconds between forced flushes to disk |

## Log File Structure

### Directory Layout
```
/var/log/swarm_blackbox/
├── session_20231201_143022/
│   ├── blackbox_20231201_143022_000.jsonl
│   ├── blackbox_20231201_143025_001.jsonl.gz
│   └── blackbox_20231201_143028_002.jsonl
└── session_20231201_150515/
    ├── blackbox_20231201_150515_000.jsonl
    └── blackbox_20231201_150520_001.jsonl.gz
```

### Log Entry Format
Each line is a JSON object:

```json
{
  "log_type": "AI_ACTION",
  "timestamp": 1701435022.123456,
  "utc_time": "2023-12-01T14:30:22.123456+00:00",
  "ros_timestamp": 1701435022.123,
  "data": {
    "linear": {"x": 1.5, "y": 0.2, "z": -0.1},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.3}
  },
  "correlated_observation": [0.1, 0.2, 0.3, ...]
}
```

## Log Analysis

### View Real-time Logs
```bash
tail -f /var/log/swarm_blackbox/session_*/blackbox_*.jsonl
```

### Extract Specific Message Types
```bash
grep '"log_type":"AI_ACTION"' blackbox_*.jsonl | jq .
```

### Python Analysis Example
```python
import json
import gzip
from pathlib import Path

def analyze_flight_data(session_dir):
    for log_file in Path(session_dir).glob("*.jsonl*"):
        if log_file.suffix == '.gz':
            file_handle = gzip.open(log_file, 'rt')
        else:
            file_handle = open(log_file, 'r')

        with file_handle as f:
            for line in f:
                entry = json.loads(line)
                if entry['log_type'] == 'AI_ACTION':
                    # Analyze AI decisions
                    print(f"AI Command: {entry['data']}")
```

## Performance Impact

The black box recorder is designed for minimal performance impact:
- **Buffered Logging**: Messages are batched before writing
- **Background Processing**: File I/O happens in separate threads
- **Efficient Serialization**: Optimized JSON encoding
- **Memory Management**: Automatic buffer overflow handling

Typical overhead: < 1% CPU, < 50MB RAM

## Troubleshooting

### Insufficient Permissions
If logs aren't being written, check directory permissions:
```bash
sudo mkdir -p /var/log/swarm_blackbox
sudo chown $USER:$USER /var/log/swarm_blackbox
```

### Disk Space Issues
Monitor disk usage, especially during long flights:
```bash
df -h /var/log
du -sh /var/log/swarm_blackbox/*
```

### Missing Log Entries
Check node status and error messages:
```bash
ros2 node info /black_box_recorder
ros2 topic echo /rosout | grep black_box
```

## Recovery and Analysis

In case of system failure, the black box logs provide complete flight reconstruction:

1. **Sensor State**: Reconstruct exact sensor readings at any timestamp
2. **AI Decisions**: Analyze what the AI model was "thinking" before incident
3. **Control Commands**: Track exactly what was sent to flight controller
4. **Safety Events**: Identify what triggered safety systems
5. **System Health**: Monitor performance leading up to issues

This data is invaluable for:
- Incident investigation
- System optimization
- AI model training data
- Safety system validation
- Performance analysis