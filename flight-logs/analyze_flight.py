#!/usr/bin/env python3
"""
Analyze flight logs to verify:
1. Movement correctness (AI actions vs GPS course)
2. Compass accuracy (attitude yaw vs GPS course)
3. Coordinate frame alignment (vx=East, vy=North)
"""

import json
import numpy as np
from collections import defaultdict

def parse_array(data_str):
    """Parse array string from blackbox log"""
    # Remove "array('f', [" and "])" and split
    data_str = data_str.replace("array('f', [", "").replace("])", "")
    return [float(x) for x in data_str.split(", ")]

def analyze_blackbox(filepath):
    """Analyze blackbox JSONL file"""

    ai_actions = []
    gps_courses = []
    attitude_eulers = []
    positions = []

    with open(filepath, 'r') as f:
        for line in f:
            try:
                log = json.loads(line.strip())
                log_type = log.get('log_type')
                timestamp = log.get('timestamp', 0)

                if log_type == 'AI_ACTION':
                    # AI action: [vx, vy, vz, speed]
                    action_data = parse_array(log['data']['data'])
                    ai_actions.append({
                        'timestamp': timestamp,
                        'vx': action_data[0],
                        'vy': action_data[1],
                        'vz': action_data[2],
                        'speed': action_data[3]
                    })

                elif log_type == 'FC_GPS_SPEED_COURSE':
                    # GPS: [speed, course]
                    gps_data = parse_array(log['data']['data'])
                    gps_courses.append({
                        'timestamp': timestamp,
                        'speed': gps_data[0],
                        'course': gps_data[1]  # degrees, 0=North, 90=East
                    })

                elif log_type == 'FC_ATTITUDE_EULER':
                    # Attitude: [roll, pitch, yaw]
                    att_data = parse_array(log['data']['data'])
                    attitude_eulers.append({
                        'timestamp': timestamp,
                        'roll': att_data[0],
                        'pitch': att_data[1],
                        'yaw': att_data[2]  # degrees
                    })

                elif log_type == 'AI_OBSERVATION_DEBUG':
                    # Position: [x, y, z, yaw, altitude, ?]
                    obs_data = parse_array(log['data']['data'])
                    positions.append({
                        'timestamp': timestamp,
                        'x': obs_data[0],  # East in ENU
                        'y': obs_data[1],  # North in ENU
                        'z': obs_data[2],  # Up in ENU
                        'yaw': obs_data[3]
                    })

            except Exception as e:
                continue

    return {
        'ai_actions': ai_actions,
        'gps_courses': gps_courses,
        'attitude_eulers': attitude_eulers,
        'positions': positions
    }

def analyze_ai_output(filepath):
    """Analyze AI output text files (ai-1.txt, etc)"""
    actions = []

    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if '/ai/action ->' in line:
                # Parse: ... /ai/action -> vx=+0.859, vy=-1.000, vz=+0.179, speed=+0.289 ...
                # Extract the part after '->'
                parts = line.split('->')[1].split('[')[0].strip()
                # Parse: vx=+0.859, vy=-1.000, vz=+0.179, speed=+0.289
                values = {}
                for item in parts.split(','):
                    item = item.strip()
                    key, val = item.split('=')
                    values[key.strip()] = float(val.strip())

                actions.append({
                    'vx': values['vx'],
                    'vy': values['vy'],
                    'vz': values['vz'],
                    'speed': values['speed']
                })

    return actions

def print_summary(data, title):
    """Print summary statistics"""
    print(f"\n{'='*60}")
    print(f"{title}")
    print(f"{'='*60}")

    if 'ai_actions' in data and data['ai_actions']:
        actions = data['ai_actions'][-100:]  # Last 100 actions
        vx_vals = [a['vx'] for a in actions]
        vy_vals = [a['vy'] for a in actions]
        vz_vals = [a['vz'] for a in actions]

        print(f"\nAI Actions (last 100):")
        print(f"  vx (East):  mean={np.mean(vx_vals):6.3f}  std={np.std(vx_vals):6.3f}  range=[{np.min(vx_vals):6.3f}, {np.max(vx_vals):6.3f}]")
        print(f"  vy (North): mean={np.mean(vy_vals):6.3f}  std={np.std(vy_vals):6.3f}  range=[{np.min(vy_vals):6.3f}, {np.max(vy_vals):6.3f}]")
        print(f"  vz (Up):    mean={np.mean(vz_vals):6.3f}  std={np.std(vz_vals):6.3f}  range=[{np.min(vz_vals):6.3f}, {np.max(vz_vals):6.3f}]")

        # Calculate heading from vx, vy
        headings = []
        for a in actions:
            if abs(a['vx']) > 0.1 or abs(a['vy']) > 0.1:
                heading = np.degrees(np.arctan2(a['vx'], -a['vy']))  # ENU: vx=E, vy=N (negated for compass)
                headings.append(heading % 360)

        if headings:
            print(f"  AI heading: mean={np.mean(headings):6.1f}°  std={np.std(headings):6.1f}°")

    if 'gps_courses' in data and data['gps_courses']:
        courses = data['gps_courses'][-20:]  # Last 20
        course_vals = [c['course'] for c in courses]
        speed_vals = [c['speed'] for c in courses]
        print(f"\nGPS Course (last 20):")
        print(f"  Course: mean={np.mean(course_vals):6.1f}°  std={np.std(course_vals):6.1f}°")
        print(f"  Speed:  mean={np.mean(speed_vals):6.3f} m/s")

    if 'attitude_eulers' in data and data['attitude_eulers']:
        attitudes = data['attitude_eulers'][-20:]  # Last 20
        yaw_vals = [a['yaw'] for a in attitudes]
        print(f"\nAttitude Yaw (last 20):")
        print(f"  Yaw: mean={np.mean(yaw_vals):6.1f}°  std={np.std(yaw_vals):6.1f}°")

def main():
    import os
    base_path = '/Users/rafaeltorrecilla/Documents/swarm/swarm-ros/flight-logs'

    print("\n" + "="*60)
    print("FLIGHT LOG ANALYSIS")
    print("="*60)

    # Analyze blackbox
    print("\n[1] BLACKBOX DATA (Overall Flight)")
    blackbox_data = analyze_blackbox(os.path.join(base_path, 'blackbox_20251020_180749_000.jsonl'))
    print_summary(blackbox_data, "Blackbox Summary")

    # Analyze individual AI output files
    test_files = [
        ('ai-1.txt', 'North Movement'),
        ('ai-2.txt', 'East Movement'),
        ('ai-3.txt', 'West Movement'),
        ('ai-4.txt', 'North-South Movement')
    ]

    for filename, description in test_files:
        filepath = os.path.join(base_path, filename)
        if os.path.exists(filepath):
            print(f"\n[{test_files.index((filename, description)) + 2}] {filename.upper()} - {description}")
            actions = analyze_ai_output(filepath)

            if actions:
                vx_vals = [a['vx'] for a in actions]
                vy_vals = [a['vy'] for a in actions]
                vz_vals = [a['vz'] for a in actions]

                print(f"  Total actions: {len(actions)}")
                print(f"  vx (East):  mean={np.mean(vx_vals):6.3f}  std={np.std(vx_vals):6.3f}")
                print(f"  vy (North): mean={np.mean(vy_vals):6.3f}  std={np.std(vy_vals):6.3f}")
                print(f"  vz (Up):    mean={np.mean(vz_vals):6.3f}  std={np.std(vz_vals):6.3f}")

                # Expected directions
                if 'North' in description and 'South' not in description:
                    print(f"  Expected: vy << 0 (North)")
                    print(f"  Result: {'✓ CORRECT' if np.mean(vy_vals) < -0.5 else '✗ INCORRECT'}")
                elif 'East' in description:
                    print(f"  Expected: vx >> 0 (East)")
                    print(f"  Result: {'✓ CORRECT' if np.mean(vx_vals) > 0.5 else '✗ INCORRECT'}")
                elif 'West' in description:
                    print(f"  Expected: vx << 0 (West)")
                    print(f"  Result: {'✓ CORRECT' if np.mean(vx_vals) < -0.5 else '✗ INCORRECT'}")

    # Analyze coordinate frame alignment
    print("\n" + "="*60)
    print("COORDINATE FRAME ANALYSIS")
    print("="*60)

    if blackbox_data['ai_actions'] and blackbox_data['gps_courses']:
        # Compare AI heading vs GPS course
        print("\nComparing AI model heading vs GPS course:")
        print("  (Should match if coordinate frame is correct)")

        # Get last few correlated measurements
        ai_actions = blackbox_data['ai_actions'][-10:]
        gps_courses = blackbox_data['gps_courses'][-10:]

        for i, (action, gps) in enumerate(zip(ai_actions, gps_courses)):
            if abs(action['vx']) > 0.1 or abs(action['vy']) > 0.1:
                # Calculate AI heading from velocity components
                # In ENU: vx=East, vy=North
                # Compass heading: atan2(vx, -vy) where 0°=North, 90°=East
                ai_heading = np.degrees(np.arctan2(action['vx'], -action['vy'])) % 360
                gps_heading = gps['course']

                diff = abs(ai_heading - gps_heading)
                if diff > 180:
                    diff = 360 - diff

                print(f"  Sample {i+1}: AI={ai_heading:5.1f}° GPS={gps_heading:5.1f}° diff={diff:5.1f}°")

if __name__ == '__main__':
    main()
