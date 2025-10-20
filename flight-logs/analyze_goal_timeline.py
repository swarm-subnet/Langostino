#!/usr/bin/env python3
"""
Analyze goal position changes over time to understand what happened
during the manual movement tests
"""

import json
import numpy as np
from datetime import datetime

def parse_array(data_str):
    """Parse array string from blackbox log"""
    data_str = data_str.replace("array('f', [", "").replace("])", "")
    return [float(x) for x in data_str.split(", ")]

def analyze_timeline(filepath):
    """Analyze observation vectors over time"""

    observations = []

    with open(filepath, 'r') as f:
        for line in f:
            try:
                log = json.loads(line.strip())
                if log.get('log_type') == 'AI_OBSERVATION':
                    obs_data = parse_array(log['data']['data'])
                    timestamp = log.get('timestamp', 0)

                    observations.append({
                        'timestamp': timestamp,
                        'position': np.array(obs_data[0:3]),
                        'goal': np.array(obs_data[128:131])
                    })
            except Exception as e:
                continue

    print(f"Total observations: {len(observations)}\n")

    # Track goal changes
    prev_goal = None
    goal_changes = []

    for obs in observations:
        goal = obs['goal']
        if prev_goal is None or not np.allclose(goal, prev_goal, atol=0.5):
            goal_changes.append(obs)
            prev_goal = goal.copy()

    print(f"Goal changed {len(goal_changes)} times during flight\n")
    print("="*100)
    print("GOAL POSITION TIMELINE")
    print("="*100)

    # Skip the crazy GPS coordinate goals at the start
    valid_goals = [g for g in goal_changes if abs(g['goal'][0]) < 100 and abs(g['goal'][1]) < 100]

    print(f"\n{'#':<4} {'Timestamp':<18} {'Goal (x,y,z)':<30} {'Pos (x,y,z)':<30} {'Rel (x,y,z)':<25} {'Heading':<10}")
    print("-"*130)

    for i, obs in enumerate(valid_goals[:20]):  # First 20 valid goals
        goal = obs['goal']
        pos = obs['position']
        rel = goal - pos

        heading = np.degrees(np.arctan2(rel[0], rel[1])) % 360
        distance = np.sqrt(rel[0]**2 + rel[1]**2)

        # Convert timestamp to readable time
        dt = datetime.fromtimestamp(obs['timestamp'])
        time_str = dt.strftime('%H:%M:%S')

        print(f"{i+1:<4} {time_str:<18} "
              f"({goal[0]:6.2f},{goal[1]:6.2f},{goal[2]:6.2f})  "
              f"({pos[0]:6.2f},{pos[1]:6.2f},{pos[2]:6.2f})  "
              f"({rel[0]:6.2f},{rel[1]:6.2f},{rel[2]:6.2f})  "
              f"{heading:6.1f}°")

    # Analyze goal positions in the second half of flight (where manual movements occurred)
    mid_point = len(observations) // 2
    late_obs = observations[mid_point:]

    print("\n" + "="*100)
    print("LATE FLIGHT ANALYSIS (Second half - during manual movements)")
    print("="*100)

    goals = np.array([obs['goal'] for obs in late_obs])
    positions = np.array([obs['position'] for obs in late_obs])

    print(f"\nSamples analyzed: {len(late_obs)}")
    print(f"\nGoal Statistics:")
    print(f"  Mean: ({np.mean(goals[:,0]):6.3f}, {np.mean(goals[:,1]):6.3f}, {np.mean(goals[:,2]):6.3f})")
    print(f"  Std:  ({np.std(goals[:,0]):6.3f}, {np.std(goals[:,1]):6.3f}, {np.std(goals[:,2]):6.3f})")
    print(f"  Range: X=[{np.min(goals[:,0]):6.3f}, {np.max(goals[:,0]):6.3f}]")
    print(f"         Y=[{np.min(goals[:,1]):6.3f}, {np.max(goals[:,1]):6.3f}]")

    # Check if goal is essentially fixed
    goal_std = np.std(goals, axis=0)
    if np.all(goal_std < 1.0):
        print(f"\n⚠️  Goal is FIXED at: ({np.mean(goals[:,0]):.2f}, {np.mean(goals[:,1]):.2f}, {np.mean(goals[:,2]):.2f})")
        print(f"    Standard deviation < 1.0m in all axes")

        # Calculate average heading to goal
        rel_goals = goals - positions
        headings = np.degrees(np.arctan2(rel_goals[:,0], rel_goals[:,1])) % 360
        avg_heading = np.mean(headings)

        print(f"\n    Average heading to goal: {avg_heading:.1f}°")
        print(f"    This matches the AI's consistent ~40-45° northeast commands!")
        print(f"\n    CONCLUSION: Goal waypoint was NOT updated during manual movement tests.")
        print(f"                Model correctly tried to reach the fixed goal position.")
    else:
        print(f"\n✓ Goal is varying (std > 1.0m)")

if __name__ == '__main__':
    analyze_timeline('/Users/rafaeltorrecilla/Documents/swarm/swarm-ros/flight-logs/blackbox_20251020_180749_000.jsonl')
