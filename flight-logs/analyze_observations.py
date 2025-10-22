#!/usr/bin/env python3
"""
Deep analysis of observation vectors to check:
1. If goal position [128:131] is updating
2. If drone position [0:3] is updating
3. If relative goal matches position changes
"""

import json
import numpy as np

def parse_array(data_str):
    """Parse array string from blackbox log"""
    data_str = data_str.replace("array('f', [", "").replace("])", "")
    return [float(x) for x in data_str.split(", ")]

def analyze_observations(filepath, sample_interval=50):
    """Analyze observation vectors"""

    observations = []

    with open(filepath, 'r') as f:
        for line in f:
            try:
                log = json.loads(line.strip())
                if log.get('log_type') == 'AI_OBSERVATION':
                    obs_data = parse_array(log['data']['data'])
                    timestamp = log.get('timestamp', 0)

                    # Parse observation vector (131 dimensions)
                    # [0:3] position (x,y,z)
                    # [3:6] euler angles (roll, pitch, yaw)
                    # [6:9] velocity
                    # [128:131] goal position

                    observations.append({
                        'timestamp': timestamp,
                        'position': obs_data[0:3],
                        'euler': obs_data[3:6],
                        'velocity': obs_data[6:9],
                        'goal': obs_data[128:131]
                    })
            except Exception as e:
                continue

    if not observations:
        print("No observations found!")
        return

    print(f"Total observations: {len(observations)}")
    print("\n" + "="*80)
    print("OBSERVATION VECTOR ANALYSIS")
    print("="*80)

    # Sample observations evenly
    indices = np.linspace(0, len(observations)-1, min(10, len(observations)), dtype=int)

    print(f"\n{'Sample':<8} {'Time':<12} {'Position (x,y,z)':<30} {'Goal (x,y,z)':<30} {'Rel Goal':<20}")
    print("-"*120)

    for i, idx in enumerate(indices):
        obs = observations[idx]
        pos = obs['position']
        goal = obs['goal']
        rel_goal = [goal[j] - pos[j] for j in range(3)]

        print(f"{i+1:<8} {obs['timestamp']:<12.2f} "
              f"({pos[0]:6.2f}, {pos[1]:6.2f}, {pos[2]:6.2f})  "
              f"({goal[0]:6.2f}, {goal[1]:6.2f}, {goal[2]:6.2f})  "
              f"({rel_goal[0]:6.2f}, {rel_goal[1]:6.2f}, {rel_goal[2]:6.2f})")

    # Statistics
    print("\n" + "="*80)
    print("STATISTICS")
    print("="*80)

    positions = np.array([obs['position'] for obs in observations])
    goals = np.array([obs['goal'] for obs in observations])

    print(f"\nPosition (x,y,z):")
    print(f"  Mean: ({np.mean(positions[:,0]):6.3f}, {np.mean(positions[:,1]):6.3f}, {np.mean(positions[:,2]):6.3f})")
    print(f"  Std:  ({np.std(positions[:,0]):6.3f}, {np.std(positions[:,1]):6.3f}, {np.std(positions[:,2]):6.3f})")
    print(f"  Range: X=[{np.min(positions[:,0]):6.3f}, {np.max(positions[:,0]):6.3f}]")
    print(f"         Y=[{np.min(positions[:,1]):6.3f}, {np.max(positions[:,1]):6.3f}]")
    print(f"         Z=[{np.min(positions[:,2]):6.3f}, {np.max(positions[:,2]):6.3f}]")

    print(f"\nGoal Position (x,y,z):")
    print(f"  Mean: ({np.mean(goals[:,0]):6.3f}, {np.mean(goals[:,1]):6.3f}, {np.mean(goals[:,2]):6.3f})")
    print(f"  Std:  ({np.std(goals[:,0]):6.3f}, {np.std(goals[:,1]):6.3f}, {np.std(goals[:,2]):6.3f})")
    print(f"  Unique goals: {len(np.unique(goals, axis=0))}")

    # Check if goal is constant
    goal_variance = np.var(goals, axis=0)
    print(f"\nGoal Variance: ({goal_variance[0]:.6f}, {goal_variance[1]:.6f}, {goal_variance[2]:.6f})")

    if np.all(goal_variance < 0.01):
        print("  ⚠️  GOAL IS ESSENTIALLY CONSTANT! Goal waypoint not being updated.")
        print(f"  Fixed goal: ({goals[0][0]:.3f}, {goals[0][1]:.3f}, {goals[0][2]:.3f})")

        # Calculate what direction this goal is from average position
        avg_pos = np.mean(positions, axis=0)
        rel_goal = goals[0] - avg_pos

        # Calculate heading to goal
        heading = np.degrees(np.arctan2(rel_goal[0], rel_goal[1])) % 360
        distance = np.sqrt(rel_goal[0]**2 + rel_goal[1]**2)

        print(f"  Relative to drone: ({rel_goal[0]:.3f}, {rel_goal[1]:.3f}, {rel_goal[2]:.3f})")
        print(f"  Heading to goal: {heading:.1f}°")
        print(f"  Horizontal distance: {distance:.3f} m")
        print(f"\n  This explains why AI always commands ~{heading:.0f}° heading!")
    else:
        print("  ✓ Goal is changing during flight")

    # Check if position is updating
    pos_variance = np.var(positions, axis=0)
    print(f"\nPosition Variance: ({pos_variance[0]:.6f}, {pos_variance[1]:.6f}, {pos_variance[2]:.6f})")

    if np.all(pos_variance < 0.01):
        print("  ⚠️  POSITION IS NOT UPDATING! GPS or position estimator might be frozen.")
    else:
        print("  ✓ Position is updating during flight")

if __name__ == '__main__':
    analyze_observations('/Users/rafaeltorrecilla/Documents/swarm/swarm-ros/flight-logs/blackbox_20251020_180749_000.jsonl')
