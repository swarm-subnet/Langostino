#!/usr/bin/env python3
"""
Observation Builder - Constructs 131-D observation arrays for AI model

This module handles:
- Building observation vectors from sensor data
- Managing action buffers
- Goal vector computation and scaling
- Observation vector assembly and validation
"""

import numpy as np
from collections import deque
from typing import Dict, Tuple, Optional


class ObservationBuilder:
    """
    Builds 131-dimensional observation arrays for the swarm AI model.

    Observation structure:
      [0:3]     - Relative position ENU (meters)
      [3:7]     - Orientation quaternion
      [7:10]    - Orientation Euler (roll, pitch, yaw in radians)
      [10:13]   - Velocity ENU (m/s)
      [13:16]   - Angular velocity (rad/s)
      [16:20]   - Last action
      [20:100]  - Action buffer (20 × 4 = 80)
      [100:112] - Padding (12 zeros)
      [112:128] - LiDAR distances (16 rays, normalized)
      [128:131] - Goal vector (ENU, normalized)
    """

    def __init__(self, action_buffer_size: int = 20, max_ray_distance: float = 10.0):
        """
        Initialize observation builder.

        Args:
            action_buffer_size: Number of past actions to store
            max_ray_distance: Maximum distance for normalization (meters)
        """
        self.action_buffer_size = action_buffer_size
        self.max_ray_distance = max_ray_distance

        # Action buffer
        self.action_buffer = deque(maxlen=action_buffer_size)
        for _ in range(action_buffer_size):
            self.action_buffer.append(np.zeros(4, dtype=np.float32))

        # Track action updates
        self.last_action_received = np.zeros(4, dtype=np.float32)
        self.action_count = 0
        self._last_action_count_seen = 0

    def update_action(self, action: np.ndarray):
        """
        Update with a new action from the AI.

        Args:
            action: 4-element action array
        """
        self.last_action_received = action.copy()
        self.action_buffer.append(action.copy())
        self.action_count += 1

    def prepare_action_for_observation(self) -> np.ndarray:
        """
        Prepare action for current observation tick.

        If no new action received since last tick, returns zeros and
        appends zeros to the buffer. Otherwise returns the last action.

        Returns:
            4-element action array for this observation
        """
        if self.action_count == self._last_action_count_seen:
            # No new action, use zeros
            zero_action = np.zeros(4, dtype=np.float32)
            self.action_buffer.append(zero_action.copy())
            last_action_for_obs = zero_action
        else:
            # New action available
            last_action_for_obs = self.last_action_received.copy()

        self._last_action_count_seen = self.action_count
        return last_action_for_obs

    def build_base_observation(
        self,
        rel_pos_enu: np.ndarray,
        quat_att: np.ndarray,
        euler_att: np.ndarray,
        velocity: np.ndarray,
        angular_velocity: np.ndarray,
        last_action: np.ndarray
    ) -> np.ndarray:
        """
        Build base 112-D observation vector.

        Args:
            rel_pos_enu: Relative position [E, N, U] in meters
            quat_att: Orientation quaternion [x, y, z, w]
            euler_att: Euler angles [roll, pitch, yaw] in radians
            velocity: Velocity [vx, vy, vz] in m/s
            angular_velocity: Angular velocity [wx, wy, wz] in rad/s
            last_action: Last action [4 values]

        Returns:
            112-element base observation array
        """
        # Flatten action buffer
        action_buffer_flat = np.concatenate(list(self.action_buffer)).astype(np.float32)

        obs = np.concatenate([
            rel_pos_enu.astype(np.float32),           # 3
            quat_att.astype(np.float32),              # 4
            euler_att.astype(np.float32),             # 3
            velocity.astype(np.float32),              # 3
            angular_velocity.astype(np.float32),      # 3
            last_action.astype(np.float32),           # 4
            action_buffer_flat,                       # 80
            np.zeros(12, dtype=np.float32)            # 12 padding
        ], dtype=np.float32)

        return obs

    def build_full_observation(
        self,
        rel_pos_enu: np.ndarray,
        quat_att: np.ndarray,
        euler_att: np.ndarray,
        velocity: np.ndarray,
        angular_velocity: np.ndarray,
        last_action: np.ndarray,
        lidar_distances: np.ndarray,
        goal_vector: np.ndarray
    ) -> np.ndarray:
        """
        Build complete 131-D observation vector.

        Args:
            rel_pos_enu: Relative position [E, N, U] in meters
            quat_att: Orientation quaternion [x, y, z, w]
            euler_att: Euler angles [roll, pitch, yaw] in radians
            velocity: Velocity [vx, vy, vz] in m/s
            angular_velocity: Angular velocity [wx, wy, wz] in rad/s
            last_action: Last action [4 values]
            lidar_distances: LiDAR distances [16 rays], normalized
            goal_vector: Goal vector [E, N, U], normalized

        Returns:
            131-element full observation array
        """
        # Build base observation (112-D)
        base_obs = self.build_base_observation(
            rel_pos_enu, quat_att, euler_att,
            velocity, angular_velocity, last_action
        )

        # Concatenate with LiDAR and goal
        full_obs = np.concatenate([
            base_obs,                           # 112
            lidar_distances.astype(np.float32), # 16
            goal_vector.astype(np.float32)      # 3
        ], dtype=np.float32)

        # Validate shape
        assert full_obs.shape == (131,), f'Observation shape mismatch: {full_obs.shape}'

        return full_obs

    def compute_goal_vector(
        self,
        goal_enu: np.ndarray,
        current_enu: np.ndarray
    ) -> np.ndarray:
        """
        Compute normalized goal vector from current position to goal.

        Args:
            goal_enu: Goal position [E, N, U] in meters
            current_enu: Current position [E, N, U] in meters

        Returns:
            Normalized goal vector [E, N, U]
        """
        # Compute offset from current to goal
        goal_offset = goal_enu - current_enu

        # Scale by max ray distance (no clipping)
        normalized_goal = (goal_offset / float(self.max_ray_distance)).astype(np.float32)

        return normalized_goal

    def normalize_lidar_distance(self, distance_m: float) -> float:
        """
        Normalize LiDAR distance to [0, 1] range.

        Args:
            distance_m: Distance in meters

        Returns:
            Normalized distance (0 = at sensor, 1 = max range or beyond)
        """
        d = max(0.0, float(distance_m))
        return float(min(1.0, d / self.max_ray_distance))

    def get_observation_breakdown(self, observation: np.ndarray) -> Dict[str, np.ndarray]:
        """
        Break down observation array into named components.

        Args:
            observation: 131-element observation array

        Returns:
            Dictionary mapping component names to arrays
        """
        assert len(observation) == 131, f'Invalid observation size: {len(observation)}'

        return {
            'rel_pos_enu': observation[0:3],
            'quat_att': observation[3:7],
            'euler_att': observation[7:10],
            'velocity': observation[10:13],
            'angular_velocity': observation[13:16],
            'last_action': observation[16:20],
            'action_buffer': observation[20:100],
            'padding': observation[100:112],
            'lidar_distances': observation[112:128],
            'goal_vector': observation[128:131]
        }

    def get_action_count(self) -> int:
        """Get total number of actions received."""
        return self.action_count

    def is_using_action(self, last_action: np.ndarray) -> bool:
        """
        Check if the observation is using a real action (not zeros).

        Args:
            last_action: Last action array

        Returns:
            True if using a real action, False if using zeros
        """
        return self.action_count > 0 and not np.allclose(last_action, 0.0)
