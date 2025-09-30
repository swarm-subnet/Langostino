#!/usr/bin/env python3
"""
PID Controller Implementation for Velocity Control

Simple, robust PID controller with anti-windup and derivative filtering.
"""

import time
from typing import Optional


class PIDController:
    """
    PID Controller with anti-windup and derivative filtering.

    The controller implements the standard PID algorithm:
    output = Kp * error + Ki * integral + Kd * derivative

    Features:
    - Anti-windup: Prevents integral term from growing unbounded
    - Derivative filtering: Smooths derivative term to reduce noise
    - Output clamping: Ensures output stays within specified limits
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_min: float = -float('inf'),
        output_max: float = float('inf'),
        integral_max: float = 100.0,
        derivative_filter_alpha: float = 0.1
    ):
        """
        Initialize PID controller.

        Parameters:
        -----------
        kp : float
            Proportional gain
        ki : float
            Integral gain
        kd : float
            Derivative gain
        output_min : float
            Minimum output value (default: -inf)
        output_max : float
            Maximum output value (default: inf)
        integral_max : float
            Maximum absolute value for integral term (anti-windup)
        derivative_filter_alpha : float
            Low-pass filter coefficient for derivative term (0-1, higher = less filtering)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_max = integral_max
        self.derivative_filter_alpha = derivative_filter_alpha

        # State variables
        self.integral = 0.0
        self.last_error = 0.0
        self.last_derivative = 0.0
        self.last_time: Optional[float] = None

        # Statistics
        self.output_history = []
        self.error_history = []

    def reset(self):
        """Reset controller state (useful when starting a new control phase)"""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_derivative = 0.0
        self.last_time = None
        self.output_history.clear()
        self.error_history.clear()

    def compute(self, error: float, dt: Optional[float] = None) -> float:
        """
        Compute PID control output.

        Parameters:
        -----------
        error : float
            Current error (setpoint - measurement)
        dt : float, optional
            Time step in seconds. If None, will be computed from system time.

        Returns:
        --------
        float
            Control output
        """
        current_time = time.time()

        # Compute dt if not provided
        if dt is None:
            if self.last_time is None:
                dt = 0.0  # First call, no derivative
            else:
                dt = current_time - self.last_time

        self.last_time = current_time

        # Avoid division by zero
        if dt < 1e-6:
            dt = 1e-6

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(-self.integral_max, min(self.integral_max, self.integral))
        i_term = self.ki * self.integral

        # Derivative term with filtering
        derivative = (error - self.last_error) / dt

        # Apply low-pass filter to derivative
        self.last_derivative = (
            self.derivative_filter_alpha * derivative +
            (1.0 - self.derivative_filter_alpha) * self.last_derivative
        )
        d_term = self.kd * self.last_derivative

        # Compute output
        output = p_term + i_term + d_term

        # Clamp output
        output = max(self.output_min, min(self.output_max, output))

        # Update state
        self.last_error = error

        # Store history (limited to last 100 samples)
        self.output_history.append(output)
        self.error_history.append(error)
        if len(self.output_history) > 100:
            self.output_history.pop(0)
            self.error_history.pop(0)

        return output

    def set_gains(self, kp: float, ki: float, kd: float):
        """Update PID gains dynamically"""
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def get_state(self) -> dict:
        """Get current controller state (for debugging/monitoring)"""
        return {
            'kp': self.kp,
            'ki': self.ki,
            'kd': self.kd,
            'integral': self.integral,
            'last_error': self.last_error,
            'last_derivative': self.last_derivative,
            'p_term': self.kp * self.last_error if self.last_error else 0.0,
            'i_term': self.ki * self.integral,
            'd_term': self.kd * self.last_derivative,
        }


class VelocityPIDController:
    """
    3-axis velocity controller using PID for each axis.

    Coordinates PID controllers for vx, vy, vz velocity control.
    """

    def __init__(
        self,
        kp_xy: float = 150.0,
        ki_xy: float = 10.0,
        kd_xy: float = 20.0,
        kp_z: float = 100.0,
        ki_z: float = 5.0,
        kd_z: float = 15.0,
        output_min: float = -400.0,
        output_max: float = 400.0
    ):
        """
        Initialize 3-axis velocity PID controller.

        Parameters:
        -----------
        kp_xy, ki_xy, kd_xy : float
            PID gains for horizontal axes (X, Y)
        kp_z, ki_z, kd_z : float
            PID gains for vertical axis (Z)
        output_min, output_max : float
            Output limits for RC channel deviation from center (1500)
        """
        # Horizontal controllers (forward/back, left/right)
        self.pid_x = PIDController(
            kp=kp_xy, ki=ki_xy, kd=kd_xy,
            output_min=output_min, output_max=output_max,
            integral_max=50.0
        )

        self.pid_y = PIDController(
            kp=kp_xy, ki=ki_xy, kd=kd_xy,
            output_min=output_min, output_max=output_max,
            integral_max=50.0
        )

        # Vertical controller (up/down)
        self.pid_z = PIDController(
            kp=kp_z, ki=ki_z, kd=kd_z,
            output_min=output_min, output_max=output_max,
            integral_max=50.0
        )

    def reset(self):
        """Reset all PID controllers"""
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()

    def compute(
        self,
        vel_cmd_x: float,
        vel_cmd_y: float,
        vel_cmd_z: float,
        vel_actual_x: float,
        vel_actual_y: float,
        vel_actual_z: float,
        dt: Optional[float] = None
    ) -> tuple[float, float, float]:
        """
        Compute RC channel deviations for velocity tracking.

        Parameters:
        -----------
        vel_cmd_x, vel_cmd_y, vel_cmd_z : float
            Commanded velocities (m/s)
        vel_actual_x, vel_actual_y, vel_actual_z : float
            Actual velocities (m/s)
        dt : float, optional
            Time step (seconds)

        Returns:
        --------
        tuple[float, float, float]
            RC channel deviations for pitch, roll, throttle
        """
        # Compute errors
        error_x = vel_cmd_x - vel_actual_x
        error_y = vel_cmd_y - vel_actual_y
        error_z = vel_cmd_z - vel_actual_z

        # Compute PID outputs
        pitch_deviation = self.pid_x.compute(error_x, dt)  # Forward velocity → pitch
        roll_deviation = -self.pid_y.compute(error_y, dt)  # Right velocity → roll (inverted)
        throttle_deviation = self.pid_z.compute(error_z, dt)  # Up velocity → throttle

        return pitch_deviation, roll_deviation, throttle_deviation

    def get_state(self) -> dict:
        """Get state of all controllers"""
        return {
            'x': self.pid_x.get_state(),
            'y': self.pid_y.get_state(),
            'z': self.pid_z.get_state()
        }