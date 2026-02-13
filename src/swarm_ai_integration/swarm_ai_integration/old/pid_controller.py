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

    output = Kp * error + Ki * integral + Kd * derivative
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_min: float = -float('inf'),
        output_max: float = float('inf'),
        integral_max: float = 100.0,
        derivative_filter_alpha: float = 0.1,
        max_history_length: int = 100
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_max = integral_max
        self.derivative_filter_alpha = derivative_filter_alpha
        self.max_history_length = max_history_length

        # State
        self.integral = 0.0
        self.last_error = 0.0
        self.last_derivative = 0.0
        self.last_time: Optional[float] = None

        # Histories (debug)
        self.output_history = []
        self.error_history = []

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_derivative = 0.0
        self.last_time = None
        self.output_history.clear()
        self.error_history.clear()

    def compute(self, error: float, dt: Optional[float] = None) -> float:
        current_time = time.time()

        if dt is None:
            if self.last_time is None:
                dt = 0.0
            else:
                dt = current_time - self.last_time
        self.last_time = current_time

        if dt < 1e-6:
            dt = 1e-6

        # PID terms
        p_term = self.kp * error

        self.integral += error * dt
        self.integral = max(-self.integral_max, min(self.integral_max, self.integral))
        i_term = self.ki * self.integral

        derivative = (error - self.last_error) / dt
        self.last_derivative = (
            self.derivative_filter_alpha * derivative +
            (1.0 - self.derivative_filter_alpha) * self.last_derivative
        )
        d_term = self.kd * self.last_derivative

        output = p_term + i_term + d_term
        output = max(self.output_min, min(self.output_max, output))

        self.last_error = error

        # logs
        self.output_history.append(output)
        self.error_history.append(error)
        if len(self.output_history) > self.max_history_length:
            self.output_history.pop(0)
            self.error_history.pop(0)

        return output

    def set_gains(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def get_state(self) -> dict:
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
    3-axis velocity controller using PID for each axis (vx, vy, vz).
    Returns RC deviations for (pitch, roll, throttle).
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
        output_max: float = 400.0,
        integral_max: float = 50.0,
        derivative_filter_alpha: float = 0.1,
        max_history_length: int = 100
    ):
        self.pid_x = PIDController(
            kp=kp_xy, ki=ki_xy, kd=kd_xy,
            output_min=output_min, output_max=output_max,
            integral_max=integral_max,
            derivative_filter_alpha=derivative_filter_alpha,
            max_history_length=max_history_length
        )
        self.pid_y = PIDController(
            kp=kp_xy, ki=ki_xy, kd=kd_xy,
            output_min=output_min, output_max=output_max,
            integral_max=integral_max,
            derivative_filter_alpha=derivative_filter_alpha,
            max_history_length=max_history_length
        )
        self.pid_z = PIDController(
            kp=kp_z, ki=ki_z, kd=kd_z,
            output_min=output_min, output_max=output_max,
            integral_max=integral_max,
            derivative_filter_alpha=derivative_filter_alpha,
            max_history_length=max_history_length
        )
        self.max_history_length = max_history_length

    def reset(self):
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
        # Errors
        err_x = vel_cmd_x - vel_actual_x
        err_y = vel_cmd_y - vel_actual_y
        err_z = vel_cmd_z - vel_actual_z

        # Velocity PID → RC deviations
        pitch_dev = self.pid_x.compute(err_x, dt)     # forward/back → Pitch
        roll_dev = -self.pid_y.compute(err_y, dt)     # right/left  → Roll (inverted)
        throttle_dev = self.pid_z.compute(err_z, dt)  # up/down     → Throttle

        return pitch_dev, roll_dev, throttle_dev

    def get_state(self) -> dict:
        return {
            'x': self.pid_x.get_state(),
            'y': self.pid_y.get_state(),
            'z': self.pid_z.get_state(),
        }
