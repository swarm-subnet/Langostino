#!/usr/bin/env python3
"""
AI Flight Node - Loads and executes the Swarm AI model for real-time flight control

This node loads the pre-trained Swarm AI model and processes 131-D observation arrays
to generate control commands for the drone. It interfaces with the flight controller
through MSP commands.

Based on swarm/core/evaluator.py:142-174 and swarm/core/secure_loader.py
"""

import sys
import os
import time
import numpy as np
from pathlib import Path
from typing import Optional, List
import threading
import queue

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy


class AIFlightNode(Node):
    """
    Loads and executes the Swarm AI model for real-time flight control.

    Subscribers:
        /ai/observation (std_msgs/Float32MultiArray): 131-D observation array
        /ai/enable (std_msgs/Bool): Enable/disable AI control
        /safety/override (std_msgs/Bool): Safety override signal

    Publishers:
        /ai/action (geometry_msgs/Twist): Control commands (velocity)
        /ai/status (std_msgs/Float32MultiArray): AI status and diagnostics
        /ai/model_ready (std_msgs/Bool): Model ready status
    """

    def __init__(self):
        super().__init__('ai_flight_node')

        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('prediction_timeout', 0.1)  # 100ms timeout
        self.declare_parameter('max_velocity', 3.0)  # m/s
        self.declare_parameter('safety_enabled', True)

        # Get parameters
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.prediction_timeout = self.get_parameter('prediction_timeout').get_parameter_value().double_value
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        self.safety_enabled = self.get_parameter('safety_enabled').get_parameter_value().bool_value

        # State variables
        self.model = None
        self.model_ready = False
        self.ai_enabled = False
        self.safety_override = False
        self.last_observation = None
        self.last_action = np.zeros(4, dtype=np.float32)
        self.prediction_count = 0
        self.error_count = 0

        # Threading for model prediction
        self.prediction_queue = queue.Queue(maxsize=1)
        self.result_queue = queue.Queue(maxsize=1)
        self.prediction_thread = None
        self.thread_running = False

        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.obs_sub = self.create_subscription(
            Float32MultiArray, '/ai/observation', self.observation_callback, reliable_qos)
        self.enable_sub = self.create_subscription(
            Bool, '/ai/enable', self.enable_callback, reliable_qos)
        self.safety_sub = self.create_subscription(
            Bool, '/safety/override', self.safety_callback, reliable_qos)

        # Publishers
        self.action_pub = self.create_publisher(
            Twist, '/ai/action', reliable_qos)
        self.status_pub = self.create_publisher(
            Float32MultiArray, '/ai/status', reliable_qos)
        self.ready_pub = self.create_publisher(
            Bool, '/ai/model_ready', reliable_qos)

        # Timer for status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)

        # Load model in separate thread to avoid blocking
        self.model_load_thread = threading.Thread(target=self.load_model)
        self.model_load_thread.daemon = True
        self.model_load_thread.start()

        self.get_logger().info('AI Flight Node initialized')

    def load_model(self):
        """Load the Swarm AI model (runs in separate thread)"""
        try:
            if not self.model_path:
                self.get_logger().error('Model path not specified. Use model_path parameter.')
                return

            model_file = Path(self.model_path)
            if not model_file.exists():
                self.get_logger().error(f'Model file not found: {model_file}')
                return

            self.get_logger().info(f'Loading AI model from: {model_file}')

            # Add swarm package to path
            swarm_path = str(Path(__file__).resolve().parent.parent.parent.parent.parent / 'swarm')
            if swarm_path not in sys.path:
                sys.path.insert(0, swarm_path)

            # Import swarm modules
            from swarm.core.secure_loader import secure_load_ppo
            from swarm.utils.env_factory import make_env
            from swarm.validator.task_gen import random_task
            from swarm.constants import SIM_DT, HORIZON_SEC

            # Create minimal environment for policy initialization
            task = random_task(sim_dt=SIM_DT, horizon=HORIZON_SEC, seed=1)
            init_env = make_env(task, gui=False)

            try:
                # Load the model
                self.model = secure_load_ppo(model_file, env=init_env, device=self.device)
                self.model_ready = True
                self.get_logger().info('AI model loaded successfully')

                # Start prediction thread
                self.thread_running = True
                self.prediction_thread = threading.Thread(target=self.prediction_worker)
                self.prediction_thread.daemon = True
                self.prediction_thread.start()

            finally:
                init_env.close()

        except Exception as e:
            self.get_logger().error(f'Failed to load AI model: {e}')
            import traceback
            self.get_logger().debug(traceback.format_exc())

    def prediction_worker(self):
        """Worker thread for model predictions"""
        while self.thread_running:
            try:
                # Wait for prediction request
                obs_array = self.prediction_queue.get(timeout=1.0)

                # Make prediction
                start_time = time.time()
                action, _ = self.model.predict(obs_array, deterministic=True)
                prediction_time = time.time() - start_time

                # Convert to numpy array and clip
                if hasattr(action, 'squeeze'):
                    action = action.squeeze()
                if hasattr(action, 'tolist'):
                    action = np.array(action, dtype=np.float32)
                else:
                    action = np.array(action, dtype=np.float32)

                # Put result in queue
                try:
                    self.result_queue.put((action, prediction_time), block=False)
                except queue.Full:
                    pass  # Drop if queue is full

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Prediction error: {e}')
                self.error_count += 1

    def observation_callback(self, msg: Float32MultiArray):
        """Process incoming observation and generate control action"""
        if not self.model_ready or not self.ai_enabled or self.safety_override:
            return

        try:
            # Convert to numpy array
            obs_array = np.array(msg.data, dtype=np.float32)

            # Validate observation size
            if len(obs_array) != 131:
                self.get_logger().warn(f'Invalid observation size: {len(obs_array)}, expected 131')
                return

            self.last_observation = obs_array

            # Queue prediction request (non-blocking)
            try:
                self.prediction_queue.put(obs_array, block=False)
            except queue.Full:
                # Drop old request and put new one
                try:
                    self.prediction_queue.get_nowait()
                    self.prediction_queue.put(obs_array, block=False)
                except queue.Empty:
                    pass

            # Check for prediction result
            try:
                action, prediction_time = self.result_queue.get_nowait()
                self.process_action(action)
                self.prediction_count += 1

                if prediction_time > self.prediction_timeout:
                    self.get_logger().warn(f'Slow prediction: {prediction_time:.3f}s')

            except queue.Empty:
                # No result available, use last action
                self.process_action(self.last_action)

        except Exception as e:
            self.get_logger().error(f'Observation processing error: {e}')
            self.error_count += 1

    def process_action(self, action):
        """Process and publish control action"""
        try:
            # Ensure action is 4-dimensional (velocity + yaw rate)
            if len(action) < 4:
                action = np.pad(action, (0, 4 - len(action)), 'constant')
            elif len(action) > 4:
                action = action[:4]

            # Apply safety limits
            if self.safety_enabled:
                # Limit velocity magnitude
                velocity_mag = np.linalg.norm(action[:3])
                if velocity_mag > self.max_velocity:
                    scale = self.max_velocity / velocity_mag
                    action[:3] *= scale

                # Limit yaw rate
                max_yaw_rate = 2.0  # rad/s
                action[3] = np.clip(action[3], -max_yaw_rate, max_yaw_rate)

            self.last_action = action

            # Create and publish Twist message
            twist_msg = Twist()
            twist_msg.linear.x = float(action[0])
            twist_msg.linear.y = float(action[1])
            twist_msg.linear.z = float(action[2])
            twist_msg.angular.z = float(action[3])

            self.action_pub.publish(twist_msg)

        except Exception as e:
            self.get_logger().error(f'Action processing error: {e}')

    def enable_callback(self, msg: Bool):
        """Enable/disable AI control"""
        self.ai_enabled = msg.data
        if self.ai_enabled:
            self.get_logger().info('AI control enabled')
        else:
            self.get_logger().info('AI control disabled')
            # Publish zero action when disabled
            twist_msg = Twist()
            self.action_pub.publish(twist_msg)

    def safety_callback(self, msg: Bool):
        """Handle safety override"""
        self.safety_override = msg.data
        if self.safety_override:
            self.get_logger().warn('Safety override activated - AI control disabled')
            # Publish zero action on safety override
            twist_msg = Twist()
            self.action_pub.publish(twist_msg)

    def publish_status(self):
        """Publish AI system status"""
        try:
            # Publish model ready status
            ready_msg = Bool()
            ready_msg.data = self.model_ready
            self.ready_pub.publish(ready_msg)

            # Publish detailed status
            status_data = [
                float(self.model_ready),
                float(self.ai_enabled),
                float(self.safety_override),
                float(self.prediction_count),
                float(self.error_count),
                float(np.linalg.norm(self.last_action) if self.last_action is not None else 0.0),
                float(time.time() % 1000)  # Timestamp
            ]

            status_msg = Float32MultiArray()
            status_msg.data = status_data
            self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Status publishing error: {e}')

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.thread_running = False
        if self.prediction_thread and self.prediction_thread.is_alive():
            self.prediction_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AIFlightNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()