#!/usr/bin/env python3
"""
AI Flight Node - Loads and executes the Swarm AI model for real-time flight control
(ROS 2, no simulator environment required at runtime)

This node loads a pre-trained Stable-Baselines3 PPO policy from an SB3 `.zip`
checkpoint **without** constructing or using the training environment.
It reads a small, safe JSON (`safe_policy_meta.json`) embedded in the zip to
reconstruct the policy architecture, then loads **weights only** (no pickle).

Observations are provided by a ROS 2 topic as a 131‑D float array and the node
publishes velocity/yaw actions as geometry_msgs/Twist.

Compatible with checkpoints saved like in your training example where you
append the SAFE metadata JSON into the SB3 zip.

Key parameters (ROS 2 params):
- model_path (string): path to the SB3 .zip file (required)
- device (string): "cpu" or "cuda" (default: "cpu")
- prediction_timeout (double): warn if a single forward pass exceeds this (s)
- max_velocity (double): linear velocity limit (m/s)
- safety_enabled (bool)
- obs_dim (int): expected observation dimension (default: 131)
- action_dim (int): expected action dimension (default: 4)

Topics:
- Subscribes:
    /ai/observation (std_msgs/Float32MultiArray): 131-D observation
    /ai/enable (std_msgs/Bool): enable/disable AI control
    /safety/override (std_msgs/Bool): safety override
- Publishes:
    /ai/action (geometry_msgs/Twist): linear x/y/z + angular z (yaw rate)
    /ai/status (std_msgs/Float32MultiArray): diagnostics
    /ai/model_ready (std_msgs/Bool): model loaded flag
"""

from __future__ import annotations

import sys
import os
import io
import json
import time
import zipfile
import threading
import queue
from pathlib import Path
from typing import Any, Dict, Mapping, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Twist

# ──────────────────────────────────────────────────────────────────────
# Minimal, safe PPO loader (no env needed for runtime; we only build a
# tiny static "spaces-only" env to wire the policy inside SB3).
# ──────────────────────────────────────────────────────────────────────

SAFE_META_CANDIDATES = ("safe_policy_meta.json",)  # primary filename used in examples


def _parse_activation(name: str):
    """Map serialized activation name to torch.nn module class."""
    import torch as th

    _ACT_MAP = {
        "relu": th.nn.ReLU,
        "tanh": th.nn.Tanh,
        "elu": th.nn.ELU,
        "leakyrelu": th.nn.LeakyReLU,
        "silu": th.nn.SiLU,
        "gelu": th.nn.GELU,
        "mish": th.nn.Mish,
        "selu": th.nn.SELU,
        "celu": th.nn.CELU,
    }
    key = name.strip().split(".")[-1].lower()
    return _ACT_MAP.get(key, th.nn.ReLU)


def _torch_supports_weights_only() -> bool:
    import torch as th
    from inspect import signature

    try:
        return "weights_only" in signature(th.load).parameters
    except Exception:
        return False


def _read_text(zf: zipfile.ZipFile, name: str) -> str:
    with zf.open(name, "r") as fh:
        return fh.read().decode("utf-8")


def _read_bytes(zf: zipfile.ZipFile, name: str) -> bytes:
    with zf.open(name, "r") as fh:
        return fh.read()


def _extract_policy_state_dict(raw_obj: Any) -> Mapping[str, "np.ndarray"]:
    """
    SB3 saves `policy.pth` usually as a plain `state_dict` for the policy;
    older formats may nest it under "policy".
    """
    import torch as th
    from collections.abc import Mapping as _Mapping  # Python <3.10 compat

    if isinstance(raw_obj, _Mapping):
        # Direct state_dict (common case)
        if "action_net.weight" in raw_obj or "mlp_extractor.policy_net.0.weight" in raw_obj:
            return raw_obj
        # Nested under "policy"
        if "policy" in raw_obj and isinstance(raw_obj["policy"], _Mapping):
            return raw_obj["policy"]

    # As a last resort, if someone saved an OrderedDict directly
    if hasattr(raw_obj, "keys") and all(isinstance(v, th.Tensor) for v in raw_obj.values()):
        return raw_obj  # type: ignore[return-value]

    raise RuntimeError("Could not interpret the loaded object as a policy state_dict.")


def _infer_ckpt_in_out_dims(state_dict: Mapping[str, Any]) -> tuple[Optional[int], Optional[int]]:
    """Infer first-layer input dim and action dim from common SB3 policy keys."""
    in_dim = None
    for k in (
        "mlp_extractor.policy_net.0.weight",   # separate policy net
        "mlp_extractor.shared_net.0.weight",   # shared trunk
        "mlp_extractor.value_net.0.weight",    # fallback
        "features_extractor.mlp.0.weight",     # if custom FE was used
    ):
        if k in state_dict:
            in_dim = int(state_dict[k].shape[1])
            break

    out_dim = None
    if "action_net.weight" in state_dict:
        out_dim = int(state_dict["action_net.weight"].shape[0])

    return in_dim, out_dim


def _make_static_spaces_env(obs_dim: int, act_dim: int):
    """
    Construct a *minimal* Gym/Gymnasium Env so SB3 can initialize a PPO with
    correct spaces. We will not use this env for stepping.
    """
    # Try gymnasium first, then gym
    try:
        import gymnasium as gym  # type: ignore
        gym_version = "gymnasium"
    except Exception:
        import gym  # type: ignore
        gym_version = "gym"

    class StaticSpacesEnv(gym.Env):  # type: ignore
        metadata = {"render_modes": []}

        def __init__(self):
            high_obs = np.full((obs_dim,), np.inf, dtype=np.float32)
            low_obs = -high_obs
            self.observation_space = gym.spaces.Box(low=low_obs, high=high_obs, dtype=np.float32)

            # Default to [-1, 1] bounds; downstream node applies real-world safety limits anyway.
            self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(act_dim,), dtype=np.float32)

        # Implement reset/step for safety; they won't be used during inference.
        def reset(self, *, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None):
            try:
                super().reset(seed=seed)  # type: ignore[misc]
            except Exception:
                pass
            obs = np.zeros((obs_dim,), dtype=np.float32)
            if gym_version == "gymnasium":
                return obs, {}
            return obs

        def step(self, action):
            obs = np.zeros((obs_dim,), dtype=np.float32)
            reward = 0.0
            terminated = True
            truncated = True
            info: Dict[str, Any] = {}
            if gym_version == "gymnasium":
                return obs, reward, terminated, truncated, info
            # legacy gym:
            done = True
            return obs, reward, done, info

    return StaticSpacesEnv()


def secure_load_ppo_from_zip_no_env(checkpoint_zip: str | Path, *, obs_dim: int, act_dim: int, device: str = "cpu"):
    """
    Securely load an SB3 PPO policy from a `.zip` using ONLY safe JSON metadata
    + tensor weights. No training env is required at runtime.

    Returns:
      model: a stable_baselines3.PPO instance ready for `.predict(...)`.
    """
    # Lazy imports to keep ROS node startup lean
    import torch as th
    from stable_baselines3 import PPO

    if not _torch_supports_weights_only():
        raise RuntimeError(
            "This PyTorch build does not support `weights_only=True` on torch.load. "
            "Please upgrade to PyTorch >= 2.0."
        )

    checkpoint_zip = str(checkpoint_zip)
    if not os.path.exists(checkpoint_zip):
        raise FileNotFoundError(f"Policy file not found: {checkpoint_zip}")

    with zipfile.ZipFile(checkpoint_zip, "r") as zf:
        names = set(zf.namelist())

        # Find the SAFE metadata JSON (filename per your training util)
        meta_name = None
        for cand in SAFE_META_CANDIDATES:
            if cand in names:
                meta_name = cand
                break
        if meta_name is None:
            raise FileNotFoundError(
                f"Missing SAFE metadata JSON inside checkpoint zip. Expected one of: {SAFE_META_CANDIDATES}"
            )
        if "policy.pth" not in names:
            raise FileNotFoundError("Missing 'policy.pth' inside the checkpoint zip.")

        meta = json.loads(_read_text(zf, meta_name))
        act_name: str = meta.get("activation_fn", "ReLU")
        net_arch: Any = meta.get("net_arch", dict(pi=[64, 64], vf=[64, 64]))
        use_sde: bool = bool(meta.get("use_sde", False))

        raw = _read_bytes(zf, "policy.pth")

    obj = th.load(io.BytesIO(raw), map_location=device, weights_only=True)
    state_dict = _extract_policy_state_dict(obj)

    ckpt_in, ckpt_out = _infer_ckpt_in_out_dims(state_dict)

    # Dimension checks (helpful for catching mismatches between drone obs & policy)
    if ckpt_in is not None and ckpt_in != int(obs_dim):
        raise RuntimeError(
            f"Observation dimension mismatch: checkpoint expects {ckpt_in}, node configured for {obs_dim}."
        )
    if ckpt_out is not None and ckpt_out != int(act_dim):
        # Not hard error: some policies include distribution parameters (e.g., log_std) separately
        # but action_net.weight rows should equal action_dim for typical Gaussian policies.
        raise RuntimeError(
            f"Action dimension mismatch: checkpoint action head outputs {ckpt_out}, node configured for {act_dim}."
        )

    # Build a tiny static env to wire SB3 policy (never used for stepping)
    fake_env = _make_static_spaces_env(obs_dim=obs_dim, act_dim=act_dim)

    # Prepare policy kwargs
    policy_kwargs: Dict[str, Any] = dict(activation_fn=_parse_activation(act_name), net_arch=net_arch)

    # Instantiate a fresh PPO (policy only will be used)
    model = PPO("MlpPolicy", fake_env, device=device, policy_kwargs=policy_kwargs, use_sde=use_sde)

    # Strictly load weights to the policy
    _ = model.policy.load_state_dict(state_dict, strict=True)

    if getattr(model, "use_sde", False):
        model.policy.reset_noise()

    return model


# ──────────────────────────────────────────────────────────────────────
# ROS 2 Node
# ──────────────────────────────────────────────────────────────────────

class AIFlightNode(Node):
    """
    Loads and executes the Swarm AI model for real-time flight control (no sim env).

    Subscribers:
        /ai/observation (std_msgs/Float32MultiArray): observation array (default 131-D)
        /ai/enable (std_msgs/Bool): Enable/disable AI control
        /safety/override (std_msgs/Bool): Safety override signal

    Publishers:
        /ai/action (geometry_msgs/Twist): Control commands (velocity + yaw rate)
        /ai/status (std_msgs/Float32MultiArray): AI status and diagnostics
        /ai/model_ready (std_msgs/Bool): Model ready status
    """

    def __init__(self):
        super().__init__('ai_flight_node')

        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('prediction_timeout', 0.1)  # 100 ms timeout
        self.declare_parameter('max_velocity', 3.0)        # m/s
        self.declare_parameter('safety_enabled', True)
        self.declare_parameter('obs_dim', 131)
        self.declare_parameter('action_dim', 4)

        # Get parameters
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.prediction_timeout = self.get_parameter('prediction_timeout').get_parameter_value().double_value
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        self.safety_enabled = self.get_parameter('safety_enabled').get_parameter_value().bool_value
        self.obs_dim = int(self.get_parameter('obs_dim').get_parameter_value().integer_value or 131)
        self.action_dim = int(self.get_parameter('action_dim').get_parameter_value().integer_value or 4)

        # State variables
        self.model = None
        self.model_ready = False
        self.ai_enabled = False
        self.safety_override = False
        self.last_observation: Optional[np.ndarray] = None
        self.last_action = np.zeros(self.action_dim, dtype=np.float32)
        self.prediction_count = 0
        self.error_count = 0

        # Threading for model prediction
        self.prediction_queue: "queue.Queue[np.ndarray]" = queue.Queue(maxsize=1)
        self.result_queue: "queue.Queue[tuple[np.ndarray, float]]" = queue.Queue(maxsize=1)
        self.prediction_thread: Optional[threading.Thread] = None
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
        self.action_pub = self.create_publisher(Twist, '/ai/action', reliable_qos)
        self.status_pub = self.create_publisher(Float32MultiArray, '/ai/status', reliable_qos)
        self.ready_pub = self.create_publisher(Bool, '/ai/model_ready', reliable_qos)

        # Timer for status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)

        # Load model in separate thread to avoid blocking
        self.model_load_thread = threading.Thread(target=self.load_model)
        self.model_load_thread.daemon = True
        self.model_load_thread.start()

        self.get_logger().info('AI Flight Node initialized (no-sim loader)')

    # ──────────────────────────────────────────────────────────────────
    # Model loading (NO environment needed for runtime)
    # ──────────────────────────────────────────────────────────────────

    def load_model(self):
        """Load the PPO policy from zip using safe, weights-only loader (runs in separate thread)."""
        try:
            if not self.model_path:
                self.get_logger().error('Model path not specified. Set the "model_path" parameter.')
                return

            model_file = Path(self.model_path)
            if not model_file.exists():
                self.get_logger().error(f'Model file not found: {model_file}')
                return

            self.get_logger().info(f'Loading AI model (no env) from: {model_file}')

            # Load securely with static spaces (obs_dim/action_dim)
            try:
                self.model = secure_load_ppo_from_zip_no_env(
                    model_file, obs_dim=self.obs_dim, act_dim=self.action_dim, device=self.device
                )
            except Exception as e:
                self.get_logger().error(f'Failed to securely load PPO policy: {e}')
                raise

            self.model_ready = True
            self.get_logger().info('AI model loaded successfully')

            # Start prediction thread
            self.thread_running = True
            self.prediction_thread = threading.Thread(target=self.prediction_worker, name="ai_pred_worker")
            self.prediction_thread.daemon = True
            self.prediction_thread.start()

        except Exception as e:
            self.get_logger().error(f'Failed to load AI model: {e}')
            import traceback
            self.get_logger().debug(traceback.format_exc())

    # ──────────────────────────────────────────────────────────────────
    # Inference worker + callbacks
    # ──────────────────────────────────────────────────────────────────

    def prediction_worker(self):
        """Worker thread for model predictions."""
        while self.thread_running:
            try:
                # Wait for prediction request
                obs_array = self.prediction_queue.get(timeout=1.0)

                # Make prediction
                start_time = time.time()
                action, _ = self.model.predict(obs_array, deterministic=True)  # type: ignore[union-attr]
                prediction_time = time.time() - start_time

                # Convert to numpy array and clip to expected length
                if hasattr(action, 'squeeze'):
                    action = action.squeeze()
                action = np.asarray(action, dtype=np.float32)

                try:
                    self.result_queue.put((action, prediction_time), block=False)
                except queue.Full:
                    # If result queue is full, drop the stale result silently
                    pass

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Prediction error: {e}')
                self.error_count += 1

    def observation_callback(self, msg: Float32MultiArray):
        """Process incoming observation and generate control action."""
        if not self.model_ready or not self.ai_enabled or self.safety_override:
            return

        try:
            # Convert to numpy array
            obs_array = np.array(msg.data, dtype=np.float32)

            # Validate observation size
            if obs_array.size != self.obs_dim:
                self.get_logger().warn(f'Invalid observation size: {obs_array.size}, expected {self.obs_dim}')
                return

            self.last_observation = obs_array

            # Queue latest observation (drop older one if queue is full)
            try:
                self.prediction_queue.put(obs_array, block=False)
            except queue.Full:
                try:
                    _ = self.prediction_queue.get_nowait()  # drop
                except queue.Empty:
                    pass
                try:
                    self.prediction_queue.put(obs_array, block=False)
                except queue.Full:
                    pass

            # Non-blocking: check if a fresh result is available
            try:
                action, prediction_time = self.result_queue.get_nowait()
                self.process_action(action)
                self.prediction_count += 1

                if prediction_time > self.prediction_timeout:
                    self.get_logger().warn(f'Slow prediction: {prediction_time:.3f}s')

            except queue.Empty:
                # No result available, reuse last action
                self.process_action(self.last_action)

        except Exception as e:
            self.get_logger().error(f'Observation processing error: {e}')
            self.error_count += 1

    def process_action(self, action: np.ndarray):
        """Process and publish control action (applies safety limits)."""
        try:
            # Ensure correct dimension
            if action.size < self.action_dim:
                action = np.pad(action, (0, self.action_dim - action.size), 'constant')
            elif action.size > self.action_dim:
                action = action[:self.action_dim]

            # Apply safety limits
            if self.safety_enabled:
                # Limit linear velocity magnitude
                v = action[:3]
                v_norm = float(np.linalg.norm(v))
                if v_norm > self.max_velocity and v_norm > 1e-6:
                    action[:3] = v * (self.max_velocity / v_norm)

                # Limit yaw rate
                max_yaw_rate = 2.0  # rad/s
                action[3] = float(np.clip(action[3], -max_yaw_rate, max_yaw_rate))

            self.last_action = action.astype(np.float32, copy=False)

            # Publish Twist
            twist_msg = Twist()
            twist_msg.linear.x = float(action[0])
            twist_msg.linear.y = float(action[1])
            twist_msg.linear.z = float(action[2])
            twist_msg.angular.z = float(action[3])
            self.action_pub.publish(twist_msg)

        except Exception as e:
            self.get_logger().error(f'Action processing error: {e}')

    def enable_callback(self, msg: Bool):
        """Enable/disable AI control."""
        self.ai_enabled = bool(msg.data)
        if self.ai_enabled:
            self.get_logger().info('AI control enabled')
        else:
            self.get_logger().info('AI control disabled')
            # Publish zero action when disabled
            self.action_pub.publish(Twist())

    def safety_callback(self, msg: Bool):
        """Handle safety override."""
        self.safety_override = bool(msg.data)
        if self.safety_override:
            self.get_logger().warn('Safety override activated - AI control disabled')
            # Publish zero action on safety override
            self.action_pub.publish(Twist())

    def publish_status(self):
        """Publish AI system status."""
        try:
            # Model ready flag
            ready_msg = Bool()
            ready_msg.data = bool(self.model_ready)
            self.ready_pub.publish(ready_msg)

            # Detailed status vector
            status_data = [
                float(self.model_ready),
                float(self.ai_enabled),
                float(self.safety_override),
                float(self.prediction_count),
                float(self.error_count),
                float(np.linalg.norm(self.last_action) if self.last_action is not None else 0.0),
                float(time.time() % 1000.0),  # lightweight timestamp
            ]

            status_msg = Float32MultiArray()
            status_msg.data = status_data
            self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Status publishing error: {e}')

    def destroy_node(self):
        """Cleanup when node is destroyed."""
        self.thread_running = False
        if self.prediction_thread and self.prediction_thread.is_alive():
            try:
                self.prediction_thread.join(timeout=1.0)
            except Exception:
                pass
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
