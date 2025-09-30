#!/usr/bin/env python3
"""
AI Flight Node (ROS 2)
- Reads 131-D observations from /ai/observation (std_msgs/Float32MultiArray)
- Loads PPO policy from a SAFE .zip (no env at runtime)
- Runs inference at 10 Hz
- Publishes [vx, vy, vz, speed] to /ai/action (std_msgs/Float32MultiArray)
- Prints each action as it's published

Hardcoded model path (per your requirement):
    /home/pi/swarm-ros/modelUID_117.zip
"""

from __future__ import annotations

import io
import json
import os
import time
import zipfile
from pathlib import Path
from typing import Any, Dict, Mapping, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Float32MultiArray

# ──────────────────────────────────────────────────────────────────────
# Safe PPO loader (no training env at runtime)
# ──────────────────────────────────────────────────────────────────────

SAFE_META_CANDIDATES = ("safe_policy_meta.json",)  # expected in your .zip


def _parse_activation(name: str):
    import torch as th
    m = {
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
    return m.get(key, th.nn.ReLU)


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
    import torch as th
    from collections.abc import Mapping as _Mapping

    if isinstance(raw_obj, _Mapping):
        if "action_net.weight" in raw_obj or "mlp_extractor.policy_net.0.weight" in raw_obj:
            return raw_obj
        if "policy" in raw_obj and isinstance(raw_obj["policy"], _Mapping):
            return raw_obj["policy"]

    if hasattr(raw_obj, "keys") and all(isinstance(v, th.Tensor) for v in raw_obj.values()):
        return raw_obj  # type: ignore[return-value]

    raise RuntimeError("Could not interpret the loaded object as a policy state_dict.")


def _infer_ckpt_in_out_dims(state_dict: Mapping[str, Any]) -> tuple[Optional[int], Optional[int]]:
    in_dim = None
    for k in (
        "mlp_extractor.policy_net.0.weight",
        "mlp_extractor.shared_net.0.weight",
        "mlp_extractor.value_net.0.weight",
        "features_extractor.mlp.0.weight",
    ):
        if k in state_dict:
            in_dim = int(state_dict[k].shape[1])
            break

    out_dim = None
    if "action_net.weight" in state_dict:
        out_dim = int(state_dict["action_net.weight"].shape[0])
    return in_dim, out_dim


def _make_static_spaces_env(obs_dim: int, act_dim: int):
    # Use gymnasium if available, else gym
    try:
        import gymnasium as gym  # type: ignore
        using = "gymnasium"
    except Exception:
        import gym  # type: ignore
        using = "gym"

    class StaticSpacesEnv(gym.Env):  # type: ignore
        metadata = {"render_modes": []}

        def __init__(self):
            high_obs = np.full((obs_dim,), np.inf, dtype=np.float32)
            self.observation_space = gym.spaces.Box(low=-high_obs, high=high_obs, dtype=np.float32)
            self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(act_dim,), dtype=np.float32)

        def reset(self, *, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None):
            try:
                super().reset(seed=seed)  # type: ignore[misc]
            except Exception:
                pass
            obs = np.zeros((obs_dim,), dtype=np.float32)
            if using == "gymnasium":
                return obs, {}
            return obs

        def step(self, action):
            obs = np.zeros((obs_dim,), dtype=np.float32)
            reward = 0.0
            terminated = True
            truncated = True
            info: Dict[str, Any] = {}
            if using == "gymnasium":
                return obs, reward, terminated, truncated, info
            return obs, reward, True, info

    return StaticSpacesEnv()


def secure_load_ppo_from_zip_no_env(checkpoint_zip: str | Path, *, obs_dim: int, act_dim: int, device: str = "cpu"):
    """Securely load an SB3 PPO policy using SAFE metadata + weights only."""
    import torch as th
    from stable_baselines3 import PPO

    if not _torch_supports_weights_only():
        raise RuntimeError(
            "This PyTorch build does not support `weights_only=True`. Please use PyTorch >= 2.0."
        )

    checkpoint_zip = str(checkpoint_zip)
    if not os.path.exists(checkpoint_zip):
        raise FileNotFoundError(f"Policy file not found: {checkpoint_zip}")

    with zipfile.ZipFile(checkpoint_zip, "r") as zf:
        names = set(zf.namelist())
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

    if ckpt_in is not None and ckpt_in != int(obs_dim):
        raise RuntimeError(f"Observation dimension mismatch: checkpoint expects {ckpt_in}, node uses {obs_dim}.")
    if ckpt_out is not None and ckpt_out != int(act_dim):
        raise RuntimeError(f"Action dimension mismatch: checkpoint outputs {ckpt_out}, node uses {act_dim}.")

    env = _make_static_spaces_env(obs_dim=obs_dim, act_dim=act_dim)
    policy_kwargs: Dict[str, Any] = dict(activation_fn=_parse_activation(act_name), net_arch=net_arch)
    model = PPO("MlpPolicy", env, device=device, policy_kwargs=policy_kwargs, use_sde=use_sde)
    _ = model.policy.load_state_dict(state_dict, strict=True)
    if getattr(model, "use_sde", False):
        model.policy.reset_noise()
    return model


# ──────────────────────────────────────────────────────────────────────
# ROS 2 Node
# ──────────────────────────────────────────────────────────────────────

class AIFlightNode(Node):
    """
    Subscribers:
        /ai/observation (std_msgs/Float32MultiArray): 131-D observation

    Publishers:
        /ai/action (std_msgs/Float32MultiArray): [vx, vy, vz, speed]

    Behavior:
        - Loads PPO policy from a fixed path
        - Inference & publish at 10 Hz
        - Logs every action line-by-line
    """

    # Hard requirements from your request
    MODEL_PATH = "/home/pi/swarm-ros/model/UID_117.zip"
    OBS_DIM = 131
    ACT_DIM = 4
    PREDICT_HZ = 10.0  # infer 10 times per second

    def __init__(self):
        super().__init__("ai_flight_node")

        # QoS: reliable, keep last
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        self.obs_sub = self.create_subscription(
            Float32MultiArray, "/ai/observation", self.observation_callback, qos
        )

        # Publisher: /ai/action -> Float32MultiArray([vx,vy,vz,speed])
        self.action_pub = self.create_publisher(Float32MultiArray, "/ai/action", qos)

        # Internal state
        self.model = None
        self.model_ready = False
        self.last_obs: Optional[np.ndarray] = None
        self.tick_counter = 0
        self.last_action = np.zeros((self.ACT_DIM,), dtype=np.float32)

        # Load model (sync at startup to keep things simple & robust)
        try:
            self.get_logger().info(f"Loading AI model from: {self.MODEL_PATH}")
            self.model = secure_load_ppo_from_zip_no_env(
                self.MODEL_PATH, obs_dim=self.OBS_DIM, act_dim=self.ACT_DIM, device="cpu"
            )
            self.model_ready = True
            self.get_logger().info("AI model loaded successfully (ready for inference).")
        except Exception as e:
            self.get_logger().error(f"Failed to load AI model: {e}")
            self.get_logger().error("Node will still run, but will publish zeros until the model is available.")

        # 10 Hz inference timer
        period = 1.0 / self.PREDICT_HZ
        self.timer = self.create_timer(period, self._inference_tick)

        self.get_logger().info("AI Flight Node started: listening on /ai/observation, publishing to /ai/action.")

    # ──────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────

    def observation_callback(self, msg: Float32MultiArray):
        """Receive latest observation. We just store it; inference runs on a fixed 10 Hz timer."""
        try:
            # Convert and validate
            obs = np.asarray(msg.data, dtype=np.float32).reshape(-1)
            if obs.size != self.OBS_DIM:
                self.get_logger().warn(
                    f"Received observation size {obs.size} but expected {self.OBS_DIM}. Ignoring this message."
                )
                return
            self.last_obs = obs
        except Exception as e:
            self.get_logger().error(f"Observation processing error: {e}")

    # ──────────────────────────────────────────────────────────────────
    # Inference + publishing at 10 Hz
    # ──────────────────────────────────────────────────────────────────

    def _inference_tick(self):
        self.tick_counter += 1

        # Prepare observation
        if self.last_obs is None or not self.model_ready:
            # No obs or no model → publish zeros but still at 10 Hz
            action = np.zeros((self.ACT_DIM,), dtype=np.float32)
            self._publish_action(action, inference_ms=None, used_obs=False)
            return

        obs = self.last_obs

        # Predict
        try:
            t0 = time.perf_counter()
            action, _ = self.model.predict(obs, deterministic=True)  # type: ignore[union-attr]
            dt_ms = (time.perf_counter() - t0) * 1e3
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")
            action = np.zeros((self.ACT_DIM,), dtype=np.float32)
            self._publish_action(action, inference_ms=None, used_obs=True)
            return

        # Sanitize, shape, and publish
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.size != self.ACT_DIM:
            # Pad or trim to [vx, vy, vz, speed]
            if action.size < self.ACT_DIM:
                action = np.pad(action, (0, self.ACT_DIM - action.size), mode="constant")
            else:
                action = action[: self.ACT_DIM]

        # Replace NaNs/Infs with safe zeros
        action = np.nan_to_num(action, nan=0.0, posinf=0.0, neginf=0.0)

        self._publish_action(action, inference_ms=dt_ms, used_obs=True)

    def _publish_action(self, action: np.ndarray, *, inference_ms: Optional[float], used_obs: bool):
        self.last_action = action.astype(np.float32, copy=False)

        # Compose ROS message
        msg = Float32MultiArray()
        # layout left empty intentionally (flat vector)
        msg.data = [float(x) for x in self.last_action]  # [vx, vy, vz, speed]

        # Publish
        self.action_pub.publish(msg)

        # Print/Log each action (so you can watch live in the node logs)
        if inference_ms is None:
            info_note = "(no model/obs)" if not used_obs else "(fallback)"
        else:
            info_note = f"{inference_ms:.2f} ms"

        vx, vy, vz, spd = [float(x) for x in self.last_action]
        self.get_logger().info(
            f"[tick {self.tick_counter:06d}] /ai/action -> "
            f"vx={vx:+.3f}, vy={vy:+.3f}, vz={vz:+.3f}, speed={spd:+.3f}   [{info_note}]"
        )

    # ──────────────────────────────────────────────────────────────────

    def destroy_node(self):
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


if __name__ == "__main__":
    main()
