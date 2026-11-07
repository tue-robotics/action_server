"""Pure helpers of the VLA backend: image packing, joint limits, the action contract, debug records.

Kept separate from backends.py so they can be tested without ROS.
"""

from __future__ import annotations

import json
from typing import List, Optional, Tuple

import numpy as np
from urdf_parser_py.urdf import URDF


def as_rgb_uint8(image) -> np.ndarray:
    """Validate and normalize an image to a contiguous (H, W, 3) uint8 array."""
    image = np.asarray(image)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("Expected an RGB image with shape (H, W, 3), got {}".format(image.shape))
    return np.ascontiguousarray(np.clip(image, 0, 255).astype(np.uint8))


def clamp_symmetric(value: float, limit: Optional[float]) -> float:
    """Clamp value into [-limit, limit]. A None/non-positive limit disables clamping.

    Used to bound the VLA base velocities, which bypass the collision-avoidance layer.
    """
    value = float(value)
    if limit is None or limit <= 0:
        return value
    return max(-float(limit), min(float(limit), value))


def apply_relative(current, delta, lower=None, upper=None, max_delta=None) -> np.ndarray:
    """Turn a relative joint action into an absolute joint target.

    The HSR-SmolVLA arm and head columns are deltas with respect to the
    measured joint state (AIRoA MoMa 'action.relative'), so the target is
    current + delta. max_delta bounds each delta, lower/upper the target.
    """
    delta = np.asarray(delta, dtype=np.float64)
    if max_delta is not None:
        max_delta = np.asarray(max_delta, dtype=np.float64)
        delta = np.clip(delta, -max_delta, max_delta)
    target = np.asarray(current, dtype=np.float64) + delta
    if lower is not None or upper is not None:
        target = np.clip(target, lower, upper)
    return target


def joint_limits(urdf_xml: str, joint_names: List[str]) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """Position and velocity limits (lower, upper, velocity) of joint_names in a URDF, in that order.

    Returns None if the URDF is empty or one of the joints has no limit.
    """
    if not urdf_xml:
        return None
    joints = {joint.name: joint for joint in URDF.from_xml_string(urdf_xml).joints}
    limits = [getattr(joints.get(name), "limit", None) for name in joint_names]
    if any(limit is None or None in (limit.lower, limit.upper, limit.velocity) for limit in limits):
        return None
    return tuple(np.array([getattr(limit, field) for limit in limits], dtype=np.float64)
                 for field in ("lower", "upper", "velocity"))


def gripper_action(value: float, close_threshold: float) -> Optional[float]:
    """Map the absolute gripper column (hand_motor_joint, ~-0.8 closed, ~1.24 open) to a command.

    The 'hybrid' gripper mode of the reference AIRoA client: below
    close_threshold grasp with effort, otherwise follow the column as a
    position. Returns None to grasp with effort, else the hand_motor_joint
    position target.
    """
    if value < close_threshold:
        return None
    return float(np.clip(value, -0.1, 1.23))


def pack_image_binary(image: np.ndarray) -> dict:
    """Encode an RGB image as msgpack-friendly raw bytes plus shape.

    Used instead of nested per-pixel lists, which are large and slow to pack.
    """
    image = as_rgb_uint8(image)
    return {"data": image.tobytes(), "shape": tuple(int(dimension) for dimension in image.shape)}


def unpack_image_binary(value) -> np.ndarray:
    """Decode a pack_image_binary() payload back into an (H, W, 3) uint8 array."""
    shape = tuple(int(dimension) for dimension in value["shape"])
    image = np.frombuffer(value["data"], dtype=np.uint8)
    return image.reshape(shape)


def query_debug_message(iteration: int, instruction: str, state, chunk, prefix_steps: int) -> str:
    """JSON record of one policy query: the observed state and the full chunk the policy returned.

    Published for rosbag recordings, so what the policy asked for can be
    compared with what the controllers did, moment to moment.
    """
    return json.dumps({
        "iteration": int(iteration),
        "instruction": str(instruction),
        "state": np.asarray(state, dtype=np.float64).tolist(),
        "chunk": np.asarray(chunk, dtype=np.float64).tolist(),
        "prefix_steps": int(prefix_steps),
    })

