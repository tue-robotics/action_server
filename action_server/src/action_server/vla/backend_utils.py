"""Shared helpers used by VLA backends (in-process and WebSocket).

Kept separate from backends.py so backend implementations stay focused on
policy loading/transport, while observation/completion primitives are
reusable and independently testable.
"""

from __future__ import annotations

import json
from typing import Optional

import numpy as np


def as_rgb_uint8(image) -> np.ndarray:
    """Validate and normalize an image to a contiguous (H, W, 3) uint8 array."""
    image = np.asarray(image)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("Expected an RGB image with shape (H, W, 3), got {}".format(image.shape))
    return np.ascontiguousarray(np.clip(image, 0, 255).astype(np.uint8))


def clamp_symmetric(value: float, limit: Optional[float]) -> float:
    """Clamp value into [-limit, limit]. A None/non-positive limit disables clamping.

    Used to bound VLA base velocities before force_drive, which bypasses the
    collision-avoidance layer.
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


def gripper_action(
    value: float, grasping: bool, close_threshold: float, open_threshold: float
) -> Optional[float]:
    """Map the absolute gripper column (hand_motor_joint, ~-0.8 closed, ~1.24 open) to a command.

    Follows the 'hybrid' gripper mode of the reference AIRoA client: below
    close_threshold grasp with effort, otherwise follow the column as a
    position. An effort grasp is only released once the column reaches
    open_threshold, because while holding an object the column stays near the
    object's width. Returns None to grasp (or keep grasping) with effort, else
    the hand_motor_joint position target.
    """
    if value < close_threshold or (grasping and value < open_threshold):
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


def gripper_occupied(sink) -> bool:
    """True if the classic Grab/Place FSM has recorded the gripper as holding an item.

    Only reflects classic bookkeeping; always False on a pure VLA run where no
    classic FSM has touched `occupied_by`.
    """
    return sink._get_arm().gripper.occupied_by is not None


def gripper_holding_object(
    commanded_state: Optional[str],
    seconds_since_command: Optional[float],
    position: Optional[float],
    empty_closed_threshold: Optional[float],
    settle_time: float,
) -> Optional[bool]:
    """Physical stand-in for `occupied_by`: is the gripper holding something?

    An open gripper also reads above the empty-closed position, so position
    alone is ambiguous: the gripper must have been commanded closed and given
    settle_time to close. It then holds an object when the fingers stopped
    above empty_closed_threshold instead of closing completely.

    Returns None when the threshold is unset (feature disabled) or the joint
    reading is unavailable, so callers can distinguish "unknown" from "empty".
    """
    if empty_closed_threshold is None or position is None:
        return None
    if commanded_state != "close" or seconds_since_command is None or seconds_since_command < settle_time:
        return False
    return position > empty_closed_threshold


def is_manipulation_done(action_name: str, occupied_now: bool, occupied_at_start: bool) -> bool:
    """Generic pick/place/hand-over completion rule from a binary gripper signal.

    pick-up succeeds on empty->grasped, place/hand-over succeed on
    grasped->empty. Other actions have no known completion signal here.
    """
    if action_name == "pick-up":
        return occupied_now and not occupied_at_start
    if action_name in ("place", "hand-over"):
        return occupied_at_start and not occupied_now
    return False
