"""Shared helpers used by VLA backends (in-process and WebSocket).

Kept separate from backends.py so backend implementations stay focused on
policy loading/transport, while observation/completion primitives are
reusable and independently testable.
"""

from __future__ import annotations

from typing import Optional

import numpy as np


def as_rgb_uint8(image) -> np.ndarray:
    """Validate and normalize an image to a contiguous (H, W, 3) uint8 array."""
    image = np.asarray(image)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("Expected an RGB image with shape (H, W, 3), got {}".format(image.shape))
    return np.ascontiguousarray(np.clip(image, 0, 255).astype(np.uint8))


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


def gripper_occupied(sink) -> bool:
    """True if the classic Grab/Place FSM has recorded the gripper as holding an item.

    Only reflects classic bookkeeping; always False on a pure VLA run where no
    classic FSM has touched `occupied_by`.
    """
    return sink._get_arm().gripper.occupied_by is not None


def gripper_grasped_by_position(
    position: Optional[float], threshold: Optional[float], direction: str
) -> Optional[bool]:
    """Physical stand-in for `occupied_by`: is the gripper holding something?

    Returns None when the threshold is unset (feature disabled) or the joint
    reading is unavailable, so callers can distinguish "unknown" from "empty".
    """
    if threshold is None or position is None:
        return None
    if direction == "below":
        return position < threshold
    return position > threshold


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
