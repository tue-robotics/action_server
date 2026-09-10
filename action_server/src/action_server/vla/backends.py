"""Concrete VLA backend implementations.

Each backend follows the contract expected by LocalBackendVLAProvider:

    __init__(self, robot)                       # robot is a live robot_skills object
    execute(self, request) -> dict              # request is a ManipulationRequest

The returned dict must contain:
    "actions":   list of executed trajectory steps (each length action_dim), for logging
    "succeeded": bool
    "message":   str (optional)

A backend owns four responsibilities:
    1. Observation  - read head/hand cameras and proprioceptive state via robot_skills.
    2. Prompting    - use ManipulationRequest.raw_sentence as the VLA language instruction.
    3. Inference    - turn (images, state, instruction) into an action chunk.
    4. Actuation    - play the action chunk on the robot's arm/gripper/head/base.

Backends are selected at runtime through the ROS param:
    /<robot_name>/action_server/vla/local_backend_class
For example:
    action_server.vla.backends:SmolVLALocalBackend
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Dict, List, Optional

import numpy as np
import rospy
from sensor_msgs.msg import Image

from .executor import ManipulationRequest


class BaseBackend(ABC):
    """Abstract base for all VLA backends.

    Kept separate from BaseVLAProvider on purpose:
    - BaseVLAProvider is the *provider interface* (action_server internal, replaceable transport).
    - BaseBackend is the *model interface* (a concrete VLA implementation).
    This lets a different VLA be dropped in by writing one subclass, without
    touching the provider layer or any challenge/action code.
    """

    def __init__(self, robot):
        self.robot = robot
        self.robot_name = robot.robot_name

    @abstractmethod
    def execute(self, request: ManipulationRequest) -> Dict:
        """Run one manipulation episode and return an outcome dict."""
        raise NotImplementedError


class HSRObservationSource:
    """Reads observations for a Toyota HSR through the robot_skills interfaces.

    The VLA was trained on:
        head_rgb    uint8 (H, W, 3)   head RGB camera
        hand_rgb    uint8 (H, W, 3)   in-hand RGB camera
        state       float32 (8,)      5 arm + 1 gripper + 2 head joints
        instruction str               natural-language prompt

    Joint names come from ROS params with HSR defaults, so the same backend
    keeps working if the URDF/controllers are renamed.
    """

    # Order must match the checkpoint's training layout (see per-group-mse-vla docs).
    DEFAULT_ARM_JOINTS = [
        "arm_lift_joint",
        "arm_flex_joint",
        "arm_roll_joint",
        "wrist_flex_joint",
        "wrist_roll_joint",
    ]
    DEFAULT_GRIPPER_JOINT = "hand_motor_joint"
    DEFAULT_HEAD_JOINTS = ["head_pan_joint", "head_tilt_joint"]

    def __init__(self, robot):
        self.robot = robot
        base = "/{}/action_server/vla".format(robot.robot_name)
        self._arm_joints = rospy.get_param(
            base + "/state_arm_joints", self.DEFAULT_ARM_JOINTS
        )
        self._gripper_joint = rospy.get_param(
            base + "/state_gripper_joint", self.DEFAULT_GRIPPER_JOINT
        )
        self._head_joints = rospy.get_param(
            base + "/state_head_joints", self.DEFAULT_HEAD_JOINTS
        )
        self._image_timeout = rospy.get_param(base + "/image_timeout", 5)
        self._bridge = None  # lazy cv_bridge

    def _cv_bridge(self):
        if self._bridge is None:
            from cv_bridge import CvBridge

            self._bridge = CvBridge()
        return self._bridge

    def _image_to_numpy(self, image_msg) -> np.ndarray:
        """Convert a sensor_msgs/Image to an (H, W, 3) uint8 RGB array."""
        return self._cv_bridge().imgmsg_to_cv2(image_msg, "rgb8")

    def get_head_rgb(self) -> Optional[np.ndarray]:
        img = self.robot.perception.get_image(timeout=self._image_timeout)
        return self._image_to_numpy(img) if img is not None else None

    def _hand_camera_cb(self, image_msg):
        self._hand_camera_last_image = image_msg

    def get_hand_rgb(self) -> Optional[np.ndarray]:
        """Return the latest in-hand RGB frame, or None if unavailable within timeout.

        Subscribes directly to the HSR hand camera topic (usb_cam, started via
        use_hand_camera:=true in hero_bringup) since it is not exposed as a
        robot_skills Perception part.
        """
        if self._hand_camera_sub is None:
            self._hand_camera_last_image = None
            self._hand_camera_sub = rospy.Subscriber(
                self._hand_camera_topic, Image, self._hand_camera_cb
            )

        rate = rospy.Rate(10)
        deadline = rospy.Time.now() + rospy.Duration(self._image_timeout)
        while self._hand_camera_last_image is None and rospy.Time.now() < deadline:
            if rospy.is_shutdown():
                return None
            rate.sleep()

        if self._hand_camera_last_image is None:
            rospy.logwarn(
                "[VLA] No hand camera image received on %s within %ss",
                self._hand_camera_topic,
                self._image_timeout,
            )
            return None

    def get_state(self) -> Optional[np.ndarray]:
        """Assemble the 8-D proprioceptive vector in the trained joint order."""
        joint_states = self.robot.get_joint_states()  # {joint_name: position}
        ordered = self._arm_joints + [self._gripper_joint] + self._head_joints
        try:
            return np.array([joint_states[name] for name in ordered], dtype=np.float32)
        except KeyError as missing:
            rospy.logwarn(
                "[VLA] Joint %s not in /joint_states; cannot build state vector",
                missing,
            )
            return None

    def get_observation(
        self, instruction: str, require_hand: bool = False
    ) -> Optional[Dict]:
        """Return a full observation dict, or None if required inputs are missing."""
        head_rgb = self.get_head_rgb()
        state = self.get_state()
        if head_rgb is None or state is None:
            return None

        obs = {"head_rgb": head_rgb, "state": state, "instruction": instruction}
        hand_rgb = self.get_hand_rgb()
        if hand_rgb is not None:
            obs["hand_rgb"] = hand_rgb
        elif require_hand:
            rospy.logwarn("[VLA] Hand camera required but not available")
            return None
        return obs


class HSRActionSink:
    """Plays a variable-width action chunk on a Toyota HSR through robot_skills.

    Supports 6/11-column chunks; columns are consumed by position:
        arm(5)  : arm_lift, arm_flex, arm_roll, wrist_flex, wrist_roll   -> arm6
        grip(1) : gripper open/close scalar                              -> arm6
        head(2) : head_pan, head_tilt                                    -> hsr11
        base(3) : base_x, base_y, base_theta                            -> hsr11
    The arm slice maps 1:1 onto _send_joint_trajectory, which prepends the
    torso (arm_lift) joint when given 5 references. Gripper/head/base are only
    applied when the chunk is wide enough, so the narrower layout is safe.
    """

    ARM_DIM = 5
    GRIP_IDX = 5
    HEAD_SLICE = slice(6, 8)
    BASE_SLICE = slice(8, 11)

    def __init__(self, robot):
        self.robot = robot
        base = "/{}/action_server/vla".format(robot.robot_name)
        self._max_joint_vel = rospy.get_param(base + "/max_joint_vel", 0.7)
        self._gripper_close_threshold = rospy.get_param(
            base + "/gripper_close_threshold", 0.5
        )
        self._base_step_duration = rospy.get_param(base + "/base_step_duration", 0.1)
        # Default to arm-only manipulation: the VLA drives the arm+gripper, not the whole robot.
        self._enable_base = rospy.get_param(base + "/enable_base_motion", False)
        self._enable_head = rospy.get_param(base + "/enable_head_motion", False)
        self._arm = None  # resolved lazily

    def _get_arm(self):
        if self._arm is None:
            self._arm = self.robot.get_arm()
        return self._arm

    def play_chunk(self, chunk: np.ndarray, step_timeout: float) -> bool:
        """Execute an (T, 11) chunk. Returns True if the arm trajectory succeeded."""
        if chunk.ndim != 2 or chunk.shape[1] < self.ARM_DIM:
            rospy.logwarn("[VLA] Unexpected chunk shape %s", chunk.shape)
            return False

        arm_ok = self._play_arm(chunk[:, : self.ARM_DIM], step_timeout)
        if chunk.shape[1] > self.GRIP_IDX:
            self._apply_gripper(chunk[-1, self.GRIP_IDX])
        if self._enable_head and chunk.shape[1] >= self.HEAD_SLICE.stop:
            self._apply_head(chunk[-1, self.HEAD_SLICE])
        if self._enable_base and chunk.shape[1] >= self.BASE_SLICE.stop:
            self._apply_base(chunk[:, self.BASE_SLICE])
        return arm_ok

    def _play_arm(self, arm_chunk: np.ndarray, step_timeout: float) -> bool:
        arm = self._get_arm()
        trajectory = [row.tolist() for row in arm_chunk]
        # _send_joint_trajectory lives on the underlying Arm; PublicArm exposes it as _arm.
        underlying = getattr(arm, "_arm", arm)
        return underlying._send_joint_trajectory(
            trajectory, max_joint_vel=self._max_joint_vel, timeout=step_timeout
        )

    def _apply_gripper(self, gripper_value: float):
        arm = self._get_arm()
        state = (
            "close" if float(gripper_value) >= self._gripper_close_threshold else "open"
        )
        arm.gripper.send_goal(state, timeout=2.0)

    def _apply_head(self, head_values: np.ndarray):
        pan, tilt = float(head_values[0]), float(head_values[1])
        # goal_type=1 commands the head joints directly (pan/tilt) via HeadReference.
        self.robot.head._setHeadReferenceGoal(1, 1.0, 0.8, 0, pan=pan, tilt=tilt)

    def _apply_base(self, base_chunk: np.ndarray):
        # PLACEHOLDER: base_x/base_y/base_theta interpretation depends on the
        # checkpoint (velocity commands vs pose deltas). This assumes the last
        # row is a body-frame velocity and drives it briefly via force_drive.
        # To finalize: confirm the training convention and, if needed, integrate
        # the whole chunk instead of only the final row.
        vx, vy, vth = (float(v) for v in base_chunk[-1])
        self.robot.base.force_drive(vx, vy, vth, self._base_step_duration)


class SmolVLALocalBackend(BaseBackend):
    """On-robot backend that runs SmolVLA in-process on the robot's GPU.

    Model loading and inference are delegated to the Server class from
    per-group-mse-vla/inference/policy_server.py, which already implements
    checkpoint loading, normalization stats and predict_action_chunk(). This
    backend adds the robot_skills observation/actuation layer and the
    closed-loop control around it.

    Configure via ROS params (all under /<robot>/action_server/vla):
        local_backend_class : action_server.vla.backends:SmolVLALocalBackend
        checkpoint_path     : /path/to/smolvla/checkpoint
        device              : cuda
        action_layout       : hsr11
        max_chunks          : max re-query iterations before giving up
        chunk_prefix_steps  : how many steps of each chunk to execute before re-querying
    """

    def __init__(self, robot):
        BaseBackend.__init__(self, robot)
        self._obs = HSRObservationSource(robot)
        self._sink = HSRActionSink(robot)
        self._server = None  # per-group-mse-vla inference.policy_server.Server

        base = "/{}/action_server/vla".format(robot.robot_name)
        self._checkpoint_path = rospy.get_param(base + "/checkpoint_path", "")
        self._device = rospy.get_param(base + "/device", "cuda")
        self._action_layout = rospy.get_param(base + "/action_layout", "hsr11")
        self._max_chunks = rospy.get_param(base + "/max_chunks", 20)
        self._chunk_prefix_steps = rospy.get_param(base + "/chunk_prefix_steps", 10)
        self._step_timeout = rospy.get_param(base + "/step_timeout", 10.0)

    # -- items 1-2: model loading + inference ------------------------------

    def _load_policy(self):
        """Lazily construct the per-group-mse-vla Server (model + tokenizer + norm stats)."""
        if self._server is not None:
            return
        if not self._checkpoint_path:
            raise RuntimeError("VLA checkpoint_path ROS param not set")

        from pathlib import Path

        # Reuse the model/inference stack from the per-group-mse-vla repo directly.
        from inference.policy_server import Server

        self._server = Server(
            checkpoint_dir=Path(self._checkpoint_path),
            device=self._device,
            action_layout=self._action_layout,
        )
        rospy.loginfo(
            "[VLA] SmolVLA policy loaded from %s on %s",
            self._checkpoint_path,
            self._device,
        )

    def _infer(self, obs: Dict) -> np.ndarray:
        """Return an (T, action_dim) action chunk in robot units."""
        return self._server.infer(obs)

    # -- item 7: episode termination --------------------------------------

    def _is_episode_done(
        self, request: ManipulationRequest, executed_steps: int
    ) -> bool:
        # PLACEHOLDER: the VLA has no intrinsic stop condition. This should
        # decide success/termination, e.g. by checking gripper.occupied_by for a
        # pick, a placement/force-sensor cue for a place, or a learned success
        # detector. For now it never stops early and relies on max_chunks.
        return False

    # -- items 5-6-8: control loop, actuation, response mapping -----------

    def execute(self, request: ManipulationRequest) -> Dict:
        self._load_policy()

        instruction = request.raw_sentence or request.action_name
        executed: List[List[float]] = []

        for _iteration in range(self._max_chunks):
            obs = self._obs.get_observation(instruction)
            if obs is None:
                return {
                    "succeeded": False,
                    "actions": executed,
                    "message": "No observation available (cameras or joint states not ready)",
                }

            chunk = self._infer(obs)  # (T, action_dim), robot units

            # Closed-loop: execute a prefix of the chunk, then re-query with a fresh observation.
            prefix = (
                chunk[: self._chunk_prefix_steps]
                if self._chunk_prefix_steps > 0
                else chunk
            )
            arm_ok = self._sink.play_chunk(prefix, step_timeout=self._step_timeout)
            executed.extend(row.tolist() for row in prefix)

            if not arm_ok:
                return {
                    "succeeded": False,
                    "actions": executed,
                    "message": "Arm trajectory execution failed during VLA rollout",
                }

            if self._is_episode_done(request, len(executed)):
                return {
                    "succeeded": True,
                    "actions": executed,
                    "message": "VLA completed '{}'".format(request.action_name),
                }

        # Ran out of iterations without a success signal.
        return {
            "succeeded": False,
            "actions": executed,
            "message": "VLA reached max_chunks without a completion signal",
        }
