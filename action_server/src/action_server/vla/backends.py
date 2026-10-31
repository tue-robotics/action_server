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
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .backend_utils import (
    apply_relative,
    as_rgb_uint8,
    clamp_symmetric,
    gripper_action,
    gripper_holding_object,
    gripper_occupied,
    is_manipulation_done,
    pack_image_binary,
    query_debug_message,
    shape_arm_delta,
)
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


class JointStateCache:
    """Latest joint positions from one persistent /<robot>/joint_states subscriber.

    robot.get_joint_states() creates a new subscriber and waits for a message on
    every call, which is too slow to stream commands at the policy rate.
    """

    def __init__(self, topic: str):
        self._positions = None
        self._subscriber = rospy.Subscriber(topic, JointState, self._callback, queue_size=1)

    def _callback(self, msg: JointState):
        self._positions = dict(zip(msg.name, msg.position))

    def get(self, timeout: float = 1.0) -> Optional[Dict[str, float]]:
        deadline = rospy.Time.now() + rospy.Duration.from_sec(timeout)
        while self._positions is None and rospy.Time.now() < deadline and not rospy.is_shutdown():
            rospy.sleep(0.01)
        return self._positions


class HSRObservationSource:
    """Reads observations for a Toyota HSR through the robot_skills interfaces.

    Default state layout (checkpoint-compatible with the released SmolVLA):
        head_rgb    uint8 (H, W, 3)   head RGB camera
        hand_rgb    uint8 (H, W, 3)   in-hand RGB camera
        state       float32 (N,)     joints from `state_joint_groups`, in order
        instruction str               natural-language prompt

    The state vector is NOT hardcoded to arm+gripper+head: `state_joint_groups`
    (a ROS param) selects which joint groups are concatenated and in what
    order, so a different VLA's state contract can be selected via config
    instead of editing this class. Joint *names* within each group also come
    from ROS params with HSR defaults, so the same backend keeps working if
    the URDF/controllers are renamed.
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
    # Default matches the released checkpoint's 8-D state (arm+gripper+head).
    DEFAULT_STATE_JOINT_GROUPS = ["arm", "gripper", "head"]

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
        # Which joint groups make up the state vector, and in what order.
        # Lets a different VLA select e.g. ["arm", "gripper"] (6-D) or
        # ["head", "arm", "gripper"] without touching this class.
        self._state_joint_groups = rospy.get_param(
            base + "/state_joint_groups", self.DEFAULT_STATE_JOINT_GROUPS
        )
        self._image_timeout = rospy.get_param(base + "/image_timeout", 5)
        self._hand_camera_topic = rospy.get_param(
            base + "/hand_camera_topic", "/{}/hand_camera/image_raw".format(robot.robot_name)
        )
        self._bridge = None  # lazy cv_bridge
        self._hand_camera_sub = None
        self._hand_camera_last_image = None

    def _cv_bridge(self):
        if self._bridge is None:
            from cv_bridge import CvBridge

            self._bridge = CvBridge()
        return self._bridge

    def _image_to_numpy(self, image_msg) -> np.ndarray:
        """Convert a sensor_msgs/Image to an (H, W, 3) uint8 RGB array."""
        return as_rgb_uint8(self._cv_bridge().imgmsg_to_cv2(image_msg, "rgb8"))

    def get_head_rgb(self) -> Optional[np.ndarray]:
        img = self.robot.perception.get_image(timeout=int(self._image_timeout))
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

        return self._image_to_numpy(self._hand_camera_last_image)

    def _joint_group(self, name: str) -> List[str]:
        if name == "arm":
            return list(self._arm_joints)
        if name == "gripper":
            return [self._gripper_joint]
        if name == "head":
            return list(self._head_joints)
        raise ValueError(
            "Unknown state_joint_groups entry '{}'; expected 'arm', 'gripper', or 'head'".format(name)
        )

    def get_state(self) -> Optional[np.ndarray]:
        """Assemble the proprioceptive vector from the configured joint groups.

        Order and composition come from `state_joint_groups` (ROS param), so
        the state contract is selected per-VLA via config, not hardcoded here.
        """
        joint_states = self.robot.get_joint_states()  # {joint_name: position}
        ordered = [
            joint_name
            for group in self._state_joint_groups
            for joint_name in self._joint_group(group)
        ]
        try:
            return np.array([joint_states[name] for name in ordered], dtype=np.float32)
        except KeyError as missing:
            rospy.logwarn(
                "[VLA] Joint %s not in /joint_states; cannot build state vector",
                missing,
            )
            return None

    def get_gripper_position(self) -> Optional[float]:
        """Raw hand_motor_joint reading, used as a physical (non-ED) grasp signal."""
        joint_states = self.robot.get_joint_states()
        return joint_states.get(self._gripper_joint)

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
    """Streams an action chunk of the HSR-SmolVLA policy onto a Toyota HSR, one row per policy step.

    The checkpoint's 'action' is the AIRoA 'action.relative' layout (its
    normalisation statistics are identical):
        arm(5)  : arm_lift, arm_flex, arm_roll, wrist_flex, wrist_roll   delta w.r.t. the measured joints [m, rad]
        grip(1) : hand_motor_joint                                       absolute, ~-0.8 closed .. ~1.24 open [rad]
        head(2) : head_pan, head_tilt                                    delta w.r.t. the measured joints [rad]
        base(3) : base_x, base_y, base_theta                             body-frame velocity [m/s, m/s, rad/s]
    Every row is played for 1/policy_fps seconds (10 Hz, like the reference
    AIRoA client airoa-org/airoa-evaluation-ICRA). The arm target is the
    measured state plus the delta, streamed to the arm trajectory controller,
    the base velocity is published directly, the gripper follows the reference
    client's 'hybrid' mode and the head is commanded once per chunk.
    Gripper/head/base are only applied when the chunk is wide enough, so a
    6-wide arm6 chunk is safe.
    """

    ARM_DIM = 5
    GRIP_IDX = 5
    HEAD_SLICE = slice(6, 8)
    BASE_SLICE = slice(8, 11)
    # HSR arm joint limits (hsrb4s.urdf) in the order of the arm columns
    ARM_LOWER = [0.0, -2.62, -2.09, -1.92, -1.92]
    ARM_UPPER = [0.69, 0.0, 3.84, 1.22, 3.67]

    def __init__(self, robot):
        self.robot = robot
        base = "/{}/action_server/vla".format(robot.robot_name)
        self._fps = float(rospy.get_param(base + "/policy_fps", 10.0))
        self._arm_joints = rospy.get_param(
            base + "/state_arm_joints", HSRObservationSource.DEFAULT_ARM_JOINTS
        )
        self._gripper_joint = rospy.get_param(
            base + "/state_gripper_joint", HSRObservationSource.DEFAULT_GRIPPER_JOINT
        )
        self._head_joints = rospy.get_param(
            base + "/state_head_joints", HSRObservationSource.DEFAULT_HEAD_JOINTS
        )
        # The arm deltas are the lead of the teleoperation command over the measured
        # state, so the arm speed is roughly delta / arm_lead_time. The default bound
        # is the URDF velocity limit times the default lead time.
        self._arm_lead_time = float(rospy.get_param(base + "/arm_lead_time", 0.2))
        self._max_arm_delta = rospy.get_param(
            base + "/max_arm_delta", [0.04, 0.24, 0.4, 0.3, 0.3]
        )
        # Corrections for the simulated arm (see shape_arm_delta), per joint or one value;
        # the defaults are the reference client.
        self._arm_delta_gain = self._arm_param(base + "/arm_delta_gain", 1.0)
        self._arm_hold_threshold = self._arm_param(base + "/arm_hold_threshold", 0.0)
        self._arm_target = None
        # Below the close threshold the gripper grasps with effort (0.26 * 0.07 m
        # moment arm = the reference client's 0.018 Nm), above it the column is
        # followed as a position. A grasp is released at the open threshold, or like
        # the reference client once the fingers closed below grasp_position_threshold
        # (nothing in the hand).
        self._gripper_close_threshold = float(
            rospy.get_param(base + "/gripper_close_threshold", 0.5)
        )
        self._gripper_open_threshold = float(rospy.get_param(base + "/gripper_open_threshold", 0.9))
        self._gripper_max_torque = float(rospy.get_param(base + "/gripper_max_torque", 0.26))
        self.grasp_position_threshold = rospy.get_param(base + "/grasp_position_threshold", None)
        self._head_vel = rospy.get_param(base + "/head_vel", 1.0)
        # How to interpret the base(3) columns:
        #   "velocity"   : row = (vx, vy, vth) body-frame velocity [m/s, m/s, rad/s]
        #   "pose_delta" : row = (dx, dy, dtheta) per-step displacement, times policy_fps
        #   "disabled"   : ignore base columns even if enable_base_motion is set
        self._base_action_mode = rospy.get_param(base + "/base_action_mode", "velocity")
        # The velocity is published straight to the base controller, bypassing
        # collision avoidance, so bound the commanded speed.
        self._max_base_velocity = rospy.get_param(base + "/max_base_velocity", 0.2)
        self._max_base_yaw_velocity = rospy.get_param(
            base + "/max_base_yaw_velocity", 0.4
        )
        # Default to arm-only manipulation: the VLA drives the arm+gripper, not the whole robot.
        self._enable_base = rospy.get_param(base + "/enable_base_motion", False)
        self._enable_head = rospy.get_param(base + "/enable_head_motion", False)

        self._joint_states = JointStateCache(
            rospy.get_param(base + "/joint_states_topic", "/{}/joint_states".format(robot.robot_name))
        )
        self._arm_pub = rospy.Publisher(
            rospy.get_param(
                base + "/arm_command_topic",
                "/{}/arm_trajectory_controller/command".format(robot.robot_name),
            ),
            JointTrajectory,
            queue_size=1,
        )
        self._base_pub = rospy.Publisher(
            rospy.get_param(base + "/base_command_topic", "/{}/base/references".format(robot.robot_name)),
            Twist,
            queue_size=1,
        )
        self._gripper_pub = rospy.Publisher(
            rospy.get_param(
                base + "/gripper_command_topic",
                "/{}/gripper_controller/command".format(robot.robot_name),
            ),
            JointTrajectory,
            queue_size=1,
        )
        self._arm = None  # resolved lazily
        # Last gripper command: 'open', 'close' (effort grasp), 'position' or None
        self.gripper_command = None
        self._gripper_command_stamp = None

    def _arm_param(self, name: str, default: float) -> np.ndarray:
        # A wrong length fails when the sink is built, before the robot moves.
        return np.broadcast_to(np.asarray(rospy.get_param(name, default), dtype=np.float64), (self.ARM_DIM,))

    def _get_arm(self):
        if self._arm is None:
            self._arm = self.robot.get_arm()
        return self._arm

    def joint_positions(self) -> Optional[Dict[str, float]]:
        return self._joint_states.get()

    def reset(self):
        """Forget the previous arm target: the sink is kept for the next rollout."""
        self._arm_target = None

    def play_chunk(self, chunk: np.ndarray) -> bool:
        """Play a (T, 6) or (T, 11) chunk at the policy rate. Returns False if it could not be played."""
        if chunk.ndim != 2 or chunk.shape[1] < self.ARM_DIM:
            rospy.logwarn("[VLA] Unexpected chunk shape %s", chunk.shape)
            return False

        drive_base = (
            self._enable_base
            and self._base_action_mode != "disabled"
            and chunk.shape[1] >= self.BASE_SLICE.stop
        )
        rate = rospy.Rate(self._fps)
        positions = None
        try:
            for row in chunk:
                if rospy.is_shutdown():
                    return False
                positions = self._joint_states.get()
                if positions is None:
                    rospy.logwarn("[VLA] No joint states received, cannot play the chunk")
                    return False
                self._send_arm(positions, row[: self.ARM_DIM])
                if chunk.shape[1] > self.GRIP_IDX:
                    self._apply_gripper(row[self.GRIP_IDX], positions.get(self._gripper_joint))
                if drive_base:
                    self._send_base(row[self.BASE_SLICE])
                rate.sleep()
        finally:
            # Always brake at the end of the prefix so the base never free-runs.
            if drive_base:
                self.stop_base()
        if self._enable_head and chunk.shape[1] >= self.HEAD_SLICE.stop:
            # head_ref cancels any in-flight goal, so the head is commanded once per chunk.
            self._apply_head(positions, chunk[-1, self.HEAD_SLICE])
        return True

    def _send_arm(self, positions: Dict[str, float], delta: np.ndarray):
        current = [positions[name] for name in self._arm_joints]
        delta = shape_arm_delta(delta, current, self._arm_delta_gain, self._arm_hold_threshold, self._arm_target)
        target = apply_relative(current, delta, self.ARM_LOWER, self.ARM_UPPER, self._max_arm_delta)
        self._arm_target = target
        point = JointTrajectoryPoint(
            positions=target.tolist(), time_from_start=rospy.Duration.from_sec(self._arm_lead_time)
        )
        # A zero header stamp starts the trajectory on receipt, replacing the previous one.
        self._arm_pub.publish(JointTrajectory(joint_names=list(self._arm_joints), points=[point]))

    def _apply_gripper(self, gripper_value: float, measured: Optional[float] = None):
        # Fingers closed on nothing hold no object, so there is no grasp to keep until the open threshold
        empty = (
            self.grasp_position_threshold is not None
            and measured is not None
            and measured <= self.grasp_position_threshold
        )
        target = gripper_action(
            float(gripper_value),
            self.gripper_command == "close" and not empty,
            self._gripper_close_threshold,
            self._gripper_open_threshold,
        )
        if target is None:
            if self.gripper_command != "close":
                # timeout=0.0: do not block the stream while the jaw closes.
                self._get_arm().gripper.send_goal(
                    "close", timeout=0.0, max_torque=self._gripper_max_torque
                )
                self.set_gripper_command("close")
            return
        point = JointTrajectoryPoint(positions=[target], time_from_start=rospy.Duration.from_sec(1.0))
        self._gripper_pub.publish(JointTrajectory(joint_names=[self._gripper_joint], points=[point]))
        if self.gripper_command != "position":
            self.set_gripper_command("position")

    def set_gripper_command(self, state: Optional[str]):
        self.gripper_command = state
        self._gripper_command_stamp = rospy.Time.now() if state is not None else None

    def seconds_since_gripper_command(self) -> Optional[float]:
        if self._gripper_command_stamp is None:
            return None
        return (rospy.Time.now() - self._gripper_command_stamp).to_sec()

    def _apply_head(self, positions: Dict[str, float], delta: np.ndarray):
        pan, tilt = apply_relative([positions[name] for name in self._head_joints], delta)
        set_head_joints(self.robot, pan, tilt, self._head_vel)

    def _send_base(self, base_row: np.ndarray):
        bx, by, bth = (float(v) for v in base_row)
        if self._base_action_mode == "pose_delta":
            bx, by, bth = bx * self._fps, by * self._fps, bth * self._fps
        twist = Twist()
        twist.linear.x = clamp_symmetric(bx, self._max_base_velocity)
        twist.linear.y = clamp_symmetric(by, self._max_base_velocity)
        twist.angular.z = clamp_symmetric(bth, self._max_base_yaw_velocity)
        self._base_pub.publish(twist)

    def stop_base(self):
        self._base_pub.publish(Twist())


def set_head_joints(robot, pan: float, tilt: float, vel: float = 1.0, timeout: float = 0.0):
    """Command head_pan_joint/head_tilt_joint [rad] through head_ref.

    head_ref's PAN_TILT goal (goal_type 1) multiplies the tilt by the joint axis
    direction, which is -1 for HERO's head_tilt_joint, so the joint tilt is negated.
    """
    robot.head._setHeadReferenceGoal(
        1, vel, vel * 0.8, 0, pan=float(pan), tilt=-float(tilt), timeout=timeout
    )


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
        state_indices       : source indices used to build the checkpoint state
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
        self._state_indices = rospy.get_param(base + "/state_indices", [])
        self._max_chunks = rospy.get_param(base + "/max_chunks", 60)
        self._chunk_prefix_steps = rospy.get_param(base + "/chunk_prefix_steps", 10)
        # The training episodes start with the arm in the HSR neutral pose and the
        # head tilted down at the table. HERO's reset and carrying poses have
        # arm_roll -1.57, ~7 standard deviations outside the training state
        # distribution. Joint values [m, rad]; an empty list skips that part.
        self._start_arm_pose = rospy.get_param(base + "/start_arm_pose", [0.2, 0.0, 0.0, -1.57, 0.0])
        self._start_head_pose = rospy.get_param(base + "/start_head_pose", [0.0, -0.6])
        # Lift [m] after a detected grasp, so the object clears the table before the robot moves.
        self._post_grasp_lift = float(rospy.get_param(base + "/post_grasp_lift", 0.1))
        # Nothing fills raw_sentence yet; the policy was trained on short commands
        # like "pick up the mug", so those are built from the semantics instead.
        self._instruction_templates = rospy.get_param(
            base + "/instruction_templates", {"pick-up": "pick up the {object}"}
        )

        # gripper.occupied_by is only ever set by the classic Grab/Place FSMs
        # (robot_smach_states), never by this VLA path, so it cannot be used on
        # its own to detect a VLA-driven grasp/release. The physical stand-in is
        # a gripper commanded closed whose fingers stopped above the empty-closed
        # position; that threshold MUST be calibrated on the actual gripper (sim
        # or hardware) before being relied on; disabled by default. The sink uses
        # the same threshold to release a grasp that closed on nothing.
        self._grasp_position_threshold = self._sink.grasp_position_threshold
        self._grasp_settle_time = float(rospy.get_param(base + "/grasp_settle_time", 1.0))
        # Consecutive chunks the completion signal must hold, against a transient reading.
        self._done_confirm_chunks = int(rospy.get_param(base + "/done_confirm_chunks", 2))
        # Set once the policy moves the robot; a failed pick-up then retracts the arm.
        self._policy_moved = False
        # Opt-in: publish every policy query (state + full chunk) as JSON, for rosbag comparisons.
        self._debug_pub = None
        if rospy.get_param(base + "/publish_debug", False):
            self._debug_pub = rospy.Publisher(base + "/debug/query", String, queue_size=10)

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
            state_indices=self._state_indices or None,
        )
        rospy.loginfo(
            "[VLA] SmolVLA policy loaded from %s on %s",
            self._checkpoint_path,
            self._device,
        )

    def _infer(self, obs: Dict) -> np.ndarray:
        """Return an (T, action_dim) action chunk in robot units."""
        return self._server.infer(obs)

    def _release_policy(self):
        """Release per-episode policy resources; the in-process model stays loaded."""

    # -- item 2: prompting -------------------------------------------------

    def _instruction(self, request: ManipulationRequest) -> str:
        if request.raw_sentence:
            return request.raw_sentence
        template = self._instruction_templates.get(request.action_name)
        target = _object_name(request.semantics.get("object"), request.context.get("object"))
        if template and target:
            return template.format(object=target)
        return request.action_name

    # -- item 7: episode termination --------------------------------------

    def _gripper_occupied(self) -> bool:
        """Classic occupied_by flag, or a VLA-driven grasp measured on the gripper joint."""
        holding = gripper_holding_object(
            self._sink.gripper_command,
            self._sink.seconds_since_gripper_command(),
            self._obs.get_gripper_position(),
            self._grasp_position_threshold,
            self._grasp_settle_time,
        )
        return gripper_occupied(self._sink) or bool(holding)

    def _is_episode_done(
        self, request: ManipulationRequest, executed_steps: int, gripper_occupied_at_start: bool
    ) -> bool:
        """Detect success from the gripper state (see backend_utils.is_manipulation_done).

        Combines two signals:
        - occupied_by transition: only fires if a classic FSM set it, so it is
          inert (always False -> False) on a pure VLA run.
        - grasp_position_threshold: an opt-in physical reading of the gripper
          joint while it is commanded closed, calibrated per-robot via ROS params.
        Other actions have no known success signal and always run to max_chunks.
        """
        return is_manipulation_done(
            request.action_name, self._gripper_occupied(), gripper_occupied_at_start
        )

    # -- items 5-6-8: control loop, actuation, response mapping -----------

    def _send_arm_pose(self, pose: List[float]):
        # _send_joint_trajectory lives on the underlying Arm; PublicArm exposes it as _arm.
        arm = self._sink._get_arm()
        getattr(arm, "_arm", arm)._send_joint_trajectory([list(pose)], timeout=10.0)

    def _current_arm_pose(self) -> List[float]:
        positions = self._sink.joint_positions()
        return [positions[name] for name in self._obs._arm_joints]

    def _open_gripper(self):
        self._sink._get_arm().gripper.send_goal("open", timeout=5.0)
        self._sink.set_gripper_command("open")

    def _move_to_start_pose(self):
        """Bring the robot into the pose the policy's training episodes start from."""
        # Cancel any local planner goal and stop the base before the policy drives it.
        self.robot.base.force_drive(0.0, 0.0, 0.0, 0.0)
        self._open_gripper()
        if self._start_arm_pose:
            # Lift first: rolling the arm at the bottom of its range can hit the table edge.
            lifted = self._current_arm_pose()
            lifted[0] = max(lifted[0], self._start_arm_pose[0])
            self._send_arm_pose(lifted)
            self._send_arm_pose(self._start_arm_pose)
        if self._start_head_pose:
            set_head_joints(self.robot, *self._start_head_pose, timeout=5.0)

    def _lift_arm(self):
        if self._post_grasp_lift > 0:
            pose = self._current_arm_pose()
            pose[0] = min(pose[0] + self._post_grasp_lift, HSRActionSink.ARM_UPPER[0])
            self._send_arm_pose(pose)

    def _retract(self):
        """Free the arm after a failed pick-up, before the classic fallback takes over."""
        if rospy.is_shutdown():
            return
        try:
            self._open_gripper()
            self._lift_arm()
            self._move_to_start_pose()
        except Exception as error:
            # Never mask the outcome or the exception of the rollout itself
            rospy.logwarn("[VLA] Retract after the failed pick-up did not complete: %s", error)

    def execute(self, request: ManipulationRequest) -> Dict:
        self._load_policy()
        self._policy_moved = False
        succeeded = False
        try:
            result = self._rollout(request)
            succeeded = bool(result.get("succeeded"))
            return result
        finally:
            self._sink.stop_base()
            # Only after the policy moved the arm: the fingers may be on the object and the arm low over the table
            if request.action_name == "pick-up" and self._policy_moved and not succeeded:
                self._retract()
            # The start pose leaves a head goal that never expires; release the head.
            self.robot.head.cancel_goal()
            self._release_policy()

    def _rollout(self, request: ManipulationRequest) -> Dict:
        self._move_to_start_pose()
        # The start pose is not played through the sink, so its previous arm target is stale
        self._sink.reset()
        instruction = self._instruction(request)
        rospy.loginfo("[VLA] Instruction: '%s'", instruction)
        executed: List[List[float]] = []
        gripper_occupied_at_start = self._gripper_occupied()
        done_count = 0

        for iteration in range(self._max_chunks):
            obs = self._obs.get_observation(instruction, require_hand=True)
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
            _log_chunk(iteration, obs["state"], prefix, self._sink.gripper_command)
            if self._debug_pub is not None:
                self._debug_pub.publish(String(data=query_debug_message(
                    iteration, instruction, obs["state"], chunk, len(prefix))))
            self._policy_moved = True
            played = self._sink.play_chunk(prefix)
            executed.extend(row.tolist() for row in prefix)

            if not played:
                return {
                    "succeeded": False,
                    "actions": executed,
                    "message": "Could not play the action chunk during the VLA rollout",
                }

            if self._is_episode_done(request, len(executed), gripper_occupied_at_start):
                done_count += 1
                if done_count >= self._done_confirm_chunks:
                    if request.action_name == "pick-up":
                        self._lift_arm()
                    return {
                        "succeeded": True,
                        "actions": executed,
                        "message": "VLA completed '{}'".format(request.action_name),
                    }
            else:
                done_count = 0

        # Ran out of iterations without a success signal.
        return {
            "succeeded": False,
            "actions": executed,
            "message": "VLA reached max_chunks without a completion signal",
        }


def _object_name(semantics_object, context_object) -> str:
    """Name of the manipulated object from the action semantics, or the found object for a reference."""
    for description in (semantics_object, context_object):
        if isinstance(description, str):
            return description
        if isinstance(description, dict):
            name = description.get("type") or description.get("id") or description.get("category")
            if name and name != "reference":
                return str(name).replace("_", " ")
    return ""


def _log_chunk(iteration: int, state, prefix: np.ndarray, gripper_command: Optional[str]):
    """One diagnostic line per chunk: what the policy asks for, per joint group."""
    arm = prefix[:, : HSRActionSink.ARM_DIM].astype(np.float64)
    # The signed sum shows the direction the policy drives each joint over the prefix
    message = "[VLA] chunk %d state=%s arm|delta|max=%s arm_sum=%s" % (
        iteration,
        np.round(np.asarray(state, dtype=np.float64), 3).tolist(),
        np.round(np.abs(arm).max(axis=0), 3).tolist(),
        np.round(arm.sum(axis=0), 3).tolist(),
    )
    if prefix.shape[1] > HSRActionSink.GRIP_IDX:
        gripper = prefix[:, HSRActionSink.GRIP_IDX]
        message += " gripper=[%.2f..%.2f] (commanded %s)" % (gripper.min(), gripper.max(), gripper_command)
    if prefix.shape[1] >= HSRActionSink.BASE_SLICE.stop:
        base = np.abs(prefix[:, HSRActionSink.BASE_SLICE]).max(axis=0)
        message += " base|v|max=%s" % np.round(base, 3).tolist()
    rospy.loginfo(message)


class SmolVLAWebSocketBackend(SmolVLALocalBackend):
    """Run SmolVLA in a separate Python process over WebSocket.

    ROS keeps ownership of observations, actuation, and the closed-loop
    rollout. The policy server can therefore run LeRobot in Python 3.12 while
    ROS Noetic remains on its Python 3.8 runtime.
    """

    def __init__(self, robot):
        SmolVLALocalBackend.__init__(self, robot)
        base = "/{}/action_server/vla".format(robot.robot_name)
        self._policy_url = rospy.get_param(base + "/policy_url", "ws://127.0.0.1:8000")
        self._policy_timeout = float(rospy.get_param(base + "/policy_timeout", 30.0))
        self._policy_socket = None

    def _load_policy(self):
        if self._policy_socket is not None:
            return
        try:
            import websocket
        except ImportError as error:
            raise RuntimeError(
                "websocket-client is required for SmolVLAWebSocketBackend"
            ) from error

        try:
            self._policy_socket = websocket.create_connection(
                self._policy_url, timeout=self._policy_timeout
            )
        except Exception:
            self._policy_socket = None
            raise
        rospy.loginfo("[VLA] Connected to policy server at %s", self._policy_url)

    def _infer(self, obs: Dict) -> np.ndarray:
        import msgpack

        head_shape = as_rgb_uint8(obs["head_rgb"]).shape
        hand_shape = as_rgb_uint8(obs["hand_rgb"]).shape

        payload = {
            "head_rgb": pack_image_binary(obs["head_rgb"]),
            "hand_rgb": pack_image_binary(obs["hand_rgb"]),
            "state": np.asarray(obs["state"], dtype=np.float32).tolist(),
            "instruction": str(obs["instruction"]),
        }
        try:
            packed_payload = msgpack.packb(payload, use_bin_type=True)
            rospy.loginfo(
                "[VLA] Sending observation head=%s hand=%s payload=%d bytes",
                head_shape,
                hand_shape,
                len(packed_payload),
            )
            self._policy_socket.send_binary(packed_payload)
            response_payload = self._policy_socket.recv()
            if isinstance(response_payload, str):
                response_payload = response_payload.encode("latin1")
            response = msgpack.unpackb(response_payload, raw=False)
        except Exception:
            self._close_policy_socket()
            raise

        if "error" in response:
            raise RuntimeError("Policy server error: {}".format(response["error"]))
        actions = np.asarray(response.get("actions"), dtype=np.float32)
        if actions.ndim != 2 or actions.shape[1] < HSRActionSink.ARM_DIM:
            raise RuntimeError(
                "Policy server returned invalid action shape {}".format(actions.shape)
            )
        return actions

    def _release_policy(self):
        # One connection per episode: between actions nobody answers the server's
        # keepalive pings, so a cached socket is dead by the next action.
        self._close_policy_socket()

    def _close_policy_socket(self):
        if self._policy_socket is not None:
            try:
                self._policy_socket.close()
            finally:
                self._policy_socket = None
