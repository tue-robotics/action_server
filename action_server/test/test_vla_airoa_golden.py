"""Golden test: HSRActionSink against the official AIRoA HSR client.

The reference is a transcription of airoa-org/airoa-evaluation-ICRA@47b4df5
deploy/hsr_policy_client/scripts/hsr_policy.py in its 10 Hz configuration
(launch args update_freq:=10 adopted_action_chunks:=10 upsample:=false
action_smoothing:=none gripper_mode:=hybrid). With reference-equivalent
parameters the sink must command exactly what the reference commands. With
the deployed parameters (hero_bringup vla.yaml) it may only differ where a
deviation is documented.
"""
import unittest
from unittest import mock

import numpy as np
import rospkg
import rospy
import yaml

from action_server.vla import backends

# Allow rospy.Time.now() without a running node
rospy.rostime.set_rostime_initialized(True)

PREFIX = "/hero/action_server/vla/"
STATE_JOINTS = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint",
                "hand_motor_joint", "head_pan_joint", "head_tilt_joint"]
TOPICS = {"arm_command_topic": "arm", "gripper_command_topic": "gripper", "base_command_topic": "base"}
# hero_bridge gripper_controller: grasp effort = -max_torque * HAND_MOMENT_ARM_LENGTH
HAND_MOMENT_ARM_LENGTH = 0.07

REFERENCE_EQUIVALENT = dict(
    TOPICS,
    policy_fps=10.0,
    arm_lead_time=0.05,  # 1 / update_freq / 2 (hsr_policy.py:608)
    max_arm_delta=[1e3] * 5,  # the reference does not bound the deltas...
    gripper_close_threshold=0.5,  # hsr_policy.py:452
    gripper_open_threshold=0.5,  # ...nor hold a grasp above the close threshold
    gripper_max_torque=0.018 / HAND_MOMENT_ARM_LENGTH,  # hsr_policy.py:665
    enable_base_motion=True,
    base_action_mode="velocity",
    max_base_velocity=0.0,  # 0 disables the clamps; the reference sends the column as is
    max_base_yaw_velocity=0.0,
)

# Mirror of the sink parameters in hero_bringup/parameters/action_server/vla.yaml
DEPLOYED = dict(
    TOPICS,
    policy_fps=10.0,
    chunk_prefix_steps=10,
    arm_lead_time=0.2,
    max_arm_delta=[0.04, 0.24, 0.4, 0.3, 0.3],
    arm_delta_gain=[1.0, 1.0, 2.5, 2.0, 1.0],
    arm_hold_threshold=[0.0, 0.0, 0.0, 0.005, 0.0],
    gripper_close_threshold=0.5,
    gripper_open_threshold=0.9,
    gripper_max_torque=0.26,
    grasp_position_threshold=0.1,
    enable_head_motion=False,
    enable_base_motion=False,
    base_action_mode="velocity",
    max_base_velocity=0.2,
    max_base_yaw_velocity=0.4,
)

# Joint state per tick, drifting so that a target anchored on the chunk start or a
# cumulative sum of the deltas would not match the reference.
STATES = np.array([0.012, -0.016, 0.0, -1.57, 0.0, 1.0, 0.0, -0.7], dtype=np.float32) + np.outer(
    np.arange(10), [0.001, -0.002, 0.001, 0.0, 0.0, 0.0, 0.0005, -0.0005]
).astype(np.float32)
# Gripper column: approach, grasp, latch, re-open, re-grasp
GRIPPER = [1.24, 1.0, 0.7, 0.45, 0.1, -0.35, -0.8, 0.6, 1.2, 0.3]
# Ticks whose arm targets the deployed bounds change (see test_deployed_parameters_deviate_only_where_documented)
DELTA_CAPPED, BELOW_LIFT_LIMIT, ABOVE_FLEX_LIMIT = 1, 2, 3


def make_chunk():
    rows = np.zeros((50, 11), dtype=np.float32)
    rows[:, 0:5] = [0.002, -0.01, 0.01, -0.01, 0.02]
    rows[:, 6:8] = [0.001, -0.002]
    rows[:10, 5] = GRIPPER
    rows[:10, 8:11] = [0.1, 0.0, 0.05]
    rows[DELTA_CAPPED, 1] = -0.30  # arm_flex delta beyond max_arm_delta 0.24
    rows[BELOW_LIFT_LIMIT, 0] = -0.02  # arm_lift target below its lower limit 0.0
    rows[ABOVE_FLEX_LIMIT, 1] = 0.04  # arm_flex target above its upper limit 0.0
    rows[1, 8] = 0.3  # beyond max_base_velocity
    rows[2, 10] = 0.5  # beyond max_base_yaw_velocity
    rows[6, 9] = -0.25
    return rows


class AiroaHybridClient(object):
    """Commands of OpenpiPolicy.act + HSREnv.execute_actions (hybrid gripper), one action row per tick."""

    def __init__(self):
        # The client's topic echo makes the state 'open' once a gripper position was sent
        self.gripper_closed = False

    def tick(self, row, joint_state):
        # Convert delta-style arm/head outputs back to absolute values (hsr_policy.py:840-842). The integer
        # zeros promote the sum of the float32 action and joint state to float64, as in the client.
        action = row + np.concatenate([joint_state[:5], np.array([0]), joint_state[6:8], np.array([0, 0, 0])])
        gripper = None
        if action[5] < 0.5:  # hsr_policy.py:661-667
            if not self.gripper_closed:
                gripper = ("grasp", -0.018)
                self.gripper_closed = True
        else:  # hsr_policy.py:671-679
            gripper = ("position", float(np.clip(action[5], -0.1, 1.23)), 1.0)
            self.gripper_closed = False
        return {"arm": (action[:5], 0.05), "head": action[6:8], "base": tuple(action[8:11]), "gripper": gripper}


def reference_commands(rows):
    client = AiroaHybridClient()
    return [client.tick(rows[k], STATES[k]) for k in range(len(rows))]


def play(params, rows):
    """Play rows through a sink built with params. Returns (rate, per-tick commands, commands after the last tick)."""
    events = []

    class Publisher(object):
        def __init__(self, topic, *args, **kwargs):
            self.topic = topic

        def publish(self, msg):
            events.append((self.topic, msg))

    class Rate(object):
        def __init__(self, hz):
            events.append(("rate", hz))

        def sleep(self):
            events.append(("tick", None))

    robot = mock.MagicMock()
    robot.robot_name = "hero"
    with mock.patch.object(backends.rospy, "get_param", lambda name, default=None: params.get(
            name[len(PREFIX):], default)), \
            mock.patch.object(backends.rospy, "Publisher", Publisher), \
            mock.patch.object(backends.rospy, "Subscriber"):
        sink = backends.HSRActionSink(robot)
    sink._joint_states = mock.MagicMock()
    sink._joint_states.get.side_effect = [dict(zip(STATE_JOINTS, map(float, state))) for state in STATES]
    sink.set_gripper_command("open")
    robot.get_arm.return_value.gripper.send_goal.side_effect = \
        lambda state, **kwargs: events.append(("grasp", (state, kwargs)))
    robot.head._setHeadReferenceGoal.side_effect = lambda *args, **kwargs: events.append(("head", kwargs))

    with mock.patch.object(backends.rospy, "Rate", Rate), \
            mock.patch.object(backends.rospy, "is_shutdown", return_value=False):
        assert sink.play_chunk(rows)

    ticks, current, rate = [], {}, None
    for topic, msg in events:
        if topic == "rate":
            rate = msg
        elif topic == "tick":
            ticks.append(current)
            current = {}
        elif topic == "arm":
            current["arm"] = (np.asarray(msg.points[0].positions), msg.points[0].time_from_start.to_sec())
        elif topic == "gripper":
            current["gripper"] = ("position", msg.points[0].positions[0], msg.points[0].time_from_start.to_sec())
        elif topic == "grasp":
            state, kwargs = msg
            assert state == "close" and kwargs["timeout"] == 0.0
            current["gripper"] = ("grasp", -kwargs["max_torque"] * HAND_MOMENT_ARM_LENGTH)
        elif topic == "base":
            current["base"] = (msg.linear.x, msg.linear.y, msg.angular.z)
        elif topic == "head":
            current["head"] = msg
    return rate, ticks, current


class TestAiroaGolden(unittest.TestCase):
    def assertGripperEqual(self, expected, actual, effort_rtol=1e-6):
        if expected is None or actual is None:
            self.assertEqual(expected, actual)
            return
        self.assertEqual(expected[0], actual[0])
        if expected[0] == "grasp":
            np.testing.assert_allclose(actual[1], expected[1], rtol=effort_rtol)
        else:
            np.testing.assert_allclose(actual[1:], expected[1:], atol=1e-6)

    def test_reference_equivalent_parameters_command_exactly_the_reference(self):
        rows = make_chunk()[:10]
        expected = reference_commands(rows)
        # The reference does not clip to the joint limits either
        with mock.patch.object(backends.HSRActionSink, "ARM_LOWER", [-np.inf] * 5), \
                mock.patch.object(backends.HSRActionSink, "ARM_UPPER", [np.inf] * 5):
            rate, ticks, after = play(REFERENCE_EQUIVALENT, rows)

        self.assertEqual(rate, 10.0)
        self.assertEqual(len(ticks), 10)
        for k, (ref, sink) in enumerate(zip(expected, ticks)):
            with self.subTest(tick=k):
                np.testing.assert_allclose(sink["arm"][0], ref["arm"][0], rtol=0, atol=1e-9)
                self.assertAlmostEqual(sink["arm"][1], ref["arm"][1])
                np.testing.assert_allclose(sink["base"], ref["base"], rtol=0, atol=1e-9)
                self.assertGripperEqual(ref["gripper"], sink.get("gripper"))
        # Documented extra: the sink brakes the base after the prefix, the reference keeps the last velocity
        self.assertEqual(after, {"base": (0.0, 0.0, 0.0)})

    def test_head_goal_is_the_reference_head_target_of_the_last_tick(self):
        rows = make_chunk()[:10]
        expected = reference_commands(rows)
        _, ticks, after = play(dict(REFERENCE_EQUIVALENT, enable_head_motion=True), rows)
        # head_ref takes one goal per chunk instead of a stream, and negates the tilt of HERO's head
        self.assertFalse(any("head" in tick for tick in ticks))
        np.testing.assert_allclose([after["head"]["pan"], -after["head"]["tilt"]], expected[-1]["head"], atol=1e-6)

    def test_deployed_parameters_deviate_only_where_documented(self):
        rows = make_chunk()[:10]
        expected = reference_commands(rows)
        rate, ticks, after = play(DEPLOYED, rows)

        self.assertEqual(rate, 10.0)
        for k, (ref, sink) in enumerate(zip(expected, ticks)):
            with self.subTest(tick=k):
                # arm_lead_time: the sim arm lags the teleoperation lead of the data
                self.assertAlmostEqual(sink["arm"][1], 0.2)
                # arm_delta_gain: the sim moves arm_roll and wrist_flex less per unit delta than the real HSR
                gain = np.array(DEPLOYED["arm_delta_gain"])
                target = np.array(ref["arm"][0], dtype=np.float64) + (gain - 1.0) * rows[k, :5]
                if k == DELTA_CAPPED:  # max_arm_delta: URDF velocity limit times arm_lead_time
                    target[1] = STATES[k][1] - 0.24
                elif k == BELOW_LIFT_LIMIT:  # URDF position limits
                    target[0] = 0.0
                elif k == ABOVE_FLEX_LIMIT:
                    target[1] = 0.0
                np.testing.assert_allclose(sink["arm"][0], target, atol=1e-6)
                # enable_head_motion / enable_base_motion are off
                self.assertNotIn("head", sink)
                self.assertNotIn("base", sink)
                if GRIPPER[k] == 0.6:
                    # gripper_open_threshold: a grasp on an object is held while the column stays at its width
                    self.assertIsNone(sink.get("gripper"))
                    self.assertEqual(ref["gripper"][0], "position")
                else:
                    # gripper_max_torque 0.26 rounds the reference's 0.018 Nm
                    self.assertGripperEqual(ref["gripper"], sink.get("gripper"), effort_rtol=0.02)
        self.assertEqual(after, {})

    def test_deployed_mirror_matches_vla_yaml(self):
        try:
            path = rospkg.RosPack().get_path("hero_bringup") + "/parameters/action_server/vla.yaml"
        except rospkg.ResourceNotFound:
            self.skipTest("hero_bringup is not available")
        with open(path) as f:
            params = yaml.safe_load(f)
        for name, value in DEPLOYED.items():
            if name not in TOPICS:
                with self.subTest(param=name):
                    self.assertEqual(params[name], value)


if __name__ == '__main__':
    unittest.main()
