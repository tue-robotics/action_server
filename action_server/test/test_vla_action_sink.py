import json
import unittest
from unittest import mock

import numpy as np
import rospy

from action_server.vla import backends

# Allow rospy.Time.now() without a running node
rospy.rostime.set_rostime_initialized(True)

PREFIX = "/hero/action_server/vla/"
ARM_JOINTS = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
STATE = {
    "arm_lift_joint": 0.1,
    "arm_flex_joint": -0.5,
    "arm_roll_joint": 0.0,
    "wrist_flex_joint": -1.57,
    "wrist_roll_joint": 0.0,
    "hand_motor_joint": 1.0,
    "head_pan_joint": 0.0,
    "head_tilt_joint": -0.7,
}
# Arm joint limits of hsrb4s.urdf: (lower, upper, velocity)
HSR_ARM_LIMITS = [(0.0, 0.69, 0.2), (-2.62, 0.0, 1.2), (-2.09, 3.84, 2.0), (-1.92, 1.22, 1.5), (-1.92, 3.67, 1.5)]
HSR_ARM_URDF = '<robot name="hsrb"><link name="link0"/>{}</robot>'.format("".join(
    '<link name="link{i}"/><joint name="{name}" type="revolute"><parent link="link{parent}"/><child link="link{i}"/>'
    '<limit lower="{lower}" upper="{upper}" velocity="{velocity}" effort="100"/></joint>'.format(
        i=i + 1, parent=i, name=name, lower=lower, upper=upper, velocity=velocity)
    for i, (name, (lower, upper, velocity)) in enumerate(zip(ARM_JOINTS, HSR_ARM_LIMITS))))


def get_param(params):
    """rospy.get_param over the VLA params (by their short name) and the robot params (by their full name)."""
    return lambda name, default=None: params.get(name[len(PREFIX):] if name.startswith(PREFIX) else name, default)


def make_sink(params):
    robot = mock.MagicMock()
    robot.robot_name = "hero"
    with mock.patch.object(backends.rospy, "get_param", get_param(params)), \
            mock.patch.object(backends.rospy, "Publisher") as publisher, \
            mock.patch.object(backends.rospy, "Subscriber"):
        publisher.side_effect = lambda topic, *args, **kwargs: mock.MagicMock(topic=topic)
        sink = backends.HSRActionSink(robot)
    sink._joint_states = mock.MagicMock()
    sink._joint_states.get.return_value = dict(STATE)
    return sink, robot


def chunk(rows, gripper=1.24, base=(0.0, 0.0, 0.0)):
    out = np.zeros((rows, 11), dtype=np.float32)
    out[:, 0:5] = [0.01, -0.02, 0.03, 0.0, -0.01]
    out[:, 5] = gripper
    out[:, 8:11] = base
    return out


def gripper_rows(values):
    rows = chunk(len(values))
    rows[:, 5] = values
    return rows


@mock.patch.object(backends.rospy, "Rate")
@mock.patch.object(backends.rospy, "is_shutdown", return_value=False)
class TestHSRActionSink(unittest.TestCase):
    def test_arm_target_is_state_plus_delta(self, _is_shutdown, rate):
        sink, _ = make_sink({"arm_lead_time": 0.2})
        self.assertTrue(sink.play_chunk(chunk(3)))

        self.assertEqual(sink._arm_pub.publish.call_count, 3)
        trajectory = sink._arm_pub.publish.call_args[0][0]
        self.assertEqual(trajectory.joint_names, ARM_JOINTS)
        np.testing.assert_allclose(trajectory.points[0].positions, [0.11, -0.52, 0.03, -1.57, -0.01], atol=1e-6)
        self.assertAlmostEqual(trajectory.points[0].time_from_start.to_sec(), 0.2)
        # One row per policy step at 10 Hz
        rate.assert_called_once_with(10.0)
        self.assertEqual(rate.return_value.sleep.call_count, 3)

    def test_arm_target_respects_the_urdf_joint_limits(self, _is_shutdown, _rate):
        sink, _ = make_sink({"/hero/robot_description": HSR_ARM_URDF})
        sink._joint_states.get.return_value = dict(STATE, arm_flex_joint=-0.1)
        rows = chunk(1)
        rows[0, 1] = 0.2  # arm_flex delta that would cross its upper limit of 0.0
        sink.play_chunk(rows)
        positions = sink._arm_pub.publish.call_args[0][0].points[0].positions
        self.assertAlmostEqual(positions[1], 0.0)

    def test_arm_delta_is_bounded_by_the_urdf_velocity_limit(self, _is_shutdown, _rate):
        sink, _ = make_sink({"/hero/robot_description": HSR_ARM_URDF, "arm_lead_time": 0.2})
        rows = chunk(1)
        rows[0, 0] = 0.3  # arm_lift moves at most 0.2 m/s, so 0.04 m within the 0.2 s lead
        sink.play_chunk(rows)
        positions = sink._arm_pub.publish.call_args[0][0].points[0].positions
        self.assertAlmostEqual(positions[0], 0.14)

    def test_without_robot_description_the_arm_is_unbounded(self, _is_shutdown, _rate):
        with mock.patch.object(backends.rospy, "logwarn") as logwarn:
            sink, _ = make_sink({})
        logwarn.assert_called_once()
        rows = chunk(1)
        rows[0, 1] = 0.6
        sink.play_chunk(rows)
        self.assertAlmostEqual(sink._arm_pub.publish.call_args[0][0].points[0].positions[1], 0.1, places=6)

    def test_gripper_open_value_is_followed_as_position(self, _is_shutdown, _rate):
        sink, robot = make_sink({})
        sink.play_chunk(chunk(5, gripper=1.24))
        robot.get_arm.return_value.gripper.send_goal.assert_not_called()
        self.assertEqual(sink._gripper_pub.publish.call_count, 5)
        trajectory = sink._gripper_pub.publish.call_args[0][0]
        self.assertEqual(trajectory.joint_names, ["hand_motor_joint"])
        self.assertAlmostEqual(trajectory.points[0].positions[0], 1.23)
        self.assertEqual(sink.gripper_command, "position")

    def test_gripper_closing_value_grasps_once(self, _is_shutdown, _rate):
        sink, robot = make_sink({})
        sink.play_chunk(chunk(5, gripper=-0.8))
        robot.get_arm.return_value.gripper.send_goal.assert_called_once_with("close", timeout=0.0, max_torque=0.26)
        self.assertEqual(sink.gripper_command, "close")

    def test_grasp_reopens_when_the_column_rises_above_the_threshold(self, _is_shutdown, _rate):
        sink, robot = make_sink({})
        sink.play_chunk(gripper_rows([0.3, 0.6]))
        robot.get_arm.return_value.gripper.send_goal.assert_called_once()
        self.assertAlmostEqual(sink._gripper_pub.publish.call_args[0][0].points[0].positions[0], 0.6, places=5)
        self.assertEqual(sink.gripper_command, "position")

    def test_base_velocity_is_clamped_and_braked(self, _is_shutdown, _rate):
        sink, _ = make_sink({"enable_base_motion": True, "max_base_velocity": 0.2})
        sink.play_chunk(chunk(2, base=(0.5, -0.1, 0.05)))
        twists = [c[0][0] for c in sink._base_pub.publish.call_args_list]
        self.assertEqual(len(twists), 3)
        self.assertAlmostEqual(twists[0].linear.x, 0.2)
        self.assertAlmostEqual(twists[0].linear.y, -0.1, places=5)
        self.assertAlmostEqual(twists[0].angular.z, 0.05, places=5)
        self.assertEqual((twists[-1].linear.x, twists[-1].linear.y, twists[-1].angular.z), (0.0, 0.0, 0.0))

    def test_base_disabled_by_default(self, _is_shutdown, _rate):
        sink, _ = make_sink({})
        sink.play_chunk(chunk(2, base=(0.1, 0.0, 0.0)))
        sink._base_pub.publish.assert_not_called()

    def test_head_delta_is_relative_to_measured_head(self, _is_shutdown, _rate):
        sink, robot = make_sink({"enable_head_motion": True})
        rows = chunk(2)
        rows[:, 6:8] = [0.01, -0.02]
        sink.play_chunk(rows)
        kwargs = robot.head._setHeadReferenceGoal.call_args[1]
        self.assertAlmostEqual(kwargs["pan"], 0.01, places=5)
        # head_ref negates the tilt of HERO's head: joint tilt -0.72 is goal tilt +0.72
        self.assertAlmostEqual(kwargs["tilt"], 0.72, places=5)

    def test_six_wide_chunk_moves_only_arm_and_gripper(self, _is_shutdown, _rate):
        sink, robot = make_sink({"enable_base_motion": True, "enable_head_motion": True})
        self.assertTrue(sink.play_chunk(chunk(2)[:, :6]))
        sink._base_pub.publish.assert_not_called()
        robot.head._setHeadReferenceGoal.assert_not_called()

    def test_stop_ends_the_chunk_before_the_next_row(self, _is_shutdown, _rate):
        sink, _ = make_sink({"enable_base_motion": True})
        sink._arm_pub.publish.side_effect = lambda _msg: sink._stop.set()
        self.assertFalse(sink.play_chunk(chunk(3)))
        self.assertEqual(sink._arm_pub.publish.call_count, 1)
        # The base is braked anyway
        twist = sink._base_pub.publish.call_args[0][0]
        self.assertEqual((twist.linear.x, twist.linear.y, twist.angular.z), (0.0, 0.0, 0.0))


def make_backend(params):
    robot = mock.MagicMock()
    robot.robot_name = "hero"
    topics = []
    with mock.patch.object(backends.rospy, "get_param", get_param(params)), \
            mock.patch.object(backends.rospy, "Publisher") as publisher, \
            mock.patch.object(backends.rospy, "Subscriber"):
        publisher.side_effect = lambda topic, *args, **kwargs: topics.append(topic) or mock.MagicMock(topic=topic)
        backend = backends.SmolVLAWebSocketBackend(robot)
    backend._obs = mock.MagicMock()
    backend._obs.get_observation.return_value = {"state": list(STATE.values()), "instruction": "pick up the apple"}
    backend._sink = mock.MagicMock(gripper_command=None)
    backend._sink.play_chunk.return_value = True
    backend._infer = mock.MagicMock(return_value=chunk(50))
    backend._move_to_start_pose = mock.MagicMock()
    return backend, topics


def pick_up_request():
    return backends.ManipulationRequest(robot_name="hero", action_name="pick-up", semantics={"object": {"type": "apple"}},
                                        context={}, output_mode="joints", action_dim=11)


class TestRolloutDebugTopic(unittest.TestCase):
    def test_every_query_is_published_with_the_full_chunk(self):
        backend, topics = make_backend({"publish_debug": True, "max_chunks": 2})
        backend._rollout(pick_up_request())

        self.assertIn("/hero/action_server/vla/debug/query", topics)
        messages = [json.loads(c[0][0].data) for c in backend._debug_pub.publish.call_args_list]
        self.assertEqual([m["iteration"] for m in messages], [0, 1])
        self.assertEqual(messages[0]["instruction"], "pick up the apple")
        self.assertEqual(np.shape(messages[0]["chunk"]), (50, 11))
        self.assertEqual(messages[0]["prefix_steps"], 10)

    def test_no_debug_topic_by_default(self):
        _, topics = make_backend({"max_chunks": 2})
        self.assertNotIn("/hero/action_server/vla/debug/query", topics)


@mock.patch.object(backends, "ActiveGraspDetector")
class TestGraspCompletion(unittest.TestCase):
    def make_grasping_backend(self, params=None, held_for=2.0):
        """Backend whose sink holds an effort grasp; lift, open and the grasp check are recorded in order."""
        backend, _ = make_backend(dict({"max_chunks": 2}, **(params or {})))
        backend._sink.gripper_command = "close"
        backend._sink.seconds_since_gripper_command.return_value = held_for
        calls = []
        backend._lift_arm = mock.MagicMock(side_effect=lambda: calls.append("lift"))

        def open_gripper():
            calls.append("open")
            backend._sink.gripper_command = "open"
        backend._open_gripper = mock.MagicMock(side_effect=open_gripper)
        return backend, calls

    def test_settled_grasp_is_lifted_then_checked(self, detector):
        backend, calls = self.make_grasping_backend()
        detector.return_value.execute.side_effect = lambda: calls.append("check") or "true"
        result = backend._rollout(pick_up_request())
        self.assertTrue(result["succeeded"])
        # The lift comes first: fingers stalled on the table edge must not pass the check
        self.assertEqual(calls, ["lift", "check"])
        self.assertEqual(backend._sink.play_chunk.call_count, 1)
        # The classic check, on the arm the sink drives
        self.assertIs(detector.call_args[0][1].resolve(), backend._sink.get_arm.return_value)

    def test_unsettled_grasp_is_not_checked(self, detector):
        backend, calls = self.make_grasping_backend(held_for=0.5)
        self.assertFalse(backend._rollout(pick_up_request())["succeeded"])
        detector.assert_not_called()
        self.assertEqual(calls, [])

    def test_failed_grasp_check_opens_the_gripper_and_continues(self, detector):
        backend, calls = self.make_grasping_backend()
        detector.return_value.execute.return_value = "cannot_determine"
        result = backend._rollout(pick_up_request())
        self.assertFalse(result["succeeded"])
        self.assertEqual(backend._sink.play_chunk.call_count, 2)
        # Opened once, then not checked again until the policy grasps again
        self.assertEqual(calls, ["lift", "open"])

    def test_grasp_minimum_position_is_passed_to_the_detector(self, detector):
        backend, _ = self.make_grasping_backend({"grasp_minimum_position": 0.3})
        detector.return_value.execute.return_value = "true"
        backend._rollout(pick_up_request())
        self.assertEqual(detector.call_args[1], {"minimum_position": 0.3})

    def test_detector_default_without_the_param(self, detector):
        backend, _ = self.make_grasping_backend()
        detector.return_value.execute.return_value = "true"
        backend._rollout(pick_up_request())
        self.assertEqual(detector.call_args[1], {})


class TestRetractAfterFailedPickUp(unittest.TestCase):
    def make_episode(self):
        """Backend whose retract steps and cleanup are recorded in order."""
        backend, _ = make_backend({"max_chunks": 1})
        calls = []
        backend._connect = mock.MagicMock()
        backend._close_policy_socket = mock.MagicMock(side_effect=lambda: calls.append("release policy"))
        backend._open_gripper = mock.MagicMock(side_effect=lambda: calls.append("open"))
        backend._lift_arm = mock.MagicMock(side_effect=lambda: calls.append("lift"))
        backend._move_to_start_pose = mock.MagicMock(side_effect=lambda: calls.append("start pose"))
        backend.robot.head.cancel_goal.side_effect = lambda: calls.append("release head")
        return backend, calls

    def test_failed_pick_up_retracts_before_the_fallback(self):
        backend, calls = self.make_episode()
        result = backend.execute(pick_up_request())
        self.assertFalse(result["succeeded"])
        # The first 'start pose' is the start of the rollout
        self.assertEqual(calls, ["start pose", "open", "lift", "start pose", "release head", "release policy"])

    def test_no_retract_without_policy_motion(self):
        backend, calls = self.make_episode()
        backend._obs.get_observation.return_value = None
        backend.execute(pick_up_request())
        self.assertEqual(calls, ["start pose", "release head", "release policy"])

    @mock.patch.object(backends, "ActiveGraspDetector")
    def test_no_retract_after_a_grasp(self, detector):
        backend, calls = self.make_episode()
        backend._sink.gripper_command = "close"
        backend._sink.seconds_since_gripper_command.return_value = 2.0
        detector.return_value.execute.return_value = "true"
        self.assertTrue(backend.execute(pick_up_request())["succeeded"])
        self.assertNotIn("open", calls)

    def test_no_retract_for_other_actions(self):
        backend, calls = self.make_episode()
        request = pick_up_request()
        request.action_name = "place"
        backend.execute(request)
        self.assertNotIn("open", calls)

    def test_exception_retracts_and_propagates(self):
        backend, calls = self.make_episode()
        backend._sink.play_chunk.side_effect = RuntimeError("controller died")
        with self.assertRaises(RuntimeError):
            backend.execute(pick_up_request())
        self.assertEqual(calls[1:], ["open", "lift", "start pose", "release head", "release policy"])

    def test_failing_retract_does_not_mask_the_outcome(self):
        backend, calls = self.make_episode()
        backend._lift_arm.side_effect = RuntimeError("arm goal rejected")
        result = backend.execute(pick_up_request())
        self.assertFalse(result["succeeded"])
        self.assertEqual(calls[-2:], ["release head", "release policy"])

    def test_cancel_stops_the_rollout_without_retract(self):
        backend, calls = self.make_episode()
        backend._max_chunks = 3
        backend._sink.play_chunk.side_effect = lambda _prefix: backend.cancel() or False
        result = backend.execute(pick_up_request())
        self.assertFalse(result["succeeded"])
        self.assertEqual(result["message"], "VLA rollout cancelled")
        self.assertEqual(backend._infer.call_count, 1)
        # Stop means stop: the arm is not moved again
        self.assertEqual(calls, ["start pose", "release head", "release policy"])

    def test_cancel_is_reset_for_the_next_episode(self):
        backend, _ = self.make_episode()
        backend.cancel()
        backend.execute(pick_up_request())
        self.assertEqual(backend._infer.call_count, 1)


class TestLogChunk(unittest.TestCase):
    def test_logs_the_signed_delta_sum_per_joint(self):
        prefix = np.zeros((2, 11), dtype=np.float32)
        prefix[:, 0:5] = [[0.01, -0.02, 0.0, 0.0, 0.0], [-0.03, -0.02, 0.0, 0.0, 0.0]]
        with mock.patch.object(backends.rospy, "loginfo") as loginfo:
            backends._log_chunk(0, np.zeros(8), prefix, "close")
        self.assertIn("arm_sum=[-0.02, -0.04, 0.0, 0.0, 0.0]", loginfo.call_args[0][0])


if __name__ == '__main__':
    unittest.main()
