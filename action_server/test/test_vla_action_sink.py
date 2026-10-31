import json
import unittest
from unittest import mock

import numpy as np
import rospy

from action_server.vla import backends

# Allow rospy.Time.now() without a running node
rospy.rostime.set_rostime_initialized(True)

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


def make_sink(params):
    robot = mock.MagicMock()
    robot.robot_name = "hero"
    prefix = "/hero/action_server/vla/"
    with mock.patch.object(backends.rospy, "get_param", lambda name, default=None: params.get(
            name[len(prefix):], default)), \
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

    def test_arm_target_respects_joint_limits(self, _is_shutdown, _rate):
        sink, _ = make_sink({})
        rows = chunk(1)
        rows[0, 1] = 0.6  # arm_flex delta that would cross its upper limit of 0.0
        sink.play_chunk(rows)
        positions = sink._arm_pub.publish.call_args[0][0].points[0].positions
        self.assertLessEqual(positions[1], 0.0)

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

    def test_grasp_is_kept_while_the_column_stays_near_the_object(self, _is_shutdown, _rate):
        sink, robot = make_sink({})
        rows = chunk(4, gripper=0.3)
        rows[2:, 5] = 0.6
        sink.play_chunk(rows)
        robot.get_arm.return_value.gripper.send_goal.assert_called_once()
        sink._gripper_pub.publish.assert_not_called()
        self.assertEqual(sink.gripper_command, "close")

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

    def test_arm_delta_gain_scales_before_the_cap(self, _is_shutdown, _rate):
        sink, _ = make_sink({"arm_delta_gain": [5.0, 1.0, 2.5, 2.0, 1.0]})
        sink.play_chunk(chunk(1))
        positions = sink._arm_pub.publish.call_args[0][0].points[0].positions
        # lift 5 * 0.01 is bounded by max_arm_delta 0.04; roll 2.5 * 0.03
        np.testing.assert_allclose(positions, [0.14, -0.52, 0.075, -1.57, -0.01], atol=1e-6)

    def test_small_delta_holds_the_previous_target(self, _is_shutdown, _rate):
        sink, _ = make_sink({"arm_hold_threshold": [0.0, 0.0, 0.0, 0.005, 0.0]})
        # The wrist sags while the policy asks it to stay, then the policy moves it
        sink._joint_states.get.side_effect = [dict(STATE, wrist_flex_joint=wrist) for wrist in (-1.57, -1.58, -1.59)]
        rows = chunk(3)
        rows[:, 3] = [0.001, 0.001, 0.02]
        sink.play_chunk(rows)
        wrist = [c[0][0].points[0].positions[3] for c in sink._arm_pub.publish.call_args_list]
        np.testing.assert_allclose(wrist, [-1.569, -1.569, -1.57], atol=1e-6)

    def test_reset_forgets_the_held_target(self, _is_shutdown, _rate):
        sink, _ = make_sink({"arm_hold_threshold": 0.005})
        rows = chunk(1)
        rows[0, 3] = 0.001
        sink.play_chunk(rows)
        sink.reset()
        sink._joint_states.get.return_value = dict(STATE, wrist_flex_joint=-1.3)
        sink.play_chunk(rows)
        self.assertAlmostEqual(sink._arm_pub.publish.call_args[0][0].points[0].positions[3], -1.299, places=6)

    def test_empty_grasp_reopens_at_the_close_threshold(self, _is_shutdown, _rate):
        sink, robot = make_sink({"grasp_position_threshold": 0.1})
        sink._joint_states.get.side_effect = [dict(STATE, hand_motor_joint=hand) for hand in (1.0, -0.07, -0.07)]
        sink.play_chunk(gripper_rows([0.3, 0.6, 0.6]))
        robot.get_arm.return_value.gripper.send_goal.assert_called_once()
        self.assertAlmostEqual(sink._gripper_pub.publish.call_args[0][0].points[0].positions[0], 0.6, places=5)
        self.assertEqual(sink.gripper_command, "position")

    def test_grasp_on_an_object_is_held_in_the_release_band(self, _is_shutdown, _rate):
        sink, robot = make_sink({"grasp_position_threshold": 0.1})
        sink._joint_states.get.side_effect = [dict(STATE, hand_motor_joint=hand) for hand in (1.0, 0.55, 0.55)]
        sink.play_chunk(gripper_rows([0.3, 0.6, 0.6]))
        robot.get_arm.return_value.gripper.send_goal.assert_called_once()
        sink._gripper_pub.publish.assert_not_called()
        self.assertEqual(sink.gripper_command, "close")

    def test_arm_param_of_the_wrong_length_fails_before_moving(self, _is_shutdown, _rate):
        with self.assertRaises(ValueError):
            make_sink({"arm_delta_gain": [1.0, 2.0]})


def make_backend(params):
    robot = mock.MagicMock()
    robot.robot_name = "hero"
    prefix = "/hero/action_server/vla/"
    topics = []
    with mock.patch.object(backends.rospy, "get_param", lambda name, default=None: params.get(
            name[len(prefix):], default)), \
            mock.patch.object(backends.rospy, "Publisher") as publisher, \
            mock.patch.object(backends.rospy, "Subscriber"):
        publisher.side_effect = lambda topic, *args, **kwargs: topics.append(topic) or mock.MagicMock(topic=topic)
        backend = backends.SmolVLALocalBackend(robot)
    backend._obs = mock.MagicMock()
    backend._obs.get_observation.return_value = {"state": list(STATE.values()), "instruction": "pick up the apple"}
    backend._sink = mock.MagicMock(gripper_command=None)
    backend._sink.play_chunk.return_value = True
    backend._infer = mock.MagicMock(return_value=chunk(50))
    backend._move_to_start_pose = mock.MagicMock()
    backend._gripper_occupied = mock.MagicMock(return_value=False)
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


class TestRetractAfterFailedPickUp(unittest.TestCase):
    def make_episode(self):
        """Backend whose retract steps and cleanup are recorded in order."""
        backend, _ = make_backend({"max_chunks": 1})
        calls = []
        backend._load_policy = mock.MagicMock()
        backend._release_policy = mock.MagicMock(side_effect=lambda: calls.append("release policy"))
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

    def test_no_retract_after_a_grasp(self):
        backend, calls = self.make_episode()
        backend._gripper_occupied.side_effect = [False, True, True]
        backend._done_confirm_chunks = 1
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


class TestLogChunk(unittest.TestCase):
    def test_logs_the_signed_delta_sum_per_joint(self):
        prefix = np.zeros((2, 11), dtype=np.float32)
        prefix[:, 0:5] = [[0.01, -0.02, 0.0, 0.0, 0.0], [-0.03, -0.02, 0.0, 0.0, 0.0]]
        with mock.patch.object(backends.rospy, "loginfo") as loginfo:
            backends._log_chunk(0, np.zeros(8), prefix, "close")
        self.assertIn("arm_sum=[-0.02, -0.04, 0.0, 0.0, 0.0]", loginfo.call_args[0][0])


if __name__ == '__main__':
    unittest.main()
