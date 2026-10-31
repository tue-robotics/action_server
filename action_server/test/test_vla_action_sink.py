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


if __name__ == '__main__':
    unittest.main()
