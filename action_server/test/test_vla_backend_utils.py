import json
import unittest

import numpy as np

from action_server.vla.backend_utils import apply_relative, gripper_action, joint_limits, query_debug_message

URDF = """<robot name="arm">
  <link name="base"/><link name="lift"/><link name="flex"/>
  <joint name="arm_lift_joint" type="prismatic">
    <parent link="base"/><child link="lift"/><limit lower="0.0" upper="0.69" velocity="0.2" effort="100"/>
  </joint>
  <joint name="arm_flex_joint" type="revolute">
    <parent link="lift"/><child link="flex"/><limit lower="-2.62" upper="0.0" velocity="1.2" effort="100"/>
  </joint>
</robot>"""


class TestApplyRelative(unittest.TestCase):
    def test_delta_is_added_to_current_state(self):
        # The checkpoint's arm/head columns are deltas w.r.t. the measured joint state, not absolute targets
        target = apply_relative([0.1, -0.5, 0.0, -1.57, 0.0], [0.01, -0.02, 0.03, 0.0, -0.01])
        np.testing.assert_allclose(target, [0.11, -0.52, 0.03, -1.57, -0.01])

    def test_target_is_clipped_to_joint_limits(self):
        target = apply_relative([0.68, -0.01], [0.05, 0.05], lower=[0.0, -2.62], upper=[0.69, 0.0])
        np.testing.assert_allclose(target, [0.69, 0.0])

    def test_delta_is_clipped_per_joint(self):
        target = apply_relative([0.0, 0.0], [0.5, -0.5], max_delta=[0.05, 0.3])
        np.testing.assert_allclose(target, [0.05, -0.3])


class TestJointLimits(unittest.TestCase):
    def test_limits_are_read_from_the_urdf_in_the_requested_order(self):
        lower, upper, velocity = joint_limits(URDF, ["arm_flex_joint", "arm_lift_joint"])
        np.testing.assert_allclose(lower, [-2.62, 0.0])
        np.testing.assert_allclose(upper, [0.0, 0.69])
        np.testing.assert_allclose(velocity, [1.2, 0.2])

    def test_missing_joint_or_description_gives_no_limits(self):
        self.assertIsNone(joint_limits(URDF, ["arm_lift_joint", "arm_roll_joint"]))
        self.assertIsNone(joint_limits("", ["arm_lift_joint"]))


class TestGripperAction(unittest.TestCase):
    # The 'hybrid' gripper mode of the reference AIRoA client (hsr_policy.py:661-679): below the close threshold
    # grasp with effort, otherwise follow the column as a hand_motor_joint position (~-0.8 closed .. ~1.24 open)

    def test_open_value_is_a_position_target(self):
        self.assertAlmostEqual(gripper_action(1.1, 0.5), 1.1)

    def test_position_is_clipped_like_the_reference_client(self):
        self.assertAlmostEqual(gripper_action(1.3, 0.5), 1.23)

    def test_closing_value_grasps_with_effort(self):
        self.assertIsNone(gripper_action(0.3, 0.5))

    def test_the_threshold_itself_is_a_position(self):
        self.assertAlmostEqual(gripper_action(0.5, 0.5), 0.5)


class TestQueryDebugMessage(unittest.TestCase):
    def test_keeps_the_full_chunk(self):
        # The comparison recordings show exactly what the policy returned, so nothing may be rounded or dropped
        chunk = np.arange(50 * 11, dtype=np.float32).reshape(50, 11) / 7.0
        message = json.loads(query_debug_message(3, "pick up the apple", np.float32([0.2, -0.3]), chunk, 10))
        self.assertEqual(message["iteration"], 3)
        self.assertEqual(message["instruction"], "pick up the apple")
        self.assertEqual(message["prefix_steps"], 10)
        np.testing.assert_allclose(message["state"], [0.2, -0.3], rtol=1e-6)
        np.testing.assert_array_equal(np.float32(message["chunk"]), chunk)


if __name__ == '__main__':
    unittest.main()
