import json
import unittest

import numpy as np

from action_server.vla.backend_utils import (
    apply_relative,
    gripper_action,
    gripper_holding_object,
    is_manipulation_done,
    query_debug_message,
)


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


class TestGripperAction(unittest.TestCase):
    # Reference AIRoA client 'hybrid' mode: below the close threshold grasp with effort,
    # above it follow the column as a hand_motor_joint position (~-0.8 closed .. ~1.24 open)

    def test_open_value_is_a_position_target(self):
        self.assertAlmostEqual(gripper_action(1.1, False, 0.5, 0.9), 1.1)

    def test_position_is_clipped_like_the_reference_client(self):
        self.assertAlmostEqual(gripper_action(1.3, False, 0.5, 0.9), 1.23)

    def test_closing_value_grasps_with_effort(self):
        self.assertIsNone(gripper_action(0.3, False, 0.5, 0.9))

    def test_grasp_is_kept_until_clearly_opening(self):
        # A held object keeps the fingers, and so the predicted column, near its width
        self.assertIsNone(gripper_action(0.6, True, 0.5, 0.9))
        self.assertAlmostEqual(gripper_action(1.0, True, 0.5, 0.9), 1.0)


class TestGripperHoldingObject(unittest.TestCase):
    def test_disabled_without_threshold(self):
        self.assertIsNone(gripper_holding_object("close", 5.0, 0.3, None, 1.0))

    def test_open_gripper_is_not_holding(self):
        # An open gripper also reads above the empty-closed position, so the commanded state must be 'close'
        self.assertFalse(gripper_holding_object("open", 5.0, 1.0, -0.5, 1.0))
        self.assertFalse(gripper_holding_object(None, None, 1.0, -0.5, 1.0))

    def test_closing_gripper_needs_time_to_settle(self):
        self.assertFalse(gripper_holding_object("close", 0.2, 0.3, -0.5, 1.0))

    def test_fingers_stopped_by_object(self):
        self.assertTrue(gripper_holding_object("close", 1.5, 0.55, 0.1, 1.0))

    def test_fingers_closed_on_nothing(self):
        # In the HERO simulation an empty grasp stalls at about -0.07
        self.assertFalse(gripper_holding_object("close", 1.5, -0.07, 0.1, 1.0))

    def test_position_mode_is_not_a_grasp(self):
        self.assertFalse(gripper_holding_object("position", 1.5, 0.55, 0.1, 1.0))


class TestIsManipulationDone(unittest.TestCase):
    def test_pick_up(self):
        self.assertTrue(is_manipulation_done("pick-up", True, False))
        self.assertFalse(is_manipulation_done("pick-up", False, False))

    def test_place(self):
        self.assertTrue(is_manipulation_done("place", False, True))
        self.assertFalse(is_manipulation_done("place", True, True))


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
