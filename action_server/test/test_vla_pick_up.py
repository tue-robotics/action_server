import unittest
from types import SimpleNamespace
from unittest import mock

import PyKDL as kdl
import rospy

from action_server.actions import pick_up
from action_server.vla.executor import VLAOutcome

# Allow rospy.Time.now() without a running node
rospy.rostime.set_rostime_initialized(True)


def make_pick_up():
    """A configured PickUp, without the knowledge and designators configure() needs."""
    action = pick_up.PickUp.__new__(pick_up.PickUp)
    action._robot = mock.MagicMock(robot_name="hero")
    arm = mock.MagicMock()
    arm.gripper = SimpleNamespace(occupied_by=None)
    arm_designator = mock.MagicMock()
    arm_designator.resolve.return_value = arm
    entity = SimpleNamespace(uuid="apple-1", etype="apple", pose=None)
    object_designator = mock.MagicMock()
    object_designator.resolve.return_value = entity
    action._config_result = SimpleNamespace(
        context={"arm-designator": arm_designator, "object": {"designator": object_designator}})
    action.context = SimpleNamespace(object=SimpleNamespace(designator=object_designator))
    action._execute_result = SimpleNamespace(succeeded=False, message="")
    action._fsm = mock.MagicMock()
    action._fsm.execute.return_value = "failed"
    action._raw_semantics = {}
    action._raw_context = {}
    return action, arm, entity


@mock.patch.object(pick_up, "maybe_run_vla_manipulation")
@mock.patch.object(pick_up, "NavigateToGrasp")
@mock.patch.object(pick_up, "is_vla_enabled_for_action", return_value=True)
class TestPickUpVLA(unittest.TestCase):
    def test_vla_grasps_after_arriving_at_the_object(self, _enabled, navigate, run_vla):
        navigate.return_value.execute.return_value = "arrived"
        run_vla.return_value = VLAOutcome(used=True, succeeded=True, message="VLA completed 'pick-up'")
        action, arm, entity = make_pick_up()
        action._start()

        action._fsm.execute.assert_not_called()
        self.assertTrue(action._execute_result.succeeded)
        self.assertIs(arm.gripper.occupied_by, entity)
        # Place composes the place pose with this pose of the object in the hand
        self.assertEqual(entity.pose.frame, kdl.Frame.Identity())
        self.assertEqual(entity.pose.header.frame_id, "map")

    def test_unreachable_object_uses_the_classic_pick_up(self, _enabled, navigate, run_vla):
        navigate.return_value.execute.return_value = "unreachable"
        action, _, _ = make_pick_up()
        action._start()
        run_vla.assert_not_called()
        action._fsm.execute.assert_called_once()

    def test_missing_arm_uses_the_classic_pick_up(self, _enabled, navigate, run_vla):
        action, _, _ = make_pick_up()
        action._config_result.context["arm-designator"].resolve.return_value = None
        action._start()
        navigate.assert_not_called()
        run_vla.assert_not_called()
        action._fsm.execute.assert_called_once()

    def test_failed_vla_falls_back_to_the_classic_pick_up(self, _enabled, navigate, run_vla):
        navigate.return_value.execute.return_value = "arrived"
        run_vla.return_value = VLAOutcome(used=False, succeeded=False, message="max_chunks")
        action, arm, _ = make_pick_up()
        action._start()
        action._fsm.execute.assert_called_once()
        self.assertIsNone(arm.gripper.occupied_by)

    def test_disabled_vla_does_not_approach(self, enabled, navigate, run_vla):
        enabled.return_value = False
        action, _, _ = make_pick_up()
        action._start()
        navigate.assert_not_called()
        run_vla.assert_not_called()
        action._fsm.execute.assert_called_once()


if __name__ == '__main__':
    unittest.main()
