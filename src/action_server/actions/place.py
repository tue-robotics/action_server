from action import Action
from util import entities_from_description

from robot_skills.arms import Arm
import robot_smach_states
from robot_smach_states.manipulation import Place as PlaceSmachState
from robot_skills.util.entity import Entity

import rospy
import threading


class Place(Action):

    def __init__(self):
        Action.__init__(self)
        self._place = None
        self._thread = None
        self._goal_entity = None

    def _configure(self, robot, config):
        self._robot = robot

        try:
            self._goal_entity = config["entity"]
        except KeyError:
            rospy.logwarn("Specify an 'entity' to place on")
            return

        (entities, error_msg) = entities_from_description(config["entity"], robot)
        if not entities:
            rospy.logwarn(error_msg)
            return
        self._place_entity = entities[0]

        try:
            side = config["side"]
        except KeyError:
            side = "right"  # Default

        if side == "left":
            self._arm = robot.leftArm
            self._goal_y = 0.2
        else:
            self._arm = robot.rightArm
            self._goal_y = -0.2

        try:
            self._height = config["height"]
        except KeyError:
            self._height = 0.8

        self._config_result.succeeded = True

    def _start(self):
        item_to_place = robot_smach_states.util.designators.Designator(self._arm.occupied_by, resolve_type=Entity)  # Which item do we want to place? The object in the hand we indicated
        arm_with_item_designator = robot_smach_states.util.designators.Designator(self._arm, resolve_type=Arm)  # No need for ArmHoldingEntityDesignator, we already know that from the config
        place_position = robot_smach_states.util.designators.EmptySpotDesignator(self._robot, robot_smach_states.util.designators.EdEntityDesignator(self._robot, id=self._place_entity.id))

        self._place = PlaceSmachState(self._robot, item_to_place, place_position, arm_with_item_designator)

        self._thread = threading.Thread(name='grab', target=self._place.execute)
        self._thread.start()

        self._thread.join()
        self._execute_result.succeeded = True

    def _cancel(self):
        if self._place.is_running:
            self._place.request_preempt()

        # Wait until canceled
        self._thread.join()

if __name__ == "__main__":
    rospy.init_node('place_test')

    import sys
    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = Place()

    config = {'action': 'place',
              'entity': {'id': 'cabinet'},
              'side': 'left',
              'height': 0.8}

    action.configure(robot, config)
    action.start()
