from action import Action
from util import PlaceDesignator

from robot_skills.arms import Arm
import robot_smach_states
from robot_smach_states.manipulation import Place as PlaceSmachState
import ed.msg

import threading

from util import entities_from_description

class Place(Action):

    def __init__(self):
        self._place = None
        self._thread = None
        self._goal_entity = None

    def _start(self, config, robot):
        print "Place!"

        try:
            self._goal_entity = config["entity"]
        except KeyError:
            return "Specify an 'entity' to place on"

        # Get 'from' location
        (entities, error_msg) = entities_from_description(config["entity"], robot)
        if not entities:
            return error_msg
        place_entity = entities[0]

        try:
            side = config["side"]
        except KeyError:
            side = "right"  # Default

        if side == "left":
            arm = robot.leftArm
            goal_y = 0.2
        else:
            arm = robot.rightArm
            goal_y = -0.2

        try:
            height = config["height"]
        except KeyError:
            height = 0.8

        item_to_place = robot_smach_states.util.designators.Designator(arm.occupied_by, resolve_type=ed.msg.EntityInfo)  # Which item do we want to place? The object in the hand we indicated
        arm_with_item_designator = robot_smach_states.util.designators.Designator(arm, resolve_type=Arm)  # No need for ArmHoldingEntityDesignator, we already know that from the config
        place_position = robot_smach_states.util.designators.EmptySpotDesignator(robot, robot_smach_states.util.designators.EdEntityDesignator(robot, id=place_entity.id))

        self._place = PlaceSmachState(robot, item_to_place, place_position, arm_with_item_designator)

        self._thread = threading.Thread(name='grab', target=self._place.execute)
        self._thread.start()

    def _cancel(self):
        if self._place.is_running:
            self._place.request_preempt()

        # Wait until canceled
        self._thread.join()
