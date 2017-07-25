from action import Action, ConfigurationData
from util import entities_from_description
from entity_description import resolve_entity_description

from robot_skills.arms import Arm
import robot_smach_states
from robot_smach_states.manipulation import Place as PlaceSmachState
from robot_skills.util.entity import Entity

import rospy
import threading


class Place(Action):
    ''' The Place class implements the action to place something on an object.

    Parameters to pass to the configure() method are:
     - `entity` (required): Entity to place the object on;
     - `arm-designator` (required): Designator resolving to the arm to place with
    '''
    def __init__(self):
        Action.__init__(self)
        self._place = None
        self._thread = None
        self._goal_entity = None

        self._required_field_prompts = {'location': " Where should I leave the object? "}

        self._required_passed_knowledge = {'arm-designator': " I won't be able to place anything before I grasped it. "
                                                             "Please tell me what to get. "}

        self._required_skills = ['arms']

    def _configure(self, robot, config):
        # TODO: remove right and left
        if not hasattr(robot, 'rightArm') or not hasattr(robot, 'leftArm'):
            rospy.logerr("Robot {} does not have attribute 'speech'".format(robot.robot_name))
            self._config_result.missing_skill = "speech"
            return

        self._robot = robot

        try:
            self._goal_entity = resolve_entity_description(config.semantics["location"])
        except KeyError:
            self._config_result.message = " Where should I place it? "
            rospy.logwarn("Specify a 'location' to place.")
            return

        (entities, error_msg) = entities_from_description(config.semantics["location"], robot)
        if not entities:
            rospy.logwarn(error_msg)
            return
        self._place_entity = entities[0]

        try:
            side = config.semantics["side"]
        except KeyError:
            side = "right"  # Default

        if side == "left":
            self._arm = robot.leftArm
            self._goal_y = 0.2
        else:
            self._arm = robot.rightArm
            self._goal_y = -0.2

        try:
            self._arm = config.semantics['arm-designator'].resolve()
        except:
            pass

        try:
            self._height = config.semantics["height"]
        except KeyError:
            self._height = 0.8

        self._config_result.succeeded = True

    def _start(self):
        # Which item do we want to place? The object in the hand we indicated
        item_to_place = robot_smach_states.util.designators.Designator(self._arm.occupied_by, resolve_type=Entity)

        # No need for ArmHoldingEntityDesignator, we already know that from the config
        arm_with_item_designator = robot_smach_states.util.designators.Designator(self._arm, resolve_type=Arm)
        place_position = robot_smach_states.util.designators.EmptySpotDesignator(
            self._robot,
            robot_smach_states.util.designators.EdEntityDesignator(self._robot, id=self._place_entity.id)
        )

        self._place = PlaceSmachState(self._robot, item_to_place, place_position, arm_with_item_designator)

        self._thread = threading.Thread(name='grab', target=self._place.execute)
        self._thread.start()

        self._thread.join()
        self._execute_result.message = " I placed the {} on the {}. ".format(item_to_place.resolve().type,
                                                                             self._place_entity.id)
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

    semantics = {'action': 'place',
                 'entity': {'id': 'cabinet'},
                 'side': 'left',
                 'height': 0.8}

    action.configure(robot, ConfigurationData(semantics))
    action.start()
