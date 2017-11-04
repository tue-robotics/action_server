from action import Action, ConfigurationData
from util import entities_from_description
from entity_description import resolve_entity_description

from robot_skills.arms import Arm
import robot_smach_states
from robot_smach_states.manipulation import Place as PlaceSmachState
from robot_skills.util.entity import Entity
from robot_smach_states.util.designators import UnoccupiedArmDesignator, EdEntityDesignator

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

        self._required_passed_knowledge = {'object-designator': " I won't be able to place anything before I grasped "
                                                                "it. Please tell me what to get. "}

        self._required_skills = ['arms']

    def _configure(self, robot, config):
        # TODO: remove right and left
        if not hasattr(robot, 'rightArm') or not hasattr(robot, 'leftArm'):
            rospy.logerr("Robot {} does not have attribute 'speech'".format(robot.robot_name))
            self._config_result.missing_skill = "speech"
            return

        self._robot = robot

        self._goal_entity = resolve_entity_description(config.semantics["location"])

        (entities, error_msg) = entities_from_description(config.semantics["location"], robot)

        if not entities:
            rospy.logwarn(error_msg)
            self._config_result.message = " I have no knowledge of a {} in my world model. ".\
                format(config.semantics['location']['id'])
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
            self._arm_designator = config.knowledge['arm-designator']
        except:
            self._arm_designator = None

        self._object_designator = config.knowledge['object-designator']

        try:
            self._height = config.semantics["height"]
        except KeyError:
            self._height = 0.8

        self._config_result.succeeded = True

    def _start(self):
        # Resolve the arm we want to place with
        if self._arm_designator:
            arm = self._arm_designator.resolve()
            if arm:
                self._arm = arm

        # Which item do we want to place? The object in the hand we indicated
        item_to_place = robot_smach_states.util.designators.Designator(self._arm.occupied_by, resolve_type=Entity)

        # In the case of a task like "Find the coke and place it on the table" grasping is implied, so grasp it here
        if item_to_place.resolve():
            self._object_designator = item_to_place
        else:
            arm_des = UnoccupiedArmDesignator(self._robot.arms, self._robot.arms['right']).lockable()
            arm_des.lock()
            self._grab_state_machine = robot_smach_states.grab.Grab(self._robot,
                                                                    item=self._object_designator,
                                                                    arm=arm_des)

            self._grab_state_machine.execute()

            self._arm = arm_des.resolve()
            if not self._arm:
                self._execute_result.message += \
                    ' I was unable to grasp the object because I believe that my grippers were occupied. '
                self._execute_result.succeeded = False
                return

            if not self._object_designator.resolve():
                self._execute_result.message += ' I was unable to grasp the object because I lost it while grasping. '
                self._execute_result.succeeded = False
                return

        # No need for ArmHoldingEntityDesignator, we already know that from the config
        arm_with_item_designator = robot_smach_states.util.designators.Designator(self._arm, resolve_type=Arm)
        place_position = robot_smach_states.util.designators.EmptySpotDesignator(
            self._robot,
            robot_smach_states.util.designators.EdEntityDesignator(self._robot, id=self._place_entity.id)
        )

        self._place = PlaceSmachState(self._robot, self._object_designator, place_position, arm_with_item_designator)

        self._thread = threading.Thread(name='grab', target=self._place.execute)
        self._thread.start()

        self._thread.join()
        self._execute_result.message = " I placed the {} on the {}. ".format(self._object_designator.resolve().type,
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
