from action import Action

from util import entities_from_description
from entity_description import resolve_entity_description

import robot_smach_states

from robot_smach_states.util.designators import UnoccupiedArmDesignator, EdEntityDesignator
import rospy
import threading


class PickUp(Action):

    def __init__(self):
        Action.__init__(self)
        self._required_field_prompts = {'object' : " I don't know what to pick up. ",
                                        'found-object-des' : " I don't expect to know what to grab when I get there. "}

    def _configure(self, robot, config):
        # TODO: remove right and left
        if not hasattr(robot, 'rightArm') or not hasattr(robot, 'leftArm'):
            rospy.logerr("Robot {} does not have attribute 'leftArm'".format(robot.robot_name))
            self._config_result.missing_skill = "arm"
            return

        self._robot = robot

        self._object = resolve_entity_description(config["object"])

        # Only filter to entities that can be picked up, e.g not furniture etc
        manipulable_object_types = [o['name'] for o in self._knowledge.objects]
        if not self._object.type in manipulable_object_types:
            self._config_result.message = " A {} is not something I can pick up. ".format(self._object.type)
            return

        self._side = config['side'] if 'side' in config else 'right'

        arm_des = UnoccupiedArmDesignator(self._robot.arms, self._robot.arms[self._side])

        self._fsm = robot_smach_states.grab.Grab(self._robot,
                                                 item=config['found-object-des'],
                                                 arm=arm_des)

        self._config_result.resulting_knowledge = {'arm-designator' : arm_des}
        self._config_result.succeeded = True

    def _start(self):
        self._thread = threading.Thread(name='pick-up', target=self._fsm.execute)
        self._thread.start()

        # TODO: implement possibility to cancel action when started, but still block while executing
        self._thread.join()
        self._execute_result.succeeded = True
        self._execute_result.message = "picked up the " + self._entity.id

    def _cancel(self):
        if self._fsm.is_running:
            self._fsm.request_preempt()

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

    action = PickUp()

    config = {'action': 'pick_up',
              'entity': {'id': 'cabinet'},
              'side': 'left',
              'height': 0.8}

    action.configure(robot, config)
    action.start()
