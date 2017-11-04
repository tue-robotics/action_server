from action import Action, ConfigurationData

from util import entities_from_description
from entity_description import resolve_entity_description
from find import Find

import robot_smach_states

from robot_smach_states.util.designators import UnoccupiedArmDesignator, EdEntityDesignator
import rospy
import threading


class PickUp(Action):
    ''' The PickUp class implements the action to grasp an object.

    Parameters to pass to the configure() method are:
     - `object` (required): the id of the object to grab
     - `object-designator` (required): a designator resolving to the object to grab
    '''
    def __init__(self):
        Action.__init__(self)
        self._required_skills = ['arms']
        self._find_action = None

    def _configure(self, robot, config):
        # Check if the task included an object to grab, otherwise try to get the information from previous actions'
        # knowledge.
        using_knowledge_for_object = False
        if 'object' in config.semantics:
            if config.semantics['object']['type'] == 'reference':
                if 'object-designator' in config.knowledge:
                    using_knowledge_for_object = True
                    pass
                else:
                    self._config_result.missing_field = 'object'
                    self._config_result.message = " What would you like me to pick up? "
                    return
            else:
                pass

        else:
            self._config_result.missing_field = 'object'
            self._config_result.message = " What would you like me to pick up? "
            return

        if not using_knowledge_for_object:
            # Check if the task included a location to grasp from, otherwise try to get the information from previous
            # actions' knowledge.
            if 'location' in config.semantics:
                pass
            elif 'location-designator' in config.knowledge:
                pass
            elif 'object-designator' in config.knowledge:
                using_knowledge_for_object = True
                pass
            else:
                self._config_result.missing_field = 'location'
                self._config_result.message = " Where would you like me to pick that up? "
                return

        if not using_knowledge_for_object:
            # Only filter to entities that can be picked up, e.g not furniture etc
            manipulable_object_types = [o['name'] for o in self._knowledge.objects]
            if not config.semantics['object']['type'] in manipulable_object_types:
                self._config_result.message = " A {} is not something I can pick up. ".\
                    format(config.semantics['object']['type'])
                return

            self._find_action = Find()
            find_config = ConfigurationData(config.semantics, config.knowledge)
            find_config_result = self._find_action.configure(robot, find_config)
            if not find_config_result.succeeded:
                self._config_result = find_config_result
                return
            config.knowledge['object-designator'] = find_config_result.resulting_knowledge['object-designator']

        # Add the found object to the knowledge that is passed to the next task
        self._config_result.resulting_knowledge['object-designator'] = config.knowledge['object-designator']

        self._robot = robot

        side = config.semantics['side'] if 'side' in config.semantics else 'right'

        arm_des = UnoccupiedArmDesignator(self._robot.arms, self._robot.arms[side]).lockable()
        arm_des.lock()

        self._fsm = robot_smach_states.grab.Grab(self._robot,
                                                 item=config.knowledge['object-designator'],
                                                 arm=arm_des)

        self._config_result.resulting_knowledge['arm-designator'] = arm_des
        self._config_result.succeeded = True

    def _start(self):
        if self._find_action:
            find_action_result = self._find_action.start()

            self._execute_result.message += find_action_result.message
            if not find_action_result.succeeded:
                return

        fsm_result = self._fsm.execute()

        if fsm_result == "done":
            self._execute_result.succeeded = True
            if self._find_action:
                self._execute_result.message += " And I picked it up. "
            elif not self._config_result.resulting_knowledge['object-designator'].resolve():
                self._execute_result.message += " I could not pick anything up. ".\
                    format(self._config_result.resulting_knowledge)
            else:
                self._execute_result.message += " I picked up the {}. ".\
                    format(self._config_result.resulting_knowledge['object-designator'].resolve().type)
        else:
            if not self._config_result.resulting_knowledge['object-designator'].resolve():
                self._execute_result.message += " I could not pick anything up. ". \
                    format(self._config_result.resulting_knowledge)
            else:
                self._execute_result.message += " I could not pick up the {}. ".\
                    format(self._config_result.resulting_knowledge['object-designator'].resolve().type)

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

    config = ConfigurationData({'action': 'pick_up',
              'entity': {'id': 'cabinet'},
              'side': 'left',
              'height': 0.8})

    action.configure(robot, config)
    action.start()
