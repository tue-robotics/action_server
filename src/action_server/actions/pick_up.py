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
        object = {}
        if 'object' in config.semantics:
            if config.semantics['object']['type'] == 'reference':
                if 'object-designator' in config.knowledge:
                    pass
                else:
                    self._config_result.missing_field = 'object'
                    self._config_result.message = " What would you like me to pick up? "
                    return
            else:
                object = config.semantics['object']['type']
        else:
            self._config_result.missing_field = 'object'
            self._config_result.message = " What would you like me to pick up? "
            return

        # Check if the task included a location to grasp from, otherwise try to get the information from previous
        # actions' knowledge.
        location = {}
        if 'location' in config.semantics:
            location = config.semantics['location']
        elif 'location-designator' in config.knowledge:
            pass
        else:
            self._config_result.missing_field = 'location'
            self._config_result.message = " Where would you like me to pick that up? "
            return

        # If we don't have an id yet, let's see if we already know of such an object
        if not 'id' in object:
            entity_description = {'type': object['type'],
                                  'location': location}
            # TODO: check if we can get entities from a description involving a location
            grab_entities, _ = entities_from_description(entity_description, robot)
            if grab_entities:
                config.knowledge['object-designator'] = EdEntityDesignator(id=grab_entities[0].id, robot=robot)
                self._config_result.resulting_knowledge['object-designator'] = config.knowledge['object-designator']

            # Otherwise we need to go and find the object
            else:
                self._find_action = Find()
                find_config = ConfigurationData(config.semantics, config.knowledge)
                find_config_result = self._find_action.configure(robot, find_config)
                config.knowledge['object-designator'] = find_config_result.resulting_knowledge['object-designator']
                if not find_config_result.succeeded:
                    self._config_result = find_config_result
                    self._config_result.message += " so I could not grasp it "
                    return

                # Add the found object to the knowledge that is passed to the next task
                self._config_result.resulting_knowledge['object-designator'] = \
                    find_config_result.resulting_knowledge['object-designator']



        if not 'object-designator' in config.knowledge:
                # Add the found object to the knowledge that is used for the current pick-up task


        self._robot = robot

        self._object = resolve_entity_description(config.semantics["object"])

        # Only filter to entities that can be picked up, e.g not furniture etc
        manipulable_object_types = [o['name'] for o in self._knowledge.objects]
        if not self._object.type in manipulable_object_types:
            self._config_result.message = " A {} is not something I can pick up. ".format(self._object.type)
            return

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
            else:
                self._execute_result.message += " I picked up the {}. ".format(self._object.type)
        else:
            self._execute_result.message += " I could not pick up the {}. ".format(self._object_type)

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
