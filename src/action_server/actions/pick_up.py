from action import Action

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
     - `found-object-des` (required): a designator resolving to the object to grab
    '''
    def __init__(self):
        Action.__init__(self)
        self._required_skills = ['arms']
        self._find_action = None

    def _configure(self, robot, config):
        if not 'found-object-des' in config:
            # If we know where to look, and we know what to find, let's first see if we already know of such an object
            if 'location' in config and 'object' in config:
                entity_description = {'type': config['object']['type'],
                                      'location': config['location']}
                grab_entities, _ = entities_from_description(entity_description, robot)
                if grab_entities:
                    config['found-object-des'] = EdEntityDesignator(id=grab_entities[0].id, robot=robot)

            # If we don't know the object already, set up a find action to go and find it
            if not 'found-object-des' in config:
                self._find_action = Find()
                find_config_result = self._find_action.configure(robot, config)
                if not find_config_result.succeeded:
                    self._config_result = find_config_result
                    return

                config['found-object-des'] = find_config_result.resulting_knowledge['found-object-des']

        self._robot = robot

        self._object = resolve_entity_description(config["object"])

        # Only filter to entities that can be picked up, e.g not furniture etc
        manipulable_object_types = [o['name'] for o in self._knowledge.objects]
        if not self._object.type in manipulable_object_types:
            self._config_result.message = " A {} is not something I can pick up. ".format(self._object.type)
            return

        side = config['side'] if 'side' in config else 'right'

        arm_des = UnoccupiedArmDesignator(self._robot.arms, self._robot.arms[side]).lockable()
        arm_des.lock()

        self._fsm = robot_smach_states.grab.Grab(self._robot,
                                                 item=config['found-object-des'],
                                                 arm=arm_des)

        self._config_result.resulting_knowledge = {'arm-designator' : arm_des}
        self._config_result.succeeded = True

    def _start(self):
        if self._find_action:
            find_action_result = self._find_action.start()

            self._execute_result.message += find_action_result.message
            if not find_action_result.succeeded:
                return

        self._thread = threading.Thread(name='pick-up', target=self._fsm.execute)
        self._thread.start()

        self._thread.join()
        self._execute_result.succeeded = True
        self._execute_result.message += " I picked up the {}. ".format(self._object.type)

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
