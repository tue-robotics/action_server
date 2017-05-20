from action import Action
from util import entities_from_description

import threading
import rospy

import robot_smach_states
from robot_smach_states.navigation import NavigateToWaypoint, NavigateToSymbolic
from robot_smach_states.util.designators import EdEntityDesignator


class NavigateTo(Action):
    ''' The NavigateTo class implements the action to navigate to a world model entity.

    Parameters to pass to the configure() method are:
     - `object` (required): the id of the entity to navigate to
    '''
    def __init__(self):
        Action.__init__(self)
        self._required_parameters = {'object' : 'Where should I go?'}

    def _configure(self, robot, config):
        # TODO: this should also check if the given robot is capable of this action.
        self._robot = robot
        self._entity_description = config['object']
        self._config_result.succeeded = True
        return

    def _start(self):
        (entities, error_msg) = entities_from_description(self._entity_description, self._robot)

        if not entities:
            rospy.loginfo("No knowledge of a {} in the world model.".format(self._entity_description["id"]))
            self._execute_result.succeeded = False
            self._execute_result.message = " I have no knowledge of a {} in my world model. ".format(self._entity_description["id"])
            return

        e = entities[0]

        if e.is_a("waypoint"):
            self._fsm = NavigateToWaypoint(self._robot,
                                          waypoint_designator=EdEntityDesignator(self._robot, id=e.id),
                                          radius=0.1)
            rospy.loginfo("Navigating to waypoint")
        else:
            if e.is_a("room"):
                area = "in"
            elif e.is_a("furniture"):
                area = "in_front_of"
            else:
                area = "near"

            entity_designator = EdEntityDesignator(robot=self._robot, id=e.id)
            self._fsm = NavigateToSymbolic(self._robot,
                                           entity_designator_area_name_map={entity_designator: area},
                                           entity_lookat_designator=entity_designator)

        result = self._fsm.execute()

        if result == 'arrived':
            self._execute_result.succeeded = True
            self._execute_result.message = " I successfully navigated to the {}".format(e.id)
            self._robot.speech.speak("I arrived at the {}".format(e.id))
        elif result == 'unreachable':
            self._execute_result.message = " I was unable to get to the {} because my path was blocked. ".format(e.id)
            self._robot.speech.speak("Oops, it seems that I can't get there right now.")
        else:
            self._execute_result.message = " I don't know why, but I couldn't find the place I should go. "
            self._robot.speech.speak("I don't know why, but I couldn't find the place I should go.")


    def _cancel(self):
        if self._fsm.is_running:
            self._fsm.request_preempt()


if __name__ == "__main__":
    rospy.init_node('navigate_to_test')

    import sys
    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = NavigateTo()

    config = {'action': 'navigate-to',
              'object': {'id': 'cabinet'}}

    action.configure(robot, config)
    action.start()
