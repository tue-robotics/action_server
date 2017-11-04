from action import Action, ConfigurationData
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
        self._required_parameters = {'object' : ' Where would you like me to go? '}
        self._required_skills = ['base']

    def _configure(self, robot, config):
        self._robot = robot
        self._goal_name = ""
        self._fsm = None

        entity_description = config.semantics['object']

        (entities, error_msg) = entities_from_description(entity_description, self._robot)

        # Check if we got any entities already...
        if 'type' in entity_description and entity_description['type'] == "reference" and \
                'object-designator' in config.knowledge:
            entity_designator = config.knowledge['object-designator']

            area = "near"
            self._fsm = NavigateToSymbolic(self._robot,
                                           entity_designator_area_name_map={entity_designator: area},
                                           entity_lookat_designator=entity_designator)

        elif not entities:
            # If not, check if we at least know a type...
            if 'type' in entity_description:
                # If we have a type, return succeeded, because that may resolve to an ID later. But remember to check
                # again before actually executing the task.
                rospy.loginfo("No knowledge of a {} in the world model yet, but who knows what I will encounter.".
                              format(entity_description['type']))
                self._config_result.succeeded = True
                self._entity_description = entity_description
            else:
                # If we don't even have a type, we cannot navigate.
                rospy.loginfo("No knowledge of a {} in the world model.".format(entity_description['id']))
                self._config_result.message = " I have no knowledge of a {} in my world model. ".format(
                    entity_description["id"])
            return
        else:
            # Take the best match and set up the state machine
            e = entities[0]
            entity_designator = EdEntityDesignator(self._robot, id=e.id)

            self._goal_name = e.id

            if e.is_a("waypoint"):
                self._fsm = NavigateToWaypoint(self._robot,
                                               waypoint_designator=entity_designator,
                                               radius=0.1)
                rospy.loginfo("Navigation set up for a waypoint")
            else:
                if e.is_a("room"):
                    area = "in"
                    rospy.loginfo("Navigation set up for a room")
                elif e.is_a("furniture"):
                    area = "in_front_of"
                    rospy.loginfo("Navigation set up for a piece of furniture")
                else:
                    area = "near"

                self._fsm = NavigateToSymbolic(self._robot,
                                               entity_designator_area_name_map={entity_designator: area},
                                               entity_lookat_designator=entity_designator)

        self._config_result.resulting_knowledge['location-designator'] = entity_designator
        self._config_result.succeeded = True


    def _start(self):
        # Check if we already set up a state machine (may not have happened if we could not get an ID earlier)
        if not self._fsm:
            (entities, error_msg) = entities_from_description(self._entity_description, self._robot)
            if not entities:
                self._execute_result.message = " I don't have knowledge of a {}.".\
                    format(self._entity_description['type'])
                return

            e = entities[0]

            self._goal_name = e.type

            entity_designator = EdEntityDesignator(robot=self._robot, id=e.id)
            self._fsm = NavigateToSymbolic(self._robot,
                                           entity_designator_area_name_map={entity_designator: "near"},
                                           entity_lookat_designator=entity_designator)

        result = self._fsm.execute()

        if result == 'arrived':
            self._execute_result.succeeded = True
            self._execute_result.message = " I successfully navigated to the {}.".format(self._goal_name)
            self._robot.speech.speak("I arrived at the {}".format(self._goal_name))
        elif result == 'unreachable':
            self._execute_result.message = " I was unable to get to the {} because my path was blocked. ".\
                format(self._goal_name)
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

    config = ConfigurationData({'action': 'navigate-to',
              'object': {'id': 'cabinet'}})

    action.configure(robot, config)
    action.start()
