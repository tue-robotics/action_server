from action import Action, ConfigurationData
from entity_description import resolve_entity_description

import rospy

from robot_smach_states.navigation import NavigateToWaypoint, NavigateToSymbolic
from robot_smach_states.util.designators import EdEntityDesignator, EntityByIdDesignator


class NavigateTo(Action):
    ''' The NavigateTo class implements the action to navigate to a world model entity.

    Parameters to pass to the configure() method are:
     - `object` (required): the id of the entity to navigate to
    '''
    def __init__(self):
        Action.__init__(self)
        self._required_parameters = {'target-location': ' Where would you like me to go? '}
        self._required_skills = ['base']

    class Semantics:
        def __init__(self):
            self.target_location = None

    @staticmethod
    def _parse_semantics(semantics_dict):
        semantics = NavigateTo.Semantics()
        semantics.target_location = resolve_entity_description(semantics_dict['target-location'])
        return semantics

    class Context:
        def __init__(self):
            self.target_location = None
            self.object = None

    @staticmethod
    def _parse_context(context_dict):
        context = NavigateTo.Context()

        # Location to navigate to
        if 'location' in context_dict:
            context.target_location = context_dict['location']

        # Object to navigate to
        if 'object-designator' in context_dict:
            context.object = context_dict['object-designator']

        return context

    def _configure(self, robot, config):
        self._robot = robot

        # If we need to navigate to "me", which resolves to "operator", plant a waypoint at the current position to
        # navigate to.
        # TODO: learn to recognize the operator so that you know you found him later on
        self._robot.ed.update_entity(id="operator", frame_stamped=self._robot.base.get_location(),
                                     type="waypoint")

        semantics = self._parse_semantics(config.semantics)
        context = self._parse_context(config.context)

        know_target = (semantics.target_location.id or
                       (semantics.target_location.type == 'reference' and context.object))

        if not know_target:
            # Request find action
            self._config_result.required_context = {'action': 'find',
                                                    'object': config.semantics['object'],
                                                    'source-location': config.semantics['source-location']}
            return
        # Now we can assume we know the navigation goal entity!

        if semantics.target_location.id:
            entity_designator = EntityByIdDesignator(self._robot, id=semantics.target_location.id)
            e = entity_designator.resolve()

            if e.is_a("waypoint"):
                self._navigation_state_machine = NavigateToWaypoint(self._robot,
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

                self._navigation_state_machine = NavigateToSymbolic(self._robot,
                                                                    entity_designator_area_name_map={
                                                                        entity_designator: area},
                                                                    entity_lookat_designator=entity_designator)
        else:
            entity_designator = context.object
            self._navigation_state_machine = NavigateToSymbolic(self._robot,
                                                                entity_designator_area_name_map={
                                                                    entity_designator: "near"},
                                                                entity_lookat_designator=entity_designator)

        self._config_result.context['location-designator'] = entity_designator
        self._config_result.context['location'] = config.semantics['target-location']
        self._config_result.succeeded = True
        return

    def _start(self):
        result = self._navigation_state_machine.execute()

        if result == 'arrived':
            self._execute_result.succeeded = True
            # self._execute_result.message = " I successfully navigated to the {}.".format(self._goal_name)
            # self._robot.speech.speak("I arrived at the {}".format(self._goal_name))
        elif result == 'unreachable':
            # self._execute_result.message = " I was unable to get to the {} because my path was blocked. ".\
                # format(self._goal_name)
            self._robot.speech.speak("Oops, it seems that I can't get there right now.")
        else:
            self._execute_result.message = " I don't know why, but I couldn't find the place I should go. "
            self._robot.speech.speak("I don't know why, but I couldn't find the place I should go.")

    def _cancel(self):
        if self._navigation_state_machine.is_running:
            self._navigation_state_machine.request_preempt()


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
                                'target-location': {'id': 'cabinet'}})

    action.configure(robot, config)
    action.start()
