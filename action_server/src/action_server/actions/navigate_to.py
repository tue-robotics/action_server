import rospy

from robot_smach_states.navigation import NavigateToSymbolic, NavigateToWaypoint
from robot_smach_states.util.designators import EntityByIdDesignator
from .action import Action, ConfigurationData
from .entity_description import resolve_entity_description


class NavigateTo(Action):
    """
    The NavigateTo class implements the action to navigate to a world model entity.

    Parameters to pass to the configure() method are:
     - `object` (required): the id of the entity to navigate to
    """

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
            context.target_location = resolve_entity_description(context_dict['location'])

        # Object to navigate to
        if 'object' in context_dict:
            context.object = resolve_entity_description(context_dict['object'])

        return context

    def _configure(self, robot, config):
        self._robot = robot

        semantics = self._parse_semantics(config.semantics)
        context = self._parse_context(config.context)

        # navigate to a room
        know_target = False
        if semantics.target_location.id in self._knowledge.rooms:
            know_target = True

        # navigate to a piece of furniture
        elif semantics.target_location.id in self._knowledge.location_names:
            know_target = True

        elif semantics.target_location.type == "waypoint":
            know_target = True

        # navigate to 'it'
        elif semantics.target_location.type == 'reference' and context.object and \
            context.object.id == semantics.target_location.id:
            know_target = True

        elif semantics.target_location.type == 'person' and context.object and \
            semantics.target_location.id == context.object.id or semantics.target_location.id == "operator":
            know_target = True

        if not know_target:
            # Request find action
            self._config_result.required_context = {'action': 'find',
                                                    'object': config.semantics['target-location']}
            if 'type' in config.semantics['target-location'] and \
                config.semantics['target-location']['type'] == 'person' and \
                'location' in config.semantics['target-location']:
                self._config_result.required_context['source-location'] = config.semantics['target-location'][
                    'location']
            elif 'source-location' in config.semantics:
                self._config_result.required_context['source-location'] = config.semantics['source-location']
            return
        # Now we can assume we know the navigation goal entity!
        # operator
        if semantics.target_location.id == 'operator':
            # If we need to navigate to "me", which resolves to "operator", plant a waypoint at the current position to
            # navigate to.
            # TODO: learn to recognize the operator so that you know you found him later on
            self._robot.ed.update_entity(uuid="operator", frame_stamped=self._robot.base.get_location(),
                                         etype="waypoint")
            entity_designator = EntityByIdDesignator(self._robot, uuid=semantics.target_location.id)
            self._navigation_state_machine = NavigateToWaypoint(self._robot,
                                                                waypoint_designator=entity_designator,
                                                                radius=0.1)
            rospy.loginfo("Navigation set up for a waypoint")

        # known room or object
        elif semantics.target_location.id and semantics.target_location.type != 'person':
            entity_designator = EntityByIdDesignator(self._robot, uuid=semantics.target_location.id)
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
        # person from context
        elif semantics.target_location.type == 'person' and context.object:
            entity_designator = context.object.designator
            self._navigation_state_machine = NavigateToWaypoint(self._robot,
                                                                waypoint_designator=entity_designator,
                                                                radius=0.7)
            rospy.loginfo("Navigation set up for a waypoint")
        # entity from context
        else:
            entity_designator = context.object.designator
            self._navigation_state_machine = NavigateToSymbolic(self._robot,
                                                                entity_designator_area_name_map={
                                                                    entity_designator: "near"},
                                                                entity_lookat_designator=entity_designator)

        self._config_result.context['location'] = config.semantics['target-location']
        self._config_result.context['location']['designator'] = entity_designator
        self._config_result.succeeded = True
        return

    def _start(self):
        result = self._navigation_state_machine.execute()

        if result == 'arrived':
            self._execute_result.succeeded = True
            self._execute_result.message = " I successfully navigated."
            self._robot.speech.speak("I arrived!")
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

    from robot_skills import get_robot_from_argv

    robot = get_robot_from_argv(1)

    action = NavigateTo()

    config = ConfigurationData({'action': 'navigate-to',
                                'target-location': {'id': 'cabinet'}})

    action.configure(robot, config)
    action.start()
