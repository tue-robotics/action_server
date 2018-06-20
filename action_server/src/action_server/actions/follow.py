from action import Action, ConfigurationData
from find import Find
from entity_description import resolve_entity_description, EntityDescription
from robot_smach_states.navigation import NavigateToSymbolic

from robot_smach_states.navigation import FollowOperator
from robot_smach_states.util.designators import EdEntityDesignator

import rospy

def navigate(robot, entity_description):
    if not entity_description.type:
        return False

    if entity_description.type == "room":
        origin_area = "in"
    else:
        origin_area = "near"

    origin_entity_designator = EdEntityDesignator(robot, id=entity_description.id)
    navigation_sm = NavigateToSymbolic(
        robot=robot,
        entity_designator_area_name_map=
        {
            origin_entity_designator: origin_area
        },
        entity_lookat_designator=origin_entity_designator
    )
    return navigation_sm.execute() == "succeeded"


class Follow(Action):
    ''' The Follow class implements the action to follow a person.

    Parameters to pass to the configure() method are:
     - `target` (required): the target id to assign to the id that is followed
     - `location-from` (optional): the location to find the target to follow
    '''
    def __init__(self):
        Action.__init__(self)
        self._required_field_prompts = {'target': " Who would you like me to follow? "}
        self._required_skills = ['base']

    class Semantics:
        def __init__(self):
            self.target = None
            self.location_from = None
            self.location_to = None

    @staticmethod
    def _parse_semantics(semantics_dict):
        semantics = Follow.Semantics()

        semantics.target = resolve_entity_description(semantics_dict['target'])

        if 'location-from' in semantics_dict:
            semantics.location_from = resolve_entity_description(semantics_dict['location-from'])

        if 'location-to' in semantics_dict:
            semantics.location_to = resolve_entity_description(semantics_dict['location-to'])

        return semantics

    class Context:
        def __init__(self):
            self.location = None

    @staticmethod
    def _parse_context(context_dict):
        context = Follow.Context()

        if 'location' in context_dict:
            context.location = resolve_entity_description(context_dict['location'])

        return context

    def _configure(self, robot, config):
        semantics = Follow._parse_semantics(config.semantics)
        context = Follow._parse_context(config.context)

        self._origin = None
        self._find_action = None
        self._target = semantics.target

        if not semantics.target or semantics.target.type == "reference":
            pass
        else:
            if not semantics.target.id == "operator" and not semantics.location_from:
                self._config_result.missing_field = "location-from"
                self._config_result.message = " Where can I find {}? ".format(semantics.target.id)
                return

            if semantics.location_from and not context.location:
                self._origin = resolve_entity_description(config.semantics["location-from"])
                self._config_result.required_context = {'action': 'find',
                                                        'source-location': {'id': self._origin.id},
                                                        'object': {'type': 'person',
                                                                   'id': semantics.target.id}}
                return

        if semantics.target.id == "operator":
            self._target.id = "you"

        self._goal = semantics.location_to

        self._robot = robot

        self._follow_sm = FollowOperator(self._robot)

        self._config_result.succeeded = True

    def _start(self):
        # Do some awesome following
        res = self._follow_sm.execute()

        # If we didn't succeed in following, navigate to the goal location if we have one
        if not res == "stopped":
            if self._goal:
                if not navigate(robot=self._robot, entity_description=self._goal):
                    self._execute_result.message += " But I failed to follow {} to the {}. ".format(self._target.id,
                                                                                                    self._goal.id)
                    return
            else:
                self._execute_result.message += " But I failed to follow {} ".format(self._target.id)
                return
        self._robot.speech.speak("Thank you for guiding me.")
        self._execute_result.message += " I successfully followed {} ".format(self._target.id)
        if self._goal:
            self._execute_result.message += " to the {}".format(self._goal.id)

        self._execute_result.succeeded = True

    def _cancel(self):
        self._follow_sm.request_preempt()


if __name__ == "__main__":
    rospy.init_node('follow_test')

    import sys
    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = Follow()

    config = ConfigurationData({'action': 'follow',
              'entity': {'special': 'me'}})

    action.configure(robot, config)
    action.start()
