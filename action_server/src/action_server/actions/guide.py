from action import Action, ConfigurationData
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from entity_description import resolve_entity_description

import rospy


class Guide(Action):
    ''' The Guide class navigates to a target, telling someone to follow the robot and about arriving at the target.

    Parameters to pass to the configure() method are:
     - `object` (required): the id of the entity to navigate to
    '''
    def __init__(self):
        Action.__init__(self)
        self._required_field_prompts = {'object': " What exactly would you like me to find? "}
        self._required_skills = ['head', 'base', 'rightArm', 'speech']
        self._follower_id = None
        self._target_id = None

    class Semantics:
        def __init__(self):
            self.object = None
            self.source_location = None
            self.target_location = None

    @staticmethod
    def _parse_semantics(semantics_dict):
        semantics = Guide.Semantics()

        semantics.object = resolve_entity_description(semantics_dict['object'])

        if 'source-location' in semantics_dict:
            semantics.source_location = resolve_entity_description(semantics_dict['source-location'])

        if 'target-location' in semantics_dict:
            semantics.target_location = resolve_entity_description(semantics_dict['target-location'])

        return semantics

    class Context:
        def __init__(self):
            self.person = None

    @staticmethod
    def _parse_context(context_dict):
        context = Guide.Context()

        if 'person' in context_dict:
            context.person = resolve_entity_description(context_dict['person'])

        return context

    def _configure(self, robot, config):
        self._robot = robot

        # We start by parsing semantics and context
        self._semantics = Guide._parse_semantics(config.semantics)
        self._context = Guide._parse_context(config.context)

        person_known = False
        if self._robot.ed.get_entity(id=self._semantics.object.id):
            person_known = True
        elif self._context.person:
            person_known = True

        # Before we start guiding, we need to know we're at the person we should guide
        # we check this by checking if we found a person earlier
        if not person_known:
            self._config_result.required_context = {
                'action': 'find',
                'object': config.semantics['object']
            }
            if self._semantics.source_location:
                self._config_result.required_context['source-location'] = config.semantics['source-location']
            return

        target_location_designator = ds.EntityByIdDesignator(self._robot, id=self._semantics.target_location.id)

        self._guide_state_machine = states.Guide(robot=self._robot,
                                                 target_location=target_location_designator,
                                                 follower=self._context.person.designator)

        self._config_result.succeeded = True

    def _start(self):
        self._guide_state_machine.execute()
        self._execute_result.succeeded = True
        self._execute_result.message = "I guided "

    def _cancel(self):
        self._guide_state_machine.request_preempt()


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

    action = Guide()

    config = ConfigurationData({'action': 'guide',
              'object': {'id': 'cabinet'}})

    action.configure(robot, config)
    action.start()
