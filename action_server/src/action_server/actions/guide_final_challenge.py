from action import Action, ConfigurationData
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from entity_description import resolve_entity_description


class GuideFinalChallenge(Action):
    """
    The GuideFinalChallenge. class navigates to a target, telling someone to follow the robot and about arriving at the target.

    Parameters to pass to the configure() method are:
     - `object` (required): the id of the entity to navigate to
    """

    def __init__(self):
        Action.__init__(self)
        self._required_field_prompts = {'object': " Who would you like me to guide? ",
                                        'target-location': " Where would you like me to guide them? "}
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
        semantics = GuideFinalChallenge.Semantics()

        semantics.object = resolve_entity_description(semantics_dict['object'])

        if 'source-location' in semantics_dict:
            semantics.source_location = resolve_entity_description(semantics_dict['source-location'])

        if 'target-location' in semantics_dict:
            semantics.target_location = resolve_entity_description(semantics_dict['target-location'])

        return semantics

    def _configure(self, robot, config):
        self._robot = robot

        # We start by parsing semantics and context
        self._semantics = GuideFinalChallenge._parse_semantics(config.semantics)

        target_location_designator = ds.EntityByIdDesignator(self._robot, id=self._semantics.target_location.id)

        area = "near"
        if self._semantics.target_location.type == "room":
            area = "in"

        self._state_machine = states.NavigateToSymbolic(robot=self._robot,
                                                        entity_designator_area_name_map={
                                                            target_location_designator: area},
                                                        entity_lookat_designator=target_location_designator)

        self._config_result.succeeded = True

    def _start(self):
        self._robot.head.look_at_standing_person()
        self._robot.head.wait_for_motion_done()
        self._robot.speech.speak("Follow me. I will guide you to the {}".format(self._semantics.target_location.id))
        self._robot.head.close()
        self._state_machine.execute()
        self._execute_result.succeeded = True
        self._execute_result.message = " I guided! "
        self._robot.speech.speak("Here you go!")

    def _cancel(self):
        self._state_machine.request_preempt()
