import rospy

from .action import Action, ConfigurationData
from .entity_description import resolve_entity_description
from .util.carrying import holds_wanted_object, object_to_fetch, reached, REFERENCE


class HandOver(Action):
    """
    The HandOver class implements the action to hand over an object to a person.

    Parameters to pass to the configure() method are 'source-location' (required), 'target-location' (required) and
    an object to bring (required).
    """

    def __init__(self):
        Action.__init__(self)
        self._required_field_prompts = {'target-location': " Who would you like me to hand the object? ",
                                        'object': " What would you like me to hand over? "}
        self._required_skills = ['base']

    class Semantics:
        def __init__(self):
            self.target_location = None
            self.object = None
            self.source_location = None

    @staticmethod
    def _parse_semantics(semantics_dict):
        semantics = HandOver.Semantics()

        semantics.target_location = resolve_entity_description(semantics_dict['target-location'])
        semantics.object = resolve_entity_description(semantics_dict['object'])

        if 'source-location' in semantics_dict:
            semantics.source_location = resolve_entity_description(semantics_dict['source-location'])

        return semantics

    class Context:
        def __init__(self):
            self.arm_designator = None
            self.object = None
            self.location = None

    @staticmethod
    def _parse_context(context_dict):
        context = HandOver.Context()

        if 'arm-designator' in context_dict:
            context.arm_designator = context_dict['arm-designator']

        if 'object' in context_dict:
            context.object = resolve_entity_description(context_dict['object'])

        if 'location' in context_dict:
            context.location = resolve_entity_description(context_dict['location'])

        return context

    def _configure(self, robot, config):
        self._robot = robot

        # Parse semantics and context to a convenient object
        self.semantics = self._parse_semantics(config.semantics)
        self.context = self._parse_context(config.context)

        # We already got the object if a previous action passed an arm and an object that fits what this step asks
        # for: a pick-up of {'type': 'coke'} provides {'category': 'drink'}, a pick-up of {'category': 'drink'} does
        # not provide {'type': 'coke'} or {'category': 'food'}, and "it" is whatever was picked up.
        got_object = holds_wanted_object(self.context, self.semantics.object, self._knowledge)
        # If precondition not met, request prior action from the task manager
        if not got_object:
            # Request pick_up action
            self._config_result.required_context = {'action': 'pick-up'}
            object_to_pick_up = object_to_fetch(config.semantics, config.context)
            if object_to_pick_up:
                self._config_result.required_context['object'] = object_to_pick_up

            if 'source-location' in config.semantics:
                self._config_result.required_context['source-location'] = config.semantics['source-location']
            if 'location' in config.context and 'id' in config.context['location']:
                self._config_result.required_context['source-location'] = config.context['location']['id']
            return

        # Now we can assume we picked up the item!
        # We should have navigated to the place where we should hand over
        if not reached(config.context, config.semantics['target-location']):
            # Request navigation action
            self._config_result.required_context = {'action': 'navigate-to',
                                                    'target-location': config.semantics['target-location']}
            return
        # We can now assume that we are at the destination for handover!

        self._robot.speech.speak("Bring the action!")

        self._config_result.succeeded = True

    def _object_name(self):
        """What to call the object: the type of the held entity ("apple"), else the type or the category ("drink")
        of the description, else of the object of "it" """
        held_name = getattr(self, "_held_name", None)
        if held_name:
            return held_name
        for description in (self.semantics.object, self.context.object):
            if description is None:
                continue
            if description.type and description.type != REFERENCE:
                return description.type
            if description.category:
                return description.category
        return "object"

    def _handover(self):
        # TODO: Move this code to the handover smach state
        arm = self.context.arm_designator.resolve()
        # Remember what the arm holds, before the hand-over empties it
        self._held_name = getattr(getattr(arm, "occupied_by", None), "etype", None) if arm else None
        self._robot.speech.speak("I will hand over the {} now".format(self._object_name()))
        arm.send_joint_goal('handover_to_human')
        arm.wait_for_motion_done()

        self._robot.speech.speak("Please take it from my gripper.", block=False)

        attempt = 0

        while not arm.handover_to_human(timeout=10) and attempt < 2:
            self._robot.speech.speak("Please take it from my gripper.", block=False)
            attempt += 1

        self._robot.speech.speak("I will open my gripper now.", block=False)

        self._robot.ed.update_entity(uuid=arm.occupied_by.uuid, action='remove')
        arm.send_gripper_goal('open')
        arm.wait_for_motion_done()

        arm.reset()
        arm.wait_for_motion_done()

        arm.occupied_by = None

    def _start(self):
        # Handover
        self._handover()

        self._execute_result.succeeded = True
        if self.semantics.source_location:
            self._execute_result.message += " I brought a {} from {} to {}. ".format(self._object_name(),
                                                                                     self.semantics.source_location.id,
                                                                                     self.semantics.target_location.id)
        else:
            self._execute_result.message += " I brought a {} to {}. ".format(self._object_name(),
                                                                             self.semantics.target_location.id)

    def _cancel(self):
        pass


if __name__ == "__main__":
    rospy.init_node('bring_test')

    from robot_skills import get_robot_from_argv

    robot = get_robot_from_argv(1)

    action = HandOver()

    config = ConfigurationData({'action': 'hand-over',
                                'object': {'location': 'cabinet'},
                                'source-location': {'id': 'cabinet'},
                                'target-location': {'id': 'dinner_table'}})

    action.configure(robot, config)
    action.start()
