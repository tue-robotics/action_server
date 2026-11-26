import rospy

from ed.entity import Entity

import robot_smach_states
from robot_skills.arm.arms import GripperTypes, PublicArm
from robot_smach_states.manipulation import Place as PlaceSmachState
from robot_smach_states.util.designators import ArmDesignator
from .action import Action, ConfigurationData
from .entity_description import resolve_entity_description
from .util.carrying import holds_wanted_object, object_to_fetch, reached, REFERENCE


class Place(Action):
    """
    The Place class implements the action to place something on an object.

    Parameters to pass to the configure() method are:
     - `entity` (required): Entity to place the object on;
     - `arm-designator` (required): Designator resolving to the arm to place with
    """

    def __init__(self):
        Action.__init__(self)

        self._required_field_prompts = {'target-location': " Where should I leave the object? "}

        self._required_skills = ['arms']

    class Semantics:
        def __init__(self):
            self.target_location = None
            self.object = None
            self.source_location = None
            self.arm = None

    @staticmethod
    def _parse_semantics(semantics_dict):
        semantics = Place.Semantics()

        semantics.target_location = resolve_entity_description(semantics_dict['target-location'])

        if 'object' in semantics_dict:
            semantics.object = resolve_entity_description(semantics_dict['object'])

        if 'source-location' in semantics_dict:
            semantics.source_location = resolve_entity_description(semantics_dict['source-location'])

        if 'arm' in semantics_dict:
            semantics.arm = semantics_dict['arm']

        return semantics

    class Context:
        def __init__(self):
            self.arm_designator = None
            self.object = None
            self.location = None

    @staticmethod
    def _parse_context(context_dict):
        context = Place.Context()

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
        # for (see hand_over.py), or if an arm holds an object of the requested type.
        got_object_in_task = holds_wanted_object(self.context, self.semantics.object, self._knowledge)

        object_in_gripper = False
        wanted_type = self.semantics.object.type if self.semantics.object else None
        # Only a real type can be looked up in the arms: a category or "it" would ask for an arm holding None, which
        # matches an empty arm, and then the object would never be fetched
        if not got_object_in_task and wanted_type and wanted_type != REFERENCE:
            arm_des = ArmDesignator(self._robot, {"required_trajectories": ["prepare_place"],
                                                  "required_objects": [wanted_type],
                                                  "required_gripper_types": [GripperTypes.GRASPING]})
            if arm_des.resolve() is not None:
                object_in_gripper = True
                self.context.arm_designator = arm_des

        got_object = got_object_in_task or object_in_gripper

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

        # We should have navigated to the place where we should place it
        if not reached(config.context, config.semantics['target-location']):
            # Request navigation action
            self._config_result.required_context = {'action': 'navigate-to',
                                                    'target-location': config.semantics['target-location']}
            return
        # We can now assume that we are at the destination for handover!

        self._config_result.succeeded = True
        return

    def _start(self):
        # We either got an arm, or we know which arm to place with
        arm_designator = None
        if self.semantics.arm:
            arm_designator = robot_smach_states.util.designators.Designator(self.semantics.arm, resolve_type=PublicArm)
        elif self.context.arm_designator:
            arm_designator = self.context.arm_designator
        else:
            arm_designator = ArmDesignator(self._robot, {"required_objects": [self.semantics.object.type],
                                                         "required_trajectories": ["prepare_place"],
                                                         "required_gripper_types": [GripperTypes.GRASPING]})

        if not arm_designator:
            self._execute_result.message = " I was unable to resolve which arm to place with."
            self._execute_result.succeeded = False
            return

        item_to_place = robot_smach_states.util.designators.Designator(arm_designator.resolve().occupied_by,
                                                                       resolve_type=Entity)

        entity_to_place_on = robot_smach_states.util.designators.EdEntityDesignator(self._robot,
                                                                                    uuid=self.context.location.id)
        self._place = PlaceSmachState(self._robot, item_to_place,
                                      entity_to_place_on,
                                      arm_designator,
                                      place_volume="on_top_of")

        state_machine_result = self._place.execute()
        if state_machine_result == 'done':
            self._execute_result.message = " I placed successfully. "
            self._execute_result.succeeded = True
        else:
            self._execute_result.message = " I failed to place. "
            self._execute_result.succeeded = True

    def _cancel(self):
        if self._place.is_running:
            self._place.request_preempt()


if __name__ == "__main__":

    rospy.init_node('place_test')

    from robot_skills import get_robot_from_argv

    robot = get_robot_from_argv(1)

    action = Place()

    semantics = {'action': 'place',
                 'entity': {'id': 'cabinet'},
                 'height': 0.8}

    action.configure(robot, ConfigurationData(semantics))
    action.start()
