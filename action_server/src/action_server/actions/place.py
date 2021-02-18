from .action import Action, ConfigurationData
from .entity_description import resolve_entity_description

from robot_skills.arm.arms import PublicArm, GripperTypes
import robot_smach_states
from robot_smach_states.manipulation import Place as PlaceSmachState
from robot_skills.util.entity import Entity
from robot_smach_states.util.designators import ArmDesignator

import rospy


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

        # Express the initial conditions in rules based on input
        # We assume we already got the object if previous action passed an arm, an object and this object has the
        # required type, or the required type is a reference.
        got_object_in_task = (
            self.context.arm_designator is not None and (self.context.object.type == self.semantics.object.type or
                                                         self.semantics.object.type == 'reference'))

        object_in_gripper = False
        if not got_object_in_task:
            arm_des = ArmDesignator(self._robot, {"required_trajectories": ["prepare_place"],
                                                  "required_objects": [self.semantics.object.type],
                                                  "required_gripper_types": [GripperTypes.GRASPING]})
            if arm_des.resolve() is not None:
                object_in_gripper = True
                self.context.arm_designator = arm_des

        got_object = got_object_in_task or object_in_gripper

        # If precondition not met, request prior action from the task manager
        if not got_object:
            # Request pick_up action
            self._config_result.required_context = {'action': 'pick-up'}
            if 'object' in config.semantics and 'type' in config.semantics['object']:
                if config.semantics['object']['type'] != 'reference':
                    self._config_result.required_context['object'] = config.semantics['object']
                elif 'object' in config.context and 'type' in config.context['object']:
                    self._config_result.required_context['object'] = {'type': config.context['object']['type']}
            if 'source-location' in config.semantics:
                self._config_result.required_context['source-location'] = config.semantics['source-location']
            if 'location' in config.context and 'id' in config.context['location']:
                self._config_result.required_context['source-location'] = config.context['location']['id']
            return
        # Now we can assume we picked up the item!

        # We should have navigated to the place where we should hand over
        at_destination = (self.context.location is not None and
                          (self.context.location == self.semantics.target_location or
                           self.semantics.target_location.type == 'reference'))

        if not at_destination:
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
                                                                                    id=self.context.location.id)
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

    import sys

    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = Place()

    semantics = {'action': 'place',
                 'entity': {'id': 'cabinet'},
                 'side': 'left',
                 'height': 0.8}

    action.configure(robot, ConfigurationData(semantics))
    action.start()
