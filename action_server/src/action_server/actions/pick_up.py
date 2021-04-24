import rospy

from robot_skills.arm.arms import GripperTypes
from robot_smach_states.manipulation import Grab
from robot_smach_states.util.designators import UnoccupiedArmDesignator
from .action import Action, ConfigurationData
from .entity_description import resolve_entity_description


class PickUp(Action):
    """
    The PickUp class implements the action to grasp an object.

    Parameters to pass to the configure() method are:
     - `object` (required): the id of the object to grab
     - `object-designator` (required): a designator resolving to the object to grab
    """

    def __init__(self):
        Action.__init__(self)
        self._required_skills = ['arms']
        self._required_field_prompts = {'object': " What would you like me to pick up? ",
                                        'source-location': " Where would you like me to pick that up? "}  # TODO: handle source location from context?

    class Semantics:
        def __init__(self):
            self.object = None
            self.source_location = None

    @staticmethod
    def _parse_semantics(semantics_dict):
        semantics = PickUp.Semantics()

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
        context = PickUp.Context()

        if 'arm-designator' in context_dict:
            context.arm_designator = context_dict['arm-designator']

        if 'object' in context_dict:
            context.object = resolve_entity_description(context_dict['object'])

        if 'location' in context_dict:
            context.location = resolve_entity_description(context_dict['location'])

        return context

    def _configure(self, robot, config):
        self._robot = robot

        semantics = PickUp._parse_semantics(config.semantics)
        context = PickUp._parse_context(config.context)

        # Check if a previous action had us find the object already...
        object_found = False
        if context.object:  # we got some object from a previous action
            if semantics.object.type:  # we got some sort of type, could still be a reference...
                if semantics.object.type != 'reference':  # an object type was specified in the task
                    if semantics.object.type == context.object.type:  # then they need to be the same
                        object_found = True
                else:  # the object was a reference, then we don't care what the type of context object is
                    object_found = True
            elif semantics.object.category:  # no type, but a category was specified
                # Then the context object must be one of that category
                if self._knowledge.get_object_category(context.object.type) == semantics.object.category:
                    object_found = True
                if context.object.category == semantics.object.category:
                    object_found = True

        if not object_found:
            self._config_result.required_context = {
                'action': 'find',
                'object': config.semantics['object']
            }
            if semantics.source_location:
                self._config_result.required_context['source-location'] = config.semantics['source-location']
            return

        # Add the found object to the context that is passed to the next task
        self._config_result.context['object'] = config.context['object']

        # Next to the arm_properties of the Pick_up action this ArmDesignator also needs the properties of the Place and
        # Hand_over actions since these actions (can) rely on the Pick_up action for the context.
        arm_des = UnoccupiedArmDesignator(self._robot, {"required_trajectories": ["prepare_grasp", "prepare_place"],
                                                        "required_goals": ["carrying_pose", "handover_to_human"],
                                                        "required_gripper_types": [GripperTypes.GRASPING]}
                                          ).lockable()
        arm_des.lock()

        self._fsm = Grab(self._robot, item=context.object.designator, arm=arm_des)

        self._config_result.context['arm-designator'] = arm_des
        self._config_result.succeeded = True

    def _start(self):
        fsm_result = self._fsm.execute()

        if fsm_result == "done":
            self._execute_result.succeeded = True
            if not self._config_result.context['object']['designator'].resolve():
                self._execute_result.message += " I could not pick anything up. ". \
                    format(self._config_result.context)
            else:
                self._execute_result.message += " I picked up the {}. ". \
                    format(self._config_result.context['object']['designator'].resolve().type)
        else:
            if not self._config_result.context['object']['designator'].resolve():
                self._execute_result.message += " I could not pick anything up. ". \
                    format(self._config_result.context)
            else:
                self._execute_result.message += " I could not pick up the {}. ". \
                    format(self._config_result.context['object']['designator'].resolve().type)

    def _cancel(self):
        if self._fsm.is_running:
            self._fsm.request_preempt()


if __name__ == "__main__":
    rospy.init_node('pickup_test')

    from robot_skills import get_robot_from_argv

    robot = get_robot_from_argv(1)

    action = PickUp()

    config = ConfigurationData({'action': 'pick_up',
                                'entity': {'id': 'cabinet'},
                                'height': 0.8})

    action.configure(robot, config)
    action.start()
