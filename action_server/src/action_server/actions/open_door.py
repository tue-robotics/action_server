from action import Action, ConfigurationData
from challenge_final import


class OpenDoor(Action):
    """
    The Open Door action wraps the state machine to open the door of the cupboard at RWC 2018

    """
    def __init__(self):
        # Call the base class constructor
        Action.__init__(self)
        # self._required_field_prompts = {'object': " What object would you like me to work with? "}

    def _configure(self, robot, config):
        self._state_machine = states.OpenDoor(robot)
        self._config_result.succeeded = True
        return

    def _start(self):
        """
        Here, the action is actually performed.
        """

        """
        We typically run the state machine here. Under the hood, this state machine fills the entity designator. Because
        this is the same object as the one in our _config_result.resulting_knowledge['object'], the context of this
        entity is passed to any following states that used this Designator.
        """
        result = self._action_fsm.execute()

        """
        After this, we may want to check the output of the state machine execution
        """
        color = self._color_designator.resolve()
        if result == 'not_available':
            self._execute_result.message = " I haven't seen any %s object yet. " % color
            self._execute_result.succeeded = False
            return

        if result == 'error':
            self._execute_result.message = " Something went wrong trying to determine which objects are %s " % color
            self._execute_result.succeeded = False
            return

        """
        Finally, we can return to the task manager that the action was performed successfully. Explicitly set succeeded
        to True here, to let the TaskManager know that the Action succeeded. The message is reported to the operator
        after the complete Task (series of Actions) is performed.
        """
        self._execute_result.message = " I successfully performed the example action. "
        self._execute_result.succeeded = True
        return

    def _cancel(self):
        """
        The _cancel method is called when the Action Server's goal is canceled and this action is still running
        """

        """
        # If the state machine is still running, we need to send it a preempt request
        if self._action_fsm.is_running():
            self._action_fsm.request_preempt()
        """

"""
The below code should not be in the Action implementation file. An action should always wrap an existing smach State
from the Robot Smach States. This is just here to be able to demonstrate the use in the above Action implementation.
"""
import smach


class DummyStateMachine(smach.State):
    def __init__(self, robot, color_designator, entity_designator, object_colors):
        """
        Select an entity with the given color.
        :param robot: robot to execute state with
        :param color_designator: A color that the resulting entity should have.
        :param entity_designator: The resulting entity with the specified color
        :param object_colors: Dict mapping from object type to color
        """
        smach.State.__init__(self, outcomes=['succeeded', 'not_available', 'error'])

        # Assign member variables
        self.robot = robot
        self._color_designator = color_designator
        self._entity_designator = entity_designator
        self._object_colors = object_colors

    def execute(self, userdata=None):
        try:
            color = self._color_designator.resolve()
            entities = self.robot.ed.get_entities()
            for e in entities:
                if self._object_colors[e.type] == color:
                    self._entity_designator.write(e)
                    return 'succeeded'
            return 'not_available'
        except:
            return 'error'
