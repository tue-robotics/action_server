from action import Action, ConfigurationData
from robot_skills.util.entity import Entity

from robot_smach_states.util.designators import VariableDesignator


class ExampleAction(Action):
    def __init__(self):
        """
        The ExampleAction class explains how Actions, as handled by the TaskManager, work.

        As by example, it takes passed context of a color passed by a previous Action, passes an Entity to the next
        Action and uses a dict mapping from object types to colors from the common knowledge. A practical use would be the
        following:

        Task: "Ask for a color, select an item of that color and grab it." (Completely made up task BTW)

        This Action would perform the middle part of the task: "select an item of that color". It takes the color from the
        Action asking for a color, and passes the selected item (Entity) to the Action for grabbing an entity. Typically,
        one would wrap higher level behavior in an Action than this, but this is just for illustration purposes.

        """
        """
        The base class constructor initializes the following fields:
         - self._config_result = ConfigurationResult()
         - self._execute_result = ActionResult()
        Their fields should be assigned meaningful values in the _configure and _start methods, respectively, by the
        implementation so the (typically the task manager) can give decent feedback to the user about the result of the
        action.

        Here, you can specify which robot skills are required to perform this action, you can specify what fields are
        required in the semantics of the task, and what information from previous tasks is required.
        """

        """
        You can set the required robot skills here. During the configure step, the given robot object is checked for
        their availability.
        """
        required_skills = ['ed']

        """
        Similarly, you can set the required keys in the context. Their availability is also checked in the configure
        step.
        """
        required_passed_knowledge = {'color': 'Which color do you mean?'}

        """
        Similarly, you can set the required keys in the semantics. Their availability is also checked in the configure
        step. However, a missing field means that the client may want to add this information by posing a follow-up
        question. Therefore, this is a dict mapping from the required fields to the prompts requesting the user for
        the specific missing information.

        If information is missing, the json path to the missing field is returned to the client, so the client knows
        which field to add.

        Actually, we don't require any fields in the configuration in this case, but this would be the way to do that.
        """
        # required_field_prompts = {'object': " What object would you like me to work with? "}
        required_field_prompts = {}
        super(ExampleAction, self).__init__(
            required_field_prompts=required_field_prompts,
            required_passed_knowledge=required_passed_knowledge,
            required_skills=required_skills,
        )

    def _configure(self, robot, config):
        """
        :param robot: The robot to use for this action
        :param config: The configuration of the action, containing fields semantics and context
        :return: ConfigurationResult

        The configure step is typically done just after the command is given. It is intended to check if the task makes
        any sense. For example, a task 'Take it to the bridge.', does not make sense as 'it' should refer to something.
        In another example, 'Find Ava', we may want to search differently than when the task is to find an object. All
        such checking can be done in the configure step.

        If a command is composed out of multiple actions (go to the kitchen and find the coffee), all actions in the
        command are first configured before the first is started. This is done to make sure the complete command makes
        sense, and the robot will not find out it doesn't make sense halfway through the execution.

        This does mean that any resulting information from earlier actions in the command is not available in the world
        model yet, in this stage.
        """

        """
        The common knowledge is available through self._knowledge. The value of self._config_result.succeeded is False
        by default, so when you simply return, the configuration is considered failed. The message is intended to be
        reported to the operator after the task.
        """
        if hasattr(self._knowledge, 'object_colors'):
            object_colors = self._knowledge.object_colors
        else:
            self._config_result.message = " Colors of objects are not in my knowledge. "
            return

        """
        Knowledge gained in previous Actions passed on to this Action can be accessed through config. The availability
        of this field is checked by the base class (because we specified 'color' as required passed context), but you
        should perform all the necessary checks on this data to make sure that it is what you expect it to be!
        """
        if not isinstance(config.knowledge['color'], VariableDesignator):
            self._config_result.message = " The ExampleAction state cannot handle a %s as a source of color. " % \
                                          type(config.knowledge['color'])
            return

        if not config.knowledge['color'].resolve_type() == str:
            self._config_result.message = " The color variable is not a string. "
            return

        self._color_designator = config.knowledge['color']

        """
        Information that should be passed on to a next action should be instantiated here. In this case we expect this
        state to result in some Entity object, so we create an empty variable designator with that resolve type. The
        designator should be 'filled' with data of an Entity during the execution of the _start method. This is
        typically done in the underlying state machine.
        """
        self._entity_designator = VariableDesignator(resolve_type=Entity)

        """
        Pass on the resulting context by putting it in the resulting context (this action results in some Entity).
        The following action can use this data by accessing its config.context['object']
        """
        self._config_result.resulting_knowledge['object'] = self._entity_designator

        """
        Typically, we instantiate a (smach) state machine here, which will do the actual work of the action. In this
        case, it will be some state machine filling an object designator based on the context passed to this Action.
        For more information on designators, how they work and why we need them, ask @LoyVanBeek.
        """
        self._action_fsm = DummyStateMachine(robot, self._color_designator, self._entity_designator, object_colors)

        """
        Explicitly set the config result to True if configuration succeeded.
        """
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
