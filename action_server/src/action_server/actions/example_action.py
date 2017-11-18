from action import Action, ConfigurationData
from robot_skills.util.entity import Entity


class ExampleAction(Action):
    """
    The ExampleAction class explains how Actions, as handled by the TaskManager, work.
    """
    def __init__(self):
        # Call the base class constructor
        Action.__init__(self)
        """
        The base class constructor initializes the following fields:
         - self._config_result = ConfigurationResult()
         - self._execute_result = ActionResult()
        Their fields should be assigned meaningful values for the caller to give decent feedback to the user.

        Here, you can specify which robot skills are required to perform this action, you can specify what fields are
        required in the semantics of the task, and what information from previous tasks is required.
        """

        """
        You can set the required robot skills here. During the configure step, the given robot object is checked for
        their availability.
        """
        self._required_skills = ['head', 'hmi', 'base']

        """
        Similarly, you can set the required keys in the knowledge. Their availability is also checked in the configure
        step.
        """
        self._required_passed_knowledge = []

        """
        Similarly, you can set the required keys in the semantics. Their availability is also checked in the configure
        step. However, a missing field means that the client may want to add this information by posing a follow-up
        question. Therefore, this is a dict mapping from the required fields to the prompts requesting the user for
        the specific missing information.

        If information is missing, the json path to the missing field is returned to the client, so the client knows
        which field to add.
        """
        self._required_field_prompts = {'object': " What object would you like me to manipulate? "}

        """
        Objects that should be passed on to a next action should be instantiated here.
        """
        self._object = Entity()

    def _configure(self, robot, config):
        """
        :param robot: The robot to use for this action
        :param config: The configuration of the action, containing fields semantics and knowledge
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
        The common knowledge is available through self._knowledge
        """
        if config.semantics['object']['type'] in self._knowledge.object_categories:
            category = config.semantics['object']['type']
        else:
            self._config_result.message = " The specified object type is not an object I know. "
            return

        """
        Pass the resulting knowledge (apparently this action results in some Entity)
        """
        self._config_result.resulting_knowledge['object'] = self._object
        self._config_result.succeeded = True

        """
        Typically, we instantiate a (smach) state machine here, which will do the actual work of the action. In this
        case, it will be some state machine filling an object designator. After executing this state machine, we can
        assign our self._object the value this designator resolves to. For more information on designators, how they
        work and why we need them in our Robot Smach States, ask @LoyVanBeek.

        self._object_designator = SomeDesignator()
        self._action_fsm = SomeStateMachine(robot, self._object_designator)
        """
        return

    def _start(self):
        """
        Here, the action is actually performed.
        """

        """
        # We typically run the state machine here:
        result = self._action_fsm.execute()
        """

        """
        After this, we may want to check the output of the state machine execution
        if result == 'failed':
            self._execute_result.message = " I failed to perform the example action. "
            self._execute_result.succeeded = False
        """

        """
        # Next, we can fill in the knowledge that is passed on to the next action:
        self._object = self._object_designator.resolve()
        """

        """
        Finally, we can return to the task manager that the action was performed successfully
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
