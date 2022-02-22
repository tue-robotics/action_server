import action_server_msgs.msg
import action_server_msgs.srv
import actionlib
import rospy


class TaskOutcome(object):
    RESULT_MISSING_INFORMATION = 0
    RESULT_TASK_EXECUTION_FAILED = 1
    RESULT_UNKNOWN = 2
    RESULT_SUCCEEDED = 3

    def __init__(self, result=RESULT_UNKNOWN, messages=None, missing_field=""):
        if not messages:
            messages = []
        self.result = result
        self.missing_field = missing_field
        self.messages = messages

    @property
    def succeeded(self):
        return self.result == self.RESULT_SUCCEEDED

    @succeeded.setter
    def succeeded(self, value):
        if value:
            self.result = self.RESULT_SUCCEEDED

    def __repr__(self):
        return "TaskOutcome(result={}, messages={}, missing_field='{}')".format(self.result,
                                                                                self.messages,
                                                                                self.missing_field)


def task_outcome_from_result(result):
    """
    Converts action_server_msgs.msg.TaskResult to TaskOutcome class

    :param result: (action_server_msgs.msg.TaskResult) result input
    :return: (TaskOutcome) result output
    """
    # Check result to return the correct outcome
    if result.result == action_server_msgs.msg.TaskResult.RESULT_MISSING_INFORMATION:

        to = TaskOutcome(TaskOutcome.RESULT_MISSING_INFORMATION,
                         result.log_messages)
        to.missing_field = result.missing_field
        return to

    elif result.result == action_server_msgs.msg.TaskResult.RESULT_TASK_EXECUTION_FAILED:
        return TaskOutcome(TaskOutcome.RESULT_TASK_EXECUTION_FAILED,
                           result.log_messages)

    elif result.result == action_server_msgs.msg.TaskResult.RESULT_UNKNOWN:
        return TaskOutcome(TaskOutcome.RESULT_UNKNOWN,
                           result.log_messages)

    elif result.result == action_server_msgs.msg.TaskResult.RESULT_SUCCEEDED:
        return TaskOutcome(TaskOutcome.RESULT_SUCCEEDED,
                           result.log_messages)

    return TaskOutcome(messages=result.log_messages)


class Client(object):
    """
    A client for the action server

    Wraps the client side of the actionlib interface so that it can be easily used in client side applications.

    Example:
        client = Client('amigo')
        semantics = "{'actions': [{'action': 'say', 'sentence': 'ROBOT_NAME'}]}"
        client.send_task(semantics)
    """
    def __init__(self, robot_name):
        action_name = "/" + robot_name + "/action_server/task"
        self._action_client = actionlib.SimpleActionClient(action_name,
                                                           action_server_msgs.msg.TaskAction)
        rospy.loginfo("Waiting for task action server to come online...")
        self._action_client.wait_for_server()
        rospy.loginfo("Connected to task action server")

        self.get_actions_proxy = rospy.ServiceProxy('get_actions', action_server_msgs.srv.GetActions)

    def get_actions(self):
        """
        Query the available actions from the action server.

        :return: List of action names as registered with the action server, or an empty list if the service call fails.
        """
        try:
            res = self.get_actions_proxy()
        except rospy.ServiceException:
            rospy.logerr("Failed to get actions from the action server.")
            res = []

        return res.actions

    def send_async_task(self, semantics, done_cb=None, feedback_cb=None):
        """
        Send a task to the action server and return immediately. A task is composed of one or multiple actions.

        :param semantics: A json string with a list of dicts, every dict in the list has at least an 'action' field,
            and depending on the type of action, several parameter fields may be required.
        :param done_cb:	(callable) Callback that gets called on transitions to Done. The callback should take one
            parameter: TaskOutCome
        :param feedback_cb: (callable)Callback that gets called whenever feedback for this goal is received. Takes one
            parameter: the feedback.
        """
        # Define the wrapped done callback
        def _wrapped_done_cb(_, result):
            taskoutcome = task_outcome_from_result(result=result)
            return done_cb(taskoutcome)

        # The wrapped done callback is only used if the provided done callback is callable. Otherwise it's useless
        _done_cb = _wrapped_done_cb if callable(done_cb) else None

        # Create and send the goal
        goal = action_server_msgs.msg.TaskGoal(recipe=semantics)
        self._action_client.send_goal(goal=goal, done_cb=_done_cb, feedback_cb=feedback_cb)

    def send_task(self, semantics) -> TaskOutcome:
        """
        Send a task to the action server. A task is composed of one or multiple actions.

        :param semantics: A json string with a list of dicts, every dict in the list has at least an 'action' field,
            and depending on the type of action, several parameter fields may be required.
        :return: result output, which provides information about success of the task execution and some
            information messages
        """
        goal = action_server_msgs.msg.TaskGoal(recipe=semantics)
        self._action_client.send_goal(goal)

        try:
            self._action_client.wait_for_result()
            result = self._action_client.get_result()

        # if user presses ctrl+C, stop waiting and cancel all goals
        except KeyboardInterrupt:
            self.cancel_all()
            raise KeyboardInterrupt

        if not isinstance(result, action_server_msgs.msg.TaskResult):
            msg = "Result not instance of 'action_server_msgs.msg.TaskResult', but {}".format(type(result))
            if result is None:
                rospy.logerr(msg)
            return TaskOutcome(messages=[msg])

        return task_outcome_from_result(result=result)

    def cancel_all(self):
        """
        Cancels all goals of the action client
        """
        rospy.logdebug("cancelling all goals...")
        self._action_client.cancel_all_goals()
        self._action_client.wait_for_result()
        rospy.logdebug("... all goals cancelled!")

    def cancel_all_async(self):
        """
        Cancels all goals of the action client and returns directly without waiting for the result
        """
        rospy.logdebug("cancelling all goals async...")
        self._action_client.cancel_all_goals()
