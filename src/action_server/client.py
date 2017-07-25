import rospy

import actionlib
import action_server.msg
import action_server.srv

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


class Client(object):
    ''' A client for the action server

    Wraps the client side of the actionlib interface so that it can be easily used in client side applications.
    '''
    def __init__(self, robot_name):
        action_name = "/" + robot_name + "/action_server/task"
        self._action_client = actionlib.SimpleActionClient(action_name,
                                                           action_server.msg.TaskAction)
        rospy.logdebug("Waiting for task action server...")
        self._action_client.wait_for_server()
        rospy.logdebug("Connected to task action server")

        self.get_actions_proxy = rospy.ServiceProxy('get_actions', action_server.srv.GetActions)

        self._feedback = []

    def _handle_feedback(self, feedback):
        for message in feedback.log_messages:
            self._feedback.append(message)


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


    def send_task(self, semantics):
        """
        Send a task to the action server.

        A task is composed of one or multiple actions.
        :param semantics: A json string with a list of dicts, every dict in the list has at least an 'action' field,
        and depending on the type of action, several parameter fields may be required.
        :return: True or false, and a message specifying the outcome of the task
        """
        self._feedback = []

        recipe = semantics

        result = None
        while not result:
            goal = action_server.msg.TaskGoal(recipe=recipe)
            self._action_client.send_goal(goal, feedback_cb=self._handle_feedback)
            self._action_client.wait_for_result()
            result = self._action_client.get_result()

        if result.result == action_server.msg.TaskResult.RESULT_MISSING_INFORMATION:
            to = TaskOutcome(TaskOutcome.RESULT_MISSING_INFORMATION,
                             result.log_messages)
            to.missing_field = result.missing_field
            return to

        elif result.result == action_server.msg.TaskResult.RESULT_TASK_EXECUTION_FAILED:
            return TaskOutcome(TaskOutcome.RESULT_TASK_EXECUTION_FAILED,
                               result.log_messages)

        elif result.result == action_server.msg.TaskResult.RESULT_UNKNOWN:
            return TaskOutcome(TaskOutcome.RESULT_UNKNOWN,
                               result.log_messages)

        elif result.result == action_server.msg.TaskResult.RESULT_SUCCEEDED:
            return TaskOutcome(TaskOutcome.RESULT_SUCCEEDED,
                               result.log_messages)

        return TaskOutcome(messages=result.log_messages)
