import rospy

import action_server.srv
import actionlib
import action_server.msg

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
        self.succeeded = (self.result == self.RESULT_SUCCEEDED)

'''
The Client is a facade to the ros interface of a client of the main.py.
'''
class Client(object):
    def __init__(self, robot_name):
        action_name = "/" + robot_name + "/action_server/task"
        self._action_client = actionlib.SimpleActionClient(action_name,
                                                           action_server.msg.TaskAction)
        rospy.logdebug("Waiting for task action server...")
        self._action_client.wait_for_server()
        rospy.logdebug("Connected to task action server")

        self._feedback = []

    def _handle_feedback(self, feedback):
        rospy.logerr("Logging feedback {}".format(feedback.log_messages))
        for message in feedback.log_messages:
            self._feedback.append(message)
        rospy.logerr("Stored feedback: {}".format(self._feedback))
        exit(0)

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

        goal = action_server.msg.TaskGoal(recipe=recipe)
        self._action_client.send_goal(goal, feedback_cb=self._handle_feedback)
        self._action_client.wait_for_result()
        result = self._action_client.get_result()

        if result.result == action_server.msg.TaskResult.RESULT_MISSING_INFORMATION:
            return TaskOutcome(TaskOutcome.RESULT_MISSING_INFORMATION,
                               self._feedback)

        elif result.result == action_server.msg.TaskResult.RESULT_TASK_EXECUTION_FAILED:
            return TaskOutcome(TaskOutcome.RESULT_TASK_EXECUTION_FAILED,
                               self._feedback)

        elif result.result == action_server.msg.TaskResult.RESULT_UNKNOWN:
            return TaskOutcome(TaskOutcome.RESULT_UNKNOWN,
                               self._feedback)

        elif result.result == action_server.msg.TaskResult.RESULT_SUCCEEDED:
            return TaskOutcome(TaskOutcome.RESULT_SUCCEEDED,
                               self._feedback)

        return TaskOutcome(messages=self._feedback)
