import rospy, yaml

import actionlib
import action_server.msg
from task_manager import TaskManager

'''
The Server wraps the TaskManager to expose a ROS actionlib interface.
'''

class Server(object):

    def __init__(self, robot):
        self._robot = robot
        self._task_manager = TaskManager(self._robot)

        # Set up actionlib interface for clients to give a task to the robot.
        self._action_name = "/" + self._robot.robot_name + "/action_server/task"
        self._action_server = actionlib.SimpleActionServer(self._action_name, action_server.msg.TaskAction,
                                                           execute_cb=self._add_action_cb, auto_start=False)
        self._feedback = action_server.msg.TaskFeedback()
        self._result = action_server.msg.TaskResult()
        self._action_server.start()
        rospy.logdebug("Started action server with action name {}".format(self._action_name))

    def _add_action_cb(self, goal):
        recipe = yaml.load(goal.recipe)
        configuration_result = self._task_manager.set_up_state_machine(recipe['actions'])

        self._feedback.log_messages = []

        if configuration_result.succeeded:
            rospy.logdebug("Setting up state machine succeeded")
        else:
            if configuration_result.missing_field:
                self._result.result = action_server.msg.TaskResult.RESULT_MISSING_INFORMATION
                self._feedback.log_messages.append(" I didn't have enough information to perform that task.")
            elif configuration_result.message:
                # TODO: this task result should not be RESULT_UNKNOWN
                self._result.result = action_server.msg.TaskResult.RESULT_UNKNOWN
                self._feedback.log_messages.append(configuration_result.message)
            else:
                self._result.result = action_server.msg.TaskResult.RESULT_UNKNOWN
                self._feedback.log_messages.append(" It seems that I am unable to perform that task. Not sure why though.")
            self._action_server.publish_feedback(self._feedback)
            self._action_server.set_aborted(self._result)
            rospy.logerr("Setting up state machine failed")
            return

        while not self._task_manager.done:
            action_result = self._task_manager.execute_next_action()
            self._feedback.log_messages.append(action_result.message)
            self._action_server.publish_feedback(self._feedback)
            # if not action_result.succeeded:
            #     # TODO: If an action failed, this does not mean the following actions are useless to do
            #     self._result.result = action_server.msg.TaskResult.RESULT_TASK_EXECUTION_FAILED
            #     self._action_server.set_aborted(self._result)
            #     rospy.logdebug("Execution of state machine aborted because action failed.")
            #     return

        rospy.logdebug("Execution of state machine succeeded.")
        self._result.result = action_server.msg.TaskResult.RESULT_SUCCEEDED
        self._action_server.set_succeeded(self._result)
