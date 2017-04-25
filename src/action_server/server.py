import rospy, yaml

import action_server.srv
import actionlib
import action_server.msg
from task_manager import TaskManager

'''
The Server wraps the TaskManager to expose a ROS actionlib interface.
'''

class Server(object):

    def __init__(self, name, robot):
        self._name = name
        self._robot = robot
        self._task_manager = TaskManager(self._robot)

        # Set up actionlib interface for clients to give a task to the robot.
        self._action_name = robot.robot_name + "/task"
        self._action_server = actionlib.SimpleActionServer(self._action_name, action_server.msg.TaskAction,
                                                           execute_cb=self._add_action_cb, auto_start=False)
        self._feedback = action_server.msg.TaskFeedback()
        self._result = action_server.msg.TaskResult()
        self._action_server.start()

    def _add_action_cb(self, goal):
        recipe = yaml.load(goal.recipe)

        configuration_result = self._task_manager.set_up_state_machine(recipe)

        if configuration_result.succeeded:
            rospy.loginfo("Setting up state machine succeeded")
        else:
            self._result.result = action_server.msg.TaskResult.RESULT_MISSING_INFORMATION
            self._action_server.set_aborted(self._result)
            rospy.loginfo("Setting up state machine failed")
            return

        self._feedback.log_messages = []

        while not self._task_manager.done:
            action_result = self._task_manager.execute_next_action()
            self._feedback.log_messages.append(action_result.message)
            if not action_result.succeeded:
                # TODO: If an action failed, this does not mean the following actions are useless to do
                self._result.result = action_server.msg.TaskResult.RESULT_TASK_EXECUTION_FAILED
                self._action_server.set_aborted(self._result)
                rospy.logwarn("Execution of state machine aborted because action failed.")
                return

        rospy.loginfo("Execution of state machine succeeded.")
        self._result.result = action_server.msg.TaskResult.RESULT_SUCCEEDED
        self._action_server.set_succeeded(self._result)
