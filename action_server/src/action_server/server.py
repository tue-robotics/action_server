import rospy, yaml

import actionlib
import action_server.msg
from action_server.srv import GetActions, GetActionsResponse
from task_manager import TaskManager


class Server(object):
    '''
    The Server wraps the TaskManager to expose a ROS actionlib interface.
    '''
    def __init__(self, robot):
        self._robot = robot
        self._task_manager = TaskManager(self._robot)

        # Set up actionlib interface for clients to give a task to the robot.
        self._action_name = "/" + self._robot.robot_name + "/action_server/task"
        self._action_server = actionlib.SimpleActionServer(self._action_name, action_server.msg.TaskAction,
                                                           execute_cb=self._add_action_cb, auto_start=False)
        self._result = action_server.msg.TaskResult()
        self._action_server.start()

        self._get_actions_srv = rospy.Service("get_actions", GetActions, self._get_actions_cb)

        rospy.logdebug("Started action server with action name {}".format(self._action_name))

    def _get_actions_cb(self, req):
        res = GetActionsResponse()
        res.actions = self._task_manager.get_actions()
        return res

    def _add_action_cb(self, goal):
        recipe = yaml.load(goal.recipe)
        rospy.logdebug("Received action recipe: {}".format(goal.recipe))
        configuration_result = self._task_manager.set_up_state_machine(recipe['actions'])
        rospy.logdebug("Result of state machine setup: {}".format(configuration_result))

        self._result.log_messages = []

        if configuration_result.succeeded:
            rospy.logdebug("Setting up state machine succeeded")
        else:
            if configuration_result.missing_field:
                self._result.result = action_server.msg.TaskResult.RESULT_MISSING_INFORMATION
                self._result.missing_field = "actions" + configuration_result.missing_field
                self._result.log_messages.append(" I don't have enough information to perform that task.")
                self._result.log_messages.append(configuration_result.message)
            elif configuration_result.message:
                # TODO: this task result should not be RESULT_UNKNOWN
                self._result.result = action_server.msg.TaskResult.RESULT_UNKNOWN
                self._result.log_messages.append(configuration_result.message)
            else:
                self._result.result = action_server.msg.TaskResult.RESULT_UNKNOWN
                self._result.log_messages.append(" It seems that I am unable to perform that task. "
                                                   "Not sure why though.")
            self._action_server.set_aborted(self._result)
            self._task_manager.clear()
            rospy.logerr("Setting up state machine failed")
            return

        while not self._task_manager.done:
            action_result = self._task_manager.execute_next_action()
            rospy.logdebug("Result of action execution: {}".format(action_result))
            self._result.log_messages.append(action_result.message)
            if not action_result.succeeded:
                self._result.result = action_server.msg.TaskResult.RESULT_TASK_EXECUTION_FAILED
                self._action_server.set_aborted(self._result)
                self._task_manager.clear()
                rospy.logdebug("Execution of state machine aborted because action failed.")
                return

        rospy.logdebug("Execution of state machine succeeded.")
        self._result.result = action_server.msg.TaskResult.RESULT_SUCCEEDED
        self._action_server.set_succeeded(self._result)
