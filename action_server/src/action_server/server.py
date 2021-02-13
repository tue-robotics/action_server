import rospy, yaml

import actionlib
import action_server_msgs.msg
from action_server_msgs.srv import GetActions, GetActionsResponse
from task_manager import TaskManager
from actions.action import ConfigurationResult


class Server(object):
    """
    The Server wraps the TaskManager to expose a ROS actionlib interface.
    """

    def __init__(self, robot):
        self._robot = robot
        self._task_manager = TaskManager(self._robot)

        # Set up actionlib interface for clients to give a task to the robot.
        self._action_name = "/" + self._robot.robot_name + "/action_server/task"
        self._action_server = actionlib.SimpleActionServer(self._action_name, action_server_msgs.msg.TaskAction,
                                                           execute_cb=self._add_action_cb, auto_start=False)

        self._action_server.register_preempt_callback(cb=self._cancel)
        self._result = action_server_msgs.msg.TaskResult()

        self._action_server.start()

        self._get_actions_srv = rospy.Service("get_actions", GetActions, self._get_actions_cb)

        rospy.logdebug("Started action server with action name {}".format(self._action_name))

    def _get_actions_cb(self, req):
        res = GetActionsResponse()
        res.actions = self._task_manager.get_actions()
        return res

    def _add_action_cb(self, goal):
        recipe = yaml.load(goal.recipe.lower())
        rospy.logdebug("Received action recipe: {}".format(goal.recipe))

        try:
            # check if we have a recipe that makes sense, otherwise set result to failed
            # note that python uses lazy evaluation of boolean conditions
            if isinstance(recipe, dict) and 'actions' in recipe and recipe['actions']:
                configuration_result = self._task_manager.set_up_state_machine(recipe['actions'])
            else:
                rospy.logerr("Recipe does not contain actions, setting configuration result to failed")
                configuration_result = ConfigurationResult()
                configuration_result.succeeded = False
                configuration_result.message = "I do not understand your command."
            rospy.logdebug("Result of state machine setup: {}".format(configuration_result))

            self._result.log_messages = []

            if configuration_result.succeeded:
                rospy.logdebug("Setting up state machine succeeded")
            else:
                if configuration_result.missing_field:
                    self._result.result = action_server_msgs.msg.TaskResult.RESULT_MISSING_INFORMATION
                    self._result.missing_field = "actions" + configuration_result.missing_field
                    self._result.log_messages.append(" I don't have enough information to perform that task.")
                    self._result.log_messages.append(configuration_result.message)
                elif configuration_result.message:
                    self._result.result = action_server_msgs.msg.TaskResult.RESULT_TASK_CONFIGURATION_FAILED
                    self._result.log_messages.append(configuration_result.message)
                else:
                    self._result.result = action_server_msgs.msg.TaskResult.RESULT_UNKNOWN
                    self._result.log_messages.append(" It seems that I am unable to perform that task. "
                                                       "Not sure why though.")
                self._action_server.set_aborted(self._result)
                self._task_manager.clear()
                rospy.logerr("Setting up state machine failed")
                return

            while not self._task_manager.done:
                # Pass feedback to client about what type of action is running
                feedback = action_server_msgs.msg.TaskFeedback()
                feedback.current_subtask = self._task_manager.get_next_action_name()
                self._action_server.publish_feedback(feedback)

                action_result = self._task_manager.execute_next_action()
                rospy.logdebug("Result of action execution: {}".format(action_result))
                self._result.log_messages.append(action_result.message)
                if not action_result.succeeded:
                    self._result.result = action_server_msgs.msg.TaskResult.RESULT_TASK_EXECUTION_FAILED
                    self._action_server.set_aborted(self._result)
                    self._task_manager.clear()
                    rospy.logdebug("Execution of state machine aborted because action failed.")
                    return
        except Exception as e:
            rospy.logerr("An error occurred using task recipe: %s\n" % goal.recipe)
            self._result.result = action_server_msgs.msg.TaskResult.RESULT_TASK_EXECUTION_FAILED
            self._result.log_messages = [' I failed to perform the task. ']
            self._action_server.set_aborted(self._result)
            raise

        rospy.logdebug("Execution of state machine succeeded.")
        self._result.result = action_server_msgs.msg.TaskResult.RESULT_SUCCEEDED
        self._action_server.set_succeeded(self._result)

    def _cancel(self):
        self._task_manager.request_preempt()
