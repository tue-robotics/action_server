import rospy, yaml, uuid

import action_server.srv
import actionlib
import action_server.msg
from task_manager import TaskManager

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

        configuration_result = self._task_manager.configure(recipe)

        if not configuration_result.succeeded:
            self._result.result = action_server.msg.TaskResult.RESULT_MISSING_INFORMATION
            self._action_server.set_aborted(self._result)

        while True:
            action_result = self._task_manager.run_next_action()
            if not action_result.succeeded:
                break
            # TODO: Send feedback here
