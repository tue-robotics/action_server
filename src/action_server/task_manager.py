#! /usr/bin/python

import rospy
import actionlib
import action_server.msg
from action_factory import ActionFactory

class TaskManager(object):
    def __init__(self, robot):
        self._task_string = None
        self._robot = robot
        self._action_factory = ActionFactory()
        self._action_sequence = []

        # Set up actionlib interface for clients to give a task to the robot.
        self._action_name = robot.robot_name + "/task/"
        self._action_server = actionlib.SimpleActionServer(self._action_name, action_server.msg.TaskAction, execute_cb=self._execute_cb, auto_start=False)
        self._feedback = action_server.msg.TaskFeedback()
        self._result = action_server.msg.TaskResult()
        self._action_server.start()

    def _execute_cb(self, goal):
        configuration_result = self._set_up_state_machine(goal.recipe)
        if configuration_result and configuration_result.succeeded:
            self._execute_state_machine()
            self._action_server.set_succeeded()
        else:
            self._action_server.set_aborted(configuration_result)

    def _set_up_state_machine(self, recipe):
        configuration_result = None
        for action_name, config in recipe.iteritems():
            # Set up the action
            Action = self._action_factory.get_action(action_name)
            action = Action()

            # Try to configure the action
            configuration_result = action.configure(config)

            # If action configuration succeeded, append configured action to action sequence
            if configuration_result.succeeded:
                self._action_sequence.append(action)
            # If action configuration failed, return the configuration result, specifying what went wrong
            else:
                return configuration_result
        return configuration_result

    def _execute_state_machine(self):
        for action in self._action_sequence:
            if not action.execute():
                return False
        return True


if __name__ == "__main__":
    rospy.init_node('task_manager')

    # Create Robot object based on argv[1]
    if len(sys.argv) < 2:
        print "Please specify a robot name 'amigo/sergio'"
        sys.exit()

    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        print "unknown robot"
        sys.exit()

    robot = Robot()

    task_manager = TaskManager(robot)

    rospy.spin()


