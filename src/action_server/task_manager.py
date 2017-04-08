#! /usr/bin/python

import rospy
from action_factory import ActionFactory

class TaskManager(object):
    def __init__(self, robot):
        self._task_string = None
        self._robot = robot
        self._action_factory = ActionFactory()
        self._action_sequence = []

    def configure(self, recipe):
        return self._set_up_state_machine(recipe)

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


