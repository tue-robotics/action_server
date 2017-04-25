#! /usr/bin/python

import rospy
from action_factory import ActionFactory

'''
The TaskManager sets up a state machine according to a task recipe and executes it.
'''

class TaskManager(object):
    def __init__(self, robot):
        self._task_string = None
        self._robot = robot
        self._action_factory = ActionFactory()
        self._action_sequence = []
        self.done = True

    def set_up_state_machine(self, recipe):
        configuration_result = None
        for instruction in recipe:
            action_name = instruction['action']

            # Set up the action
            Action = self._action_factory.get_action(action_name)
            action = Action()

            # Try to configure the action
            configuration_result = action.configure(self._robot, instruction)

            # If action configuration succeeded, append configured action to action sequence
            if configuration_result.succeeded:
                self._action_sequence.append(action)
            # If action configuration failed, return the configuration result, specifying what went wrong
            else:
                return configuration_result

        self.done = False

        return configuration_result

    def execute_next_action(self):
        result = self._action_sequence.pop(0).start()
        if not self._action_sequence:
            self.done = True
        return result
