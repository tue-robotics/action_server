from action_factory import ActionFactory
from actions.action import ConfigurationResult, ConfigurationData

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
        self._active_action = None

    def get_actions(self):
        return self._action_factory.get_action_names()

    def clear(self):
        self._task_string = None
        self._action_sequence = []
        self.done = True
        self._active_action = None

    def set_up_state_machine(self, recipe):
        configuration_result = ConfigurationResult()

        i = 0

        for instruction in recipe:
            try:
                action_name = instruction['action']
            except KeyError:
                return configuration_result

            # Set up the action
            Action = self._action_factory.get_action(action_name)
            action = Action()

            config_data = ConfigurationData(instruction, configuration_result.resulting_knowledge)

            # Try to configure the action
            configuration_result = action.configure(self._robot, config_data)

            # If action configuration succeeded, append configured action to action sequence
            if configuration_result.succeeded:
                self._action_sequence.append(action)
            # If action configuration failed, return the configuration result, specifying what went wrong
            elif configuration_result.missing_field:
                configuration_result.missing_field = "[{}].".format(i) + configuration_result.missing_field
                return configuration_result
            else:
                return configuration_result
            i += 1

        self.done = False

        return configuration_result

    def get_next_action_name(self):
        if self._action_sequence:
            return self._action_sequence[0].get_name()
        else:
            return None

    def execute_next_action(self):
        self._active_action = self._action_sequence.pop(0)
        result = self._active_action.start()
        if not self._action_sequence:
            self.done = True
        return result

    def request_preempt(self):
        self._action_sequence = []
        if self._active_action:
            self._active_action.cancel()
        self.clear()
