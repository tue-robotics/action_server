import rospy
from action_factory import ActionFactory
from actions.action import ConfigurationResult, ConfigurationData

"""
The TaskManager sets up a state machine according to a task recipe and executes it.
"""


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

    def get_action_from_context(self, context):
        # TODO: The content of 'context' should be thought out better.
        action_name = context['action']
        return self._action_factory.get_action(action_name)()

    def recursive_configure(self, action, configuration_data):
        preconditions_met = False
        action_list = []  # List of configured actions
        configuration_result = ConfigurationResult()

        # Try configuring the action until all of its preconditions are met
        while not preconditions_met:
            configuration_result = action.configure(self._robot, configuration_data)

            # If context is required, we will need to configure an action prior to the current one
            if configuration_result.required_context:
                rospy.logwarn("More context required: {}".format(configuration_result.required_context))

                # We derive the required action from the required context
                # TODO: Using postconditions of actions, we could infer what kind of prior action is necessary
                prior_action = self.get_action_from_context(configuration_result.required_context)

                # Configure the prior action using the same recursive strategy
                prior_config_data = ConfigurationData(configuration_result.required_context)
                prior_action_list, prior_action_config_result = self.recursive_configure(prior_action,
                                                                                         prior_config_data)

                # Add all of the resulting actions from this configuration to the action list
                action_list += prior_action_list

                # If configuration did not succeed, we need to stop and report what went wrong there.
                if not prior_action_config_result.succeeded:
                    configuration_result = prior_action_config_result
                    break

                # Add the resulting context to the context of the current action, to get ahead in its configuration
                configuration_data.context.update(prior_action_config_result.context)

            # If configuration succeeded and no more context is required, we can append the current action to the list
            else:
                if configuration_result.succeeded:
                    action_list.append(action)
                    rospy.loginfo("Configuration succeeded!\n")
                preconditions_met = True

        # Return the list of configured actions and the current configuration result
        return action_list, configuration_result

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

            config_data = ConfigurationData(instruction, configuration_result.context)

            # Try to configure the action
            try:
                action_list, configuration_result = self.recursive_configure(action, config_data)
            except Exception as e:
                # if the action crashes, assume that the other actions in the sequence become invalid,
                # so clear the action sequence before re-raising the exception
                rospy.logerr('Action configuration crashed, requesting preempt of action sequence')
                self.request_preempt()
                raise

            # If action configuration succeeded, append configured action to action sequence
            if configuration_result.succeeded:
                self._action_sequence += action_list
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
        try:
            result = self._active_action.start()
        except Exception as e:
            # if the action crashes, assume that the other actions in the sequence become invalid,
            # so clear the action sequence before re-raising the exception
            rospy.logerr('Action execution crashed, requesting preempt of action sequence')
            self.request_preempt()
            raise

        if not self._action_sequence:
            self.done = True
        return result

    def request_preempt(self):
        self._action_sequence = []
        if self._active_action:
            try:
                self._active_action.cancel()
            except Exception as e:
                rospy.logerr("Cancelling action failed:\n{}".format(e))
        self.clear()
