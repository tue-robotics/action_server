import rospy
from voluptuous import Schema, Required
from robot_skills.robot import Robot
from robocup_knowledge import load_knowledge


class ConfigurationData(object):
    """
    The ConfigurationData class defines the input data structure for configuration of an action.
    """
    def __init__(self, semantics, context=None):
        """
        :param semantics: Dictionary of the following structure: {'action': <action-name>, 'param1': <param-value>}.
        :param context: Dictionary of parameter names to context provided by previous actions.
        """
        self.semantics = semantics
        if context:
            self.context = context
        else:
            self.context = {}

    def __repr__(self):
        return "ConfigurationData(semantics={}, context={})".format(self.semantics, self.context)


class ConfigurationResult(object):
    """
    The ConfigurationResult class defines the data structure that is returned by the configure() methods of actions
    """
    def __init__(self, succeeded=False, context=None):
        if context is None:
            context = {}

        self.succeeded = succeeded
        self.context = context
        self.required_context = None
        self.missing_field = ""
        self.missing_skill = None
        self.message = ""

    def __repr__(self):
        return "ConfigurationResult(succeeded={}, context={}, required_context={}, missing_field={}, " \
               "missing_skill={}, message={})".format(self.succeeded, self.context, self.required_context,
                                                  self.missing_field, self.missing_skill, self.message)


# Check Issue 23 for Alberts grammar checking: https://github.com/tue-robotics/action_server/issues/23
class ActionResult(object):
    """
    The ActionResult class defines the data structure that is returned by the run() methods of actions.
    """
    def __init__(self, succeeded=False, message=""):
        self.succeeded = succeeded
        self.message = message

    def __repr__(self):
        return "ActionResult(succeeded=%s, message=%s)" % (self.succeeded, self.message)


class Action(object):
    def __init__(self, required_field_prompts, required_passed_knowledge, required_skills):
        # type: (dict, dict, list) -> None
        """
        The Action class defines the interface of actions that can be configured and started by the task_manager.

        :param required_field_prompts: maps required fields in the semantics to corresponding error messages that will
        be fed back to the user. E.g., in case of the 'Say' action, this might be {"sentence": "What would you like me
        to say?"}. N.B.: no defaults are defined to force inheriting classes to specify this.
        :param required_passed_knowledge:
        :param required_skills:
        """
        # Action dependent requirements
        self._required_field_prompts = required_field_prompts
        self._required_passed_knowledge = required_passed_knowledge
        self._required_skills = required_skills

        # Static knowledge
        self._knowledge = load_knowledge('common')

        # State
        self._config_result = ConfigurationResult()
        self._execute_result = ActionResult()

    def _check_parameters(self, config):

        schema_params = {Required(k): str for k in self._required_field_prompts.iterkeys()}
        schema_params["action"] = str
        schema = Schema(schema_params)
        schema(config.semantics)

        for k, v in self._required_field_prompts.items():
            if k not in config.semantics:
                rospy.logerr("Missing required parameter {}".format(k))
                self._config_result.missing_field = k
                self._config_result.message = v
                return False

        for k, v in self._required_passed_knowledge.items():
            if k not in config.context:
                rospy.logerr("Missing required context {}".format(k))
                self._config_result.missing_field = k
                self._config_result.message = v
                return False

        # Check for superfluous parameters
        for k, v in config.semantics.iteritems():
            if k == "action":
                continue
            if k not in self._required_field_prompts:
                print("Parameter {}: {} superfluous, this is probably wrong".format(
                    k, v
                ))
                return False

        return True

    def _check_skills(self, robot):
        for skill in self._required_skills:
            if not hasattr(robot, skill):
                rospy.logerr("Robot {} does not have attribute '{}'".format(robot.robot_name, skill))
                self._config_result.missing_skill = skill
                self._config_result.message = " I am missing the required skill {}. ".format(skill)
                return False
        return True

    def configure(self, robot, config):
        """
        Configure the action with a robot and configuration data
        :param robot: The robot to use for this action
        :type robot: Robot
        :param config: The configuration data. Contains semantics from input and implied context from previous tasks.
        :type config: ConfigurationData
        :return: The result of configuration
        :rtype: ConfigurationResult
        """
        self._config_result = ConfigurationResult()

        if not isinstance(config, ConfigurationData):
            rospy.logerr("Action: the specified config should be ConfigurationData! I received: %s" % str(config))
            self._config_result.message = " Something's wrong with my wiring. I'm so sorry, but I cannot do this. "
            return self._config_result

        rospy.loginfo("Configuring action {} with semantics {} and context {}.".
                      format(self.get_name(), config.semantics, config.context))

        if not isinstance(robot, Robot):
            rospy.logerr("Action: the specified robot should be a Robot! I received: %s" % str(robot))
            self._config_result.message = " I don't know what to say. I'm having an identity crisis. I'm so sorry. "
            return self._config_result

        if self._check_parameters(config) and self._check_skills(robot):
            self._configure(robot, config)

        rospy.loginfo("Resulting context = {}".format(self._config_result.context))

        return self._config_result

    def _configure(self, robot, config):
        raise NotImplementedError("Implement me in {}.".format(self.get_name()))

    def start(self):
        """
        Runs the execution of the action. Blocks until the action is finished or canceled.
        :return: ActionResult
        """
        rospy.loginfo("Starting executing of action {}.".format(self.get_name()))
        self._start()
        return self._execute_result

    def _start(self):
        raise NotImplementedError("Implement me in {}.".format(self.get_name()))

    def cancel(self):
        """
        Cancels the execution of the action.
        """
        rospy.loginfo("Canceling executing of action {}.".format(self.get_name()))
        self._cancel()

    def _cancel(self):
        raise NotImplementedError("Implement me in {}.".format(self.get_name()))

    def get_name(self):
        return self.__class__.__name__
