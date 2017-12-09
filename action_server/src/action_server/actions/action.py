import rospy

from robot_skills.robot import Robot
from robocup_knowledge import load_knowledge


class ConfigurationData(object):
    """
    The ConfigurationData class defines the input data structure for configuration of an action.
    """
    def __init__(self, semantics, knowledge=None):
        """
        :param semantics: Dictionary of the following structure: {'action': <action-name>, 'param1': <param-value>}.
        :param knowledge: Dictionary of parameter names to knowledge provided by previous actions.
        """
        self.semantics = semantics
        if knowledge:
            self.knowledge = knowledge
        else:
            self.knowledge = {}

    def __repr__(self):
        return "ConfigurationData(semantics=%s, knowledge=%s)" % (self.semantics, self.knowledge)


class ConfigurationResult(object):
    """
    The ConfigurationResult class defines the data structure that is returned by the configure() methods of actions
    """
    def __init__(self, succeeded=False, resulting_knowledge=None):
        if resulting_knowledge is None:
            resulting_knowledge = {}

        self.succeeded = succeeded
        self.resulting_knowledge = resulting_knowledge
        self.missing_field = ""
        self.missing_skill = None
        self.message = ""


class ActionResult(object):
    """
    The ActionResult class defines the data structure that is returned by the run() methods of actions.
    """
    def __init__(self, succeeded=False, message=""):
        self.succeeded = succeeded
        self.message = message


class Action(object):
    """
    The Action class defines the interface of actions that can be configured and started by the task_manager.
    """
    def __init__(self):
        self._config_result = ConfigurationResult()
        self._execute_result = ActionResult()
        self._required_field_prompts = {}
        self._required_passed_knowledge = {}
        self._required_skills = []
        self._knowledge = load_knowledge('common')

    def _check_parameters(self, config):
        for k, v in self._required_field_prompts.items():
            if k not in config.semantics:
                rospy.logerr("Missing required parameter {}".format(k))
                self._config_result.missing_field = k
                self._config_result.message = v
                return False

        for k, v in self._required_passed_knowledge.items():
            if k not in config.knowledge:
                rospy.logerr("Missing required knowledge {}".format(k))
                self._config_result.missing_field = k
                self._config_result.message = v
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
        :param config: The configuration data. Contains semantics from input and implied knowledge from previous tasks.
        :type config: ConfigurationData
        :return: The result of configuration
        :rtype: ConfigurationResult
        """
        rospy.loginfo("Configuring action {} with semantics {} and knowledge {}.".
                      format(self.get_name(), config.semantics, config.knowledge))
        if not isinstance(config, ConfigurationData):
            rospy.logerr("Action: the specified config should be ConfigurationData! I received: %s" % str(config))
            self._config_result.message = " Something's wrong with my wiring. I'm so sorry, but I cannot do this. "
            return self._config_result

        if not isinstance(robot, Robot):
            rospy.logerr("Action: the specified robot should be a Robot! I received: %s" % str(robot))
            self._config_result.message = " I don't know what to say. I'm having an identity crisis. I'm so sorry. "
            return self._config_result

        if self._check_parameters(config) and self._check_skills(robot):
            self._configure(robot, config)

        rospy.loginfo("Resulting knowledge = {}".format(self._config_result.resulting_knowledge))

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
