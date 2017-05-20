import rospy

from robot_skills.robot import Robot
from robocup_knowledge import load_knowledge


class ConfigurationResult(object):
    '''
    The ConfigurationResult class defines the data structure that is returned by the configure() methods of actions
    '''
    def __init__(self, succeeded=False, resulting_knowledge=None):
        if resulting_knowledge is None:
            resulting_knowledge = {}

        self.succeeded = succeeded
        self.resulting_knowledge = resulting_knowledge
        self.missing_field = None
        self.missing_skill = None
        self.message = ""


class ActionResult(object):
    '''
    The ActionResult class defines the data structure that is returned by the run() methods of actions.
    '''
    def __init__(self, succeeded=False, message=""):
        self.succeeded = succeeded
        self.message = message


class Action(object):
    '''
    The Action class defines the interface of actions that can be configured and started by the task_manager.
    '''
    def __init__(self):
        self._config_result = ConfigurationResult()
        self._execute_result = ActionResult()
        self._required_field_prompts = {}
        self._knowledge = load_knowledge('common')

    def _check_parameters(self, config):
        for k, v in self._required_field_prompts.items():
            if k not in config:
                rospy.logerr("Missing required parameter {}".format(k))
                self._config_result.missing_field = k
                self._config_result.message = v
                return False
        return True

    def configure(self, robot, config):
        # TODO: push to debug
        rospy.loginfo("Configuring action {} with config {}.".format(self.__class__.__name__, config))
        if not isinstance(config, dict):
            rospy.logerr("Action: the specified config should be a dictionary! I received: %s" % str(config))
            return False

        if not isinstance(robot, Robot):
            rospy.logerr("Action: the specified robot should be a Robot! I received: %s" % str(robot))
            return False

        if not self._check_parameters(config):
            return self._config_result

        self._configure(robot, config)
        return self._config_result

    def _configure(self, robot, config):
        raise NotImplementedError

    def start(self):
        rospy.loginfo("Starting executing of action {}.".format(self.__class__.__name__))
        self._start()
        return self._execute_result

    def _start(self):
        raise NotImplementedError

    def cancel(self):
        rospy.loginfo("Canceling executing of action {}.".format(self.__class__.__name__))
        return self._cancel()

    def _cancel(self):
        raise NotImplementedError
