import rospy

from robot_skills.robot import Robot


class ConfigurationResult(object):
    def __init__(self, succeeded=False, resulting_knowledge=None):
        if resulting_knowledge is None:
            resulting_knowledge = {}

        self.succeeded = succeeded
        self.resulting_knowledge = resulting_knowledge
        self.missing_field = None


class ActionResult(object):
    def __init__(self, succeeded=False, message=""):
        self.succeeded = succeeded
        self.message = message


class Action:
    def __init__(self):
        self._config_result = ConfigurationResult()
        self._execute_result = ActionResult()

    def configure(self, robot, config):
        if not isinstance(config, dict):
            rospy.logerr("Action: the specified config should be a dictionary! I received: %s" % str(config))
            return False

        if not isinstance(robot, Robot):
            rospy.logerr("Action: the specified robot should be a Robot! I received: %s" % str(robot))
            return False

        self._configure(robot, config)
        return self._config_result

    def _configure(self, robot, config):
        raise NotImplementedError

    def start(self):
        self._start()
        return self._execute_result

    def _start(self):
        raise NotImplementedError

    def cancel(self):
        return self._cancel()

    def _cancel(self):
        raise NotImplementedError
