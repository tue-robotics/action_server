import rospy, threading

from robot_skills.robot import Robot
import smach

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

# ----------------------------------------------------------------------------------------------------

class FSMAction(Action):

    def __init__(self):
        self._fsm = None

    def _init_fsm(self, robot):
        raise NotImplementedError

    def _start(self, robot):
        err = self._init_fsm(config, robot)
        if err:
            return err

        self._thread = threading.Thread(name='fsm', target=self._run)
        self._thread.start()

    def _run(self):
        self._fsm.execute()
        self._fsm = None

    def _cancel(self):
        if self._fsm and isinstance(self._fsm, smach.StateMachine) and self._fsm.is_running:
            self._fsm.request_preempt()

        # Wait until canceled
        self._thread.join()
