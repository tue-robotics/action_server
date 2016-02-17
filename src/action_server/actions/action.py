import rospy, threading

from robot_skills.robot import Robot
import smach

class Action:

    def start(self, config, robot):
        if not isinstance(config, dict):
            rospy.logerr("Action: the specified config should be a dictionary! I received: %s" % str(config))
            return False

        if not isinstance(robot, Robot):
            rospy.logerr("Action: the specified robot should be a Robot! I received: %s" % str(robot))
            return False

        return self._start(config, robot)

    def _start(self, config, robot):
        raise NotImplementedError

    def cancel(self):
        return self._cancel()

    def _cancel(self):
        raise NotImplementedError

# ----------------------------------------------------------------------------------------------------

class FSMAction(Action):

    def __init__(self):
        self._fsm = None

    def _init_fsm(self, config, robot):
        raise NotImplementedError

    def _start(self, config, robot):
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
