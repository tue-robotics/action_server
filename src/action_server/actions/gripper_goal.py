from action import Action

import rospy
import threading


class GripperGoal(Action):

    def __init__(self):
        Action.__init__(self)
        self._arm = None
        self._goal = None

    def _configure(self, robot, config):
        if not "side" in config:
            rospy.logwarn("Please provide 'side'")
            self._config_result.missing_field = "side"

        if config['side'] == 'left':
            self._arm = robot.leftArm
        else:
            self._arm = robot.rightArm

        if not "goal" in config:
            rospy.logwarn("Please specify 'goal'")
            self._config_result.missing_field = "goal"

        self._goal = config["goal"]

        self._config_result.succeeded = True

    def _start(self):
        self._thread = threading.Thread(name='gripper-goal', target=self._execute)
        self._thread.start()

        self._thread.join()
        self._execute_result.succeeded = True

    def _execute(self):
        self._arm.send_gripper_goal(self._goal)

    def _cancel(self):
        pass
