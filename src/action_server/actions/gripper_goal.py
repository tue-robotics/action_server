from action import Action

import threading

class GripperGoal(Action):

    def __init__(self):
        self._arm = None
        self._goal = None

    def _start(self, config, robot):
        if not "side" in config:
            return "Please provide 'side'"

        if config['side'] == 'left':
            self._arm = robot.leftArm
        else:
            self._arm = robot.rightArm

        if not "goal" in config:
            return "Please specify 'goal'"

        self._goal = config["goal"]

        self._thread = threading.Thread(name='gripper-goal', target=self._execute)
        self._thread.start()

    def _execute(self):
        self._arm.send_gripper_goal(self._goal)

    def _cancel(self):
        pass
