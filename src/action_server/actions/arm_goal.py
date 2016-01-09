from action import Action

import threading

class ArmGoal(Action):

    def __init__(self):
        self._arm = None
        self._symbolic_goal = None

    def _start(self, config, robot):
        if not "side" in config:
            return "Please provide 'side'"

        if config['side'] == 'left':
            self._arm = robot.leftArm
        else:
            self._arm = robot.rightArm

        if not "symbolic" in config:
            return "Please provide 'symbolic' keyword."

        self._symbolic_goal = config["symbolic"]

        self._thread = threading.Thread(name='arm-goal', target=self._execute)
        self._thread.start()

    def _execute(self):
        if self._symbolic_goal:
            self._arm.send_joint_goal(self._symbolic_goal)

    def _cancel(self):
        pass
