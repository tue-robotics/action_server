from action import Action

class ResetWM(Action):

    def _start(self, config, robot):
        robot.ed.reset()

    def _cancel(self):
        pass
