from action import Action, ConfigureResult

class ResetWM(Action):

    def _configure_(self, robot, config):
        self._robot = robot # TODO: this should also check if the given robot is capable of this action.

        self._config_result.succeeded = True
        return

    def _start(self):
        self._robot.ed.reset()

    def _cancel(self):
        pass
