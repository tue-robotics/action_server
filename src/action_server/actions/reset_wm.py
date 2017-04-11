from action import Action


class ResetWM(Action):
    def __init__(self):
        Action.__init__(self)

    def _configure(self, robot, config):
        self._robot = robot  # TODO: this should also check if the given robot is capable of this action.

        self._config_result.succeeded = True
        return

    def _start(self):
        self._robot.ed.reset()
        self._execute_result.succeeded = True

    def _cancel(self):
        pass
