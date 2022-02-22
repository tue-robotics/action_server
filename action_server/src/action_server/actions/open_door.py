from challenge_storing_groceries.open_door import OpenDoorMachine

from .action import Action


class OpenDoor(Action):
    """
    The Open Door action wraps the state machine to open the door of the cupboard at RWC 2018.

    """

    def __init__(self):
        # Call the base class constructor
        Action.__init__(self)
        # self._required_field_prompts = {'object': " What object would you like me to work with? "}

    def _configure(self, robot, config):
        self._state_machine = OpenDoorMachine(robot, "cupboard", "in_front_of", "shelf6")
        self._config_result.succeeded = True
        return

    def _start(self):
        self._state_machine.execute()

        self._execute_result.message = " I opened the cupboard door. "
        self._execute_result.succeeded = True
        return

    def _cancel(self):
        pass
