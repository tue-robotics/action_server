from action import Action
from challenge_storing_groceries.open_door import OpenDoorMachine


class OpenDoor(Action):
    def __init__(self):
        """
        The Open Door action wraps the state machine to open the door of the cupboard at RWC 2018.

        """
        super(OpenDoor, self).__init__(
            required_field_prompts={},
            required_passed_knowledge={},
            required_skills=[],
        )

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
