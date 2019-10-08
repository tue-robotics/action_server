from action import Action
# ToDo: move dishwasher stuff or remove action?
from challenge_dishwasher.dishwasher import NavigateAndPlaceDishwasher, GrabRobust, NavigateAndOpenDishwasher


class ClearTable(Action):
    def __init__(self):
        super(ClearTable, self).__init__(
            required_field_prompts={},
            required_passed_knowledge={},
            required_skills=[],
        )

    def _configure(self, robot, config):
        self._state_machines = [GrabRobust(robot), NavigateAndOpenDishwasher(robot), NavigateAndPlaceDishwasher(robot)]

        self._config_result.succeeded = True
        self._active_state_machine = None
        return

    def _start(self):
        for sm in self._state_machines:
            self._active_state_machine = sm
            self._active_state_machine.execute()

        self._active_state_machine = None

        self._execute_result.message = " I cleaned the table! "
        self._execute_result.succeeded = True
        return

    def _cancel(self):
        if self._active_state_machine:
            self._active_state_machine.request_preempt()
