from action import Action

from util import entities_from_description

import robot_smach_states
import threading

import rospy


class Inspect(Action):
    def __init__(self):
        Action.__init__(self)

    def _configure(self, robot, config):
        if "entity" not in config:
            self._config_result.missing_field = "entity"
            return

        self._robot = robot  # TODO: this should also check if the given robot is capable of this action.
        self._entity_description = config["entity"]

        self._config_result.succeeded = True
        return

    def _start(self):
        (entities, error_msg) = entities_from_description(self._entity_description, self._robot)
        if not entities:
            return error_msg

        entity = entities[0]

        self._fsm = robot_smach_states.world_model.Inspect(self._robot,
                                                           entityDes=robot_smach_states.util.designators.EdEntityDesignator(
                                                               self._robot, id=entity.id))

        self._thread = threading.Thread(name='inspect', target=self._fsm.execute)
        self._thread.start()

        self._thread.join()
        self._execute_result.succeeded = True

    def _cancel(self):
        if self._fsm.is_running:
            self._fsm.request_preempt()

        # # Wait until canceled
        # self._thread.join()


if __name__ == "__main__":
    rospy.init_node('inspect_test')

    import sys
    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = Inspect()

    config = {'action': 'inspect',
              'entity': {'id': 'cabinet'}}

    action.configure(robot, config)
    action.start()
