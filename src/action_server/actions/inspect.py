from action import Action, ConfigurationData

from util import entities_from_description

import robot_smach_states
import threading

import rospy


class Inspect(Action):
    """ The Inspect class implements the action to inspect an area.

    Parameters to pass to the configure() method are:
     - `entity` (required): an entity with a segmentation area to inspect
    """
    def __init__(self):
        Action.__init__(self)
        self._required_field_prompts = {'entity': " What would you like me to inspect? "}
        self._required_skills = ['head', 'ed', 'base']

    def _configure(self, robot, config):
        self._robot = robot
        self._entity_description = config.semantics["entity"]

        self._config_result.succeeded = True
        return

    def _start(self):
        (entities, error_msg) = entities_from_description(self._entity_description, self._robot)
        if not entities:
            return error_msg

        entity = entities[0]

        if entity.is_a('furniture'):
            area = 'in_front_of'
        else:
            area = ''

        self._fsm = robot_smach_states.world_model.Inspect(self._robot,
                                                           entityDes=robot_smach_states.util.designators.EdEntityDesignator(
                                                               self._robot, id=entity.id),
                                                           navigation_area=area)

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

    config = ConfigurationData({'action': 'inspect',
              'entity': {'id': 'cabinet'}})

    action.configure(robot, config)
    action.start()
