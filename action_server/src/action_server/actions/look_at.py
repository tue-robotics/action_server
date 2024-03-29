import threading

from pykdl_ros import VectorStamped
import rospy

from .action import Action, ConfigurationData
from .util import entities_from_description


class LookAt(Action):
    """
    The LookAt class implements the action to look at a world model entity.

    Parameters to pass to the configure() method are:
     - `entity` (optional): the entity id to look at.
    """

    def __init__(self):
        Action.__init__(self)
        self._required_field_prompts = {'entity': " What would you like me to look at? "}
        self._required_skills = ['head']

    def _configure(self, robot, config):
        self._robot = robot

        (entities, error_msg) = entities_from_description(config.semantics["entity"], robot)
        if not entities:
            rospy.logwarn(error_msg)
            return

        self._entity = entities[0]

        self._config_result.succeeded = True

    def _start(self):
        self._thread = threading.Thread(name='head-goal', target=self._execute)
        self._thread.start()

        self._thread.join()

    def _execute(self):
        self._robot.head.cancel_goal()

        if self._entity:
            self._robot.head.look_at_point(VectorStamped.from_framestamped(self._entity.pose), timeout=10)

        self._execute_result.succeeded = True

    def _cancel(self):
        self._robot.head.cancel_goal()


if __name__ == "__main__":
    rospy.init_node('look_at_test')

    from robot_skills import get_robot_from_argv

    robot = get_robot_from_argv(1)

    action = LookAt()

    config = ConfigurationData({'action': 'look_at',
                                'entity': {'id': 'cabinet'}})

    action.configure(robot, config)
    action.start()
