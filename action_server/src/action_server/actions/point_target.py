import rospy

from robocup_knowledge import load_knowledge
from robot_smach_states.human_interaction import GetFurnitureFromOperatorPose
from robot_smach_states.util.designators import VariableDesignator
from ed.entity import Entity
from .action import Action, ConfigurationData


class PointTarget(Action):
    """
    The DemoPresentation class wraps the demo presentation state machine..
    """

    def __init__(self):
        Action.__init__(self)

    def _configure(self, robot, config):
        self._robot = robot
        self._knowledge = load_knowledge('common')
        self._entity_des = VariableDesignator(resolve_type=Entity).writeable
        self._point_sm = GetFurnitureFromOperatorPose(robot, self._entity_des, self._knowledge.location_names)
        self._config_result.succeeded = True

    def _start(self):
        outcome = self._point_sm.execute()

        if outcome == 'succeeded':
            self._execute_result.message = "I saw what you pointed at!"
            self._execute_result.succeeded = True
        elif outcome == 'failed':
            self._execute_result.message = "Something went wrong. Sorry!"
            self._execute_result.succeeded = False

        return

    def _cancel(self):
        self._point_sm.request_preempt()


if __name__ == "__main__":
    rospy.init_node('demo_point-at_test')

    from robot_skills import get_robot_from_argv

    robot = get_robot_from_argv(1)

    action = PointTarget()
    config = ConfigurationData({'action': 'point-at'})

    action.configure(robot, config)
    action.start()
