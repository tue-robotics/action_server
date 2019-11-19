from action import Action, ConfigurationData

import rospy
from robot_smach_states import GetFurnitureFromOperatorPose


class PointTarget(Action):
    """ The DemoPresentation class wraps the demo presentation state machine..

    """
    def __init__(self):
        Action.__init__(self)
        # self._required_skills = ['speech', 'hmi']

    def _configure(self, robot, config):
        self._robot = robot
        self._point_sm = GetFurnitureFromOperatorPose(robot)
        self._config_result.succeeded = True
        return

    def _start(self):
        outcome = self._point_sm.execute()

        if outcome == 'done':
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

    import sys

    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    elif robot_name == 'hero':
        from robot_skills.hero import Hero as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = GetFurniture()
    config = ConfigurationData({'action': 'point-at'})

    action.configure(robot, config)
    action.start()
