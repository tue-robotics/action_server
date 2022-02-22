import rospy

from .action import Action, ConfigurationData


class ResetWM(Action):
    """
    The ResetWM class implements the action reset the robot's world model.
    """

    def __init__(self):
        Action.__init__(self)
        self._required_skills = ['ed']

    def _configure(self, robot, config):
        self._robot = robot

        self._config_result.succeeded = True
        return

    def _start(self):
        self._robot.ed.reset()
        self._execute_result.succeeded = True

    def _cancel(self):
        pass


if __name__ == "__main__":
    rospy.init_node('reset_wm_test')

    from robot_skills import get_robot_from_argv

    robot = get_robot_from_argv(1)

    action = ResetWM()

    config = ConfigurationData({'action': 'reset-wm'})

    action.configure(robot, config)
    action.start()
