from action import Action

import rospy

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

if __name__ == "__main__":
    rospy.init_node('reset_wm_test')

    import sys
    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = ResetWM()

    config = {'action': 'reset_wm'}

    action.configure(robot, config)
    action.start()
