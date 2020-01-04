from action import Action, ConfigurationData
from robot_smach_states import ShowOpenPoseLoop
import rospy


class ShowOpenposeAction(Action):
    """
    The ShowOpenposeAction class implements the action to show openpose on the screen of Hero
    """
    def __init__(self):
        Action.__init__(self)

    def _configure(self, robot, config):
        if 'iterations' in config.semantics:
            self._iterations = int(config.semantics['iterations'])
        else:
            self._iterations = 15

        self._config_result.succeeded = True
        self._state_machine = ShowOpenPoseLoop(robot, iterations=self._iterations)
        return

    def _start(self):
        rospy.loginfo('Starting ShowOpenPoseAction action')
        outcome = self._state_machine.execute()
        if outcome == 'done':
            self._execute_result.succeeded = True
        else:
            self._execute_result.succeeded = False


if __name__ == "__main__":
    rospy.init_node('show_openpose_test')

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

    action = ShowOpenposeAction()

    config = ConfigurationData(
        semantics={'action': 'show-openpose',
                   'iterations': '15'}
    )

    action.configure(robot, config)
    action.start()
