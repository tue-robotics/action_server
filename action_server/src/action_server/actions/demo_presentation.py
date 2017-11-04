from action import Action, ConfigurationData

import rospy
from challenge_presentation import PresentationMachine


class DemoPresentation(Action):
    """ The DemoPresentation class wraps the demo presentation state machine..

    """
    def __init__(self):
        Action.__init__(self)
        # self._required_skills = ['speech', 'ears']

    def _configure(self, robot, config):
        self._robot = robot
        if 'language' in config.semantics:
            language = config.semantics['language']
        else:
            language = 'en'
        self._presentation_sm = PresentationMachine(robot, language=language)
        self._config_result.succeeded = True
        return

    def _start(self):
        outcome = self._presentation_sm.execute()

        if outcome == 'done':
            self._execute_result.message = " I executed the demo presentation! Don't you think I was awesome? "
            self._execute_result.succeeded = True
        elif outcome == 'aborted':
            self._execute_result.message = " Something went wrong during initialization. Sorry! "
            self._execute_result.succeeded = False

        return

    def _cancel(self):
        pass


if __name__ == "__main__":
    rospy.init_node('demo_presentation_test')

    import sys

    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = DemoPresentation()
    config = ConfigurationData({'action': 'demo-presentation'})

    action.configure(robot, config)
    action.start()
