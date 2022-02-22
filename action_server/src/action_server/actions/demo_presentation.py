import rospy

from challenge_presentation import PresentationMachine, PresentationMachineHero
from .action import Action, ConfigurationData


class DemoPresentation(Action):
    """
    The DemoPresentation class wraps the demo presentation state machine
    """

    def __init__(self):
        Action.__init__(self)
        # self._required_skills = ['speech', 'hmi']

    def _configure(self, robot, config):
        self._robot = robot
        if 'language' in config.semantics:
            language = config.semantics['language']
        else:
            language = 'en'
        if robot.robot_name == 'hero':
            self._presentation_sm = PresentationMachineHero(robot, language=language)
        else:
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
        self._presentation_sm.request_preempt()


if __name__ == "__main__":
    rospy.init_node('demo_presentation_test')

    from robot_skills import get_robot_from_argv

    robot = get_robot_from_argv(1)

    action = DemoPresentation()
    config = ConfigurationData({'action': 'demo-presentation'})

    action.configure(robot, config)
    action.start()
