from action import Action, ConfigurationData

import rospy

class FindOutAndReport(Action):
    ''' The Say class implements the action to say something.

    Parameters to pass to the configure() method are:
     - `sentence` (required): The sentence to speak. May be a keyword to tell something more intelligent.
    '''
    def __init__(self):
        Action.__init__(self)
        self._required_skills = ['speech']

    def _configure(self, robot, config):
        self._robot = robot

        self._config_result.message = "Sorry, I can't do that yet. Please give me another command."
        self._config_result.missing_field = ""

        rospy.loginfo('Robot cannot perform this action yet, so saying so and returning')

        self._config_result.succeeded = False

    def _start(self):
        self._execute_result.succeeded = False
        self._execute_result.message = ""

    def _cancel(self):
        pass

if __name__ == "__main__":
    rospy.init_node('find_out_and_report_test')

    import sys
    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = FindOutAndReport()

    config = ConfigurationData({}, {})

    action.configure(robot, config)
    action.start()
