from action import ConfigurationData
from navigate_to import NavigateTo

import rospy


class Guide(NavigateTo):
    ''' The Guide class navigates to a target, telling someone to follow the robot and about arriving at the target.

    Parameters to pass to the configure() method are:
     - `object` (required): the id of the entity to navigate to
    '''
    def __init__(self):
        NavigateTo.__init__(self)

    def _configure(self, robot, config):
        if 'object' in config.semantics and 'id' in config.semantics['object']:
            self._id = config.semantics['object']['id']
        else:
            self._id = None

        return NavigateTo._configure(self, robot, config)

    def _start(self):
        self._robot.speech.speak("Follow me if you want to live", block=True)

        res = NavigateTo._start(self)

        if self._id:
            self._robot.speech.speak("Here is the {}".format(self._id))

        return res

    def _cancel(self):
        return NavigateTo._cancel(self)


if __name__ == "__main__":
    rospy.init_node('navigate_to_test')

    import sys
    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = NavigateTo()

    config = ConfigurationData({'action': 'guide',
              'object': {'id': 'cabinet'}})

    action.configure(robot, config)
    action.start()
