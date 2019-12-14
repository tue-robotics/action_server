from action import Action, ConfigurationData
from robot_smach_states import ShowOpenPose
import threading

import rospy


class ShowOpenposeAction(Action):
    """
    The ShowOpenposeAction class implements the action to turn towards the sound source provided by SSL
    """
    def __init__(self):
        Action.__init__(self)
        self._canceled = False

    def show_openpose(self):
        while not self._canceled:
            self._state_machine.execute(userdata={})
            rospy.sleep(rospy.Duration(1))

    def _configure(self, robot, config):
        if 'duration' in config.semantics:
            self._duration = rospy.Duration(int(config.semantics['duration']))
        else:
            self._duration = rospy.Duration(10)

        self._config_result.succeeded = True
        self._state_machine = ShowOpenPose(robot)
        self._thread = threading.Thread(target=self.show_openpose)
        return

    def _start(self):
        rospy.loginfo('Starting ShowOpenPoseAction action')
        self._thread.start()
        rospy.sleep(self._duration)
        self._cancel()
        self._thread.join()
        self._execute_result.succeeded = True

    def _cancel(self):
        self._canceled = True

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
                   'duration': '30'}
    )

    action.configure(robot, config)
    action.start()
