from action import Action, ConfigurationData
from robot_smach_states import SSLLookatAndRotate
import threading

import rospy


class TurnTowardSound(Action):
    """
    The TurnTowardSound class implements the action to turn towards the sound source provided by SSL
    """
    def __init__(self):
        Action.__init__(self)
        self._canceled = False

    def listen_and_rotate(self):
        while not self._canceled:
            self._state_machine.execute(userdata={})
            rospy.sleep(rospy.Duration(1))

    def _configure(self, robot, config):
        self._config_result.succeeded = True
        self._state_machine = SSLLookatAndRotate(robot)
        self._thread = threading.Thread(target=self.listen_and_rotate)
        return

    def _start(self):
        rospy.loginfo('Starting TurnTowardSound action')
        self._thread.start()
        rospy.sleep(rospy.Duration(10))
        self._cancel()
        self._thread.join()
        self._execute_result.succeeded = True

    def _cancel(self):
        self._canceled = True

if __name__ == "__main__":
    rospy.init_node('turn_toward_sound_test')

    import sys
    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = TurnTowardSound()

    config = ConfigurationData(
        semantics={'action': 'turn-toward-sound'}
    )

    action.configure(robot, config)
    action.start()
