import threading

import rospy

from robot_smach_states.human_interaction import SSLLookatAndRotate
from .action import Action, ConfigurationData


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
        if 'duration' in config.semantics:
            self._duration = rospy.Duration(int(config.semantics['duration']))
        else:
            self._duration = rospy.Duration(10)

        self._config_result.succeeded = True
        self._state_machine = SSLLookatAndRotate(robot)
        self._thread = threading.Thread(target=self.listen_and_rotate)
        return

    def _start(self):
        rospy.loginfo('Starting TurnTowardSound action')
        self._thread.start()
        rospy.sleep(self._duration)
        self._cancel()
        self._thread.join()
        self._execute_result.succeeded = True

    def _cancel(self):
        self._canceled = True


if __name__ == "__main__":
    rospy.init_node('turn_toward_sound_test')

    from robot_skills import get_robot_from_argv

    robot = get_robot_from_argv(1)

    action = TurnTowardSound()

    config = ConfigurationData(
        semantics={'action': 'turn-toward-sound',
                   'duration': '30'}
    )

    action.configure(robot, config)
    action.start()
