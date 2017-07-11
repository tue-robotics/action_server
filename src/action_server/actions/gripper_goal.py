from action import Action, ConfigurationData

import rospy
import threading


class GripperGoal(Action):
    """ The GripperGoal class implements the action to open or close the gripper.

    Parameters to pass to the configure() method are:
     - `side` (required): the gripper's side (left or right)
     - `goal` (required): the goal (open or close).
    """
    def __init__(self):
        Action.__init__(self)
        self._required_field_prompts = {'side' : " Which gripper should I move? ",
                                        'goal' : " Should I open my gripper, or close it? "}
        self._required_skills = ['arms']

    def _configure(self, robot, config):
        config = config.semantics
        side = config['side']
        try:
            self._arm = robot.arms[side]
        except KeyError:
            self._config_result.message = " I don't have a {} arm with grippers to close. ".format(side)
            self._config_result.missing_skill = side + "Arm"

        self._goal = config["goal"]

        self._config_result.succeeded = True

    def _start(self):
        self._thread = threading.Thread(name='gripper-goal', target=self._execute)
        self._thread.start()

        self._thread.join()
        self._execute_result.succeeded = True

    def _execute(self):
        self._arm.send_gripper_goal(self._goal)

    def _cancel(self):
        pass


if __name__ == "__main__":
    rospy.init_node('gripper_goal_test')

    import sys
    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = GripperGoal()

    config = ConfigurationData({'action': 'gripper_goal',
              'side': 'left',
              'goal': 'close'})

    action.configure(robot, config)
    action.start()
