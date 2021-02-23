from action import Action, ConfigurationData

import rospy
import threading


class GripperGoal(Action):
    """
    The GripperGoal class implements the action to open or close the gripper.

    Parameters to pass to the configure() method are:
     - `arm_name` (required): the arm to use
     - `goal` (required): the goal (open or close).
    """

    def __init__(self):
        Action.__init__(self)
        self._required_field_prompts = {'arm_name': " Which gripper should I move? ",
                                        'goal': " Should I open my gripper, or close it? "}
        self._required_skills = ['arms']

    def _configure(self, robot, config):
        arm_name = config.semantics['arm_name']
        try:
            self._arm = robot.arms[arm_name]
        except KeyError:
            self._config_result.message = " I don't have an arm, {} with grippers to close. ".format(arm_name)
            self._config_result.missing_skill = arm_name

        self._goal = config.semantics["goal"]

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
                                'arm_name': 'arm_center',
                                'goal': 'close'})

    action.configure(robot, config)
    action.start()
