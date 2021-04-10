import threading

import rospy

from .action import Action, ConfigurationData


class ArmGoal(Action):
    """
    The ArmGoal class implements the action of moving the arm to a predefined joint position.

    Parameters to pass to the configure method are 'side' (optional) and 'symbolic' (required). The 'symbolic'
     parameter must be a string as defined in Arm.default_joint_configurations.
    """

    def __init__(self):
        Action.__init__(self)
        self._required_field_prompts = {'side': " Which arm should I move? ",
                                        'symbolic': " What should I do with my arm? "}
        self._required_skills = ['arms']

    def _configure(self, robot, config):
        if config.semantics['side'] == 'left':
            self._arm = robot.leftArm
        else:
            self._arm = robot.rightArm

        self._symbolic_goal = config.semantics["symbolic"]
        self._config_result.succeeded = True

    def _start(self):
        self._thread = threading.Thread(name='arm-goal', target=self._execute)
        self._thread.start()

        self._thread.join()

    def _execute(self):
        if self._symbolic_goal:
            self._arm.send_joint_goal(self._symbolic_goal)
            self._execute_result.succeeded = True
        else:
            self._execute_result.succeeded = False

    def _cancel(self):
        pass


if __name__ == "__main__":
    rospy.init_node('arm_goal_test')

    from robot_skills import get_robot_from_argv

    robot = get_robot_from_argv(1)

    action = ArmGoal()

    semantics = {'action': 'arm_goal',
                 'side': 'right',
                 'symbolic': 'carrying_box_pose'}

    action.configure(robot, ConfigurationData(semantics))
    action.start()
