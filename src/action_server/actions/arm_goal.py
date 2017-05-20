from action import Action

import threading
import rospy


class ArmGoal(Action):
    """
    The ArmGoal class implements the action of moving the arm to a predefined joint position.

    Parameters to pass to the configure method are 'side' (optional) and 'symbolic' (required). The 'symbolic'
     parameter must be a string as defined in Arm.default_joint_configurations.
    """

    def __init__(self):
        Action.__init__(self)
        self._arm = None
        self._symbolic_goal = None

    def _configure(self, robot, config):
        if not "side" in config:
            rospy.logwarn( "Please provide 'side'")
            self._config_result.missing_field = "side"
            return

        if config['side'] == 'left':
            self._arm = robot.leftArm
        else:
            self._arm = robot.rightArm

        if not "symbolic" in config:
            rospy.logwarn("Please provide 'symbolic' keyword.")

        self._symbolic_goal = config["symbolic"]
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

    import sys
    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = ArmGoal()

    config = {'action': 'arm_goal',
              'side': 'right',
              'symbolic': 'carrying_box_pose'}

    action.configure(robot, config)
    action.start()
