#!/usr/bin/env python

"""
The Action Server ROS node running an instance of the server for a specific robot.
"""

import rospy
import sys
from .server import Server


if __name__ == "__main__":
    rospy.init_node('action_server')

    try:
        robot_name = rospy.get_param('~robot_name').lower()
    except KeyError:
        rospy.logerr("Please provide a 'robot_name'")
        exit(0)

    rospy.loginfo("Parameters:")
    rospy.loginfo("robot_name = {}".format(robot_name))

    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    elif robot_name == 'hero':
        from robot_skills.hero import Hero as Robot
    else:
        rospy.logerr("'robot_name' must be 'amigo', 'sergio' or 'hero'")
        sys.exit()

    robot = Robot()

    server = Server(robot)

    rospy.spin()
