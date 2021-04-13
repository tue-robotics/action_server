#!/usr/bin/env python

"""
The Action Server ROS node running an instance of the server for a specific robot.
"""

import rospy

from robot_skills import get_robot

from action_server import Server

if __name__ == "__main__":
    rospy.init_node('action_server')

    try:
        robot_name = rospy.get_param('~robot_name').lower()
    except KeyError:
        rospy.logerr("Please provide a 'robot_name'")
        exit(0)

    rospy.loginfo("Parameters:")
    rospy.loginfo("robot_name = {}".format(robot_name))

    robot = get_robot(robot_name)

    server = Server(robot)

    rospy.spin()
