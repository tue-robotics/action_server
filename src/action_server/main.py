#!/usr/bin/env python

'''
The Action Server ROS node running an instance of the server for a specific robot.
'''

import rospy, sys
from server import Server

if __name__ == "__main__":
    rospy.init_node('action_server')

    # Create Robot object based on argv[1]
    if len(sys.argv) < 2:
        print "Please specify a robot name 'amigo/sergio'"
        sys.exit()

    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        print "unknown robot"
        sys.exit()

    robot = Robot()

    server = Server("state_machine", robot)

    rospy.spin()
