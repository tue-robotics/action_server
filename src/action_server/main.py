#!/usr/bin/env python

'''
A central place to dispatch actions received from various sources. These sources could come from
GUIs, command line tools or natural language.
'''

import rospy, sys, inspect, re

import actions
from server import Server

def class_name_camelcase_to_dashes(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1-\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1-\2', s1).lower()

if __name__ == "__main__":
    rospy.init_node('action_server_py')

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

    server = Server("action_server_py", robot)

    # Register all actions that can be found in the actions module
    for name, obj in inspect.getmembers(actions):
        if inspect.isclass(obj):
            server.register_skill(class_name_camelcase_to_dashes(name), obj)

    server.connect('action_server/register_action_server')

    rospy.spin()
