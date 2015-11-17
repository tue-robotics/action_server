#! /usr/bin/env python
import roslib; 

from robot_skills.ears import Ears
from robot_skills.speech import Speech
from robot_skills.head import Head

import random
import time

from action_server.srv import AddAction

import sys

import rospy

if len(sys.argv) < 2:
    print "Please specify a robot name"
    sys.exit()

robot_name = sys.argv[1]

rospy.init_node("action_server_speech_client")
if robot_name == "amigo":
    from robot_skills.amigo import Amigo
    robot = Amigo()
elif robot_name == "sergio":
    from robot_skills.sergio import Sergio
    robot = Sergio()
else:
    print "Unknown robot '%s'"%robot_name
    sys.exit()

rospy.wait_for_service('/%s/action_server/add_action'%robot_name)

e = robot.ears
s = robot.speech
robot.head.look_at_standing_person()

drinks = ['coke','fanta']
places = ['table','cabinet']

def ask():
    s.speak("Say %s and I will listen to you!"%robot_name)
    spec = robot_name
    r = e.recognize(spec, {}, time_out=rospy.Duration(60))

    if r:
        s.speak("What's up")
        spec = "Bring me a <drink> from the <place>"
        choices = {"drink" : drinks, "place" : places}
        r = e.recognize(spec, choices)
        if r:
            s.speak("You said: %s"%r.result)
            try:
                add_action = rospy.ServiceProxy('/%s/action_server/add_action'%robot_name, AddAction)
                add_action(action=r.choices["drink"], parameters="{entity:%s}"%r.choices['place'])
                sys.exit()
            except rospy.ServiceException, error:
                print "Service call failed: %s"%error
        else:
            s.speak("I didn't quite get that")

while not rospy.is_shutdown():
    ask()
