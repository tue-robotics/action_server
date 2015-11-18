#! /usr/bin/env python
import roslib; 

from robot_skills.ears import Ears
from robot_skills.speech import Speech
from robot_skills.head import Head

import robot_skills.util.msg_constructors as msgs

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

drinks = ['coke','fanta']
places = ['table','cabinet']

def ask():
    s.speak("Hello! What can I do for you?")

    while not rospy.is_shutdown():
        spec = "Bring me the <drink> from the <place>"
        choices = {"drink" : drinks, "place" : places}
        r = e.recognize(spec, choices)
        if not r:
            s.speak("I didn't quite get that")
            return

        s.speak("Do you want me to bring you the %s from the %s?" % (r.choices["drink"], r.choices["place"]))
        r = e.recognize("<yesno>", {"yesno": ["yes", "no"]})
        if not r:
            s.speak("I'll take that as a yes")
            break
        elif r.choices["yesno"] == "no":
            s.speak("I'm sorry I didn't understand! Can you please tell me again?")
        else:
            s.speak("Alright!")
            break

    # Cancel the head goal
    robot.head.cancel_goal()

    try:
        add_action = rospy.ServiceProxy('/%s/action_server/add_action'%robot_name, AddAction)
        #add_action(action=r.choices["drink"], parameters="{entity:%s}"%r.choices['place'])

        params = {"entity": {}, "from": {"id": "dinnertable"}, "from_room" : "living_room",
                  "to": {"id": "operator"}, "to_radius": 1.5}

        import json
        add_action(action="bring", parameters=json.dumps(params))
        sys.exit()
    except rospy.ServiceException, error:
        print "Service call failed: %s"%error

while not rospy.is_shutdown():
    ask()
