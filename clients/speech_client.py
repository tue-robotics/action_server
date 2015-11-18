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
import json

if len(sys.argv) < 2:
    print "Please specify a robot name"
    sys.exit()

speech_recognition = True
if len(sys.argv) >= 3:
    if sys.argv[2] == "--no-ears":
        speech_recognition = False

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
add_action = rospy.ServiceProxy('/%s/action_server/add_action'%robot_name, AddAction)

e = robot.ears
s = robot.speech

drinks = ['drink']
places = ['cabinet']

def ask():

    # Look at the operator
    add_action(action="look-at", parameters=json.dumps({"entity": {"id": "operator"}}))

    s.speak("Hello! What can I do for you?")

    if speech_recognition:
        while not rospy.is_shutdown():
            spec = "Bring me the <drink> from the <place>"
            choices = {"drink" : drinks, "place" : places}
            r = e.recognize(spec, choices)
            if not r:
                s.speak("I didn't quite get that")
                return

            if r.choices["place"] == "cabinet":
                from_id = "cabinet"
                from_room = "kitchen"
            if r.choices["place"] == "table":
                from_id = "dinnertable"
                from_room = "living_room"

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

    else:
        # Default
        from_id = "dinnertable"
        from_room = "living_room"

        time.sleep(4.0)

        s.speak("Do you want me to bring you the %s from the %s?" % ("drink", "table"))

        time.sleep(2.0)

        s.speak("Alright!")

    # Cancel the head goal
    robot.head.cancel_goal()

    try:
        params = {"entity": {}, "from": {"id": from_id}, "from_room" : from_room,
                  "to": {"id": "operator"}, "to_radius": 1.5}

        add_action(action="bring", parameters=json.dumps(params))
        sys.exit()
    except rospy.ServiceException, error:
        print "Service call failed: %s"%error

while not rospy.is_shutdown():
    ask()
