#!/usr/bin/env python

import rospy
import action_server.srv

import yaml
import thread
import time
import threading
import sys

# -------------------------------------
# For PickUp:
from robot_skills.util import transformations
import robot_skills.util.msg_constructors as msgs

from robot_smach_states.navigation import NavigateToObserve, NavigateToWaypoint
from robot_smach_states.manipulation import Grab, Place
from cb_planner_msgs_srvs.msg import PositionConstraint, OrientationConstraint
from robot_smach_states.util.designators import Designator, UnoccupiedArmDesignator, EdEntityDesignator, ArmHoldingEntityDesignator
import ed.msg
from robot_skills.arms import Arm
# -------------------------------------

global server


class PickUp:

    def __init__(self):
        self.grab = None
        self.thread = None

    def create_action(self, action_type, config, robot):
        print 'PickUp!'

        try:
            entity_id = config['entity']
        except KeyError:
            print 'No object given'
            return False

        side = config['side'] if 'side' in config else 'right'

        if side == 'left':
            arm = robot.leftArm
        else:
            arm = robot.rightArm

        self.grab = Grab(robot, arm=UnoccupiedArmDesignator(robot.arms, arm), item=EdEntityDesignator(robot, id=entity_id))    
        self.thread = threading.Thread(name='grab', target=self.grab.execute)
        self.thread.start()

    def cancel(self):
        if self.grab.is_running:
            self.grab.request_preempt()

        # Wait until canceled
        self.thread.join()

# ----------------------------------------------------------------------------------------------------

class Put:

    def __init__(self):
        self.place = None
        self.thread = None

    def create_action(self, action_type, config, robot):
        print "Place!"

        try:
            side = config["side"]
        except KeyError:
            side = "right" # Default

        if side == "left":
            arm = robot.leftArm
            goal_y = 0.2
        else:
            arm = robot.rightArm
            goal_y = -0.2

        try:
            height = config["height"]
        except KeyError:
            height = 0.8

        x = 0.2
        place_pos = msgs.PointStamped(x, goal_y, height + 0.2, 0.0, 0.0, 0.0, frame_id="/amigo/base_link")

        item_to_place = Designator(arm.occupied_by, resolve_type=ed.msg.EntityInfo)  # Which item do we want to place? The object in the hand we indicated
        arm_with_item_designator = Designator(arm,  resolve_type=Arm) #No need for ArmHoldingEntityDesignator, we already know that from the config
        place_position = Designator(place_pos, resolve_type=msgs.PointStamped)
        self.place = Place(robot, item_to_place, place_position, arm_with_item_designator)
        
        self.place = Place(robot, item_to_place, place_position, arm_with_item_designator)    
        self.thread = threading.Thread(name='grab', target=self.place.execute)
        self.thread.start()

    def cancel(self):
        if self.place.is_running:
            self.place.request_preempt()

        # Wait until canceled
        self.thread.join()  

# ----------------------------------------------------------------------------------------------------

class NavigateTo:

    def __init__(self):
        self.nwc = None
        self.goal_entity = None
        self.goal_type = None

    def create_action(self, action_type, config, robot):

        try:
            self.goal_entity = config["entity"]
            self.goal_type = config["entity_type"]
        except KeyError:
            print "No object given"
            return False

        if self.goal_type == "waypoint":
            self.nwc = NavigateToWaypoint(robot, waypoint_designator=EdEntityDesignator(robot, id=self.goal_entity), radius=0.1)
            rospy.logwarn("ACTION_SERVER: Navigating to waypoint")
        else:
            self.nwc = NavigateToObserve(robot, entity_designator=EdEntityDesignator(robot, id=self.goal_entity), radius=.5)
            rospy.logwarn("ACTION_SERVER: Navigating to observe")

        self.thread = threading.Thread(name='navigate', target=self.nwc.execute)
        self.thread.start()

    def cancel(self):
        if self.nwc.is_running:
            self.nwc.request_preempt()

        # Wait until canceled
        self.thread.join()

# ----------------------------------------------------------------------------------------------------

class Server:

    def __init__(self, robot): # Robot should no be in the constructor but should be in the add_action -- REIN
        self.action_type_to_skill = {}
        self.last_action = None
        self.robot = robot # Should be in the add action, not in the constructor -- REIN

    def register_skill(self, action_type, skill):
        self.action_type_to_skill[action_type] = skill
    
    def add_action(self, action_type, config): # We should include the robot here as arguments, not pass it in the constructor if we want to use multiple robots, this is desired :) -- REIN
        try:
            self.action = self.action_type_to_skill[action_type]

            if self.last_action:
               self.last_action.cancel()

            self.action.create_action(action_type, config, self.robot)

            self.last_action = self.action
        except KeyError:
            return False

        return True

def srv_add_action(req):
    config = yaml.load(req.parameters)

    global server
    server.add_action(req.action, config)

    return action_server.srv.AddActionResponse("", "")

# ----------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    node_name = 'action_server_py'
    rospy.init_node(node_name)

    add_action_service_name = '/' + node_name + '/add_action'
    srv = rospy.Service(add_action_service_name, action_server.srv.AddAction, srv_add_action)

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

    global server
    server = Server(robot)

    # Register components
    pick_up = PickUp()
    server.register_skill("pick-up", pick_up)

    put = Put()
    server.register_skill("place", put) #The original state from robot_smach_states is also called Place so there we need a different name

    navigate_to = NavigateTo()
    server.register_skill("navigate-to", navigate_to)

    # Register this server at the main (c++) action server
    print "Waiting for connection with 'action_server/register_action_server'..."
    rospy.wait_for_service('action_server/register_action_server')
    print "... Connected."  

    try:
        register_client = rospy.ServiceProxy('action_server/register_action_server', action_server.srv.RegisterActionServer)
        resp = register_client(add_action_service_name, '/' + node_name + '/get_action_status')

        if resp.error_msg:
            print "ERROR: " + resp.error_msg
            sys.exit(1)            
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    rospy.spin()
