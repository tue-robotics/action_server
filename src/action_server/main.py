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
from robot_smach_states.manipulation import Grab
from cb_planner_msgs_srvs.msg import PositionConstraint, OrientationConstraint
from robot_smach_states.util.designators import *
# -------------------------------------

global server


class PickUp:

    def __init__(self):
        pass

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

class Place:

    def __init__(self):
        pass

    def create_action(self, action_type, config, robot):
        print "Place!"

        try:
            side = config["side"]
        except KeyError:
            side = "right" # Default

        if side == "left":
            arm = robot.leftArm
        else:
            arm = robot.rightArm

        try:
            height = config["height"]
        except KeyError:
            height = 0.8

        # Torso to highest position
        robot.spindle.high()

        if side == "left":
            goal_y = 0.2
        else:
            goal_y = -0.2

        dx = 0.5

        x = 0.2
        while x <= dx:
            if not arm.send_goal(x, goal_y, height + 0.2, 0.0, 0.0, 0.0, timeout=20, pre_grasp=False, frame_id="/amigo/base_link"):
                print "Failed pre-drop"
                return
            x += 0.1       

        if not arm.send_goal(dx, goal_y, height + 0.1, 0.0, 0.0, 0.0, timeout=20, pre_grasp=False, frame_id="/amigo/base_link"):
            print "drop"
            return   

        # Open gripper
        arm.send_gripper_goal('open')

        x = dx
        while x > 0.3:
            if not arm.send_goal(x, goal_y, height + 0.2, 0.0, 0.0, 0.0, timeout=20, pre_grasp=False, frame_id="/amigo/base_link"):
                print "Failed pre-drop"
                return
            x -= 0.1       	

        if not arm.send_goal(0.2, goal_y, height + 0.05, 0.0, 0.0, 0.0, timeout=20, pre_grasp=False, frame_id="/amigo/base_link"):
            print "Failed after-drop"
            return

        # Close gripper
        arm.send_gripper_goal('close')

        arm.reset()

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

    place = Place()
    server.register_skill("place", place)

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
