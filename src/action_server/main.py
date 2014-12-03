#!/usr/bin/env python

import rospy
import action_server.srv

import yaml

global server

# -------------------------------------
# For PickUp:
from robot_skills.amigo import Amigo
from robot_skills.util import transformations
import robot_skills.util.msg_constructors as msgs

from robot_smach_states.navigation import NavigateToObserve
from cb_planner_msgs_srvs.msg import PositionConstraint, OrientationConstraint
# -------------------------------------

# ----------------------------------------------------------------------------------------------------

class PickUp:

    def __init__(self, robot):
        self._robot = robot

    def create_action(self, action_type, config):
        print "PickUp!"

        try:
            entity_id = config["entity"]
        except KeyError:
            print "No object given"
            return False

        try:
            side = config["side"]
        except KeyError:
            side = "right" # Default

        if side == "left":
            arm = self._robot.leftArm
        else:
            arm = self._robot.rightArm

        # goal in map frame
        goal_map = msgs.Point(0, 0, 0)

        # Transform to base link frame
        goal_bl = transformations.tf_transform(goal_map, entity_id, "/amigo/base_link", tf_listener=self._robot.tf_listener)
        if goal_bl == None:
            return 'failed'

        print goal_bl

        # Arm to position in a safe way
        arm.send_joint_trajectory([
            [-0.1,-0.6,0.1,1.2,0.0,0.1,0.0],
            [-0.1,-0.8,0.1,1.6,0.0,0.2,0.0],
            [-0.1,-1.0,0.1,2.0,0.0,0.3,0.0],
            [-0.1,-0.5,0.1,2.0,0.0,0.3,0.0],
            ], timeout=20)

        # Open gripper
        arm.send_gripper_goal(ArmState.OPEN, timeout=5)

        # Pre-grasp
        if not arm.send_goal(goal_bl.x, goal_bl.y, goal_bl.z, 0, 0, 0, frame_id="/amigo/base_link", timeout=20, pre_grasp=True, first_joint_pos_only=True):
            print "Pre-grasp failed"
            arm.reset_arm()
            arm.send_gripper_goal(ArmState.CLOSE, timeout=0.01)
            return

        # Grasp
        if not arm.send_goal(goal_bl.x, goal_bl.y, goal_bl.z, 0, 0, 0, frame_id="/amigo/base_link", timeout=120, pre_grasp = True):
            self._robot.speech.speak("I am sorry but I cannot move my arm to the object position", block=False)
            print "Grasp failed"
            arm.reset_arm()
            arm.send_gripper_goal(ArmState.CLOSE, timeout=0.01)
            return

        # Close gripper
        arm.send_gripper_goal(ArmState.CLOSE, timeout=5)

        # Lift
        if not arm.send_goal( goal_bl.x, goal_bl.y, goal_bl.z + 0.1, 0.0, 0.0, 0.0, timeout=20, pre_grasp=False, frame_id="/amigo/base_link"):
            print "Failed lift"

        # Retract
        if not arm.send_goal( goal_bl.x - 0.1, goal_bl.y, goal_bl.z + 0.1, 0.0, 0.0, 0.0, timeout=20, pre_grasp=False, frame_id="/amigo/base_link"):
            print "Failed retract"

        # Carrying pose
        if side == "left":
            y_home = 0.2
        else:
            y_home = -0.2

        print "y_home = " + str(y_home)
        
        rospy.loginfo("start moving to carrying pose")        
        if not arm.send_goal(0.18, y_home, goal_bl.z + 0.1, 0, 0, 0, 60):            
            print 'Failed carrying pose'                 


        #machine = robot_smach_states.manipulation.GrabMachineWithoutBase(side=side, robot=self._robot, grabpoint_query=query)
        #machine.execute()

# ----------------------------------------------------------------------------------------------------

class Place:

    def __init__(self, robot):
        self._robot = robot

    def create_action(self, action_type, config):
        print "Place!"

        try:
            side = config["side"]
        except KeyError:
            side = "right" # Default

        if side == "left":
            arm = self._robot.leftArm
        else:
            arm = self._robot.rightArm

        try:
            height = config["height"]
        except KeyError:
            height = 0.8

        # Torso to highest position
        self._robot.spindle.high()

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
        arm.send_gripper_goal(ArmState.OPEN, timeout=5)

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
        arm.send_gripper_goal(ArmState.CLOSE, timeout=5)

        arm.reset_arm()

# ----------------------------------------------------------------------------------------------------

class NavigateTo:

    def __init__(self, robot):
        self._robot = robot

    def create_action(self, action_type, config):
        print "NavigateTo!"

        try:
            entity_id = config["entity"]
        except KeyError:
            print "No object given"
            return False

        nwc = NavigateToObserve(self._robot, entity_id=entity_id, radius=1)
        nwc.execute()

# ----------------------------------------------------------------------------------------------------

class Server:

    def __init__(self):
        self.action_type_to_skill = {}

    def register_skill(self, action_type, skill):
        self.action_type_to_skill[action_type] = skill
    
    def add_action(self, action_type, config):
        try:
            self.action_type_to_skill[action_type].create_action(action_type, config)
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

    global server
    server = Server()

    # Create AMIGO object
    amigo = Amigo(dontInclude = ['head', 'base', 'base2', 'perception', 'ebutton', 'lights', 'reasoner'], wait_services=False)  

    # Register components
    pick_up = PickUp(amigo)
    server.register_skill("pick-up", pick_up)

    place = Place(amigo)
    server.register_skill("place", place)

    navigate_to = NavigateTo(amigo)
    server.register_skill("navigate-to", navigate_to)

    # Register this server at the main (c++) action server
    print "Waiting for connection with '/action_server/register_action_server'..."
    rospy.wait_for_service('/action_server/register_action_server')
    print "... Connected."  

    try:
        register_client = rospy.ServiceProxy('/action_server/register_action_server', action_server.srv.RegisterActionServer)
        resp = register_client(add_action_service_name, '/' + node_name + '/get_action_status')

        if resp.error_msg:
            print "ERROR: " + resp.error_msg
            sys.exit(1)            
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    rospy.spin()
