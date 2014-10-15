#!/usr/bin/env python

import rospy
import action_server.srv

import yaml

global server

# -------------------------------------
# For PickUp:
from robot_skills.amigo import Amigo
from robot_skills.arms import State as ArmState
from robot_skills.util import transformations
import robot_skills.util.msg_constructors as msgs
#import robot_smach_states
# -------------------------------------

class PickUp:

    def __init__(self):
        self._robot = Amigo(dontInclude = ['head', 'base', 'base2', 'perception', 'ebutton', 'lights', 'reasoner'], wait_services=False)  

    def create_action(self, action_type, config):
        try:
            entity_id = config["entity"]
        except KeyError:
            print "No object given"
            return False

        try:
            side = config["side"]
        except KeyError:
            side = "left" # Default

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
        if not arm.send_goal(0.18, y_home, 0.75, 0, 0, 0, 60):            
            print 'Failed carrying pose'                 


        #machine = robot_smach_states.manipulation.GrabMachineWithoutBase(side=side, robot=self._robot, grabpoint_query=query)
        #machine.execute()

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


if __name__ == "__main__":
    node_name = 'action_server_py'
    rospy.init_node(node_name)

    add_action_service_name = '/' + node_name + '/add_action'
    srv = rospy.Service(add_action_service_name, action_server.srv.AddAction, srv_add_action)

    global server
    server = Server()

    # Register components
    pick_up = PickUp()
    server.register_skill("pick_up", pick_up)

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
