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
import geometry_msgs.msg as gm
import math
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

class PlaceDesignator(Designator):
    def __init__(self, robot=None, goal_entity=None):
        super(PlaceDesignator, self).__init__(resolve_type=gm.PoseStamped)

        self._robot = robot
        self._goal_entity = goal_entity
        self._eps = 0.05 # Threshold for navigation constraint
        self._edge_distance = 0.1

    def resolve(self):

        # Get entity
        e = self._robot.ed.get_entity(self._goal_entity, parse=False)
        if not e:
            rospy.logerr("No such entity")
            return None

        ''' First, check if a place pose is defined in the entity model'''

        ''' If not: derive one '''
        place_pose = self.derivePlacePose(e)
        rospy.logdebug("Place pose = {0}".format(place_pose))

        return place_pose

    def derivePlacePose(self, e):

        # Determine radius
        grasp_offset = self._robot.grasp_offset
        radius = math.hypot(grasp_offset.x, grasp_offset.y)

        # Get plan to constraint
        ch = e.convex_hull

        x = e.pose.position.x
        y = e.pose.position.y

        if len(ch) > 0:

            ch.append(ch[0])

            pci = ""

            for i in xrange(len(ch) - 1):
                dx = ch[i+1].x - ch[i].x
                dy = ch[i+1].y - ch[i].y

                length = math.hypot(dx, dy)

                pci_cur = "("

                # Parallel to the polygon
                xs = ch[i].x + (dy/length)*(radius+self._eps)
                ys = ch[i].y - (dx/length)*(radius+self._eps)
                pci_cur = pci_cur + "-(x-%f)*%f+(y-%f)*%f > 0.0 "%(xs, dy, ys, dx)

                xs = ch[i].x + (dy/length)*(radius-self._eps)
                ys = ch[i].y - (dx/length)*(radius-self._eps)
                pci_cur = pci_cur + ' and ' + "-(x-%f)*%f+(y-%f)*%f < 0.0 "%(xs, dy, ys, dx)

                # Orthogonal to the polygon
                pci_cur = pci_cur + ' and ' + "(y-%f)*%f+(x-%f)*%f > %f "%(ch[i].y, dy, ch[i].x, dx, self._edge_distance)
                pci_cur = pci_cur + ' and ' + "(y-%f)*%f+(x-%f)*%f < -%f "%(ch[i+1].y, dy, ch[i+1].x, dx, self._edge_distance)

                pci_cur = pci_cur + ")"

                if i != 0:
                    pci = pci + ' or '
                pci = pci + pci_cur

        else:
            ro = "(x-%f)^2+(y-%f)^2 < %f^2"%(x, y, radius+0.05)
            ri = "(x-%f)^2+(y-%f)^2 > %f^2"%(x, y, radius-0.05)
            pci = ri+" and "+ro

        pc = PositionConstraint(constraint=pci, frame="/map")
        #oc = OrientationConstraint(look_at=Point(x, y, 0.0), frame="/map")
        
        plan = self._robot.base.global_planner.getPlan(pc)
        base_position = plan[-1].pose.position

        #return pc, oc

        ''' For all segments of the polyon, compute the distance to the computed base pose
            Go along the line PERPENDICULAR to the segment, for distance + offset to edge. 
            This is your pose '''
        x = base_position.x
        y = base_position.y

        # ToDo: additional check to make sure we don't use the wrong edge
        poses  = []
        for i in xrange(len(ch) - 1):
                dx = ch[i+1].x - ch[i].x
                dy = ch[i+1].y - ch[i].y

                length = math.hypot(dx, dy)
                distance = math.fabs(dy*x - dx*y + ch[i+1].x*ch[i].y - ch[i+1].y*ch[i].x)/length

                ''' Possible constraints are within an threshold from the originally computed radius '''
                if (distance > radius - self._eps and distance < radius + self._eps):
                    place_pose = msgs.PoseStamped()
                    place_pose.header.frame_id = "/map"
                    place_pose.pose.position.x = base_position.x - dy/length * (distance+self._edge_distance)
                    place_pose.pose.position.y = base_position.y + dx/length * (distance+self._edge_distance)
                    place_pose.pose.position.z = e.z_max   
                    ''' Compute the distance of the place pose to the entity center '''
                    place_pose_to_center = math.hypot((place_pose.pose.position.x - e.pose.position.x), (place_pose.pose.position.x - e.pose.position.x))
                    poses  += [(place_pose, place_pose_to_center)]

        ''' Usually, there are one or two distances within the threshold. Typically, the one closest to the center is correct'''
        best = min(poses, key=lambda pose: pose[1])
        place_pose, place_pose_to_center = best

        return place_pose


class Put:

    def __init__(self):
        self.place = None
        self.thread = None
        self.goal_entity = None

    def create_action(self, action_type, config, robot):
        print "Place!"

        try:
            self.goal_entity = config["entity"]
        except KeyError:
            print "No object given"
            return False

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

        item_to_place = Designator(arm.occupied_by, resolve_type=ed.msg.EntityInfo)  # Which item do we want to place? The object in the hand we indicated
        arm_with_item_designator = Designator(arm,  resolve_type=Arm) #No need for ArmHoldingEntityDesignator, we already know that from the config
        place_position = PlaceDesignator(robot, self.goal_entity)
        
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
