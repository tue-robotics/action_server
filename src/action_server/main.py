#!/usr/bin/env python

'''
A central place to dispatch actions received from various sources. These sources could come from
GUIs, command line tools or natural language.
'''

import rospy
import action_server.srv

import yaml
import time
import threading
import sys
import uuid

import robot_skills.util.msg_constructors as msgs
from robot_skills.util import transformations

from robot_smach_states.navigation import NavigateToObserve, NavigateToWaypoint, NavigateToGrasp
from robot_smach_states.manipulation import Grab, Place

from cb_planner_msgs_srvs.msg import PositionConstraint, OrientationConstraint
from robot_smach_states.util.designators import Designator, ArmDesignator, UnoccupiedArmDesignator, EdEntityDesignator, ArmHoldingEntityDesignator

import ed.msg
from robot_skills.arms import Arm
from robot_skills.arms import ArmState
import geometry_msgs.msg as gm
import math


def length_sq(x, y):
    return x * x + y * y


def entities_from_description(entity_descr, robot):
    '''
    Query entities with various methods

    @param entity_descr: A dict that contains an 'id' or 'type' field
    @param robot: The robot object

    @return: (entities, error_msg)
        entities  - list of entities that fulfill the description
                    (each element has type EntityInfo)
        error_msg - If something goes wrong, this contains the message
    '''
    if "id" in entity_descr:
        e = robot.ed.get_entity(id=entity_descr["id"], parse=False)
        if not e:
            return ([], "No entity with id '%s'" % entity_descr["id"])
        entities = [e]
    elif "type" in entity_descr:
        entities = robot.ed.get_entities(type=entity_descr["type"], parse=False)
    else:
        entities = robot.ed.get_entities(parse=False)

    if not entities:
        return ([], "No such entity")

    robot_pos = robot.base.get_location().pose.position

    # Sort entities by distance
    entities = sorted(entities,
                      key=lambda entity: length_sq(
                          robot_pos.x - entity.pose.position.x,
                          robot_pos.y - entity.pose.position.y))

    print entities

    return (entities, "")


class PickUp(object):

    def __init__(self):
        self.grab = None
        self.thread = None
        self.arm = None

    def start(self, config, robot):


        self._robot = robot

        if not "entity" in config:
            return "No entity given"

        entity_descr = config["entity"]
        (entities, error_msg) = entities_from_description(entity_descr, robot)
        if not entities:
            return error_msg

        # Only filter to entities that do not have a shape but do have a convex hull
        entities = [e for e in entities if not e.has_shape and len(e.convex_hull) > 0]

        if not entities:
            return "Inpossible to grab that object"

        e = entities[0]
        self.entity_id = e.id

        self.side = config['side'] if 'side' in config else 'right'

        if self.side == 'left':
            self.arm = robot.leftArm
        else:
            self.arm = robot.rightArm

        self.thread = threading.Thread(name='grab', target=self.execute)

        self.thread.start()

    def execute(self):

        self.ntg = NavigateToGrasp(self._robot, EdEntityDesignator(self._robot, id=self.entity_id), ArmDesignator(robot.arms, robot.arms[self.side]))
        self.ntg.execute()

        arm = self.arm

        # goal in map frame
        goal_map = msgs.Point(0, 0, 0)

        # Transform to base link frame
        goal_bl = transformations.tf_transform(goal_map, self.entity_id, "/amigo/base_link", tf_listener=self._robot.tf_listener)
        if goal_bl == None:
            return 'failed'

        print goal_bl

        # Arm to position in a safe way
        arm._send_joint_trajectory([
            [-0.1,-0.6,0.1,1.2,0.0,0.1,0.0],
            [-0.1,-0.8,0.1,1.6,0.0,0.2,0.0],
            [-0.1,-1.0,0.1,2.0,0.0,0.3,0.0],
            [-0.1,-0.5,0.1,2.0,0.0,0.3,0.0],
            ])#, timeout=20)

        # Open gripper
        arm.send_gripper_goal(ArmState.OPEN, timeout=5)

        # Pre-grasp
        if not arm.send_goal(goal_bl.x, goal_bl.y, goal_bl.z, 0, 0, 0, frame_id="/amigo/base_link", timeout=20, pre_grasp=True, first_joint_pos_only=True):
            print "Pre-grasp failed"
            arm.reset()
            arm.send_gripper_goal(ArmState.CLOSE, timeout=0.01)
            return

        # Grasp
        if not arm.send_goal(goal_bl.x, goal_bl.y, goal_bl.z, 0, 0, 0, frame_id="/amigo/base_link", timeout=120, pre_grasp = True):
            self._robot.speech.speak("I am sorry but I cannot move my arm to the object position", block=False)
            print "Grasp failed"
            arm.reset()
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
        if self.side == "left":
            y_home = 0.2
        else:
            y_home = -0.2

        print "y_home = " + str(y_home)
        
        rospy.loginfo("start moving to carrying pose")        
        if not arm.send_goal(0.18, y_home, goal_bl.z + 0.1, 0, 0, 0, 60):            
            print 'Failed carrying pose'      

    def cancel(self):
        if not self.grab:
            return

        if self.grab.is_running:
            self.grab.request_preempt()

        # Wait until canceled
        self.thread.join()


class Inspect(object):

    def __init__(self):
        self.nwc = None

    def start(self, config, robot):

        if "entity" not in config:
            return "No entity given"

        entity_descr = config["entity"]
        (entities, error_msg) = entities_from_description(entity_descr, robot)
        if not entities:
            return error_msg

        self.entity = entities[0]

        self.robot = robot

        self.thread = threading.Thread(name='inspect', target=self._run)
        self.thread.start()

    def _run(self):
        # Navigate to the location
        self.nwc = NavigateToObserve(robot,
                                     entity_designator=EdEntityDesignator(
                                         robot,
                                         id=self.entity.id),
                                     radius=1.0)
        self.nwc.execute()

        # Make sure the head looks at the entity
        pos = self.entity.pose.position
        self.robot.head.look_at_point(msgs.PointStamped(pos.x, pos.y, 0.8, "/map"), timeout=10)

        # This is needed because the head is not entirely still when the
        # look_at_point function finishes
        time.sleep(1)

        # Inspect 'on top of' the entity
        segm_res = self.robot.ed.update_kinect("on_top_of %s" % self.entity.id)

        # Cancel the head goal
        self.robot.head.cancel_goal()

        # Classify
        ids = list(set(segm_res.new_ids) | set(segm_res.updated_ids))
        print self.robot.ed.classify(ids=ids)

    def cancel(self):
        if self.nwc and self.nwc.is_running:
            self.nwc.request_preempt()

        # Wait until canceled
        self.thread.join()


class PlaceDesignator(Designator):
    def __init__(self, robot=None, goal_entity=None):
        super(PlaceDesignator, self).__init__(resolve_type=gm.PoseStamped)

        self._robot = robot
        self._goal_entity = goal_entity
        self._eps = 0.05  # Threshold for navigation constraint
        self._edge_distance = 0.1

    def resolve(self):

        # Get entity
        e = self._robot.ed.get_entity(self._goal_entity, parse=False)
        if not e:
            rospy.logerr("No such entity")
            return None

        # First, check if a place pose is defined in the entity model
        # If not: derive one
        place_pose = self.derive_place_pose(e)
        rospy.logdebug("Place pose = {0}".format(place_pose))

        return place_pose

    def derive_place_pose(self, e):

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
                dx = ch[i + 1].x - ch[i].x
                dy = ch[i + 1].y - ch[i].y

                length = math.hypot(dx, dy)

                pci_cur = "("

                # Parallel to the polygon
                xs = ch[i].x + (dy / length) * (radius + self._eps)
                ys = ch[i].y - (dx / length) * (radius + self._eps)
                pci_cur = pci_cur + "-(x-%f)*%f+(y-%f)*%f > 0.0 " % (xs, dy, ys, dx)

                xs = ch[i].x + (dy / length) * (radius - self._eps)
                ys = ch[i].y - (dx / length) * (radius - self._eps)
                pci_cur = pci_cur + ' and ' + "-(x-%f)*%f+(y-%f)*%f < 0.0 " % (xs, dy, ys, dx)

                # Orthogonal to the polygon
                pci_cur = pci_cur + ' and ' + "(y-%f)*%f+(x-%f)*%f > %f " % (ch[i].y, dy, ch[i].x, dx, self._edge_distance)
                pci_cur = pci_cur + ' and ' + "(y-%f)*%f+(x-%f)*%f < -%f " % (ch[i + 1].y, dy, ch[i + 1].x, dx, self._edge_distance)

                pci_cur = pci_cur + ")"

                if i != 0:
                    pci = pci + ' or '
                pci = pci + pci_cur

        else:
            ro = "(x-%f)^2+(y-%f)^2 < %f^2" % (x, y, radius + 0.05)
            ri = "(x-%f)^2+(y-%f)^2 > %f^2" % (x, y, radius - 0.05)
            pci = ri + " and " + ro

        pc = PositionConstraint(constraint=pci, frame="/map")
        # oc = OrientationConstraint(look_at=Point(x, y, 0.0), frame="/map")

        plan = self._robot.base.global_planner.getPlan(pc)
        base_position = plan[-1].pose.position

        # return pc, oc

        '''
        For all segments of the polyon, compute the distance to the computed base pose
        Go along the line PERPENDICULAR to the segment, for distance + offset to edge.
        This is your pose
        '''
        x = base_position.x
        y = base_position.y

        # ToDo: additional check to make sure we don't use the wrong edge
        poses = []
        for i in xrange(len(ch) - 1):
            dx = ch[i + 1].x - ch[i].x
            dy = ch[i + 1].y - ch[i].y

            length = math.hypot(dx, dy)
            distance = math.fabs(dy * x - dx * y + ch[i + 1].x * ch[i].y - ch[i + 1].y * ch[i].x) / length

            '''
            Possible constraints are within an threshold from the originally computed radius
            '''
            if distance > radius - self._eps and distance < radius + self._eps:
                place_pose = msgs.PoseStamped()
                place_pose.header.frame_id = "/map"
                place_pose.pose.position.x = base_position.x - dy / length * (distance + self._edge_distance)
                place_pose.pose.position.y = base_position.y + dx / length * (distance + self._edge_distance)
                place_pose.pose.position.z = e.z_max
                '''
                Compute the distance of the place pose to the entity center
                '''
                place_pose_to_center = math.hypot(
                    (place_pose.pose.position.x - e.pose.position.x),
                    (place_pose.pose.position.x - e.pose.position.x))
                poses += [(place_pose, place_pose_to_center)]

        '''
        Usually, there are one or two distances within the threshold. Typically, the one closest to
        the center is correct
        '''
        best = min(poses, key=lambda pose: pose[1])
        place_pose, place_pose_to_center = best

        return place_pose


class Put(object):

    def __init__(self):
        self.place = None
        self.thread = None
        self.goal_entity = None

    def start(self, config, robot):
        print "Place!"

        try:
            self.goal_entity = config["entity"]
        except KeyError:
            print "No object given"
            return False

        try:
            side = config["side"]
        except KeyError:
            side = "right"  # Default

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
        arm_with_item_designator = Designator(arm, resolve_type=Arm)  # No need for ArmHoldingEntityDesignator, we already know that from the config
        place_position = PlaceDesignator(robot, self.goal_entity)

        self.place = Place(robot, item_to_place, place_position, arm_with_item_designator)

        self.thread = threading.Thread(name='grab', target=self.place.execute)
        self.thread.start()

    def cancel(self):
        if self.place.is_running:
            self.place.request_preempt()

        # Wait until canceled
        self.thread.join()


class NavigateTo(object):

    def __init__(self):
        self.nwc = None

    def start(self, config, robot):

        if "entity" not in config:
            return "No entity given"

        entity_descr = config["entity"]
        (entities, error_msg) = entities_from_description(entity_descr, robot)
        if not entities:
            return error_msg

        e = entities[0]

        if "waypoint" in e.types:
            self.nwc = NavigateToWaypoint(robot,
                                          waypoint_designator=EdEntityDesignator(robot, id=e.id),
                                          radius=0.1)
            rospy.logwarn("ACTION_SERVER: Navigating to waypoint")
        else:
            self.nwc = NavigateToObserve(robot,
                                         entity_designator=EdEntityDesignator(robot, id=e.id),
                                         radius=.5)
            rospy.logwarn("ACTION_SERVER: Navigating to observe")

        self.thread = threading.Thread(name='navigate', target=self.nwc.execute)
        self.thread.start()

    def cancel(self):
        if self.nwc.is_running:
            self.nwc.request_preempt()

        # Wait until canceled
        self.thread.join()


class Server(object):

    def __init__(self, name, robot):  # Robot should no be in the constructor but should be in the add_action -- REIN
        self.name = name
        self.action_type_to_skill = {}
        self.action = None
        self.robot = robot  # Should be in the add action, not in the constructor -- REIN
        self.cl_register = None

        self.add_action_service_name = self.name + '/add_action'
        self.get_action_status_service_name = self.name + '/get_action_status'
        self.srv_add_action = rospy.Service(self.add_action_service_name,
                                            action_server.srv.AddAction,
                                            self.add_action_cb)

    def register_skill(self, action_type, skill):
        self.action_type_to_skill[action_type] = skill

    def connect(self, register_service_name):
        rospy.loginfo("Waiting for registration service ('%s') ..." % register_service_name)
        rospy.wait_for_service(register_service_name)
        rospy.loginfo("...found")

        self.cl_register = rospy.ServiceProxy(register_service_name,
                                              action_server.srv.RegisterActionServer)

        try:
            resp = self.cl_register(self.add_action_service_name,
                                    self.get_action_status_service_name,
                                    [k for k in self.action_type_to_skill.iterkeys()])
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return False

        if resp.error_msg:
            rospy.logerr("Response from action server: " + resp.error_msg)
            return False

        return True

    def add_action_cb(self, req):
        try:
            action_class = self.action_type_to_skill[req.action]
        except KeyError:
            return action_server.srv.AddActionResponse("", "Action type '%s' not found." % req.action)

        config = yaml.load(req.parameters)

        if self.action:
            self.action.cancel()

        action = action_class()

        try:
            err = action.start(config, robot)
            if not err:
                self.action = action
                id = str(uuid.uuid1())
                return action_server.srv.AddActionResponse(id, "")
            else:
                return action_server.srv.AddActionResponse("", "Could not construct action: %s" % err)
        except Exception as err:
            error_msg = "%s\n\n" % err
            import traceback
            error_msg += traceback.format_exc()
            return action_server.srv.AddActionResponse("", "Error while starting action: %s" % error_msg)


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

    # Register components
    server.register_skill("pick-up", PickUp)
    # The original state from robot_smach_states is also called Place
    # so there we need a different name
    server.register_skill("place", Put)
    server.register_skill("navigate-to", NavigateTo)
    server.register_skill("inspect", Inspect)

    server.connect('action_server/register_action_server')

    rospy.spin()
