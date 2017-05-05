from action import Action
from entity_description import resolve_entity_description

import rospy
import math

import robot_smach_states as states
from robot_smach_states.util.designators import EdEntityDesignator, VariableDesignator
from robot_skills.util.entity import Entity

import robot_skills.util.kdl_conversions as kdl


class Find(Action):
    def __init__(self):
        Action.__init__(self)
        # TODO: change this to a python dictionary schema
        self._required_field_prompts = {'object': " I didn't get what you want me to find. ",
                                        'location': " I didn't get where I should look. "}

    def _point_at_person(self, person):
        person_pose = kdl.poseMsgToKdlFrame(person.pose)
        person_pose_stamped = kdl.FrameStamped(person_pose, 'map')
        pose_base_link_kdl = person_pose_stamped.projectToFrame(self._robot.robot_name + '/base_link',
                                                                self._robot.tf_listener)
        pose_base_link = kdl.kdlFrameStampedToPoseStampedMsg(pose_base_link_kdl)

        x = pose_base_link.pose.position.x
        y = pose_base_link.pose.position.y

        th = math.atan2(y, x)
        vth = 0.5

        self._robot.head.cancel_goal()
        self._robot.base.force_drive(0, 0, math.copysign(1, th) * vth, abs(th / vth))

        self._robot.speech.speak("Found you!")
        self._robot.speech.speak("I will point at you now.")

        self._robot.head.look_at_ground_in_front_of_robot(distance=100)
        self._robot.rightArm._send_joint_trajectory([[0,1.0,0.3,0.8,0,0,0]])

        self._robot.speech.speak("You're right there!")

        self._robot.rightArm.send_joint_goal('reset')

    def _configure(self, robot, config):
        self._robot = robot
        self._object = resolve_entity_description(config['object'])
        self._location = resolve_entity_description(config['location'])

        self._location_designator = EdEntityDesignator(self._robot, id=self._location.id)

        # If we need to find a manipulable item, the location should also be manipulable
        if not self._object.type == "person" and self._location.id not in self._knowledge.manipulation_locations:
            rospy.logwarn("Not going to look for a {} at the {}".format(self._object.type, self._location.id))
            self._config_result.message = " I don't think it makes sense to look for a {} at the {}. ".format(
                self._object.type, self._location.id)
            return

        # Set up designator for area
        if self._location.id in self._knowledge.location_rooms:
            self._area = "in"
            self._nav_area = self._area
        elif self._object.type == "person":
            self._area = "near"
            self._nav_area = self._area
        else:
            # TODO: inspect other areas of the same object
            self._nav_area = "in_front_of"
            self._area = "on_top_of"

        print "area = {}".format(self._area)
        self._area_designator = VariableDesignator(self._area)

        # Set up the designator with the object description
        self._description_designator = VariableDesignator({'type': self._object.type})

        # Set up designator to be filled with the found entity
        self._found_entity_designator = VariableDesignator(resolve_type=Entity)

        self._navigation_area_designator = VariableDesignator(self._nav_area)

        # Set the resulting knowledge of the Find action to this found object designator
        self._config_result.resulting_knowledge['found_entity'] = self._found_entity_designator

        # Set up the Find state machine
        self._fsm = states.Find(robot=self._robot,
                                source_entity_designator=self._location_designator,
                                description_designator=self._description_designator,
                                area_name_designator=self._area_designator,
                                navigation_area_designator=self._navigation_area_designator,
                                found_entity_designator=self._found_entity_designator)

        self._config_result.succeeded = True

    def _start(self):
        res = self._fsm.execute()

        if res == 'succeeded':
            self._execute_result.message = " I found what you wanted me to find. "
            self._execute_result.succeeded = True

            if self._object.type == "person":
                self._point_at_person(self._found_entity_designator.resolve())
            else:
                self._robot.speech.speak("Hey, I found a {}!".format(self._object.type))
            return
        else:
            if self._object.type == "person":
                self._robot.speech.speak(" I don't see anyone here. ")
            else:
                self._robot.speech.speak("I don't see what I am looking for here.")
            self._execute_result.message = " I couldn't find what you wanted me to find. "

    def _cancel(self):
        pass


if __name__ == "__main__":
    rospy.init_node('find_test')

    import sys

    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = Find()

    config = {'action': 'find',
              'location': {'id': 'cabinet',
                           'area': 'on_top_of'},
              'object': {'type': 'coke'}}

    action.configure(robot, config)
    action.start()

    config = {'action': 'find',
              'location': {'id': 'livingroom',
                           'area': 'in'},
              'object': {'type': 'person'}}

    action.configure(robot, config)
    action.start()
