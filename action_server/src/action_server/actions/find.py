from action import Action, ConfigurationData

from entity_description import resolve_entity_description, EntityDescription
from util import entities_from_description

import rospy
import math

import robot_smach_states as states
from robot_smach_states.util.designators import EdEntityDesignator, VariableDesignator
from robot_skills.util.entity import Entity

import robot_skills.util.kdl_conversions as kdl


class Find(Action):
    """ The Find class implements the action to find an object or person at a specified location.

    Parameters to pass to the configure() method are:
     - `location` (required): location to find the object (room or pre-existing world model entity)
     - `object` (required): the object to find.
    """
    def __init__(self):
        Action.__init__(self)
        # TODO: change this to a python dictionary schema
        self._required_field_prompts = {'object': " What exactly would you like me to find? "}
        self._required_skills = ['head', 'base', 'rightArm', 'speech']

    def _point_at_person(self, person):
        pose_base_link_kdl = person.pose.projectToFrame(self._robot.robot_name + '/base_link',
                                                        self._robot.tf_listener)
        pose_base_link = kdl.kdl_frame_stamped_to_pose_stamped_msg(pose_base_link_kdl)

        x = pose_base_link.pose.position.x
        y = pose_base_link.pose.position.y

        th = math.atan2(y, x)
        vth = 0.5

        self._robot.head.cancel_goal()
        self._robot.base.force_drive(0, 0, math.copysign(1, th) * vth, abs(th / vth))

        self._robot.speech.speak("Found you!")
        self._robot.speech.speak("I will point at you now.")

        self._robot.head.look_at_ground_in_front_of_robot(distance=100)
        self._robot.rightArm._send_joint_trajectory([[-0.2, -0.5, 0.2, 2.0, 0, 0.1, 0.0]])

        self._robot.speech.speak("You're right there!")

        self._robot.rightArm.send_joint_goal('reset')

    def _configure(self, robot, config):
        self._robot = robot
        self._object = resolve_entity_description(config.semantics['object'])
        self._location = None

        if 'source-location' in config.semantics:
            self._location = resolve_entity_description(config.semantics['source-location'])
        elif 'location-designator' in config.context:
            e = config.context['location-designator'].resolve()
            if not e:
                if self._object.id:
                    self._config_result.message = " Where should I look for the {}?".format(self._object.id)
                else:
                    self._config_result.message = " Where should I look for the {}?".format(self._object.type)
                self._config_result.missing_field = 'source-location'
                return
            self._location = EntityDescription(id=e.id)
        else:
            self._config_result.message = " Where should I look for the {}?".format(self._object.id)
            self._config_result.missing_field = 'source-location'
            return

        # We (safely) assume that self._location is an EntityDescription object
        # If we need to find a manipulable item, the location should also be manipulable
        if not self._object.type == "person" and self._location.id not in self._knowledge.manipulation_locations \
            and self._location.id not in self._knowledge.location_rooms:
            self._config_result.message = " I can't grasp anything from the {}".format(self._location.id)
            return

        # Set up designator for areas
        self._areas = {}
        self._nav_areas = {}
        if self._location.id in self._knowledge.location_rooms:
            if not self._object.type == "person":
                locations = self._knowledge.get_locations(self._location.id)
                for location in locations:
                    self._areas[location] = self._knowledge.get_inspect_areas(location)
                    self._nav_areas[location] = self._knowledge.get_inspect_position(location)
            else:
                self._areas[self._location.id] = ["in"]
                self._nav_areas[self._location.id] = "in"
        elif self._object.type == "person":
            self._areas[self._location.id] = ["near"]
            self._nav_areas[self._location.id] = "near"
        else:
            self._areas[self._location.id] = self._knowledge.get_inspect_areas(self._location.id)
            self._nav_areas[self._location.id] = self._knowledge.get_inspect_position(self._location.id)

        # Set up the designator with the object description
        entity_description = {'type': self._object.type}
        description_designator = VariableDesignator(entity_description)

        # Set up designator to be filled with the found entity
        self._found_entity_designator = VariableDesignator(resolve_type=Entity)

        self._find_state_machines = []
        for loc, areas in self._areas.iteritems():
            location_designator = EdEntityDesignator(self._robot, id=loc)
            nav_area = self._nav_areas[loc]
            for area in areas:
                area_designator = VariableDesignator(area)

                navigation_area_designator = VariableDesignator(nav_area)

                # Set up the Find state machine
                rospy.loginfo("Setting up state machine with loc = {}, area = {}, nav_area = {}".format(loc, area, nav_area))
                self._find_state_machines.append(states.Find(robot=self._robot,
                                                             source_entity_designator=location_designator,
                                                             description_designator=description_designator,
                                                             area_name_designator=area_designator,
                                                             navigation_area_designator=navigation_area_designator,
                                                             found_entity_designator=self._found_entity_designator))

        self._config_result.context['object-designator'] = self._found_entity_designator
        self._config_result.context['location-designator'] = location_designator
        self._config_result.succeeded = True

    def _start(self):
        # e = self._found_entity_designator.resolve()
        # if e:
        #     self._execute_result.message = " I already knew where to find {}. ".format(
        #         self._object.id if self._object.id else "a " + self._object.type)
        #     self._execute_result.succeeded = True
        #     return

        for fsm in self._find_state_machines:
            res = fsm.execute()

            if res == 'succeeded':
                self._execute_result.message = " I found {}. ".format(
                    self._object.id if self._object.id else "a " + self._object.type)
                self._execute_result.succeeded = True

                if self._object.type == "person":
                    self._point_at_person(self._found_entity_designator.resolve())
                else:
                    self._robot.speech.speak("Hey, I found a {}!".format(self._object.type))
                return
            elif res == 'not_found':
                if self._object.type == "person":
                    self._robot.speech.speak(" I don't see anyone here. ")
                else:
                    self._robot.speech.speak("I don't see what I am looking for here.")

                self._execute_result.message = " I couldn't find {} {} the {} ".format(
                    self._object.id if self._object.id and not self._object.id == "None" else \
                        "a " + self._object.type if self._object.type else "a " + self._object.category,
                    "in" if self._location.id in self._knowledge.location_rooms else "at",
                    self._location.id
                )
            else:
                self._robot.speech.speak(" I'm unable to inspect the {} ".format(self._location.id))
                self._execute_result.message = " I was unable to inspect the {} to find {}. ".format(
                    self._location.id,
                    self._object.id if self._object.id else "a " + self._object.type if self._object.type else \
                        "a " + self._object.category
                )

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

    config = ConfigurationData({'action': 'find',
              'source-location': {'id': 'cabinet',
                           'area': 'on_top_of'},
              'object': {'type': 'coke'}})

    action.configure(robot, config)
    action.start()

    config = ConfigurationData({'action': 'find',
              'source-location': {'id': 'livingroom',
                           'area': 'in'},
              'object': {'type': 'person'}})

    action.configure(robot, config)
    action.start()
