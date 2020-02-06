from action import Action, ConfigurationData

from entity_description import resolve_entity_description, EntityDescription

import rospy
import math

import robot_smach_states as rss
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

        self._robot.speech.speak("I will point at you now.")

        self._robot.head.look_at_ground_in_front_of_robot(distance=100)
        self._robot.rightArm._send_joint_trajectory([[-0.2, -0.5, 0.2, 2.0, 0, 0.1, 0.0]])

        self._robot.speech.speak("You're right there!")

        self._robot.rightArm.send_joint_goal('reset')

    class Semantics:
        def __init__(self):
            self.object = None
            self.source_location = None

        def __repr__(self):
            return str(self.__dict__)

    @staticmethod
    def _parse_semantics(semantics_dict):
        semantics = Find.Semantics()

        semantics.object = resolve_entity_description(semantics_dict['object'])

        if 'source-location' in semantics_dict:
            semantics.source_location = resolve_entity_description(semantics_dict['source-location'])

        return semantics

    class Context:
        def __init__(self):
            self.location = None

    @staticmethod
    def _parse_context(context_dict):
        context = Find.Context()

        if 'location' in context_dict:
            context.location = resolve_entity_description(context_dict['location'])

        return context

    def _configure(self, robot, config):
        self._robot = robot

        self._semantics = Find._parse_semantics(config.semantics)
        self._context = Find._parse_context(config.context)

        # Check if we know the source location
        if self._semantics.source_location:
            pass
        elif self._context.location:
            e = self._context.location.designator.resolve()
            if e:
                self._semantics.source_location = self._context.location
            else:
                if self._semantics.object.id:
                    self._config_result.message = " Where should I look for the {}?".format(self._semantics.object.id)
                else:
                    self._config_result.message = " Where should I look for the {}?".format(self._semantics.object.type)
                self._config_result.missing_field = 'source-location'
                return
        else:
            self._config_result.message = " Where should I look for the {}?".format(self._semantics.object.type)
            self._config_result.missing_field = 'source-location'
            return

        # We can now assume we know the source location
        # Set up context about the found entity
        self._config_result.context['object'] = config.semantics['object']

        # Check if we need to find a person or an object
        if self._semantics.object.type == 'person':
            # If no source-location is specified, check if a location is specified in the object spec
            if not self._semantics.source_location:
                if self._semantics.object.location:
                    self._semantics.source_location = self._semantics.object.location
                elif self._context.location:
                    self._semantics.source_location = self._context.location
                else:
                    self._config_result.message = " Where should I look for {}?".format(self._semantics.object.id)
                    self._config_result.missing_field = "source-location"
                    return

            # if we need to find the person near a furniture object, get the room where it is located
            if self._semantics.source_location.id not in self._knowledge.location_rooms:
                self._semantics.source_location.id = self._knowledge.get_room(self._semantics.source_location.id)

            # person in room
            if not self._semantics.object.id:
                self._semantics.object.id = 'someone'
                discard_other_labels = False
            else:
                discard_other_labels = True
            self._found_entity_designator = VariableDesignator(resolve_type=Entity)
            self._find_state_machines = [
                rss.human_interaction.FindPersonInRoom(robot, self._semantics.source_location.id, self._semantics.object.id,
                                                       discard_other_labels, self._found_entity_designator.writeable)]
            self._config_result.context['location'] = {
                'designator': EdEntityDesignator(self._robot, id=self._semantics.source_location.id)
            }
            if self._semantics.source_location.id:
                self._config_result.context['location']['id'] = self._semantics.source_location.id

            self._found_entity_designator = EdEntityDesignator(self._robot, id=self._semantics.object.id)
            self._config_result.context['object']['designator'] = self._found_entity_designator
            self._config_result.succeeded = True

            # TODO: Robocup hack to make sure the robot moves to the found person
            self._navigation_state_machine = rss.navigation.NavigateToWaypoint(
                self._robot,
                waypoint_designator=self._found_entity_designator,
                radius=1.0,
                look_at_designator=self._found_entity_designator
            )
            return

        # We need to find an object
        # we need to navigate to all pieces of furniture and inspect all of their inspection areas
        self._areas = {}
        self._nav_areas = {}
        if self._semantics.source_location.id in self._knowledge.location_rooms:
            locations = self._knowledge.get_locations(self._semantics.source_location.id, True)
            for location in locations:
                self._areas[location] = self._knowledge.get_inspect_areas(location)
                self._nav_areas[location] = self._knowledge.get_inspect_position(location)
        else:
            self._areas[self._semantics.source_location.id] = self._knowledge.get_inspect_areas(
                self._semantics.source_location.id)
            self._nav_areas[self._semantics.source_location.id] = self._knowledge.get_inspect_position(
                self._semantics.source_location.id)

        # Set up the designator with the object description
        entity_description = {'type': self._semantics.object.type, 'category': self._semantics.object.category}
        description_designator = VariableDesignator(entity_description)

        location_designator = None
        self._find_state_machines = []
        self._found_entity_designator = VariableDesignator(resolve_type=Entity)
        for loc, areas in self._areas.iteritems():
            location_designator = EdEntityDesignator(self._robot, id=loc)
            nav_area = self._nav_areas[loc]
            for area in areas:
                area_designator = VariableDesignator(area)

                navigation_area_designator = VariableDesignator(nav_area)

                # Set up the Find state machine
                rospy.loginfo("Setting up state machine with loc = {}, area = {}, nav_area = {}".format(loc, area,
                                                                                                        nav_area))
                self._find_state_machines.append(rss.navigation.Find(robot=self._robot, knowledge=self._knowledge,
                                                                     source_entity_designator=location_designator,
                                                                     description_designator=description_designator,
                                                                     area_name_designator=area_designator,
                                                                     navigation_area_designator=navigation_area_designator,
                                                                     found_entity_designator=self._found_entity_designator))

        self._config_result.context['object'] = {'designator': self._found_entity_designator,
                                                 'type': self._semantics.object.type,
                                                 'category': self._semantics.object.category}
        self._config_result.context['location'] = {'designator': location_designator}
        if self._semantics.source_location.id:
            self._config_result.context['location']['id'] = self._semantics.source_location.id
        self._config_result.succeeded = True

    def _start(self):
        for fsm in self._find_state_machines:
            res = fsm.execute()

            location = None
            if self._semantics.source_location.id:
                location = self._semantics.source_location.id

            if res in ['succeeded', 'found']:
                if self._semantics.object.type == 'person':
                    if self._semantics.object.id:
                        if location:
                            self._execute_result.message = " I found {} at the {}. ".format(self._semantics.object.id,
                                                                                            location)
                        else:
                            self._execute_result.message = " I found {}. ".format(self._semantics.object.id)
                    else:
                        if location:
                            self._execute_result.message = " I found a person at the {}. ".format(location)
                        else:
                            self._execute_result.message = " I found a person. "
                    self._point_at_person(self._found_entity_designator.resolve())
                    self._navigation_state_machine.execute()
                    self._robot.speech.speak("Hi there!")
                else:
                    self._robot.speech.speak("Hey, I found a {}!".format(self._semantics.object.type if
                                                                         self._semantics.object.type else
                                                                         self._semantics.object.category))
                    self._execute_result.message = " I found a {} ".format(self._semantics.object.type if
                                                                           self._semantics.object.type else
                                                                           self._semantics.object.category)
                self._execute_result.succeeded = True
                return
            elif res == 'not_found':
                if self._semantics.object.type == "person":
                    self._robot.speech.speak(" I don't see anyone here. ")
                else:
                    self._robot.speech.speak("I don't see what I am looking for here.")

                self._execute_result.message = " I couldn't find {} {} the {} ".format(
                    self._semantics.object.id if self._semantics.object.id and not self._semantics.object.id == "None" else \
                        "a " + self._semantics.object.type if self._semantics.object.type else "a " + self._semantics.object.category,
                    "in" if self._semantics.source_location.id in self._knowledge.location_rooms else "at",
                    self._semantics.source_location.id
                )
            else:
                self._robot.speech.speak(" I'm unable to inspect the {} ".format(self._semantics.source_location.id))
                self._execute_result.message = " I was unable to inspect the {} to find {}. ".format(
                    self._semantics.source_location.id,
                    self._semantics.object.id if self._semantics.object.id else "a " + self._semantics.object.type if self._semantics.object.type else \
                        "a " + self._semantics.object.category
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
