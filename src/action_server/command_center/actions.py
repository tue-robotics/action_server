import os
import sys
import yaml
import cfgparser
import rospy
import random
import time

import robot_smach_states
from robot_smach_states.navigation import NavigateToObserve, NavigateToWaypoint, NavigateToSymbolic
from robot_smach_states import SegmentObjects, Grab, Place, HandoverToHuman
from robot_smach_states.util.designators import EdEntityDesignator, EntityByIdDesignator, VariableDesignator, DeferToRuntime, analyse_designators, UnoccupiedArmDesignator, EmptySpotDesignator, OccupiedArmDesignator
from robot_skills.util import transformations
from robot_skills.classification_result import ClassificationResult
from robocup_knowledge import load_knowledge
from find_person import FindPerson
from datetime import datetime, timedelta
import robot_smach_states.util.designators as ds

from robot_smach_states import LookAtArea, StartChallengeRobust

from robot_smach_states import FollowOperator

speech_data = load_knowledge('challenge_speech_recognition')

# ------------------------------------------------------------------------------------------------------------------------

class EntityDescription(object):
    def __init__(self, id=None, type=None, location=None):
        self.id = id
        self.type = type
        self.location = location

# ------------------------------------------------------------------------------------------------------------------------

def not_implemented(robot, parameters):
    rospy.logerr("This was not implemented, show this to Sjoerd: {}".format(parameters))
    robot.speech.speak("Not implemented! Warn Sjoerd", block=False)
    return

# ------------------------------------------------------------------------------------------------------------------------

def resolve_entity_description(world, parameters):
    descr = EntityDescription()

    if isinstance(parameters, str):
        descr.id = parameters

    elif "special" in parameters:
        special = parameters["special"]
        if special =="it":
            descr = world.last_entity
        elif special == "operator":
            descr.id = "gpsr_starting_pose"
    else:
        if "id" in parameters:
            descr.id = parameters["id"]
        if "type" in parameters:
            descr.type = parameters["type"]
        if "loc" in parameters:
            descr.location = resolve_entity_description(world, parameters["loc"])

    return descr

# ------------------------------------------------------------------------------------------------------------------------

def move_robot(robot, world, id=None, type=None, nav_area=None, room=None):

    if world.is_room(id):
        # Driving to a room

        nwc =  NavigateToSymbolic(robot,
                                        { EntityByIdDesignator(robot, id=id) : "in" },
                                          EntityByIdDesignator(robot, id=id))
        nwc.execute()
    elif type == "person":
        # Driving to a person

        if id:
            nwc =  NavigateToSymbolic(robot,
                                            { EntityByIdDesignator(robot, id=id) : "in" },
                                              EntityByIdDesignator(robot, id=id))
        elif room:
            room_des = EdEntityDesignator(robot, id=room)
            f = FindPerson(robot, room_des)
            result = f.execute()
            # if result == 'succeeded':
            #     robot.speech.speak("I found you!")
        else:
            robot.speech.speak("I don't know where I can find the person")

    elif world.is_location(id):
        # Driving to a location

        if not nav_area:
            nav_area = world.get_inspect_position(id)

        location_des = ds.EntityByIdDesignator(robot, id=id)

        room_id = world.get_room(id)

        if room_id:
            room_des = ds.EntityByIdDesignator(robot, id=room_id)
            nwc = NavigateToSymbolic(robot, {location_des : nav_area, room_des : "in"}, location_des)
        else:
            nwc = NavigateToSymbolic(robot, {location_des : nav_area}, location_des)            
        nwc.execute()
    else:
        # Driving to anything else (e.g. a waypoint)
        nwc = NavigateToObserve(robot, EntityByIdDesignator(robot, id=id))
        nwc.execute()

# ------------------------------------------------------------------------------------------------------------------------

def navigate(robot, world, parameters):
    entity_descr = resolve_entity_description(world, parameters["entity"])

    if not entity_descr.location:
        entity_descr.location = world.last_location

    if entity_descr.type == "person":
        move_robot(robot, world, entity_descr.id, entity_descr.type, room=entity_descr.location.id)

    elif not entity_descr.id:
        not_implemented(robot, parameters)

    else:
        robot.speech.speak("I am going to the %s" % entity_descr.id, block=False)
        move_robot(robot, world, entity_descr.id, entity_descr.type)
        world.last_location = entity_descr

# ------------------------------------------------------------------------------------------------------------------------

def follow(robot, world, parameters):
    entity_descr = resolve_entity_description(world, parameters["entity"])

    if not entity_descr.location:
        entity_descr.location = world.last_location

    if entity_descr.type == "person":
        move_robot(robot, world, entity_descr.id, entity_descr.type, room=entity_descr.location.id)

    elif not entity_descr.id:
        not_implemented(robot, parameters)

    else:
        robot.speech.speak("I am going to the %s" % entity_descr.id, block=False)
        move_robot(robot, world, entity_descr.id, entity_descr.type)
        world.last_location = entity_descr

    follow_operator_state = FollowOperator(robot)
    ret = follow_operator_state.execute({})

    if ret == "stopped":
        robot.speech.speak("I succesfully followed you", block=False)

# ------------------------------------------------------------------------------------------------------------------------

def answer_question(robot, world, parameters):

    robot.head.look_at_standing_person()
    robot.head.wait_for_motion_done()

    robot.speech.speak("What is your question?")

    res = robot.ears.recognize(spec=speech_data.spec,
                               choices=speech_data.choices,
                               time_out=rospy.Duration(15))

    if not res:
        robot.speech.speak("My ears are not working properly, sorry!")

    if res:
        if "question" in res.choices:
            rospy.loginfo("Question was: '%s'?"%res.result)
            robot.speech.speak("The answer is %s" % speech_data.choice_answer_mapping[res.choices['question']])
        else:
            robot.speech.speak("Sorry, I do not understand your question")

# ------------------------------------------------------------------------------------------------------------------------

def say(robot, world, parameters):
    sentence = parameters["sentence"]
    rospy.loginfo('Answering %s', sentence)

    if sentence == 'TIME':
        hours = datetime.now().hour
        minutes = datetime.now().minute
        line = "The time is {} {}".format(hours, minutes)
    elif sentence == "ROBOT_NAME":
        line = 'My name is %s' % robot.robot_name
    elif sentence == 'TODAY':
        line = datetime.today().strftime('Today is %A %B %d')
    elif sentence == 'TOMORROW':
        line = (datetime.today() + timedelta(days=1)).strftime('Tomorrow is %A %B %d')
    elif sentence == 'DAY_OF_MONTH':
        line = datetime.now().strftime('It is day %d of the month')
    elif sentence == 'DAY_OF_WEEK':
        line = datetime.today().strftime('Today is a %A')
    else:
        line = sentence

    robot.speech.speak(line)

# ------------------------------------------------------------------------------------------------------------------------

def find_and_pick_up(robot, world, parameters, pick_up=True):
    entity_descr = resolve_entity_description(world, parameters["entity"])

    if not entity_descr.location:
        entity_descr.location = world.last_location

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if entity_descr.type == "person":

        if world.is_room(entity_descr.location.id):
            room = entity_descr.location.id
        else:
            room = world.get_room(entity_descr.location.id)

        move_robot(robot, world, id=entity_descr.id, type=entity_descr.type, room=room)
        return

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    world.last_entity = entity_descr

    if entity_descr.location:
        room_or_location = entity_descr.location.id

        if world.is_room(room_or_location):
            locations = world.get_locations(room=room_or_location, pick_location=True)
        else:
            locations = [room_or_location]

        locations_with_areas = []
        for location in locations:
            locations_with_areas += [(location, world.get_inspect_areas(location))]
    else:
        obj_cat = world.get_object_category(entity_descr.type)

        (location, area_name) = world.get_object_category_location(obj_cat)

        locations_with_areas = [(location, [area_name])]

        robot.speech.speak("The {} is a {}, which is stored on the {}".format(entity_descr.type, obj_cat, location), block=False)

    location_defined = (len(locations_with_areas) == 1)

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    possible_entities = []

    for loc_and_areas in locations_with_areas:

        (location, area_names) = loc_and_areas

        robot.speech.speak("Going to the %s" % location, block=False)

        last_nav_area = None

        for area_name in area_names:

            nav_area = world.get_inspect_position(location, area_name)

            if nav_area != last_nav_area:

                # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
                # Move to the location

                move_robot(robot, world, id=location, nav_area=nav_area)
                last_nav_area = nav_area

            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            # Look at the area

            look_sm = LookAtArea(robot,
                                 EdEntityDesignator(robot, id=location),
                                 area_name)
            look_sm.execute()

            import time
            time.sleep(1)

            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            # Segment

            segmented_entities = robot.ed.update_kinect("{} {}".format(area_name, location))

            found_entity_ids = segmented_entities.new_ids + segmented_entities.updated_ids

            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            # Classify

            entity_types_and_probs = robot.ed.classify(ids=found_entity_ids,
                                                       types=world.get_objects())

            best_prob = 0
            for det in entity_types_and_probs:
                if det.type == entity_descr.type and det.probability > best_prob:
                    entity_descr.id = det.id
                    best_prob = det.probability

            if not entity_descr.id:
                possible_entities += found_entity_ids
            else:
                robot.speech.speak("Found the {}!".format(entity_descr.type), block=False)

            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            if entity_descr.id:
                break

        if entity_descr.id:
            break

    if not entity_descr.id:
        if not possible_entities:
            robot.speech.speak("I really can't find the {}!".format(entity_descr.type), block=False)
        else:
            closest_entity_id = None
            closest_distance = None
            for entity_id in possible_entities:
                entity = robot.ed.get_entity(id=entity_id, parse=False)
                if not entity:
                    continue

                p = transformations.tf_transform(entity.pose.position, "/map",
                                             robot.robot_name+"/base_link",
                                             robot.tf_listener)
                distance = p.x*p.x + p.y*p.y

                if not closest_entity_id or distance < closest_distance:
                    closest_entity_id = entity_id
                    closest_distance = distance

            entity_descr.id = closest_entity_id

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if pick_up and entity_descr.id:

        robot.speech.speak("Going to grab the {}".format(entity_descr.type))

        # grab it
        grab = Grab(robot, EdEntityDesignator(robot, id=entity_descr.id),
             UnoccupiedArmDesignator(robot.arms, robot.leftArm, name="empty_arm_designator"))
        result = grab.execute()

# ------------------------------------------------------------------------------------------------------------------------

def bring(robot, world, parameters):

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # Check if need to grab an entity and if so, do so

    if "entity" in parameters:
        entity_descr = resolve_entity_description(world, parameters["entity"])

        if not world.last_entity or entity_descr.type != world.last_entity.type:
            find_and_pick_up(robot, world, parameters)

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # Deliver it

    to_descr = resolve_entity_description(world, parameters["to"])

    if to_descr.type == "person" or to_descr.id == "gpsr_starting_pose":
        if to_descr.location:
            move_robot(robot, world, id=to_descr.id, type=to_descr.type, room=to_descr.location.id)
        else:
            move_robot(robot, world, id=to_descr.id, type=to_descr.type)

        arm_des = OccupiedArmDesignator(robot.arms, robot.leftArm)

        if not arm_des.resolve():
            robot.speech.speak("I don't have anything to give to you")
        else:
            h = HandoverToHuman(robot, arm_des)
            result = h.execute()
    else:
        # Move to the location
        move_robot(robot, world, id=to_descr.id, nav_area="in_front_of")

        # place
        arm = OccupiedArmDesignator(robot.arms, robot.leftArm)

        if not arm.resolve():
            robot.speech.speak("I don't have anything to place")
        else:
            current_item = EdEntityDesignator(robot)
            location_des = EntityByIdDesignator(robot, id=to_descr.id)
            place_position = EmptySpotDesignator(robot, location_des, area='on_top_of')
            p = Place(robot, current_item, place_position, arm)
            result = p.execute()

            if result != 'done':
                robot.speech.speak("Sorry, my fault")

    world.last_location = None
    world.last_entity = None

# ------------------------------------------------------------------------------------------------------------------------

def find(robot, world, parameters):
    find_and_pick_up(robot, world, parameters, pick_up=False)