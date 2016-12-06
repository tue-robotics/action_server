#! /usr/bin/python

# ------------------------------------------------------------------------------------------------------------------------
# By Sjoerd van den Dries, 2016
# ------------------------------------------------------------------------------------------------------------------------

# TODO:
# - correctly deal with resolving:
#   - Always resolve 'it', etc
#   - if ask_missing_info is True, also ask questions about object categories, etc

import os
import sys
import yaml
import rospy
import random
import argparse
import time
import traceback
import actions

import hmi_server.api

from command_recognizer import CommandRecognizer

# ------------------------------------------------------------------------------------------------------------------------

class World:

    def __init__(self):
        self.entity_ids = []
        self.entity_type_to_id = {}
        self.object_to_location = {}
        self.last_location = None
        self.last_entity = None
        self.knowledge = None

    # Returns if 'id' is a room
    def is_room(self, id):
        return (id in self.knowledge.rooms)

    # Returns if 'id' is a location
    def is_location(self, id):
        return self.knowledge.is_location(id)

    # Returns the room 'id' is in
    def get_room(self, id):
        return self.knowledge.get_room(id)

    def get_inspect_position(self, location, area=""):
        return self.knowledge.get_inspect_position(id, area)

    def get_locations(self, room=None, pick_location=None, place_location=None):
        return self.knowledge.get_locations(room=room, pick_location=pick_location, place_location=place_location)

    def get_objects(self, category=None):
        return self.knowledge.get_objects(category=category)

    def get_inspect_areas(self, location):
        return self.knowledge.get_inspect_areas(location)

    # Returns (location, area_name)
    def get_object_category_location(self, obj_cat):
        return self.knowledge.get_object_category_location(obj_cat)

    def get_object_category(self, obj):
        return self.knowledge.get_object_category(obj)

# ------------------------------------------------------------------------------------------------------------------------

class CommandCenter:

    def __init__(self, robot=None):

        self.robot = robot
        self.world = World()

        self.action_functions = {
            "navigate" : actions.navigate,
            "find" : actions.find,
            "follow" : actions.follow,
            "answer-question" : actions.answer_question,
            "pick-up" : actions.find_and_pick_up,
            "bring" : actions.bring,
            "say" : actions.say
        }

    # ------------------------------------------------------------------------------------------------------------------------

    def start_challenge(self, robot):
        s = StartChallengeRobust(robot, self.knowledge.initial_pose)
        s.execute()


    # ------------------------------------------------------------------------------------------------------------------------

    def set_grammar(self, grammar_file, knowledge):
        self.world.knowledge = knowledge
        self.command_recognizer = CommandRecognizer(grammar_file, knowledge)

    # ------------------------------------------------------------------------------------------------------------------------

    # Functions must have signature (robot, world, parameters)
    def register_action(self, name, function):
        print "Registering action {}".format(name)
        self.action_functions[name] = function

    # ------------------------------------------------------------------------------------------------------------------------

    # returns: None if getting command failed, otherwise (command_words, command_semantics)
    def request_command(self, ask_confirmation=True, ask_missing_info=False, timeout=600, sentences=None, n_tries=100):

        if not sentences or len(sentences) < 2:
            sentences = ["Welcome to this challenge, Still loading grammar. Ok I am done. . What can I do for you?",
                         "I am so so sorry. Can you please speak louder? Wait for the ping!"]

        def prompt_once(sentence):
            self.robot.head.look_at_standing_person()
            self.robot.head.wait_for_motion_done()

            res = None
            while not res:
                if rospy.is_shutdown():
                    return None

                self.robot.speech.speak(sentence, block=False)
                res = self.command_recognizer.recognize(self.robot, timeout=timeout)
                if not res:
                    pass
                    #self.robot.speech.speak("Sorry, I could not understand", block=True)

            # print "Sentence: %s" % res[0]
            # print "Semantics: %s" % res[1]

            return res

        def ask_confirm(tries=3):
            sentence = " ".join(words)

            self.robot.speech.speak("Is that correct?", block=True)

            # self.robot.speech.speak("Do you want me to %s?" % sentence.replace(" your", " my").replace(" me", " you"), block=True)
            for i in range(0, tries):
                if rospy.is_shutdown():
                    return None

                try:
                    result = self.robot.hmi.query("", "<choice>", {"choice":["yes","no"]}, timeout=10)
                    return result["choice"] == "yes"
                except hmi_server.api.TimeoutException:
                    self.robot.speech.speak("Please say yes or no")

            return False

        n_current_tries = 0

        while True:
            i_sentence = n_current_tries

            if i_sentence >= len(sentences):
                i_sentence = (i_sentence - 1) % (len(sentences) - 1) + 1

            (words, semantics) = prompt_once(sentences[i_sentence])
            n_current_tries += 1

            if not ask_confirmation:
                return None

            if ask_confirm():
                # We heard correctly
                break

            if n_current_tries >= n_tries:
                self.robot.speech.speak("Sorry")
                return None

        semantics = self.command_recognizer.resolve(semantics, self.robot)

        return (words, semantics)

    # ------------------------------------------------------------------------------------------------------------------------

    # returns: command_semantics, or None if sentence could not be parsed
    def parse_command(self, command_sentence):
        res = self.command_recognizer.parse(command_sentence, self.robot)
        if not res:
            return None

        (words, semantics) = res
        return semantics

    # ------------------------------------------------------------------------------------------------------------------------

    def execute_command(self, command_semantics, blocking=True):
        try:
            # print "command_semantics = {}".format(command_semantics)
            actions = command_semantics["actions"]
            for a in actions:
                action_type = a["action"]

                if action_type in self.action_functions:
                    self.action_functions[action_type](self.robot, self.world, a)
                else:
                    print "Unknown action type: '%s'" % action_type

            return True
        except KeyboardInterrupt as e:
            rospy.logwarn('keyboard interupt')
        except Exception as e:
            rospy.logerr("{0}".format(e.message))
            rospy.logerr("%s", traceback.format_exc(sys.exc_info()))

        return False

    # ------------------------------------------------------------------------------------------------------------------------

    def wait_for_command(self):
        pass
