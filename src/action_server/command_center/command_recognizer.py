#! /usr/bin/python

import os
import cfgparser
import sys
import rospy

import yaml

import hmi_server.api

# from robocup_knowledge import load_knowledge
# challenge_knowledge = load_knowledge('challenge_gpsr')

# ----------------------------------------------------------------------------------------------------

class EntityDescription(object):
    def __init__(self, id=None, type=None, location=None, category=None):
        self.id = id
        self.type = type
        self.location = location
        self.category = category
        self.is_undefined = False

    def serialize(self):
        d = {}
        if self.id:
            d["id"] = self.id
        if self.type:
            d["type"] = self.type
        if self.location:
            d["loc"] = self.location.serialize()
        if self.category:
            d["cat"] = self.category

        return d

    def __repr__(self):
        return "(id={}, type={}, location={}, category={})".format(self.id, self.type,
                                                             self.location, self.category)

# ----------------------------------------------------------------------------------------------------

def resolve_entity_description(parameters):
    descr = EntityDescription()

    if isinstance(parameters, str):
        descr.id = parameters

    elif "special" in parameters:
        special = parameters["special"]
        if special =="it":
            descr.is_undefined = True
        elif special == "operator":
            descr.id = "gpsr_starting_pose"
    else:
        if "id" in parameters:
            descr.id = parameters["id"]
        if "type" in parameters:
            descr.type = parameters["type"]
        if "loc" in parameters:
            descr.location = resolve_entity_description(parameters["loc"])
        if "cat" in parameters:
            descr.category = parameters["cat"]

    return descr

# ----------------------------------------------------------------------------------------------------

def hear(robot, sentence, options):
    for i in range(0, 3):
        robot.speech.speak(sentence)

        try:
            result = robot.hmi.query(sentence, "<choice>", {"choice":options}, timeout=10)
            choice = result["choice"]
            robot.speech.speak("OK, {}".format(choice))
            return choice
        except hmi_server.api.TimeoutException:
            robot.speech.speak("Sorry, I could not understand")

    robot.speech.speak("I will just assume {}".format(options[0]))
    return options[0]

# ----------------------------------------------------------------------------------------------------


def fill_in_gaps(robot, knowledge, parameters, last_entity, last_location, context):
    new_last_loc = None

    entity = resolve_entity_description(parameters)

    if entity.is_undefined:
        if not last_entity:
            entity.id = hear(robot, "What do you want me to {}?".format(context["action"]),
                              knowledge.object_names)
        else:
            entity = last_entity

    elif entity.id:
        if entity.type == "person":
            if not entity.location:
                entity.location = last_location
            if not entity.location:
                loc =hear(robot, "Where can I find {}?".format(entity.id),
                          knowledge.rooms)
                entity.location = EntityDescription(id=loc)
        else:
            new_last_loc = EntityDescription(id=entity.id)

    elif entity.type:
        if not entity.location:
            entity.location = last_location
        if not entity.location:
                loc = hear(robot, "Where can I find {}?".format(entity.id),
                          list(set([o["room"] for o in knowledge.locations])))
                entity.location = EntityDescription(id=loc)
    elif entity.category:
        entity.type = hear(robot, "What kind of {}?".format(entity.category), knowledge.object_names)

    # print entity.serialize()

    return (entity.serialize(), entity, new_last_loc)

# ----------------------------------------------------------------------------------------------------

def unwrap_grammar(lname, parser):
    if not lname in parser.rules:
        return ""

    rule = parser.rules[lname]

    s = ""

    opt_strings = []
    for opt in rule.options:
        conj_strings = []

        ok = True
        for conj in opt.conjuncts:
            if conj.is_variable:
                unwrapped_string = unwrap_grammar(conj.name, parser)
                if unwrapped_string:
                    conj_strings.append(unwrapped_string)
                else:
                    ok = False
            else:
                conj_strings.append(conj.name)

        if ok:
            opt_strings.append(" ".join(conj_strings))

    if not opt_strings:
        return ""

    s = "|".join(opt_strings)

    if len(opt_strings) > 1:
        s = "(" + s + ")"

    return s

# ----------------------------------------------------------------------------------------------------

def resolve_name(name, knowledge):
    if name in knowledge.translations:
        return knowledge.translations[name]
    else:
        return name.replace("_", " ")

# ----------------------------------------------------------------------------------------------------

class CommandRecognizer:

    def __init__(self, grammar_file, challenge_knowledge):
        self.parser = cfgparser.CFGParser.fromfile(grammar_file)

        for obj in challenge_knowledge.object_names:
            self.parser.add_rule("SMALL_OBJECT[\"%s\"] -> %s" % (obj, resolve_name(obj, challenge_knowledge)))

        for loc in challenge_knowledge.get_locations():
            print loc
            #parser.add_rule("FURNITURE[\"%s\"] -> %s" % (furniture, furniture))
            self.parser.add_rule("FURNITURE[\"%s\"] -> %s" % (loc, resolve_name(loc, challenge_knowledge)))

        for name in challenge_knowledge.names:
            self.parser.add_rule("NAME[\"%s\"] -> %s" % (name.lower(), name.lower()))

            # for obj in objects:
            #     #parser.add_rule("SMALL_OBJECT[\"%s\"] -> %s" % (obj, obj))
            #     self.parser.add_rule("SMALL_OBJECT[\"%s\"] -> the %s" % (obj, obj))
            #     self.parser.add_rule("SMALL_OBJECT[\"%s\"] -> a %s" % (obj, obj))

        for room in challenge_knowledge.rooms:
            #parser.add_rule("ROOM[\"%s\"] -> %s" % (rooms, rooms))
            self.parser.add_rule("ROOM[\"%s\"] -> %s" % (room, resolve_name(room, challenge_knowledge)))

        for obj_cat in challenge_knowledge.object_categories:
            self.parser.add_rule("OBJ_CAT[\"%s\"] -> %s" % (obj_cat, obj_cat))

        for container in challenge_knowledge.get_objects(category="container"):
            self.parser.add_rule("CONTAINER[\"%s\"] -> %s" % (container, container))

        # for (alias, obj) in challenge_knowledge.object_aliases.iteritems():
        #     #parser.add_rule("NP[\"%s\"] -> %s" % (obj, alias))
        #     self.parser.add_rule("NP[\"%s\"] -> the %s" % (obj, alias))
        #     self.parser.add_rule("NP[\"%s\"] -> a %s" % (obj, alias))

        self.grammar_string = unwrap_grammar("T", self.parser)

        self.knowledge = challenge_knowledge

        # print len(self.grammar_string)

    # Returns: semantics
    def resolve(self, semantics, robot):
        if not semantics:
            return None

        actions = semantics["actions"]

        last_location = None
        last_entity = None

        actions_resolved = []
        for a in actions:
            if "entity" in a:
                (e_new, le, ll) = fill_in_gaps(robot, self.knowledge, a["entity"], last_entity, last_location, a)
                a["entity"] = e_new
                last_location = ll
                last_entity = le
            if "to" in a:
                (e_new, le, ll) = fill_in_gaps(robot, self.knowledge, a["to"], last_entity, last_location, a)
                a["to"] = e_new
                last_location = ll
                last_entity = le

        print "RESOLVED ACTIONS = {}".format(actions)

        return { "actions" : actions }

    # Returns (words, semantics)
    def parse(self, sentence, robot):
        if isinstance(sentence, str):
            words = sentence.lower().strip().split(" ")
        else:
            words = sentence

        semantics_str = self.parser.parse("T", words)

        if not semantics_str:
            return None

        semantics_str = semantics_str.replace("<", "[")
        semantics_str = semantics_str.replace(">", "]")

        semantics = yaml.load(semantics_str)

        return (words, semantics)

    # Returns (words, semantics)
    def recognize(self, robot, timeout):
        result = robot.ears.recognize(self.grammar_string, time_out=rospy.Duration(timeout))

        if not result:
            return None

        sentence = result.result

        if not sentence:
            return None

        return self.parse(sentence, robot)