import os
import unittest

from ed_msgs.msg import EntityInfo

from action_server.test_tools import test_grammar
from robot_skills.mockbot import Mockbot
from robot_skills.util.entity import from_entity_info

# noinspection Py
GRAMMAR = """
T[{actions : <A1>}] -> C[A1]

C[{A}] -> VP[A]

V_GUIDE -> guide | escort | take | lead | accompany

DET -> the | a | an | some
NUMBER -> one | two | three
MANIPULATION_AREA_DESCRIPTION -> on top of | at | in | on | from

LOCATION[couch_table] -> couch_table
LOCATION[dinner_table] -> dinner_table
LOCATION[bar] -> bar
LOCATION[bookcase] -> bookcase
LOCATION[cabinet] -> cabinet
LOCATION[trashbin] -> trashbin
LOCATION[plant] -> plant
LOCATION[bed] -> bed
LOCATION[nightstand] -> nightstand
LOCATION[flight_case] -> flight_case
LOCATION[battery_table] -> battery_table
LOCATION[workbench] -> workbench
LOCATION[hallway_table] -> hallway_table
NAMED_OBJECT[cookies] -> cookies
NAMED_OBJECT[apple] -> apple
NAMED_OBJECT[tea] -> tea
NAMED_OBJECT[deodorant] -> deodorant
NAMED_OBJECT[sponge] -> sponge
NAMED_OBJECT[brush] -> brush
NAMED_OBJECT[hairspray] -> hairspray
NAMED_OBJECT[cup] -> cup
NAMED_OBJECT[toothpaste] -> toothpaste
NAMED_OBJECT[bowl] -> bowl
NAMED_OBJECT[beer] -> beer
NAMED_OBJECT[fanta] -> fanta
NAMED_OBJECT[orange] -> orange
NAMED_OBJECT[coke] -> coke
NAMED_OBJECT[pringles] -> pringles
NAMED_OBJECT[chopsticks] -> chopsticks
NAMED_OBJECT[fork] -> fork
NAMED_OBJECT[plate] -> plate
NAMED_OBJECT[pepper] -> pepper
NAMED_OBJECT[bifrutas] -> bifrutas
NAMED_OBJECT[hair_gel] -> hair_gel
NAMED_OBJECT[corn] -> corn
NAMED_OBJECT[peas] -> peas
NAMED_OBJECT[water] -> water
NAMED_OBJECT[cloth] -> cloth
NAMED_OBJECT[spoon] -> spoon
NAMED_OBJECT[ice_tea] -> ice_tea
NAMED_OBJECT[cereal] -> cereal
NAMED_OBJECT[pasta] -> pasta
NAMED_OBJECT[towel] -> towel
NAMED_OBJECT[banana] -> banana
NAMED_OBJECT[mango] -> mango
NAMED_OBJECT[mentos] -> mentos
NAMED_OBJECT[knife] -> knife
NAMED_OBJECT[basket] -> basket
NAMED_OBJECT[crackers] -> crackers
NAMED_OBJECT[salt] -> salt
MANIPULATION_AREA_LOCATION[couch_table] -> MANIPULATION_AREA_DESCRIPTION the couch_table
MANIPULATION_AREA_LOCATION[dinner_table] -> MANIPULATION_AREA_DESCRIPTION the dinner_table
MANIPULATION_AREA_LOCATION[bar] -> MANIPULATION_AREA_DESCRIPTION the bar
MANIPULATION_AREA_LOCATION[bookcase] -> MANIPULATION_AREA_DESCRIPTION the bookcase
MANIPULATION_AREA_LOCATION[cabinet] -> MANIPULATION_AREA_DESCRIPTION the cabinet
MANIPULATION_AREA_LOCATION[bed] -> MANIPULATION_AREA_DESCRIPTION the bed
MANIPULATION_AREA_LOCATION[nightstand] -> MANIPULATION_AREA_DESCRIPTION the nightstand
MANIPULATION_AREA_LOCATION[workbench] -> MANIPULATION_AREA_DESCRIPTION the workbench
MANIPULATION_AREA_LOCATION[hallway_table] -> MANIPULATION_AREA_DESCRIPTION the hallway_table
OBJECT_CATEGORY[cleaning_stuff] -> cleaning_stuff
OBJECT_CATEGORY[snack] -> snack
OBJECT_CATEGORY[container] -> container
OBJECT_CATEGORY[food] -> food
OBJECT_CATEGORY[drink] -> drink
OBJECT_CATEGORY[cutlery] -> cutlery
NAMED_PERSON[anna] -> anna
NAMED_PERSON[beth] -> beth
NAMED_PERSON[carmen] -> carmen
NAMED_PERSON[jennifer] -> jennifer
NAMED_PERSON[jessica] -> jessica
NAMED_PERSON[kimberly] -> kimberly
NAMED_PERSON[kristina] -> kristina
NAMED_PERSON[laura] -> laura
NAMED_PERSON[mary] -> mary
NAMED_PERSON[sarah] -> sarah
NAMED_PERSON[alfred] -> alfred
NAMED_PERSON[charles] -> charles
NAMED_PERSON[daniel] -> daniel
NAMED_PERSON[james] -> james
NAMED_PERSON[john] -> john
NAMED_PERSON[luis] -> luis
NAMED_PERSON[paul] -> paul
NAMED_PERSON[richard] -> richard
NAMED_PERSON[robert] -> robert
NAMED_PERSON[steve] -> steve
V_PRESENT -> introduce yourself | present yourself | perform a demonstration | give a presentation
ENGLISH['en'] -> english
DUTCH['nl'] -> dutch
LANGUAGE[X] -> ENGLISH[X] | DUTCH[X]
VP["action": "demo-presentation", 'language': 'en'] -> V_PRESENT
VP["action": "demo-presentation", "language": X] -> V_PRESENT in LANGUAGE[X]

V_FIND -> find | locate | look for

VP["action": "find", "object": {"type": X}, "location": {"id": Y}] -> V_FIND DET NAMED_OBJECT[X] MANIPULATION_AREA_LOCATION[Y]
VP["action": "find", "object": {"type": X}] -> V_FIND DET NAMED_OBJECT[X]

V_GOPL -> go to | navigate to | drive to

VP["action": "navigate-to", "target-location": {"id": X}] -> V_GOPL the LOCATION[X]


VP["action": "inspect", "entity": {"id": X}] -> inspect the LOCATION[X]

OBJECT_TO_BE_BROUGHT -> NAMED_OBJECT | DET NAMED_OBJECT

V_BRING -> bring | deliver | take | carry | transport | give | hand | hand over

VP["action": "place", "source-location": {"id": X}, "target-location": {"id": Y}, "object": {"type": Z}] -> V_BRING OBJECT_TO_BE_BROUGHT[Z] from the LOCATION[X] to the LOCATION[Y]
VP["action": "hand-over", "source-location": {"id": X}, "target-location": {"id": "operator"}, "object": {"type": Z}] -> V_BRING me OBJECT_TO_BE_BROUGHT[Z] from the LOCATION[X] | V_BRING OBJECT_TO_BE_BROUGHT[Z] from the LOCATION[X] to me

V_SAY -> tell | say

VP["action": "say", "sentence": X] -> V_SAY SAY_SENTENCE[X]

SAY_SENTENCE["ROBOT_NAME"] -> your name
SAY_SENTENCE["TIME"] -> the time | what time it is | what time is it
SAY_SENTENCE["my team is tech united"] -> the name of your team
SAY_SENTENCE["DAY_OF_MONTH"] -> the day of the month
SAY_SENTENCE["DAY_OF_WEEK"] -> the day of the week
SAY_SENTENCE["TODAY"] -> what day is today | me what day it is | the date
SAY_SENTENCE["JOKE"] -> a joke
VP["action": "turn-toward-sound", "duration": "30"] -> show your sound source localization | look at me when i am talking to you

"""


class TestGrammarTest(unittest.TestCase):
    def test_grammar_test(self):
        """
        Tests if the grammar test tool works as expected.

        The test is based on the current (at the time of writing) grammar of the demo challenge (which is therefore
        copied to this file)
        """
        # Export a (default) robot env. This is necessary because the action server
        # loads knowledge on construction of actions.
        # It is desirable to improve this in the future.
        os.environ["ROBOT_ENV"] = "robotics_testlabs"

        # Construct a Mockbot object and add a number of static entities
        robot = Mockbot()
        robot.ed._static_entities = {
            "couch_table": from_entity_info(EntityInfo(id="couch_table")),
            "operator": from_entity_info(EntityInfo(id="operator")),
        }
        robot.ed._dynamic_entities = dict()

        # Perform the actual test
        test_grammar(robot=robot, grammar=GRAMMAR, grammar_target="T")
