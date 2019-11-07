import unittest
import voluptuous

# from grammar_parser.cfgparser import CFGParser, Conjunct, Option, Rule
from robot_skills.mockbot import Mockbot
# from action_server.actions import Say
from action_server.task_manager import TaskManager
from action_server.actions.util.entities_from_description import EntitySchema
#
#
# GRAMMAR_DICT = {
#     Say: [
#         'V_SAY -> tell | say',
#         'SAY_SENTENCE["my team is tech united"] -> the name of your team',
#         'VP["action": "say", "sentence": X] -> V_SAY SAY_SENTENCE[X]',
#     ]
# }
#
#
# class TestActions(unittest.TestCase):
#     @classmethod
#     def setUpClass(cls):
#         """
#         Constructs the entire grammar and the parser. This is separated instead of defining the grammar in a single
#         multi-line string to clearly indicate to which action a certain part of the grammar belongs to
#         """
#         cls.grammar = 'T[{actions : <A1>}] -> C[A1]\n'
#         cls.grammar += 'C[{A}] -> VP[A]\n'
#         for action in [Say]:
#             cls.add_action_to_grammar(action)
#         cls.parser = CFGParser.fromstring(cls.grammar)
#         cls.grammar_target = 'T'
#         cls.task_manager = TaskManager(robot=Mockbot())
#
#     @classmethod
#     def add_action_to_grammar(cls, action):
#         for line in GRAMMAR_DICT[action]:
#             cls.grammar += line
#             cls.grammar += "\n"
#
#     def test_say(self):
#         config = self.parser.parse_raw(
#             target=self.grammar_target,
#             words='tell the name of your team',
#         )
#         config_result = self.task_manager.set_up_state_machine(recipe=config["actions"])
#         self.assertTrue(config_result.succeeded)


class TestActions(unittest.TestCase):
    """
    Tests a number of actions by setting up the corresponding state machine
    """

    def setUp(self):
        self.task_manager = TaskManager(robot=Mockbot())

    def test_entity_schema(self):
        # Check 'correct' options
        EntitySchema({"id": "foo"})
        EntitySchema({"type": "foo"})
        EntitySchema({"location": {
            "id": "bar",
        }})
        EntitySchema({"location": {
            "id": "bar",
            "area": "on",
        }})

        # Make sure that bad cases fail
        # Key "foo" does not make any sense
        with self.assertRaises(voluptuous.Invalid):
            EntitySchema({"foo": "bar"})

        # Location does not have an ID defined (but is directly ID)
        # EntitySchema({"location": "foo"})
        with self.assertRaises(voluptuous.Invalid):
            EntitySchema({"location": "foo"})

        # Test empty dict
        with self.assertRaises(voluptuous.InInvalid):
            EntitySchema({})

        # id and type are exclusive
        with self.assertRaises(voluptuous.InInvalid):
            EntitySchema({"id": "foo", "type": "bar"})

    def test_find(self):
        config_data = {"action": "find"}
        config_result = self.task_manager.set_up_state_machine(recipe=[config_data])
        self.assertTrue(config_result.succeeded, "Configuration of find failed: {}".format(config_result))

    def test_look_at(self):
        print("Testing look at")
        # Usual situation
        config_data = {"action": "look-at",
                       "entity": {"id": "cabinet"}
                       }
        config_result = self.task_manager.set_up_state_machine(recipe=[config_data])
        self.assertTrue(config_result.succeeded, "Configuration of lookat failed: {}".format(config_result))

        # Missing information
        config_data = {"action": "look-at",
                       }
        config_result = self.task_manager.set_up_state_machine(recipe=[config_data])
        self.assertFalse(config_result.succeeded, "Configuration of lookat failed: {}".format(config_result))
        # The following '].' is due to peculiarities in TaskManager.set_up_state_machine
        self.assertIn("].entity", config_result.missing_field)

        # Wrong definition of entity
        with self.assertRaises(Exception):
            config_data = {"action": "look-at",
                           "entity": "cabinet",
                           }
            self.task_manager.set_up_state_machine(recipe=[config_data])

        # Superfluous data
        with self.assertRaises(Exception):
            config_data = {"action": "look-at",
                           "entity": {"id": "cabinet"},
                           "foo": "bar",
                           }
            self.task_manager.set_up_state_machine(recipe=[config_data])

    @unittest.skip("Cannot test 'Say' before being able to test 'Find'")
    def test_say(self):
        pass
