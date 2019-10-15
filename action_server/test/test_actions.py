import unittest

from grammar_parser.cfgparser import CFGParser, Conjunct, Option, Rule
from robot_skills.mockbot import Mockbot
from action_server.actions import Say
from action_server.task_manager import TaskManager


GRAMMAR_DICT = {
    Say: [
        'V_SAY -> tell | say',
        'SAY_SENTENCE["my team is tech united"] -> the name of your team',
        'VP["action": "say", "sentence": X] -> V_SAY SAY_SENTENCE[X]',
    ]
}


class TestActions(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """
        Constructs the entire grammar and the parser. This is separated instead of defining the grammar in a single
        multi-line string to clearly indicate to which action a certain part of the grammar belongs to
        """
        cls.grammar = 'T[{actions : <A1>}] -> C[A1]\n'
        cls.grammar += 'C[{A}] -> VP[A]\n'
        for action in [Say]:
            cls.add_action_to_grammar(action)
        cls.parser = CFGParser.fromstring(cls.grammar)
        cls.grammar_target = 'T'
        cls.task_manager = TaskManager(robot=Mockbot())

    @classmethod
    def add_action_to_grammar(cls, action):
        for line in GRAMMAR_DICT[action]:
            cls.grammar += line
            cls.grammar += "\n"

    def test_say(self):
        config = self.parser.parse_raw(
            target=self.grammar_target,
            words='tell the name of your team',
        )
        config_result = self.task_manager.set_up_state_machine(recipe=config["actions"])
        self.assertTrue(config_result.succeeded)


