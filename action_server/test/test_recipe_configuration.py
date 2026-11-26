"""
Regression tests of the task format: which actions a recipe configures into, on the mock robot.

A recipe is what the parsers (the LLM planner, the GPSR grammar, the NER) send to the action server. The task manager
turns it into a chain of actions, adding the steps a recipe leaves out: "bring me the coke from the dinner table" is one
hand-over, which configures into Find > PickUp > NavigateTo > HandOver. These tests pin those chains, so that a change
of the format or of an action shows up as a changed chain.

No robot and no roscore are needed: the mock robot only has to know the furniture and rooms of the environment. Every
recipe is configured in a child process with a time limit, so a recipe that makes the task manager loop forever fails
its test instead of blocking the whole run.

The tests marked expectedFailure describe known bugs: they state the behaviour we want, fail today, and must lose the
mark when the bug is fixed (an unexpected success fails the run).
"""
import copy
import io
import contextlib
import logging
import multiprocessing
import os
import unittest

os.environ["ROBOT_ENV"] = "impuls"

from ed.entity import from_entity_info  # noqa: E402
from ed_msgs.msg import EntityInfo  # noqa: E402
from grammar_parser.cfgparser import CFGParser  # noqa: E402
from robocup_knowledge import knowledge_loader  # noqa: E402
from robot_skills import get_robot  # noqa: E402

from action_server.llm import compile_plan  # noqa: E402
from action_server.llm.knowledge import load_vocabulary  # noqa: E402
from action_server.task_manager import TaskManager  # noqa: E402

TIME_LIMIT = 20.0  # seconds to configure one recipe; a normal recipe takes well under one
HANG = "hang"

_FORK = multiprocessing.get_context("fork")
_ROBOT = None


def _robot():
    """The mock robot, with the furniture and rooms of the environment as static entities"""
    global _ROBOT
    if _ROBOT is None:
        logging.getLogger("rosout").setLevel(logging.CRITICAL)
        common = knowledge_loader.load_knowledge("common")
        names = [location["name"] for location in common.locations] + list(common.location_rooms) + ["operator"]
        _ROBOT = get_robot("mockbot")
        _ROBOT.ed._static_entities = {name: from_entity_info(EntityInfo(id=name)) for name in names}
        _ROBOT.ed._dynamic_entities = {}
    return _ROBOT


def _configure_in_child(robot, recipe, queue):
    task_manager = TaskManager(robot=robot)
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        try:
            result = task_manager.set_up_state_machine(recipe=copy.deepcopy(recipe))
            queue.put({"succeeded": result.succeeded, "missing_field": result.missing_field,
                       "message": result.message.strip(),
                       "chain": [action.get_name() for action in task_manager._action_sequence]})
        except Exception as e:  # a crash is a result too
            queue.put({"succeeded": False, "missing_field": "", "message": "crashed: {}: {}".format(
                type(e).__name__, e), "chain": []})


def configure(recipe):
    """
    Configure a recipe (the list of actions) on the mock robot.

    Example: configure([{"action": "pick-up", "object": {"type": "coke"}, "source-location": {"id": "dinner_table"}}])
    gives {"succeeded": True, "missing_field": "", "message": "", "chain": ["Find", "PickUp"]}.

    :return: dict with succeeded, missing_field (e.g. "[0].source-location"), message and chain (the action names),
        or HANG when the task manager did not finish within TIME_LIMIT
    """
    robot = _robot()
    queue = _FORK.Queue()
    child = _FORK.Process(target=_configure_in_child, args=(robot, recipe, queue))
    child.start()
    child.join(TIME_LIMIT)
    if child.is_alive():
        child.kill()
        child.join()
        return HANG
    return queue.get(timeout=5)


# Short ways to write recipes
def at(furniture):
    return {"id": furniture}


def room(name):
    return {"type": "room", "id": name}


def person(name, location=None):
    p = {"type": "person", "id": name}
    if location:
        p["location"] = location
    return p


OPERATOR = person("operator")
COKE = {"type": "coke"}
APPLE = {"type": "apple"}
DRINK = {"category": "drink"}
FOOD = {"category": "food"}
IT = {"type": "reference"}


class RecipeTestCase(unittest.TestCase):
    def assertChain(self, recipe, chain):
        """The recipe configures, into exactly this chain of actions"""
        result = configure(recipe)
        self.assertNotEqual(HANG, result, "configuring {} did not finish".format(recipe))
        self.assertTrue(result["succeeded"], "{} did not configure: {}".format(recipe, result))
        self.assertEqual(chain, result["chain"], recipe)

    def assertAsks(self, recipe, missing_field):
        """The recipe does not configure: HERO has to ask for this field"""
        result = configure(recipe)
        self.assertNotEqual(HANG, result, "configuring {} did not finish".format(recipe))
        self.assertFalse(result["succeeded"], recipe)
        self.assertEqual(missing_field, result["missing_field"], recipe)


class TestObjects(RecipeTestCase):
    def test_find_an_object_by_type(self):
        self.assertChain([{"action": "find", "object": COKE, "source-location": at("dinner_table")}], ["Find"])

    def test_find_an_object_by_category(self):
        self.assertChain([{"action": "find", "object": DRINK, "source-location": at("dinner_table")}], ["Find"])

    def test_pick_up_by_type(self):
        self.assertChain([{"action": "pick-up", "object": COKE, "source-location": at("dinner_table")}],
                         ["Find", "PickUp"])

    def test_pick_up_by_category(self):
        self.assertChain([{"action": "pick-up", "object": DRINK, "source-location": at("dinner_table")}],
                         ["Find", "PickUp"])

    def test_pick_up_needs_a_source(self):
        self.assertAsks([{"action": "pick-up", "object": COKE}], "[0].source-location")

    def test_hand_over_fetches_the_object(self):
        self.assertChain([{"action": "hand-over", "object": COKE, "source-location": at("dinner_table"),
                           "target-location": OPERATOR}],
                         ["Find", "PickUp", "NavigateTo", "HandOver"])

    def test_hand_over_to_a_named_person_finds_the_person(self):
        self.assertChain([{"action": "hand-over", "object": COKE, "source-location": at("dinner_table"),
                           "target-location": person("anna", at("couch_long"))}],
                         ["Find", "PickUp", "Find", "NavigateTo", "HandOver"])

    def test_place_fetches_the_object(self):
        self.assertChain([{"action": "place", "object": COKE, "source-location": at("dinner_table"),
                           "target-location": at("cabinet")}],
                         ["Find", "PickUp", "NavigateTo", "Place"])

    def test_place_what_was_picked_up(self):
        self.assertChain([{"action": "pick-up", "object": COKE, "source-location": at("dinner_table")},
                          {"action": "place", "object": IT, "target-location": at("cabinet")}],
                         ["Find", "PickUp", "NavigateTo", "Place"])

    def test_place_what_was_found(self):
        # The second Find is redundant (an action added before another gets no context), but it is today's chain
        self.assertChain([{"action": "find", "object": COKE, "source-location": at("dinner_table")},
                          {"action": "place", "object": IT, "target-location": at("cabinet")}],
                         ["Find", "Find", "PickUp", "NavigateTo", "Place"])

    def test_count_objects(self):
        self.assertChain([{"action": "count-and-tell", "object": COKE, "location": at("dinner_table")}],
                         ["CountAndTell"])

    def test_inspect_furniture(self):
        self.assertChain([{"action": "inspect", "entity": at("dinner_table")}], ["Inspect"])


class TestCategories(RecipeTestCase):
    """An object asked for by category ("a drink"): HERO fetches one, and knows what it holds"""

    def test_hand_over_a_category_fetches_it(self):
        self.assertChain([{"action": "hand-over", "object": DRINK, "source-location": at("dinner_table"),
                           "target-location": OPERATOR}],
                         ["Find", "PickUp", "NavigateTo", "HandOver"])

    def test_place_a_category_fetches_it(self):
        # Before, an arm "holding None" matched an empty arm, so place skipped the fetch
        self.assertChain([{"action": "place", "object": DRINK, "source-location": at("dinner_table"),
                           "target-location": at("cabinet")}],
                         ["Find", "PickUp", "NavigateTo", "Place"])

    def test_hand_over_what_was_found_by_category(self):
        # Before, pick-up and hand-over disagreed on "already holding it" and asked each other forever
        self.assertChain([{"action": "find", "object": DRINK, "source-location": at("dinner_table")},
                          {"action": "hand-over", "object": IT, "target-location": OPERATOR}],
                         ["Find", "Find", "PickUp", "NavigateTo", "HandOver"])

    def test_a_held_coke_is_a_drink(self):
        self.assertChain([{"action": "pick-up", "object": COKE, "source-location": at("dinner_table")},
                          {"action": "hand-over", "object": DRINK, "target-location": OPERATOR}],
                         ["Find", "PickUp", "NavigateTo", "HandOver"])

    def test_a_held_drink_is_not_food(self):
        # Before, "None == None" made the held drink count as the requested food; now the food must be fetched, and
        # the step does not say where from
        self.assertAsks([{"action": "pick-up", "object": DRINK, "source-location": at("dinner_table")},
                         {"action": "hand-over", "object": FOOD, "target-location": OPERATOR}],
                        "[1].source-location")

    def test_a_held_drink_is_not_necessarily_a_coke(self):
        self.assertAsks([{"action": "pick-up", "object": DRINK, "source-location": at("dinner_table")},
                         {"action": "hand-over", "object": COKE, "target-location": OPERATOR}],
                        "[1].source-location")

    def test_an_object_without_type_or_category_fails_instead_of_hanging(self):
        result = configure([{"action": "pick-up", "object": {}, "source-location": at("dinner_table")}])
        self.assertNotEqual(HANG, result)
        self.assertFalse(result["succeeded"])
        self.assertEqual("", result["missing_field"])


class TestKnownBugs(RecipeTestCase):
    """Known bugs of today's actions; each test states the wanted behaviour"""

    @unittest.expectedFailure
    def test_an_explicit_drive_keeps_the_held_object(self):
        # Today: a step only inherits the context of the step before it, so after a navigate-to step HERO forgets what
        # it holds and asks "What would you like me to pick up?"
        self.assertChain([{"action": "pick-up", "object": COKE, "source-location": at("dinner_table")},
                          {"action": "navigate-to", "target-location": at("cabinet")},
                          {"action": "place", "object": IT, "target-location": at("cabinet")}],
                         ["Find", "PickUp", "NavigateTo", "Place"])


class TestPeopleAndPlaces(RecipeTestCase):
    def test_navigate_to_a_room(self):
        self.assertChain([{"action": "navigate-to", "target-location": room("kitchen")}], ["NavigateTo"])

    def test_navigate_to_furniture(self):
        self.assertChain([{"action": "navigate-to", "target-location": at("dinner_table")}], ["NavigateTo"])

    def test_navigate_to_the_operator(self):
        self.assertChain([{"action": "navigate-to", "target-location": OPERATOR}], ["NavigateTo"])

    def test_find_someone(self):
        self.assertChain([{"action": "find", "object": {"type": "person"}, "source-location": room("kitchen")}],
                         ["Find"])

    def test_find_a_named_person(self):
        self.assertChain([{"action": "find", "object": person("anna"), "source-location": room("kitchen")}],
                         ["Find"])

    def test_guide_finds_the_person_first(self):
        self.assertChain([{"action": "guide", "object": person("anna"), "source-location": room("kitchen"),
                           "target-location": room("living_room")}],
                         ["Find", "Guide"])

    def test_follow_the_operator(self):
        self.assertChain([{"action": "follow", "target": OPERATOR}], ["Follow"])

    def test_follow_a_named_person_finds_the_person_first(self):
        self.assertChain([{"action": "follow", "target": person("anna"), "location-from": room("kitchen")}],
                         ["Find", "Follow"])

    def test_tell_the_name_of_a_person(self):
        self.assertChain([{"action": "tell-name-of-person", "location": room("kitchen")}],
                         ["Find", "TellNameOfPerson"])

    def test_say(self):
        self.assertChain([{"action": "say", "sentence": "hello"}], ["Say"])


class TestGrammarRecipes(RecipeTestCase):
    """The GPSR grammar of the environment: the recipe of a sentence, and its chain (golden recipes)"""

    @classmethod
    def setUpClass(cls):
        knowledge = knowledge_loader.load_knowledge("challenge_gpsr")
        cls.parser = CFGParser.fromstring(knowledge.grammar)
        cls.target = knowledge.grammar_target

    def parse(self, sentence):
        return self.parser.parse_raw(self.target, sentence.split())["actions"]

    def test_find_a_category_in_a_room(self):
        recipe = self.parse("find the drink in the kitchen")
        self.assertEqual([{"action": "find", "object": DRINK, "source-location": room("kitchen")}], recipe)
        self.assertChain(recipe, ["Find"])

    def test_bring_me(self):
        recipe = self.parse("bring me the coke from the dinner_table")
        self.assertEqual([{"action": "hand-over", "source-location": at("dinner_table"),
                           "target-location": OPERATOR, "object": COKE}], recipe)
        self.assertChain(recipe, ["Find", "PickUp", "NavigateTo", "HandOver"])

    def test_bring_to_a_person_without_a_source(self):
        recipe = self.parse("bring the coke to anna at the couch_long")
        self.assertEqual([{"action": "hand-over", "target-location": person("anna", at("couch_long")),
                           "object": COKE}], recipe)
        self.assertAsks(recipe, "[0].source-location")

    def test_put_it(self):
        recipe = self.parse("put it on the dinner_table")
        self.assertEqual([{"action": "place", "target-location": at("dinner_table"), "object": IT}], recipe)
        # Nothing is held and "it" refers to nothing, so HERO asks what to pick up
        self.assertAsks(recipe, "[0].object")

    def test_go_to_a_room(self):
        recipe = self.parse("go to the kitchen")
        self.assertEqual([{"action": "navigate-to", "target-location": room("kitchen")}], recipe)
        self.assertChain(recipe, ["NavigateTo"])

    def test_take_and_put(self):
        recipe = self.parse("take the apple from the dinner_table and put it on the cabinet")
        self.assertEqual([{"action": "pick-up", "object": APPLE, "source-location": at("dinner_table")},
                          {"action": "place", "target-location": at("cabinet"), "object": IT}], recipe)
        self.assertChain(recipe, ["Find", "PickUp", "NavigateTo", "Place"])


class TestPlannerRecipes(RecipeTestCase):
    """Plans of the LLM planner, compiled for the environment: the recipe the planner sends, and its chain"""

    @classmethod
    def setUpClass(cls):
        cls.vocabulary = load_vocabulary()

    def compile(self, *steps):
        full_steps = []
        for s in steps:
            full = {"action": None, "object": None, "person": None, "source": None, "destination": None,
                    "sentence": None}
            full.update(s)
            full_steps.append(full)
        plan = {"understanding": "", "substitutions": [], "status": "ok", "steps": full_steps, "reply": ""}
        return compile_plan(plan, self.vocabulary)["actions"]

    def test_bring_me_a_coke(self):
        recipe = self.compile({"action": "hand-over", "object": "coke", "person": "operator",
                               "source": "dinner_table"})
        self.assertChain(recipe, ["Find", "PickUp", "NavigateTo", "HandOver"])

    def test_find_a_drink(self):
        recipe = self.compile({"action": "find", "object": "drink", "source": "dinner_table"})
        self.assertEqual(DRINK, recipe[0]["object"])
        self.assertChain(recipe, ["Find"])

    def test_bring_me_a_drink(self):
        recipe = self.compile({"action": "hand-over", "object": "drink", "person": "operator",
                               "source": "dinner_table"})
        self.assertEqual(DRINK, recipe[0]["object"])
        self.assertChain(recipe, ["Find", "PickUp", "NavigateTo", "HandOver"])

    def test_put_some_food_on_the_cabinet(self):
        recipe = self.compile({"action": "place", "object": "food", "source": "dinner_table",
                               "destination": "cabinet"})
        self.assertEqual(FOOD, recipe[0]["object"])
        self.assertChain(recipe, ["Find", "PickUp", "NavigateTo", "Place"])

    def test_pick_up_and_place_it(self):
        recipe = self.compile({"action": "pick-up", "object": "apple", "source": "dinner_table"},
                              {"action": "place", "object": "it", "destination": "cabinet"})
        self.assertChain(recipe, ["Find", "PickUp", "NavigateTo", "Place"])

    def test_guide_a_person(self):
        recipe = self.compile({"action": "guide", "person": "anna", "source": "kitchen",
                               "destination": "living_room"})
        self.assertChain(recipe, ["Find", "Guide"])

    def test_go_to_a_room(self):
        recipe = self.compile({"action": "navigate-to", "destination": "kitchen"})
        self.assertChain(recipe, ["NavigateTo"])


if __name__ == "__main__":
    unittest.main()
