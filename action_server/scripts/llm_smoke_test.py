#!/usr/bin/env python
"""
Check the LLM task planner without a robot: send commands to the model and print the
plan it makes and the recipe the action server would get.

    llm_smoke_test.py --ping                one tiny request: are the key, the address and the model right?
    llm_smoke_test.py                       a few built-in commands
    llm_smoke_test.py "bring me a pepsi"    your own commands
    llm_smoke_test.py --configure ...       also give every recipe to the action server's task manager, with the
                                            mock robot, and show whether it accepts it and which actions it adds
    llm_smoke_test.py --show-prompt         print the instructions and the schema; no request is made

Needs the settings of .env/llm.env, see .env/README.md, and for everything but --ping also ROBOT_ENV.
--configure also needs a running roscore.
"""
import argparse
import copy
import json
import logging
import os
import sys

# Use the code of the checkout this script is in, also when that is a git worktree
# and the catkin workspace points at another checkout.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from action_server.llm import BackendError, KnowledgeError, LLMPlanner, PlanError, SettingsError, \
    load_settings, load_vocabulary  # noqa: E402
from action_server.llm.knowledge import find_knowledge_file  # noqa: E402
from action_server.llm.planner import create_backend  # noqa: E402
from action_server.llm.settings import read_values  # noqa: E402

COMMANDS = ["go to the kitchen",
            "bring me a coke from the dinner table",
            "could you get me a pepsi",
            "i am thirsty",
            "put the apple on the cabinet and then tell me a joke",
            "find anna in the living room and follow her",
            "bring me the stapler",
            "paint the wall blue"]


class Unconnected(object):
    """Stands in for the model when only the prompt is shown"""
    def complete(self, system_prompt, messages, schema):
        raise BackendError("no request is made with --show-prompt")


def make_configure(vocabulary):
    """
    :param vocabulary: Vocabulary; its rooms and furniture become the world model of the mock robot
    :return: function that gives a recipe to the task manager of the action server and describes the result
    """
    from ed.entity import from_entity_info
    from ed_msgs.msg import EntityInfo
    from robot_skills import get_robot

    from action_server.task_manager import TaskManager

    # The robot and the actions log a lot while they are set up
    logging.getLogger("rosout").setLevel(logging.CRITICAL)

    robot = get_robot("mockbot")
    robot.ed._static_entities = {name: from_entity_info(EntityInfo(id=name))
                                 for name in vocabulary.locations + ["operator"]}
    robot.ed._dynamic_entities = {}

    def configure(recipe):
        task_manager = TaskManager(robot=robot)
        try:
            result = task_manager.set_up_state_machine(recipe=copy.deepcopy(recipe["actions"]))
        except Exception as e:
            return "CRASHED on it. {}: {}".format(type(e).__name__, e)
        if result.succeeded:
            return "accepts it: " + " > ".join(action.get_name() for action in task_manager._action_sequence)
        if result.missing_field:
            return "would ask the user: {} ({})".format(result.message.strip(), result.missing_field)
        return "REJECTED it: {}".format(result.message)

    return configure


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("commands", nargs="*", help="commands to plan; default a built-in set")
    parser.add_argument("--configure", action="store_true",
                        help="also give every recipe to the task manager of the action server, with the mock robot; "
                             "needs a running roscore")
    parser.add_argument("--ping", action="store_true",
                        help="send one tiny request to check the key, the address and the model, and stop")
    parser.add_argument("--show-prompt", action="store_true",
                        help="print the instructions and the schema sent to the model, without calling it")
    parser.add_argument("--knowledge-file", help="instructions file; default LLM_KNOWLEDGE_FILE, then "
                                                 "$ROBOT_BRINGUP_PATH/parameters/llm/task_planner.md")
    args = parser.parse_args()

    if args.knowledge_file:
        os.environ["LLM_KNOWLEDGE_FILE"] = args.knowledge_file

    try:
        if args.ping:
            settings = load_settings()
            print("Settings: {}".format(settings.describe()))
            try:
                answer = create_backend(settings).ping()
            except BackendError as e:
                print("FAILED: {}".format(e))
                return 1
            print("OK: the model answered '{}' in {:.1f} s".format(answer.text.strip(), answer.latency))
            return 0

        if args.show_prompt:
            # The prompt does not depend on the keys, so this also works before they are filled in
            knowledge_file = find_knowledge_file(read_values()[0].get("LLM_KNOWLEDGE_FILE"))
            planner = LLMPlanner(Unconnected(), load_vocabulary(), knowledge_file.read_text())
            print("# Instructions ({})\n\n{}\n".format(knowledge_file, planner.system_prompt))
            print("# Schema\n\n{}".format(json.dumps(planner.schema, indent=2)))
            return 0

        settings = load_settings()
        print("Settings: {}".format(settings.describe()))
        print("ROBOT_ENV: {}\n".format(os.environ.get("ROBOT_ENV")))
        planner = LLMPlanner.from_settings(settings)
    except (SettingsError, KnowledgeError) as e:
        print("Cannot start: {}".format(e))
        return 2

    configure = None
    if args.configure:
        import rosgraph
        if not rosgraph.is_master_online():
            print("Cannot start: --configure needs a running roscore")
            return 2
        configure = make_configure(load_vocabulary())

    failures = 0
    for command in args.commands or COMMANDS:
        print("> {}".format(command))
        try:
            plan = planner.plan(command)
        except (BackendError, PlanError) as e:
            failures += 1
            print("  FAILED: {}\n".format(e))
            if isinstance(e, BackendError):
                # The service itself does not work; the other commands would fail the same way
                break
            continue

        tokens = sum((c.prompt_tokens or 0) + (c.completion_tokens or 0) for c in plan.completions)
        print("  understanding: {}".format(plan.answer.get("understanding")))
        for substitution in plan.answer.get("substitutions", []):
            print("  substitution:  {heard} -> {used}".format(**substitution))
        print("  status:        {}".format(plan.status))
        print("  reply:         {}".format(plan.reply))
        print("  recipe:        {}".format(json.dumps(plan.recipe)))
        if configure and plan.ok:
            verdict = configure(plan.recipe)
            failures += verdict.startswith(("CRASHED", "REJECTED"))
            print("  action server: {}".format(verdict))
        print("  {:.1f} s, {} tokens, {} request(s)\n".format(plan.latency, tokens, len(plan.completions)))

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
