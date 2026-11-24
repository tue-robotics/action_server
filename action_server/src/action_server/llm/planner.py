"""
Turns a command in natural language into a recipe for the action server, with a language model.
"""
import json

import rospy

from .backend import OpenAIBackend
from .claude_backend import ClaudeBackend
from .claude_code_backend import ClaudeCodeBackend
from .knowledge import describe_vocabulary, find_knowledge_file, load_vocabulary, render_prompt
from .schema import PlanError, SAY_KEYWORDS, STATUS_OK, build_plan_schema, compile_plan, describe_actions
from .settings import CLAUDE_PROVIDERS, load_settings


def create_backend(settings):
    """
    :param settings: Settings
    :return: the backend for the provider in the settings
    """
    if settings.provider == "claude_code":
        return ClaudeCodeBackend(settings)
    if settings.provider in CLAUDE_PROVIDERS:
        return ClaudeBackend(settings)
    return OpenAIBackend(settings)


def describe_request(command, follow_ups=()):
    """
    :return: the command with the questions and answers that followed it, as told to the model. The robot's
        instructions file explains this form to the model.
    """
    lines = [command]
    for question, answer in follow_ups:
        lines += ['The robot asked: "{}"'.format(" ".join(question.split())),
                  'The person answered: "{}"'.format(" ".join(answer.split()))]
    return "\n".join(lines)


class Plan(object):
    def __init__(self, command, answer, recipe, completions):
        """
        :param command: what the user said
        :param answer: the plan as the model wrote it, see schema.build_plan_schema
        :param recipe: {"actions": [...]} for the action server; no actions when the status is not ok
        :param completions: the answers of the model that led to this plan, one per request
        """
        self.command = command
        self.answer = answer
        self.recipe = recipe
        self.completions = completions

    @property
    def status(self):
        return self.answer["status"]

    @property
    def ok(self):
        """True when there is a recipe to execute"""
        return self.status == STATUS_OK

    @property
    def reply(self):
        """What the robot should say to the user: a confirmation, a question, or why it cannot do it"""
        return self.answer.get("reply", "")

    @property
    def latency(self):
        return sum(c.latency for c in self.completions)


class LLMPlanner(object):
    def __init__(self, backend, vocabulary, instructions):
        """
        :param backend: object with complete(system_prompt, messages, schema), e.g. OpenAIBackend
        :param vocabulary: Vocabulary of the environment
        :param instructions: text of the robot's instructions file, with {{placeholders}}
        """
        self._backend = backend
        self._vocabulary = vocabulary
        self.schema = build_plan_schema(vocabulary)

        values = describe_vocabulary(vocabulary)
        values["actions"] = describe_actions()
        values["say_keywords"] = ", ".join(SAY_KEYWORDS)
        self.system_prompt = render_prompt(instructions, values)

    @classmethod
    def from_settings(cls, settings=None):
        """
        Planner as configured in .env/llm.env, for the environment in ROBOT_ENV

        :param settings: Settings, default load_settings()
        """
        settings = settings or load_settings()
        instructions = find_knowledge_file(settings.knowledge_file).read_text()
        rospy.loginfo("LLM planner: {}".format(settings.describe()))
        return cls(create_backend(settings), load_vocabulary(), instructions)

    def plan(self, command, follow_ups=()):
        """
        :param command: what the user said
        :param follow_ups: what was asked and answered about this command since, as (question of the robot, answer
            of the user) pairs. The model plans the whole command again with those answers.
        :return: Plan
        :raises BackendError: when the model cannot be reached
        :raises PlanError: when the model gives a plan that cannot be executed, also after it was told why
        """
        messages = [{"role": "user", "content": describe_request(command, follow_ups)}]
        completions = []

        # One correction: a plan that cannot be compiled is sent back once with the reason
        for attempt in range(2):
            completion = self._backend.complete(self.system_prompt, messages, self.schema)
            completions.append(completion)
            try:
                answer = json.loads(completion.text)
                recipe = compile_plan(answer, self._vocabulary)
            except (ValueError, PlanError) as e:
                rospy.logwarn("LLM planner: plan for '{}' rejected: {}".format(command, e))
                if attempt == 1:
                    raise PlanError("No usable plan for '{}': {}".format(command, e))
                messages += [{"role": "assistant", "content": completion.text},
                             {"role": "user", "content": "That plan cannot be executed: {}. "
                                                         "Give a corrected plan.".format(e)}]
                continue

            rospy.loginfo("LLM planner: '{}' -> {} ({:.1f} s)".format(command, recipe, completion.latency))
            return Plan(command, answer, recipe, completions)
