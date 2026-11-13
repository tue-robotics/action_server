import json
import os
import tempfile
import unittest
from types import SimpleNamespace

from action_server.action_factory import ActionFactory
from action_server.llm import BackendError, ClaudeBackend, LLMPlanner, OpenAIBackend, PlanError, Settings, \
    SettingsError, Vocabulary, build_plan_schema, compile_plan, load_settings
from action_server.llm.backend import Completion
from action_server.llm.planner import create_backend
from action_server.llm.knowledge import KnowledgeError, render_prompt
from action_server.llm.schema import ACTIONS

VOCABULARY = Vocabulary(
    rooms=["kitchen", "living_room"],
    furniture=[{"name": "dinner_table", "room": "kitchen", "manipulation": True},
               {"name": "cabinet", "room": "kitchen", "manipulation": True},
               {"name": "couch", "room": "living_room", "manipulation": False}],
    objects=[{"name": "coke", "category": "drink"}, {"name": "apple", "category": "food"}],
    people=["anna"],
    usual_locations={"drink": "dinner_table"})


def step(action, **roles):
    """A step as the model writes it: every role present, null when unused"""
    full = {"action": action, "object": None, "person": None, "source": None, "destination": None, "sentence": None}
    full.update(roles)
    return full


def plan(*steps, **fields):
    full = {"understanding": "", "substitutions": [], "status": "ok", "steps": list(steps), "reply": ""}
    full.update(fields)
    return full


def actions(*steps):
    return compile_plan(plan(*steps), VOCABULARY)["actions"]


class FakeBackend(object):
    """Gives prepared answers and records what it was asked"""
    def __init__(self, *answers):
        self._answers = list(answers)
        self.requests = []

    def complete(self, system_prompt, messages, schema):
        self.requests.append(list(messages))
        return Completion(text=json.dumps(self._answers.pop(0)), latency=0.1)


class TestSchema(unittest.TestCase):
    def test_every_action_exists_in_the_action_server(self):
        registered = set(ActionFactory().get_action_names())
        self.assertEqual(set(), {spec.name for spec in ACTIONS} - registered)

    def test_schema_follows_the_rules_of_structured_output(self):
        """Every object lists all its properties as required and allows no others"""
        def check(node):
            if isinstance(node, dict):
                if node.get("type") == "object":
                    self.assertEqual(sorted(node["properties"]), sorted(node["required"]))
                    self.assertIs(node["additionalProperties"], False)
                for value in node.values():
                    check(value)
            elif isinstance(node, list):
                for value in node:
                    check(value)

        check(build_plan_schema(VOCABULARY))

    def test_roles_can_only_take_known_names(self):
        definitions = build_plan_schema(VOCABULARY)["$defs"]
        self.assertEqual(["coke", "apple", "drink", "food", "it"], definitions["object"]["enum"])
        self.assertEqual(["operator", "someone", "anna"], definitions["person"]["enum"])
        self.assertEqual(["kitchen", "living_room", "dinner_table", "cabinet", "couch"],
                         definitions["location"]["enum"])


class TestCompile(unittest.TestCase):
    """The recipes are the ones the GPSR grammar gives for the same commands"""

    def test_navigate_to_room_and_furniture(self):
        self.assertEqual([{"action": "navigate-to", "target-location": {"id": "kitchen", "type": "room"}}],
                         actions(step("navigate-to", destination="kitchen")))
        self.assertEqual([{"action": "navigate-to", "target-location": {"id": "cabinet"}}],
                         actions(step("navigate-to", destination="cabinet")))

    def test_bring_to_operator(self):
        self.assertEqual([{"action": "hand-over",
                           "object": {"type": "coke"},
                           "source-location": {"id": "dinner_table"},
                           "target-location": {"type": "person", "id": "operator"}}],
                         actions(step("hand-over", object="coke", person="operator", source="dinner_table")))

    def test_bring_to_person_at_location(self):
        self.assertEqual({"type": "person", "id": "anna", "location": {"id": "living_room", "type": "room"}},
                         actions(step("hand-over", object="coke", person="anna",
                                      destination="living_room"))[0]["target-location"])

    def test_pick_up_then_place_it(self):
        self.assertEqual([{"action": "pick-up", "object": {"type": "apple"}, "source-location": {"id": "cabinet"}},
                          {"action": "place", "object": {"type": "reference"},
                           "target-location": {"id": "dinner_table"}}],
                         actions(step("pick-up", object="apple", source="cabinet"),
                                 step("place", object="it", destination="dinner_table")))

    def test_place_without_object_places_the_held_object(self):
        self.assertEqual({"type": "reference"}, actions(step("place", destination="cabinet"))[0]["object"])

    def test_find_person_and_category(self):
        self.assertEqual([{"action": "find", "object": {"type": "person"}},
                          {"action": "find", "object": {"type": "person", "id": "anna"},
                           "source-location": {"id": "kitchen", "type": "room"}},
                          {"action": "find", "object": {"category": "drink"},
                           "source-location": {"id": "dinner_table"}}],
                         actions(step("find", person="someone"),
                                 step("find", person="anna", source="kitchen"),
                                 step("find", object="drink", source="dinner_table")))

    def test_say_keeps_the_recipe_parseable(self):
        self.assertEqual([{"action": "say", "sentence": "joke"}, {"action": "say", "sentence": "its a nice day"}],
                         actions(step("say", sentence="joke"), step("say", sentence="it's a \"nice\"\nday")))

    def test_count_follow_and_guide(self):
        self.assertEqual([{"action": "count-and-tell", "object": {"type": "apple"}, "location": {"id": "cabinet"}},
                          {"action": "follow", "target": {"type": "person", "id": "operator"}},
                          {"action": "guide", "object": {"type": "person", "id": "anna"},
                           "source-location": {"id": "kitchen", "type": "room"},
                           "target-location": {"id": "couch"}}],
                         actions(step("count-and-tell", object="apple", source="cabinet"),
                                 step("follow", person="operator"),
                                 step("guide", person="anna", source="kitchen", destination="couch")))

    def test_no_actions_when_the_robot_cannot_do_it(self):
        for status in ("needs_clarification", "unsupported"):
            self.assertEqual({"actions": []}, compile_plan(plan(status=status), VOCABULARY))

    def test_rejects_what_the_action_server_cannot_execute(self):
        rejected = {
            "unknown action": step("fly-to", destination="kitchen"),
            "unknown object": step("pick-up", object="stapler"),
            "unknown place": step("navigate-to", destination="garage"),
            "unknown person": step("follow", person="bob"),
            "navigate-to without target": step("navigate-to"),
            "say without sentence": step("say"),
            "category where an object is needed": step("hand-over", object="drink", person="operator"),
        }
        for reason, bad_step in rejected.items():
            with self.assertRaises(PlanError, msg=reason):
                compile_plan(plan(bad_step), VOCABULARY)

        with self.assertRaises(PlanError):
            compile_plan(plan(), VOCABULARY)  # ok without steps
        with self.assertRaises(PlanError):
            compile_plan(plan(status="maybe"), VOCABULARY)


class TestPlanner(unittest.TestCase):
    INSTRUCTIONS = "<!-- for people -->\nActions:\n{{actions}}\nRooms: {{rooms}}\nObjects:\n{{objects}}"

    def test_prompt_has_the_names_of_the_environment(self):
        prompt = LLMPlanner(FakeBackend(), VOCABULARY, self.INSTRUCTIONS).system_prompt
        self.assertNotIn("for people", prompt)
        self.assertIn("Rooms: kitchen, living_room", prompt)
        self.assertIn("- drink (usually on dinner_table): coke", prompt)
        self.assertIn("- `hand-over`", prompt)

    def test_unknown_placeholder_is_an_error(self):
        with self.assertRaises(KnowledgeError):
            render_prompt("{{romos}}", {"rooms": "kitchen"})

    def test_plan_gives_the_recipe(self):
        backend = FakeBackend(plan(step("navigate-to", destination="kitchen"), reply="I will go to the kitchen"))
        result = LLMPlanner(backend, VOCABULARY, self.INSTRUCTIONS).plan("go to the kitchen")

        self.assertTrue(result.ok)
        self.assertEqual("I will go to the kitchen", result.reply)
        self.assertEqual({"actions": [{"action": "navigate-to", "target-location": {"id": "kitchen", "type": "room"}}]},
                         result.recipe)
        self.assertEqual([[{"role": "user", "content": "go to the kitchen"}]], backend.requests)

    def test_follow_ups_are_told_to_the_model_with_the_command(self):
        backend = FakeBackend(plan(step("pick-up", object="apple", source="cabinet")))
        LLMPlanner(backend, VOCABULARY, self.INSTRUCTIONS).plan(
            "pick that up", follow_ups=[("Which object would you like me to pick up?", "the apple\n")])

        self.assertEqual('pick that up\n'
                         'The robot asked: "Which object would you like me to pick up?"\n'
                         'The person answered: "the apple"', backend.requests[0][0]["content"])

    def test_a_rejected_plan_is_sent_back_once_with_the_reason(self):
        backend = FakeBackend(plan(step("navigate-to")), plan(step("navigate-to", destination="kitchen")))
        result = LLMPlanner(backend, VOCABULARY, self.INSTRUCTIONS).plan("go there")

        self.assertTrue(result.ok)
        self.assertEqual(2, len(result.completions))
        self.assertIn("needs destination or person", backend.requests[1][-1]["content"])

    def test_gives_up_after_the_second_rejected_plan(self):
        backend = FakeBackend(plan(step("navigate-to")), plan(step("navigate-to")))
        with self.assertRaises(PlanError):
            LLMPlanner(backend, VOCABULARY, self.INSTRUCTIONS).plan("go there")
        self.assertEqual(2, len(backend.requests))


class StubClient(object):
    """Stands in for openai.OpenAI: records the request and returns one prepared message"""
    def __init__(self, content='{"status": "ok"}', refusal=None):
        self.request = None
        self._message = SimpleNamespace(content=content, refusal=refusal)
        self.chat = SimpleNamespace(completions=SimpleNamespace(create=self._create))

    def _create(self, **request):
        self.request = request
        return SimpleNamespace(choices=[SimpleNamespace(message=self._message, finish_reason="stop")],
                               usage=SimpleNamespace(prompt_tokens=10, completion_tokens=5))


class TestBackend(unittest.TestCase):
    SCHEMA = {"type": "object", "properties": {}, "required": [], "additionalProperties": False}
    MESSAGES = [{"role": "user", "content": "go to the kitchen"}]

    def _settings(self, **overrides):
        settings = Settings(provider="openai", model="some-model", api_key="secret")
        for name, value in overrides.items():
            setattr(settings, name, value)
        return settings

    def test_asks_for_structured_output(self):
        client = StubClient()
        completion = OpenAIBackend(self._settings(), client).complete("instructions", self.MESSAGES, self.SCHEMA)

        self.assertEqual('{"status": "ok"}', completion.text)
        self.assertEqual((10, 5), (completion.prompt_tokens, completion.completion_tokens))
        self.assertEqual("some-model", client.request["model"])
        self.assertEqual([{"role": "system", "content": "instructions"}] + self.MESSAGES, client.request["messages"])
        self.assertEqual({"type": "json_schema",
                          "json_schema": {"name": "plan", "strict": True, "schema": self.SCHEMA}},
                         client.request["response_format"])
        self.assertNotIn("temperature", client.request)  # reasoning models reject it

    def test_ping_is_the_smallest_request(self):
        client = StubClient(content="ok")
        answer = OpenAIBackend(self._settings(), client).ping()

        self.assertEqual("ok", answer.text)
        self.assertEqual({"model": "some-model",
                          "messages": [{"role": "user", "content": "Reply with the word ok."}]}, client.request)

    def test_temperature_is_sent_when_set(self):
        client = StubClient()
        OpenAIBackend(self._settings(temperature=0.0), client).complete("instructions", self.MESSAGES, self.SCHEMA)
        self.assertEqual(0.0, client.request["temperature"])

    def test_without_structured_output_the_schema_is_in_the_instructions(self):
        client = StubClient()
        OpenAIBackend(self._settings(response_format="json_object"), client).complete(
            "instructions", self.MESSAGES, self.SCHEMA)

        self.assertEqual({"type": "json_object"}, client.request["response_format"])
        self.assertIn(json.dumps(self.SCHEMA), client.request["messages"][0]["content"])

    def test_refusal_and_empty_answer_are_errors(self):
        for client in (StubClient(refusal="I cannot help with that"), StubClient(content=None)):
            with self.assertRaises(BackendError):
                OpenAIBackend(self._settings(), client).complete("instructions", self.MESSAGES, self.SCHEMA)


class StubPost(object):
    """Stands in for requests.Session: records the request and returns one prepared response"""
    def __init__(self, answer=None, status_code=200):
        self.headers = {}
        self.request = None
        self._answer = answer if answer is not None else {
            "stop_reason": "end_turn",
            "content": [{"type": "thinking", "thinking": ""}, {"type": "text", "text": '{"status": "ok"}'}],
            "usage": {"input_tokens": 10, "output_tokens": 5}}
        self._status_code = status_code

    def post(self, url, json, timeout):
        self.request = SimpleNamespace(url=url, headers=dict(self.headers), body=json, timeout=timeout)
        return SimpleNamespace(status_code=self._status_code, json=self._json, text=str(self._answer),
                               headers={"request-id": "req_1"})

    def _json(self):
        if isinstance(self._answer, str):
            raise ValueError("not JSON")
        return self._answer


class TestClaudeBackend(unittest.TestCase):
    SCHEMA = TestBackend.SCHEMA
    MESSAGES = TestBackend.MESSAGES

    def _settings(self, **overrides):
        settings = Settings(provider="foundry", model="my-deployment", api_key="secret",
                            base_url="https://resource.services.ai.azure.com/anthropic")
        for name, value in overrides.items():
            setattr(settings, name, value)
        return settings

    def test_asks_for_structured_output(self):
        post = StubPost()
        completion = ClaudeBackend(self._settings(), post).complete("instructions", self.MESSAGES, self.SCHEMA)

        self.assertEqual('{"status": "ok"}', completion.text)  # the text block, not the thinking block before it
        self.assertEqual((10, 5), (completion.prompt_tokens, completion.completion_tokens))
        self.assertEqual("https://resource.services.ai.azure.com/anthropic/v1/messages", post.request.url)
        self.assertEqual({"x-api-key": "secret", "anthropic-version": "2026-10-21"}, post.request.headers)
        self.assertEqual((3.05, 20.0), post.request.timeout)  # (to connect, to answer)
        self.assertEqual({"model": "my-deployment", "max_tokens": 16000, "system": "instructions",
                          "messages": self.MESSAGES,
                          "output_config": {"format": {"type": "json_schema", "schema": self.SCHEMA}}},
                         post.request.body)

    def test_ping_is_the_smallest_request(self):
        post = StubPost({"stop_reason": "end_turn", "content": [{"type": "text", "text": "ok"}],
                         "usage": {"input_tokens": 15, "output_tokens": 5}})
        answer = ClaudeBackend(self._settings(effort="low"), post).ping()

        self.assertEqual("ok", answer.text)
        self.assertEqual({"model": "my-deployment", "max_tokens": 64,
                          "messages": [{"role": "user", "content": "Reply with the word ok."}]}, post.request.body)

    def test_ping_fails_on_a_wrong_key(self):
        post = StubPost({"error": {"message": "invalid key"}}, status_code=401)
        with self.assertRaises(BackendError):
            ClaudeBackend(self._settings(), post).ping()

    def test_effort_is_sent_when_set(self):
        post = StubPost()
        ClaudeBackend(self._settings(effort="low"), post).complete("instructions", self.MESSAGES, self.SCHEMA)
        self.assertEqual("low", post.request.body["output_config"]["effort"])

    def test_without_structured_output_the_schema_is_in_the_instructions(self):
        post = StubPost()
        ClaudeBackend(self._settings(response_format="json_object"), post).complete(
            "instructions", self.MESSAGES, self.SCHEMA)

        self.assertNotIn("output_config", post.request.body)
        self.assertIn(json.dumps(self.SCHEMA), post.request.body["system"])

    def test_failures_are_backend_errors(self):
        failures = {
            "refusal": StubPost({"stop_reason": "refusal", "content": [], "stop_details": {"category": None}}),
            "cut off": StubPost({"stop_reason": "max_tokens", "content": [{"type": "text", "text": '{"sta'}]}),
            "no text": StubPost({"stop_reason": "end_turn", "content": []}),
            "http error": StubPost({"error": {"message": "invalid key"}}, status_code=401),
            "not json": StubPost("<html>gateway</html>"),
        }
        for reason, post in failures.items():
            with self.assertRaises(BackendError, msg=reason):
                ClaudeBackend(self._settings(), post).complete("instructions", self.MESSAGES, self.SCHEMA)

    def test_error_names_the_cause_but_not_the_key(self):
        post = StubPost({"error": {"message": "invalid key"}}, status_code=401)
        with self.assertRaises(BackendError) as context:
            ClaudeBackend(self._settings(), post).complete("instructions", self.MESSAGES, self.SCHEMA)
        self.assertIn("HTTP 401", str(context.exception))
        self.assertIn("invalid key", str(context.exception))
        self.assertNotIn("secret", str(context.exception))


class TestSettings(unittest.TestCase):
    def _env_file(self, text):
        handle, path = tempfile.mkstemp(suffix=".env")
        with os.fdopen(handle, "w") as f:
            f.write(text)
        self.addCleanup(os.remove, path)
        return path

    def test_openai_from_file(self):
        settings = load_settings(self._env_file("# comment\nLLM_MODEL=some-model\nOPENAI_API_KEY='secret'\n"),
                                 environ={})
        self.assertEqual(("openai", "some-model", "secret", "json_schema"),
                         (settings.provider, settings.model, settings.api_key, settings.response_format))
        self.assertNotIn("secret", settings.describe())

    def test_environment_overrules_the_file(self):
        settings = load_settings(self._env_file("LLM_MODEL=from-file\nOPENAI_API_KEY=secret\n"),
                                 environ={"LLM_MODEL": "from-environment"})
        self.assertEqual("from-environment", settings.model)

    def test_azure_needs_its_own_settings(self):
        path = self._env_file("LLM_PROVIDER=azure\nLLM_MODEL=deployment\nAZURE_OPENAI_API_KEY=secret\n"
                              "OPENAI_API_VERSION=2024-10-21\n")
        with self.assertRaises(SettingsError) as context:
            load_settings(path, environ={})
        self.assertIn("AZURE_OPENAI_ENDPOINT", str(context.exception))

    def test_foundry_accepts_the_resource_name_or_the_target_uri_of_the_portal(self):
        common = "LLM_PROVIDER=foundry\nLLM_MODEL=deployment\nANTHROPIC_FOUNDRY_API_KEY=secret\nLLM_EFFORT=low\n"
        expected = "https://resource.services.ai.azure.com/anthropic"
        for location in ("ANTHROPIC_FOUNDRY_RESOURCE=resource",
                         "ANTHROPIC_FOUNDRY_BASE_URL=https://resource.services.ai.azure.com/anthropic/",
                         "ANTHROPIC_FOUNDRY_BASE_URL=https://resource.services.ai.azure.com/anthropic/v1/messages"):
            settings = load_settings(self._env_file(common + location + "\n"), environ={})
            self.assertEqual((expected, "secret", "low"), (settings.base_url, settings.api_key, settings.effort),
                             msg=location)

        with self.assertRaises(SettingsError) as context:
            load_settings(self._env_file(common), environ={})
        self.assertIn("ANTHROPIC_FOUNDRY_RESOURCE", str(context.exception))

    def test_anthropic_uses_the_claude_api(self):
        settings = load_settings(self._env_file("LLM_PROVIDER=anthropic\nLLM_MODEL=m\nANTHROPIC_API_KEY=secret\n"),
                                 environ={})
        self.assertEqual("https://api.anthropic.com", settings.base_url)

    def test_the_provider_decides_the_backend(self):
        self.assertIsInstance(create_backend(Settings("foundry", "m", "k", base_url="https://x/anthropic")),
                              ClaudeBackend)
        self.assertIsInstance(create_backend(Settings("openai", "m", "k")), OpenAIBackend)

    def test_missing_key_names_the_setting_and_the_file(self):
        path = self._env_file("LLM_MODEL=some-model\nOPENAI_API_KEY=\n")
        with self.assertRaises(SettingsError) as context:
            load_settings(path, environ={})
        self.assertIn("OPENAI_API_KEY", str(context.exception))
        self.assertIn(path, str(context.exception))


if __name__ == "__main__":
    unittest.main()
