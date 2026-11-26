"""
The plan the language model writes, and its translation to a recipe of the action server.

The model does not write the recipe itself. It fills in a small frame per step:

    {"action": "hand-over", "object": "coke", "person": "operator",
     "source": "dinner_table", "destination": null, "sentence": null}

An action plus five roles is all it has to decide. Every role except the sentence is a
choice from the names of the environment, so with structured output the model cannot
name an action, object, place or person that the robot does not know. compile_plan then
turns the frames into the recipe, and is the one place that knows how the action server
wants an entity described ({"type": "person", "id": "operator"}, {"type": "reference"}, ...).
"""

OBJECT = "object"
PERSON = "person"
SOURCE = "source"
DESTINATION = "destination"
SENTENCE = "sentence"
ROLES = (OBJECT, PERSON, SOURCE, DESTINATION, SENTENCE)

#: The object handled by the previous step ("it")
PREVIOUS_OBJECT = "it"
#: The person who gives the command ("me")
OPERATOR = "operator"
#: A person whose name is not known
SOMEONE = "someone"

STATUS_OK = "ok"
STATUS_NEEDS_CLARIFICATION = "needs_clarification"
STATUS_UNSUPPORTED = "unsupported"
STATUSES = (STATUS_OK, STATUS_NEEDS_CLARIFICATION, STATUS_UNSUPPORTED)

#: Sentences the say action composes itself. Any other sentence is spoken as it is.
SAY_KEYWORDS = ("time", "today", "tomorrow", "day_of_week", "day_of_month", "joke", "robot_name", "team_name",
                "country", "team_affiliation", "something_about_self")


class PlanError(Exception):
    """The plan of the model cannot be turned into a recipe"""


def _location(name, vocabulary):
    if name in vocabulary.rooms:
        return {"id": name, "type": "room"}
    return {"id": name}


def _object(name, vocabulary):
    if name == PREVIOUS_OBJECT:
        return {"type": "reference"}
    if name in vocabulary.categories:
        return {"category": name}
    return {"type": name}


def _person(name, vocabulary, location=None):
    if name == OPERATOR:
        # navigate-to only recognises the operator by this id; it then returns to where the task was given
        return {"type": "person", "id": OPERATOR}
    person = {"type": "person"}
    if name != SOMEONE:
        person["id"] = name
    if location:
        person["location"] = _location(location, vocabulary)
    return person


def _with_locations(recipe, step, vocabulary, **fields):
    """Add the location roles that are given, e.g. source="source-location" """
    for role, field in fields.items():
        if step[role]:
            recipe[field] = _location(step[role], vocabulary)
    return recipe


def _navigate_to(step, v):
    if step[PERSON]:
        return {"target-location": _person(step[PERSON], v, location=step[DESTINATION])}
    return {"target-location": _location(step[DESTINATION], v)}


def _find(step, v):
    target = _person(step[PERSON], v) if step[PERSON] else _object(step[OBJECT], v)
    return _with_locations({"object": target}, step, v, source="source-location")


def _pick_up(step, v):
    return _with_locations({"object": _object(step[OBJECT], v)}, step, v, source="source-location")


def _place(step, v):
    recipe = {"object": _object(step[OBJECT] or PREVIOUS_OBJECT, v)}
    return _with_locations(recipe, step, v, source="source-location", destination="target-location")


def _hand_over(step, v):
    recipe = {"object": _object(step[OBJECT] or PREVIOUS_OBJECT, v),
              "target-location": _person(step[PERSON], v, location=step[DESTINATION])}
    return _with_locations(recipe, step, v, source="source-location")


def _say(step, v):
    # The recipe travels as text; quotes and backslashes in a sentence would break it
    sentence = " ".join(step[SENTENCE].split())
    return {"sentence": "".join(c for c in sentence if c not in "'\"\\")}


def _follow(step, v):
    recipe = {"target": _person(step[PERSON], v)}
    return _with_locations(recipe, step, v, source="location-from", destination="location-to")


def _guide(step, v):
    recipe = {"object": _person(step[PERSON], v)}
    return _with_locations(recipe, step, v, source="source-location", destination="target-location")


def _count_and_tell(step, v):
    return _with_locations({"object": _object(step[OBJECT], v)}, step, v, source="location")


def _tell_name_of_person(step, v):
    return _with_locations({}, step, v, source="location")


def _inspect(step, v):
    return {"entity": _location(step[SOURCE], v)}


class ActionSpec(object):
    def __init__(self, name, roles, summary, requires, build):
        """
        :param name: name of the action in the action server
        :param roles: how the action uses the roles, as told to the model
        :param summary: what the action does, as told to the model
        :param requires: groups of roles; of each group at least one must be given
        :param build: function (step, vocabulary) -> the fields of the recipe
        """
        self.name = name
        self.roles = roles
        self.summary = summary
        self.requires = requires
        self.build = build


ACTIONS = [
    ActionSpec("navigate-to", "destination, or person", "Go to a room, a piece of furniture or a person.",
               [(DESTINATION, PERSON)], _navigate_to),
    ActionSpec("find", "object or person; source = where to look", "Look for an object or a person.",
               [(OBJECT, PERSON)], _find),
    ActionSpec("pick-up", "object; source = where it is", "Take an object in the gripper and keep holding it.",
               [(OBJECT,)], _pick_up),
    ActionSpec("place", "object; destination = furniture to put it on; source = where it is",
               "Put an object on a piece of furniture.",
               [(DESTINATION,)], _place),
    ActionSpec("hand-over", "object; person = who gets it; source = where it is; destination = where that person is",
               "Bring an object to a person and give it to them.",
               [(PERSON,)], _hand_over),
    ActionSpec("say", "sentence", "Say something.",
               [(SENTENCE,)], _say),
    ActionSpec("follow", "person; source = where the person is; destination = where they are going",
               "Drive behind a person.",
               [(PERSON,)], _follow),
    ActionSpec("guide", "person; source = where the person is; destination = where to take them",
               "Lead a person to a place.",
               [(PERSON,), (DESTINATION,)], _guide),
    ActionSpec("count-and-tell", "object; source = furniture to count on",
               "Count the objects of one kind on a piece of furniture and say the number.",
               [(OBJECT,), (SOURCE,)], _count_and_tell),
    ActionSpec("tell-name-of-person", "source = where the person is",
               "Ask the name of the person at a place and report it.",
               [], _tell_name_of_person),
    ActionSpec("inspect", "source = furniture to inspect", "Look at a piece of furniture to see what is on it.",
               [(SOURCE,)], _inspect),
]
# Not offered to the model: answer-question, which only configures in an environment with 'challenge_spr'
# knowledge, and the actions that are no commands of a user (arm-goal, gripper-goal, reset-wm, ...).
_ACTIONS_BY_NAME = {spec.name: spec for spec in ACTIONS}

#: The actions that can be asked for "a drink": HERO picks a drink when it sees what is there. Count-and-tell counts
#: objects of one type, and inspect and the others take no object.
ACTIONS_THAT_TAKE_A_CATEGORY = ("find", "pick-up", "place", "hand-over")


def describe_actions():
    """
    :return: the actions and how each uses the roles, as text for the model
    """
    return "\n".join("- `{}` ({}): {}".format(spec.name, spec.roles, spec.summary) for spec in ACTIONS)


def _choices(vocabulary):
    """The values each role may take"""
    return {OBJECT: vocabulary.object_names + vocabulary.categories + [PREVIOUS_OBJECT],
            PERSON: [OPERATOR, SOMEONE] + vocabulary.people,
            SOURCE: vocabulary.locations,
            DESTINATION: vocabulary.locations}


def build_plan_schema(vocabulary):
    """
    The JSON schema of a plan. It follows the rules of OpenAI structured output: every property is required
    (a role that is not used is null) and no other properties are allowed.

    :param vocabulary: Vocabulary; its names are the only values the model can give for a role
    :return: JSON schema as a dict
    """
    choices = _choices(vocabulary)

    def nullable(definition):
        return {"anyOf": [{"$ref": "#/$defs/" + definition}, {"type": "null"}]}

    def strict_object(properties):
        return {"type": "object", "properties": properties, "required": list(properties),
                "additionalProperties": False}

    step = strict_object({
        "action": {"type": "string", "enum": [spec.name for spec in ACTIONS]},
        OBJECT: nullable("object"),
        PERSON: nullable("person"),
        SOURCE: nullable("location"),
        DESTINATION: nullable("location"),
        SENTENCE: {"anyOf": [{"type": "string"}, {"type": "null"}]},
    })

    schema = strict_object({
        "understanding": {"type": "string",
                          "description": "The command in one sentence, in the names the robot knows"},
        "substitutions": {"type": "array",
                          "description": "Words of the user that were replaced by a name the robot knows",
                          "items": strict_object({"heard": {"type": "string"}, "used": {"type": "string"}})},
        "status": {"type": "string", "enum": list(STATUSES)},
        "steps": {"type": "array", "description": "Empty unless status is ok", "items": {"$ref": "#/$defs/step"}},
        "reply": {"type": "string", "description": "One short sentence the robot says to the user"},
    })
    schema["$defs"] = {"step": step,
                       "object": {"type": "string", "enum": choices[OBJECT]},
                       "person": {"type": "string", "enum": choices[PERSON]},
                       "location": {"type": "string", "enum": choices[SOURCE]}}
    return schema


def _check_step(index, step, choices, vocabulary):
    """
    :return: (ActionSpec, the step with every role present)
    :raises PlanError: when the step names something unknown or misses a role its action needs
    """
    if not isinstance(step, dict):
        raise PlanError("step {} is not an object".format(index))

    spec = _ACTIONS_BY_NAME.get(step.get("action"))
    if spec is None:
        raise PlanError("step {} has unknown action '{}'; choose from: {}".format(
            index, step.get("action"), ", ".join(_ACTIONS_BY_NAME)))

    checked = {role: step.get(role) or None for role in ROLES}
    for role, allowed in choices.items():
        if checked[role] is not None and checked[role] not in allowed:
            raise PlanError("step {} ({}) has unknown {} '{}'; choose from: {}".format(
                index, spec.name, role, checked[role], ", ".join(allowed)))
    if checked[SENTENCE] is not None and not isinstance(checked[SENTENCE], str):
        raise PlanError("step {} ({}) has a sentence that is not text".format(index, spec.name))
    if checked[OBJECT] in vocabulary.categories and spec.name not in ACTIONS_THAT_TAKE_A_CATEGORY:
        raise PlanError("step {} ({}) needs one specific object, not the category '{}'".format(
            index, spec.name, checked[OBJECT]))

    for group in spec.requires:
        if not any(checked[role] for role in group):
            raise PlanError("step {} ({}) needs {}".format(index, spec.name, " or ".join(group)))

    return spec, checked


def compile_plan(plan, vocabulary):
    """
    Turn the plan of the model into a recipe for the action server.

    :param plan: the plan as a dict, following build_plan_schema
    :param vocabulary: Vocabulary the plan was made with
    :return: {"actions": [...]}; no actions when the status of the plan is not ok
    :raises PlanError: when the plan cannot be executed as it is
    """
    if not isinstance(plan, dict):
        raise PlanError("the plan is not an object")
    if plan.get("status") not in STATUSES:
        raise PlanError("unknown status '{}'; choose from: {}".format(plan.get("status"), ", ".join(STATUSES)))
    if plan["status"] != STATUS_OK:
        return {"actions": []}

    steps = plan.get("steps")
    if not isinstance(steps, list) or not steps:
        raise PlanError("status is ok but there are no steps")

    choices = _choices(vocabulary)
    actions = []
    for index, step in enumerate(steps):
        spec, checked = _check_step(index, step, choices, vocabulary)
        action = {"action": spec.name}
        action.update(spec.build(checked, vocabulary))
        actions.append(action)
    return {"actions": actions}
