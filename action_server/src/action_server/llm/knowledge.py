"""
What the language model is told about the robot and its environment.

The instructions are a markdown file kept with the robot's parameters
(<robot>_bringup/parameters/llm/task_planner.md). The names of the rooms, furniture,
objects and people are not written in that file: they are taken from robocup_knowledge
for the active ROBOT_ENV, the same source as the grammars and the NER entity linker,
and filled in where the file has {{placeholders}}.
"""
import os
import re
from pathlib import Path

KNOWLEDGE_FILE_IN_BRINGUP = Path("parameters") / "llm" / "task_planner.md"

_COMMENT = re.compile(r"<!--.*?-->\n?", re.DOTALL)
_PLACEHOLDER = re.compile(r"{{\s*(\w+)\s*}}")


class KnowledgeError(Exception):
    """The instructions or the vocabulary of the environment cannot be loaded"""


class Vocabulary(object):
    """The names the robot knows in its environment"""

    def __init__(self, rooms, furniture, objects, people, usual_locations=None):
        """
        :param rooms: room names
        :param furniture: list of {"name": ..., "room": ..., "manipulation": bool}
        :param objects: list of {"name": ..., "category": ...}
        :param people: names of known people
        :param usual_locations: {category: furniture name}, where objects of a category are normally kept
        """
        self.rooms = list(rooms)
        self.furniture = list(furniture)
        self.objects = list(objects)
        self.people = list(people)
        self.usual_locations = dict(usual_locations or {})

    @property
    def locations(self):
        """Every place the robot can be sent to: rooms and furniture"""
        return self.rooms + [f["name"] for f in self.furniture]

    @property
    def object_names(self):
        return [o["name"] for o in self.objects]

    @property
    def categories(self):
        categories = []
        for o in self.objects:
            if o["category"] not in categories:
                categories.append(o["category"])
        return categories


def load_vocabulary():
    """
    :return: Vocabulary of the active environment (ROBOT_ENV), from robocup_knowledge
    :raises KnowledgeError: when the knowledge of the environment cannot be loaded
    """
    try:
        from robocup_knowledge import knowledge_loader
        common = knowledge_loader.load_knowledge("common")
    except Exception as e:
        raise KnowledgeError("Could not load robocup_knowledge 'common' for ROBOT_ENV='{}': {}".format(
            os.environ.get("ROBOT_ENV"), e))
    if common is None:
        raise KnowledgeError("robocup_knowledge has no 'common' knowledge for ROBOT_ENV='{}'".format(
            os.environ.get("ROBOT_ENV")))

    furniture = [{"name": l["name"], "room": l.get("room", ""), "manipulation": l.get("manipulation") == "yes"}
                 for l in common.locations]
    objects = [{"name": o["name"], "category": o.get("category", "")} for o in common.objects]
    usual_locations = {category: next(iter(locations))
                       for category, locations in getattr(common, "category_locations", {}).items() if locations}

    return Vocabulary(rooms=common.location_rooms, furniture=furniture, objects=objects, people=common.names,
                      usual_locations=usual_locations)


def find_knowledge_file(configured=None):
    """
    :param configured: the LLM_KNOWLEDGE_FILE setting, if it is set
    :return: Path of the instructions file; default $ROBOT_BRINGUP_PATH/parameters/llm/task_planner.md
    :raises KnowledgeError: when no file is configured or the file does not exist
    """
    if configured:
        path = Path(configured).expanduser()
    elif os.environ.get("ROBOT_BRINGUP_PATH"):
        path = Path(os.environ["ROBOT_BRINGUP_PATH"]) / KNOWLEDGE_FILE_IN_BRINGUP
    else:
        raise KnowledgeError("No instructions file: set LLM_KNOWLEDGE_FILE, or ROBOT_BRINGUP_PATH to use "
                             "<bringup>/{}".format(KNOWLEDGE_FILE_IN_BRINGUP))
    if not path.is_file():
        raise KnowledgeError("Instructions file {} does not exist".format(path))
    return path


def describe_vocabulary(vocabulary):
    """
    :param vocabulary: Vocabulary
    :return: dict placeholder name -> text
    """
    furniture = []
    for f in vocabulary.furniture:
        details = [f["room"]] if f["room"] else []
        if not f["manipulation"]:
            details.append("no picking or placing")
        furniture.append("{} ({})".format(f["name"], ", ".join(details)) if details else f["name"])

    objects = []
    for category in vocabulary.categories:
        names = [o["name"] for o in vocabulary.objects if o["category"] == category]
        usual = vocabulary.usual_locations.get(category)
        header = "{} (usually on {})".format(category, usual) if usual else category
        objects.append("- {}: {}".format(header, ", ".join(names)))

    return {"rooms": ", ".join(vocabulary.rooms),
            "furniture": ", ".join(furniture),
            "objects": "\n".join(objects),
            "people": ", ".join(vocabulary.people)}


def render_prompt(template, values):
    """
    Fill the {{placeholders}} of the instructions and drop the <!-- comments -->, which are for the people
    who edit the file.

    :param template: text of the instructions file
    :param values: dict placeholder name -> text
    :raises KnowledgeError: when the template has a placeholder that is not in values
    """
    def fill(match):
        name = match.group(1)
        if name not in values:
            raise KnowledgeError("Unknown placeholder {{{{{}}}}} in the instructions; known: {}".format(
                name, ", ".join(sorted(values))))
        return values[name]

    return _PLACEHOLDER.sub(fill, _COMMENT.sub("", template)).strip()
