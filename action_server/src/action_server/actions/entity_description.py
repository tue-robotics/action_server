class EntityDescription(object):
    """
    What a step of a task says about an entity. Examples: {'type': 'coke'}, {'category': 'drink'},
    {'type': 'person', 'id': 'anna'}, and "the waving person in the kitchen":
    {'type': 'person', 'location': {'id': 'kitchen'}, 'attributes': {'gesture': 'waving'}}.

    :ivar attributes: dict attribute -> value describing a person, e.g. {'gesture': 'waving'}; None when not given
        (see robot_smach_states.util.entity_descriptions.PERSON_ATTRIBUTES)
    """
    def __init__(self, id=None, type=None, location=None, area=None, category=None, attributes=None):
        self.id = id
        self.type = type
        self.location = location
        self.area = area
        self.category = category
        self.attributes = attributes

    def __repr__(self):
        return ("EntityDescription(id={id},type={type},location={location},area={area}, category={category}, "
                "attributes={attributes})").format(
            id=self.id,
            type=self.type,
            location=self.location,
            area=self.area,
            category=self.category,
            attributes=self.attributes)

    def __eq__(self, other):
        return self.__dict__ == other.__dict__


def resolve_entity_description(parameters):
    description = EntityDescription()

    if isinstance(parameters, str):
        description.id = parameters

    elif "special" in parameters:
        special = parameters["special"]
        if special == "it":
            description = world.last_entity
        elif special == "operator":
            description.id = "gpsr_starting_pose"
    else:
        if "id" in parameters:
            description.id = parameters["id"]
        if "type" in parameters:
            description.type = parameters["type"]
        if "category" in parameters:
            description.category = parameters["category"]
        if parameters.get("attributes"):
            description.attributes = dict(parameters["attributes"])
        if "location" in parameters:
            description.location = resolve_entity_description(parameters["location"])
        if "designator" in parameters:
            description.designator = parameters["designator"]

    return description
