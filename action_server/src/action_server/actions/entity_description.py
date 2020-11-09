
class EntityDescription(object):
    def __init__(self, id=None, type=None, location=None, area=None, category=None):
        self.id = id
        self.type = type
        self.location = location
        self.area = area
        self.category = category

    def __repr__(self):
        return "EntityDescription(id={id},type={type},location={location},area={area}, category={category})".format(
                    id=self.id,
                    type=self.type,
                    location=self.location,
                    area=self.area,
                    category=self.category)

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
        if "location" in parameters:
            description.location = resolve_entity_description(parameters["location"])
        if "designator" in parameters:
            description.designator = parameters["designator"]

    return description
