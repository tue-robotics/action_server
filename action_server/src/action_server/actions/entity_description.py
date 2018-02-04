
class EntityDescription(object):
    def __init__(self, id=None, type=None, location=None, area=None):
        self.id = id
        self.type = type
        self.location = location
        self.area = area

    def __repr__(self):
        return "EntityDescription(id={id},type={type},location={location},area={area})".format(id=self.id,
                                                                                               type=self.type,
                                                                                               location=self.location,
                                                                                               area=self.area)

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
        if "loc" in parameters:
            description.location = resolve_entity_description(parameters["loc"])

    return description
