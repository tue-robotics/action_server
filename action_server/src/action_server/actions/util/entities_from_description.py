from pykdl_ros import VectorStamped

from robot_skills.robot import Robot


def length_sq(x, y):
    return x * x + y * y


# ----------------------------------------------------------------------------------------------------

def entities_from_description(entity_descr, robot):
    """
    Query entities with various methods

    @param entity_descr: A dict that contains an 'id' or 'type' field
    @param robot: The robot object

    @return: (entities, error_msg)
        entities  - list of entities that fulfill the description
                    (each element has type EntityInfo)
        error_msg - If something goes wrong, this contains the message
    """
    if not isinstance(entity_descr, dict):
        return ([],
                "entities_from_description: the specified entity_descr should be a dictionary! I received: %s" % str(
                    entity_descr))

    if not isinstance(robot, Robot):
        return [], "entities_from_description: the specified robot should be a Robot! I received: %s" % str(robot)

    if "id" in entity_descr:
        e = robot.ed.get_entity(id=entity_descr["id"])
        if not e:
            return [], "No entity with id '%s'" % entity_descr["id"]
        entities = [e]
    elif "type" in entity_descr:
        entities = robot.ed.get_entities(type=entity_descr["type"])
    else:
        entities = robot.ed.get_entities()

    if "location" in entity_descr:
        location_entity = robot.ed.get_entity(uuid=entity_descr["location"]["uuid"])

        if location_entity:
            if "area" in entity_descr and entity_descr["area"] in location_entity.volumes:
                area = entity_descr["area"]
            else:
                area = "on_top_of"
            entities = [e for e in entities if
                        location_entity.in_volume(volume_id=area, point=VectorStamped.from_framestamped(e.pose))]

    if not entities:
        return [], "No such entity"

    robot_location = robot.base.get_location()
    robot_pos = robot_location.frame.p

    # Sort entities by distance
    entities = sorted(entities, key=lambda entity: entity.distance_to_2d(robot_pos))

    return entities, ""
