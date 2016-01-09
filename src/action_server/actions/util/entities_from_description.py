import rospy
from robot_skills.robot import Robot

def length_sq(x, y):
    return x * x + y * y

# ----------------------------------------------------------------------------------------------------

def entities_from_description(entity_descr, robot):
    '''
    Query entities with various methods

    @param entity_descr: A dict that contains an 'id' or 'type' field
    @param robot: The robot object

    @return: (entities, error_msg)
        entities  - list of entities that fulfill the description
                    (each element has type EntityInfo)
        error_msg - If something goes wrong, this contains the message
    '''
    if not isinstance(entity_descr, dict):
        return ([], "entities_from_description: the specified entity_descr should be a dictionary! I received: %s" % str(entity_descr))

    if not isinstance(robot, Robot):
        return ([], "entities_from_description: the specified robot should be a Robot! I received: %s" % str(robot))

    if "id" in entity_descr:
        e = robot.ed.get_entity(id=entity_descr["id"], parse=False)
        if not e:
            return ([], "No entity with id '%s'" % entity_descr["id"])
        entities = [e]
    elif "type" in entity_descr:
        entities = robot.ed.get_entities(type=entity_descr["type"], parse=False)
    else:
        entities = robot.ed.get_entities(parse=False)

    if not entities:
        return ([], "No such entity")

    robot_pos = robot.base.get_location().pose.position

    # Sort entities by distance
    entities = sorted(entities,
                      key=lambda entity: length_sq(
                          robot_pos.x - entity.pose.position.x,
                          robot_pos.y - entity.pose.position.y))

    return (entities, "")
