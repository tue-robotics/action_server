from action import FSMAction

from util import entities_from_description

import robot_smach_states

class Inspect(FSMAction):

    def _init_fsm(self, config, robot):
        if "entity" not in config:
            return "No entity given"

        entity_descr = config["entity"]
        (entities, error_msg) = entities_from_description(entity_descr, robot)
        if not entities:
            return error_msg

        entity = entities[0]

        self._fsm = robot_smach_states.world_model.Inspect(robot, entityDes = robot_smach_states.util.designators.EdEntityDesignator(robot, id=entity.id))
