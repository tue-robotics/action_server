from action import FSMAction

from util import entities_from_description

import robot_smach_states

from cb_planner_msgs_srvs.msg import PositionConstraint, OrientationConstraint
from robot_smach_states.util.designators import UnoccupiedArmDesignator, EdEntityDesignator

class PickUp(FSMAction):

    def _init_fsm(self, config, robot):

        if not "entity" in config:
            return "No entity given"

        entity_descr = config["entity"]
        (entities, error_msg) = entities_from_description(entity_descr, robot)
        if not entities:
            return error_msg

        # Only filter to entities that can be picked up, e.g not furniture etc
        entities = [e for e in entities if not e.is_a("furniture")]

        if not entities:
            return "Inpossible to grab that object"

        entity = entities[0]

        side = config['side'] if 'side' in config else 'right'

        # self._fsm = robot_smach_states.grab.SjoerdsGrab(robot,
        #     item_des = EdEntityDesignator(robot, id = entity.id),
        #     arm_des = UnoccupiedArmDesignator(robot.arms, robot.arms[side]))
        self._fsm = robot_smach_states.grab.Grab(robot,
            item=EdEntityDesignator(robot, id=entity.id),
            arm=UnoccupiedArmDesignator(robot.arms, robot.arms[side]))
