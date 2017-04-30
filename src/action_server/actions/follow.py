from action import Action
from entity_description import resolve_entity_description

from robot_smach_states import FollowOperator, EdEntityDesignator

import rospy

def navigate(robot, entity_description):
    if not entity_description.type:
        return False

    if entity_description.type == "room":
        origin_area = "in"
    else:
        origin_area = "near"

    origin_entity_designator = EdEntityDesignator(id=entity_description.id)
    navigation_sm = NavigateToSymbolic(
        robot=robot,
        entity_designator_area_name_map=
        {
            origin_entity_designator: origin_area
        },
        lookat_entity_designator=origin_entity_designator
    )
    return navigation_sm.execute() == "succeeded"

class Follow(Action):

    def __init__(self):
        Action.__init__(self)

    def _configure(self, robot, config):
        if not "entity" in config:
            rospy.logwarn("Please provide required field in config. Got config: {}".format(config))
            self._config_result.missing_field = "entity"
            return

        self._entity_description = resolve_entity_description(config["entity"])

        self._origin = None
        if not "origin" in config:
            rospy.logdebug("No 'origin' entity given in config. Got config: {}".format(config))
        else:
            self._origin = resolve_entity_description(config["origin"])

        self._goal = None
        if not "goal" in config:
            rospy.logdebug("No 'goal' entity given in config. Got config: {}".format(config))
        else:
            self._origin = resolve_entity_description(config["goal"])

        self._robot = robot

        self._config_result.succeeded = True

    def _start(self):
        # If we need to navigate to some origin location, do that first TODO: Use the navigate action for this
        if self._origin:
            if not navigate(robot=self._robot, entity_description=self._origin):
                self._execute_result.message += "failed to get to the origin location to follow"
                return
            self._execute_result.message += "navigated to the origin location to follow the operator and "

        # Do some awesome following
        follow_sm = FollowOperator(self._robot)
        res = follow_sm.execute()

        # If we didn't succeed in following, navigate to the goal location if we have one
        if not res == "succeeded":
            if self._goal:
                if not navigate(robot=self._robot, entity_description=self._goal):
                    self._execute_result.message += "failed to follow to the {}".format(self._goal.id)
                    return
            else:
                self._execute_result.message += "failed to follow the operator"
                return

        self._execute_result.message += "followed the operator"
        if self._goal:
            self._execute_result.message += " to the {}".format(self._goal.id)

        self._execute_result.succeeded = True

    def _cancel(self):
        pass
