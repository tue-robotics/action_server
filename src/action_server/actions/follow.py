from action import Action
from entity_description import resolve_entity_description

from robot_smach_states import FollowOperator
from robot_smach_states.util.designators import EdEntityDesignator

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
        self._required_field_prompts = {'target': "I don't know who to follow"}

    def _configure(self, robot, config):
        # if not "entity" in config:
        #     rospy.logwarn("Please provide required field in config. Got config: {}".format(config))
        #     self._config_result.missing_field = "entity"
        #     return

        self._target = resolve_entity_description(config["target"])

        if not self._target.id == "operator" and not "location-from" in config:
            self._config_result.missing_field = "location-from"
            self._config_result.message = " Where can I find {}? ".format(self._target.id)
            return

        # TODO: if id and id is operator, directly follow instead of first navigating to location-from
        # TODO: if id is not operator, go to location-from and then follow

        self._origin = resolve_entity_description(config["location-from"])

        self._goal = resolve_entity_description(config["location-to"])

        self._robot = robot

        self._config_result.succeeded = True

    def _start(self):
        # If we need to navigate to some origin location, do that first TODO: Use the navigate action for this
        if not self._target.id == "operator":
            if not navigate(robot=self._robot, entity_description=self._origin):
                self._execute_result.message += "i cannot resolve the origin location"
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


if __name__ == "__main__":
    rospy.init_node('follow_test')

    import sys
    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = Follow()

    config = {'action': 'follow',
              'entity': {'special': 'me'}}

    action.configure(robot, config)
    action.start()
