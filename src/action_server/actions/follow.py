from action import Action
from find import Find
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
    ''' The Follow class implements the action to follow a person.

    Parameters to pass to the configure() method are:
     - `target` (required): the target id to assign to the id that is followed
     - `location-from` (optional): the location to find the target to follow
    '''
    def __init__(self):
        Action.__init__(self)
        self._required_field_prompts = {'target': " Who would you like me to follow? "}

    def _configure(self, robot, config):
        self._target = resolve_entity_description(config["target"])

        if not self._target.id == "operator" and not "location-from" in config:
            self._config_result.missing_field = "location-from"
            self._config_result.message = " Where can I find {}? ".format(self._target.id)
            return

        if "location-from" in config:
            self._origin = resolve_entity_description(config["location-from"])
            self._find_action = Find()
            find_config = {'location': {'id' : self._origin.id},
                           'object': {'type': 'person',
                                      'id': self._target.id}}
            find_config_result = self._find_action.configure(robot, find_config)
            if not find_config_result.succeeded:
                self._config_result.message = " I don't know how to find {} in the {}. ".format(self._origin.id,
                                                                                                self._target.id)
                return
        else:
            self._origin = None
            self._find_action = None

        if self._target.id == "operator":
            self._target.id = "you"

        if "location-to" in config:
            self._goal = resolve_entity_description(config["location-to"])
        else:
            self._goal = None

        self._robot = robot

        self._config_result.succeeded = True

    def _start(self):
        # If we need to navigate to some origin location, do that first TODO: Use the find action for this
        if self._find_action:
            find_result = self._find_action.start()
            self._execute_result.message += find_result.message
            if not find_result.succeeded:
                return
            # TODO: Navigate to the found person before following

        # Do some awesome following
        follow_sm = FollowOperator(self._robot)
        res = follow_sm.execute()

        # If we didn't succeed in following, navigate to the goal location if we have one
        if not res == "succeeded":
            if self._goal:
                if not navigate(robot=self._robot, entity_description=self._goal):
                    self._execute_result.message += " But I failed to follow {} to the {}. ".format(self._target.id,
                                                                                                    self._goal.id)
                    return
            else:
                self._execute_result.message += " But I failed to follow {} ".format(self._target.id)
                return

        self._execute_result.message += " I successfully followed {} ".format(self._goal.id)
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
