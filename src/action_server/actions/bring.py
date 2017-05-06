from action import Action
from find import Find
from pick_up import PickUp
from navigate_to import NavigateTo
from place import Place
from entity_description import resolve_entity_description

import rospy

class Bring(Action):
    def __init__(self):
        Action.__init__(self)
        self._required_field_prompts = {'source-location': " I don't know where you want me to get it. ",
                                        'target-location': " I don't know where you want me to take it. ",
                                        'object' : " I don't know what you want me to bring"}

    def _configure(self, robot, config):
        self._robot = robot

        self._source_location = resolve_entity_description(config['source-location'])
        self._target_location = resolve_entity_description(config['target-location'])
        self._object = resolve_entity_description(config['object'])

        # TODO Passing on knowledge needs to be automated in the future...
        self._find_action = Find()
        find_config = {'object': config['object'],
                       'location': config['source-location']}
        find_config_result = self._find_action.configure(self._robot, find_config)
        if not find_config_result.succeeded:
            self._config_result.message = find_config_result.message
            return
        self._found_object_designator = find_config_result.resulting_knowledge['found-object-des']

        self._grab_action = PickUp()
        grab_config = {'object': config['object'], 'found-object-des': self._found_object_designator}
        grab_config_result = self._grab_action.configure(self._robot, grab_config)
        if not grab_config_result.succeeded:
            self._config_result.message = grab_config_result.message
            return
        self._arm_designator = grab_config_result.resulting_knowledge['arm-designator']

        self._nav_action = NavigateTo()
        nav_config = {'object': config['target-location']}
        nav_config_result = self._nav_action.configure(self._robot, nav_config)
        if not nav_config_result.succeeded:
            self._config_result.message = nav_config_result.message
            return

        self._place_action = Place()
        place_config = {'object': config['target-location'],
                        'arm-designator': self._arm_designator}
        place_config_result = self._place_action.configure(self._robot, place_config)
        if not place_config_result.succeeded:
            self._config_result.message = place_config_result.message
            return

        self._config_result.succeeded = True
        self._robot.speech.speak("Bring the action!")

    def _handover(self):
        self._robot.speech.speak("I will hand over the {} now".format(self._object.type))
        arm = self._arm_designator.resolve()
        arm.handover_to_human()
        self._robot.speech.speak("Here you go!", block=False)
        arm.reset()

    def _start(self):
        # Find
        find_result = self._find_action.start()

        if not find_result.succeeded:
            self._execute_result.message = " I was unable to find the {}".format(self._object.type)
            return

        # Grab
        grab_result = self._grab_action.start()

        if not grab_result.succeeded:
            self._execute_result.message = " I was unable to grab the {}".format(self._object.type)
            return

        # Navigate
        nav_result = self._nav_action.start()

        if not nav_result.succeeded:
            self._execute_result.message = " I was unable to go to the {}".format(self._target_location.id)
            return

        # Handover or place
        if self._target_location.type == "person":
            self._handover()
        else:
            place_result = self._place_action.start()

            if not place_result.succeeded:
                self._execute_result.message = " I was unable to place the {}".format(self._object.type)
                return

        self._execute_result.succeeded = True
        self._execute_result.message = " I brought a {} from {} to {}. ".format(self._object.type,
                                                                                self._source_location.id,
                                                                                self._target_location.id)

if __name__ == "__main__":
    rospy.init_node('bring_test')

    import sys
    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = Bring()

    config = {'action': 'bring',
              'entity': {'location': 'cabinet'},
              'from': {'id': 'cabinet'},
              'to': {'id': 'dinner_table'}}

    action.configure(robot, config)
    action.start()
