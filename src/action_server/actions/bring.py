from action import Action, ConfigurationData
from find import Find
from pick_up import PickUp
from navigate_to import NavigateTo
from place import Place
from entity_description import resolve_entity_description

import rospy

class Bring(Action):
    """
    The Bring class implements the action to bring an object from a source to a target location.

    Parameters to pass to the configure() method are 'source-location' (required), 'target-location' (required) and
     an object to bring (required).
    """
    def __init__(self):
        Action.__init__(self)
        self._required_field_prompts = {'source-location': " Where would you like me to get it? ",
                                        'target-location': " Where would you like me to take it? ",
                                        'object' : " What would you like me to bring, exactly? "}
        self._required_skills = ['base']

    def _configure(self, robot, config):
        self._robot = robot

        self._source_location = resolve_entity_description(config.semantics['source-location'])
        self._target_location = resolve_entity_description(config.semantics['target-location'])
        self._object = resolve_entity_description(config.semantics['object'])

        # TODO Passing on knowledge needs to be automated in the future...
        self._find_action = Find()
        find_config = ConfigurationData({'object': config.semantics['object'],
                       'location': config.semantics['source-location']})
        find_config_result = self._find_action.configure(self._robot, find_config)
        if not find_config_result.succeeded:
            self._config_result.message = find_config_result.message
            return
        self._found_object_designator = find_config_result.resulting_knowledge['found-object-des']

        self._grab_action = PickUp()
        grab_config = ConfigurationData({'object': config.semantics['object'], 'found-object-des': self._found_object_designator})
        grab_config_result = self._grab_action.configure(self._robot, grab_config)
        if not grab_config_result.succeeded:
            self._config_result.message = grab_config_result.message
            return
        self._arm_designator = grab_config_result.resulting_knowledge['arm-designator']

        self._nav_action = NavigateTo()
        nav_config = ConfigurationData({'object': config.semantics['target-location']})
        nav_config_result = self._nav_action.configure(self._robot, nav_config)
        if not nav_config_result.succeeded:
            self._config_result.message = nav_config_result.message
            return

        target_location = resolve_entity_description(config.semantics['target-location'])
        self._drop_waypoint_after_find = False
        if target_location.type == "person":
            # TODO: Also handle bringing something to someone else than the operator
            self._robot.ed.update_entity(id="operator", frame_stamped=self._robot.base.get_location(), type="waypoint")
        else:
            self._place_action = Place()
            place_config = {'entity': config.semantics['target-location'],
                            'arm-designator': self._arm_designator}
            place_config_result = self._place_action.configure(self._robot, place_config)
            if not place_config_result.succeeded:
                self._config_result.message = place_config_result.message
                return

        self._config_result.succeeded = True

    def _handover(self):
        self._robot.speech.speak("I will hand over the {} now".format(self._object.type))
        arm = self._arm_designator.resolve()
        arm.send_joint_goal('handover_to_human')
        arm.wait_for_motion_done()

        self._robot.speech.speak("Please take it from my gripper.", block=False)

        attempt = 0

        while not arm.handover_to_human(timeout=10) and attempt < 2:
            self._robot.speech.speak("Please take it from my gripper.", block=False)
            attempt += 1

        self._robot.speech.speak("I will open my gripper now.", block=False)

        arm.send_gripper_goal('open')
        arm.wait_for_motion_done()

        arm.reset()
        arm.wait_for_motion_done()

        arm.occupied_by = None

    def _start(self):
        self._robot.speech.speak("Bring the action!")

        # Find
        find_result = self._find_action.start()

        if not find_result.succeeded:
            self._execute_result.message = " I was unable to find the {}. ".format(self._object.type)
            return

        # Grab
        grab_result = self._grab_action.start()

        if not grab_result.succeeded:
            self._execute_result.message = " I was unable to grab the {}. ".format(self._object.type)
            return

        # Navigate
        nav_result = self._nav_action.start()

        if not nav_result.succeeded:
            self._execute_result.message = " I was unable to go to the {}. ".format(self._target_location.id)

        # Handover or place
        if self._target_location.type == "person":
            self._handover()
        else:
            place_result = self._place_action.start()

            if not place_result.succeeded:
                self._execute_result.message = " I was unable to place the {}".format(self._object.type)
                return

        self._execute_result.succeeded = True
        self._execute_result.message += " I brought a {} from {} to {}. ".format(self._object.type,
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

    config = ConfigurationData({'action': 'bring',
              'entity': {'location': 'cabinet'},
              'from': {'id': 'cabinet'},
              'to': {'id': 'dinner_table'}})

    action.configure(robot, config)
    action.start()
