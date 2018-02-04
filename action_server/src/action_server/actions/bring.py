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
        self._required_field_prompts = {'target-location': " Where would you like me to take it? ",
                                        'object' : " What would you like me to bring, exactly? "}
        self._required_skills = ['base']

    def _configure(self, robot, config):
        self._robot = robot
        self._source_location = None
        self._object = resolve_entity_description(config.semantics['object'])
        self._find_action = None
        self._grab_action = None
        self._nav_action = None
        self._find_person_action = None

        if 'arm-designator' in config.knowledge:
            self._arm_designator = config.knowledge['arm-designator']
            if 'object-designator' in config.knowledge:
                self._found_object_designator = config.knowledge['object-designator']
            else:
                rospy.logfatal("No object designator while there is an arm designator. How can this be happening?!")
                self._config_result.message = "No object designator while there is an arm designator. How can this be happening?!"
                return
        else:
            # If we got a source location in the semantics, use that to find the object
            if 'source-location' in config.semantics or 'location-designator' in config.knowledge:
                self._find_action = Find()

                # Put the knowledge passed to the bring action to the find action.
                find_semantics = {}
                if 'source-location' in config.semantics:
                    find_semantics['location'] = config.semantics['source-location']
                find_semantics['object'] = config.semantics['object']

                find_config = ConfigurationData(find_semantics, config.knowledge)
                find_config_result = self._find_action.configure(self._robot, find_config)
                if not find_config_result.succeeded:
                    self._config_result = find_config_result
                    return

                # Use the object designator from the find action to resolve to the object we want to bring
                self._found_object_designator = find_config_result.context['object-designator']
                self._source_location = resolve_entity_description(config.semantics['source-location'])

            # If the task is something like "... and bring it to me", the "it" refers to something we already found or even
            # grasped the past, meaning we don't need to do a find action, but we need to use that object.
            elif config.semantics['object']['type'] == 'reference' and 'object-designator' in config.knowledge:
                rospy.loginfo("object is a reference, but I have an object designator")
                self._found_object_designator = config.knowledge['object-designator']
            else:
                rospy.loginfo("No source given, trying to find at default location.")
                self._find_action = Find()

                # Put the knowledge passed to the bring action to the find action.
                find_semantics = {}
                if config.semantics['object']['type'] in self._knowledge.object_categories:
                    category = config.semantics['object']['type']
                else:
                    category = self._knowledge.get_object_category(config.semantics['object']['type'])

                expected_object_location = self._knowledge.get_object_category_location(category)[0]

                find_semantics['location'] = expected_object_location
                find_semantics['object'] = config.semantics['object']

                find_config = ConfigurationData(find_semantics)
                find_config_result = self._find_action.configure(self._robot, find_config)
                if not find_config_result.succeeded:
                    self._config_result = find_config_result
                    return

                # Use the object designator from the find action to resolve to the object we want to bring
                self._found_object_designator = find_config_result.context['object-designator']

                # rospy.loginfo("I really don't know where to get this thing. ")
                # self._config_result.message = " Where would you like me to get it? "
                # self._config_result.missing_field = 'source-location'
                # self._config_result.succeeded = False
                # return

            self._grab_action = PickUp()
            grab_config = ConfigurationData({'object': config.semantics['object']},
                                            {'object-designator': self._found_object_designator})
            grab_config_result = self._grab_action.configure(self._robot, grab_config)
            if not grab_config_result.succeeded:
                self._config_result = grab_config_result
                return
            self._arm_designator = grab_config_result.context['arm-designator']


        self._target_location = resolve_entity_description(config.semantics['target-location'])
        if self._target_location.type == "person":
            print self._target_location.type
            print self._target_location.id
            print self._target_location.location
            if self._target_location.id and self._target_location.id == "operator":
                self._robot.ed.update_entity(id="operator", frame_stamped=self._robot.base.get_location(),
                                             type="waypoint")
                self._nav_action = NavigateTo()
                nav_config = ConfigurationData({'object': config.semantics['target-location']})
                nav_config_result = self._nav_action.configure(self._robot, nav_config)
                if not nav_config_result.succeeded:
                    self._config_result = nav_config_result
                    return

            # If we need to bring to someone else than the operator, we need to go and find that person
            elif self._target_location.id and self._target_location.location:
                self._find_person_action = Find()

                find_person_semantics = {}
                find_person_semantics['location'] = {'id' : self._target_location.location.id}
                find_person_semantics['object'] = {'id' : self._target_location.id,
                                                   'type' : self._target_location.type}

                find_person_config = ConfigurationData(find_person_semantics)
                find_person_config_result = self._find_person_action.configure(self._robot, find_person_config)
                if not find_person_config_result.succeeded:
                    self._config_result = find_person_config_result
                    return
            elif self._target_location.id:
                self._config_result.message = \
                    "Please give me the assignment again and then also tell me were to find {}".\
                        format(self._target_location.id)
                return
        else:
            self._place_action = Place()
            place_config = ConfigurationData({'location': config.semantics['target-location']},
                                             {'arm-designator': self._arm_designator,
                                              'object-designator': self._found_object_designator})
            place_config_result = self._place_action.configure(self._robot, place_config)
            if not place_config_result.succeeded:
                self._config_result = place_config_result
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

        self._robot.ed.update_entity(id=arm.occupied_by.id, action='remove')
        arm.send_gripper_goal('open')
        arm.wait_for_motion_done()

        arm.reset()
        arm.wait_for_motion_done()

        arm.occupied_by = None

    def _start(self):
        self._robot.speech.speak("Bring the action!")

        # Find
        if self._find_action:
            find_result = self._find_action.start()

            if not find_result.succeeded:
                if self._object.type == "reference":
                    self._object.type = self._found_object_designator.resolve().type
                self._execute_result.message = " I was unable to find the {}. ".format(self._object.type)
                return

        if self._object.type == "reference":
            self._object.type = self._found_object_designator.resolve().type

        # Grab
        if self._grab_action:
            grab_result = self._grab_action.start()

            if not grab_result.succeeded:
                self._execute_result.message = " I was unable to grab the {}. ".format(self._object.type)
                return

        # Navigate
        if self._nav_action:
            nav_result = self._nav_action.start()

            if not nav_result.succeeded:
                self._execute_result.message = " I was unable to go to the {}. ".format(self._target_location.id)
        elif self._find_person_action:
            find_person_result = self._find_person_action.start()

            if not find_person_result.succeeded:
                self._execute_result.message = " I found {}. ".format(self._target_location.id)

        # Handover or place
        if self._target_location.type == "person":
            self._handover()
        else:
            place_result = self._place_action.start()

            if not place_result.succeeded:
                self._execute_result.message = " I was unable to place the {}".format(self._object.type)
                return

        self._execute_result.succeeded = True
        if self._source_location:
            self._execute_result.message += " I brought a {} from {} to {}. ".format(self._object.type,
                                                                                     self._source_location.id,
                                                                                     self._target_location.id)
        else:
            self._execute_result.message += " I brought a {} to {}. ".format(self._object.type,
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
