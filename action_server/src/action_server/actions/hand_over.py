from action import Action, ConfigurationData
from find import Find
from pick_up import PickUp
from navigate_to import NavigateTo
from place import Place
from entity_description import resolve_entity_description

import rospy


class HandOver(Action):
    """
    The HandOver class implements the action to hand over an object to a person.

    Parameters to pass to the configure() method are 'source-location' (required), 'target-location' (required) and
    an object to bring (required).
    """
    def __init__(self):
        Action.__init__(self)
        self._required_field_prompts = {'target-location': " Who would you like me to hand the object? ",
                                        'object': " What would you like me to hand over? "}
        self._required_skills = ['base']

    class Semantics:
        def __init__(self):
            self.target_location = None
            self.object = None
            self.source_location = None

    @staticmethod
    def _parse_semantics(semantics_dict):
        semantics = HandOver.Semantics()

        semantics.target_location = resolve_entity_description(semantics_dict['target-location'])
        semantics.object = resolve_entity_description(semantics_dict['object'])

        if 'source-location' in semantics_dict:
            semantics.source_location = resolve_entity_description(semantics_dict['source-location'])

        return semantics

    class Knowledge:
        def __init__(self):
            self.arm_designator = None
            self.object_designator = None
            self.object_type = None
            self.location_designator = None

    @staticmethod
    def _parse_knowledge(knowledge_dict):
        knowledge = HandOver.Knowledge()

        if 'arm-designator' in knowledge_dict:
            knowledge.arm_designator = knowledge_dict['arm-designator']

        if 'object-designator' in knowledge_dict:
            knowledge.arm_designator = knowledge_dict['object-designator']

        if 'object-type' in knowledge_dict:
            knowledge.object_type = knowledge_dict['object-type']

        if 'location-designator' in knowledge_dict:
            knowledge.location_designator = knowledge_dict['location-designator']

        return knowledge

    def _configure(self, robot, config):
        self._robot = robot
        self._pick_action = None
        self._nav_action = None

        # Parse semantics and knowledge to a convenient object
        self.semantics = self._parse_semantics(config.semantics)
        self.knowledge = self._parse_knowledge(config.knowledge)

        # We assume we already got the object if previous action passed an arm, an object and this object has the
        # required type, or the required type is a reference.
        got_object = (
            self.knowledge.arm_designator is not None and (self.knowledge.object_type == self.semantics.object.type or
                                                           self.semantics.object.type == 'reference'))

        if not got_object:
            # Request pick_up action
            self._config_result._required_prior_action = {'action': 'pick-up',
                                                          'object': self.semantics.object}
            self._pick_action = PickUp()
            pick_up_configuration_data = ConfigurationData(config.semantics, config.knowledge)
            pick_up_configuration_result = self._pick_action.configure(self._robot, pick_up_configuration_data)
            if not pick_up_configuration_result.succeeded:
                self._config_result = pick_up_configuration_result
                return
            self.knowledge.arm_designator = pick_up_configuration_result.resulting_knowledge['arm-designator']

        # Now we can assume we picked up the item!

        at_destination = False  # TODO: Replace this hack with a decent check for the planned location of the robot.

        if not at_destination:
            # Request navigation action
            self._nav_action = NavigateTo()
            nav_config = ConfigurationData({'object': config.semantics['target-location']})
            nav_config_result = self._nav_action.configure(self._robot, nav_config)
            if not nav_config_result.succeeded:
                self._config_result = nav_config_result
                return

        # We can now assume that we are at the destination for handover!

        self._config_result.succeeded = True

    def _handover(self):
        # TODO: Move this code to the handover smach state
        self._robot.speech.speak("I will hand over the {} now".format(self.semantics.object.type))
        arm = self.knowledge.arm_designator.resolve()
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
        # Grab
        if self._pick_action:
            grab_result = self._pick_action.start()

            if not grab_result.succeeded:
                self._execute_result.message = grab_result.message
                return

        # Navigate
        if self._nav_action:
            nav_result = self._nav_action.start()

            if not nav_result.succeeded:
                self._execute_result.message = " I was unable to go to the {}. ".format(
                    self.semantics.target_location.id)

        # Handover
        self._handover()

        self._execute_result.succeeded = True
        if self.semantics.source_location:
            self._execute_result.message += " I brought a {} from {} to {}. ".format(self.semantics.object.type,
                                                                                     self.semantics.source_location.id,
                                                                                     self.semantics.target_location.id)
        else:
            self._execute_result.message += " I brought a {} to {}. ".format(self.semantics.object.type,
                                                                             self.semantics.target_location.id)

    def _cancel(self):
        pass


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

    action = HandOver()

    config = ConfigurationData({'action': 'hand-over',
                                'object': {'location': 'cabinet'},
                                'source-location': {'id': 'cabinet'},
                                'target-location': {'id': 'dinner_table'}})

    action.configure(robot, config)
    action.start()
