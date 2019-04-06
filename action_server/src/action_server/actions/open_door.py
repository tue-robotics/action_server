from action import Action, ConfigurationData
from robot_smach_states.manipulation import OpenDoor as OpenDoorMachine
from entity_description import resolve_entity_description

class OpenDoor(Action):
    """
    The Open Door action wraps the state machine to open the door of the cupboard at RWC 2018.

    """
    def __init__(self):
        # Call the base class constructor
        Action.__init__(self)
        self._required_field_prompts = {'furniture_id':
                                            " From what piece of furniture would you like me to open the door? "}
        self._required_field_prompts = {'furniture_navigate_area':
                                            " Where near that piece of furniture should I navigate to? "}
        self.furniture_inspect_area = {'furniture_inspect_area':
                                           " What near that piece of furniture can be used to inspect? "}

    class Semantics:
        def __init__(self):
            self.furniture_id = None
            self.furniture_navigate_area = None
            self.furniture_inspect_area = None

    @staticmethod
    def _parse_semantics(semantics_dict):
        semantics = OpenDoor.Semantics()

        if 'furniture_id' in semantics_dict:
            semantics.furniture_id = resolve_entity_description(semantics_dict['furniture_id'])
        if 'furniture_navigate_area' in semantics_dict:
            semantics.furniture_navigate_area = resolve_entity_description(semantics_dict['furniture_navigate_area'])
        if 'furniture_inspect_area' in semantics_dict:
            semantics.furniture_inspect_area = resolve_entity_description(semantics_dict['furniture_inspect_area'])

        return semantics

    def _configure(self, robot, config):
        self._robot = robot

        semantics = OpenDoor._parse_semantics(config.semantics)

        self._state_machine = OpenDoorMachine(self._robot, semantics.furniture_id,
                                              semantics.furniture_navigate_area, semantics.furniture_inspect_area)
        self._config_result.succeeded = True

    def _start(self):
        self._result = self._state_machine.execute()
        if self._result == "succeeded":
            self._execute_result.message = " I opened a door. "
            self._execute_result.succeeded = True
        else:
            self._execute_result.message = " I could not open a door. "
            self._execute_result.succeeded = False
        return

    def _cancel(self):
        pass

if __name__ == "__main__":
    rospy.init_node('open_door_test')

    import sys
    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    elif robot_name == 'hero':
        from robot_skills.sergio import Hero as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = OpenDoor()

    config = ConfigurationData({'action': 'open-door',
                                'furniture_id': 'cabinet',
                                'furniture_navigate_area': 'in_front_of',
                                'furniture_inspect_area': 'on_top_of'})

    action.configure(robot, config)
    action.start()
