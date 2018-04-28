from action import Action, ConfigurationData
import rospy
import threading

import robot_smach_states

from entity_description import resolve_entity_description


class SendPicture(Action):
    """ The Inspect class implements the action to send a picture over a whatsapp client.

    Parameters to pass to the configure() method are:
     - `target-location` (required): an entity with a segmentation area to inspect
    """
    def __init__(self):
        Action.__init__(self)

    class Semantics:
        def __init__(self):
            self.target_location = None

    @staticmethod
    def _parse_semantics(semantics_dict):
        semantics = SendPicture.Semantics()
        semantics.target_location = resolve_entity_description(semantics_dict['target-location'])

        return semantics

    class Context:
        def __init__(self):
            self.location_designator = None

    @staticmethod
    def _parse_context(context_dict):
        context = SendPicture.Context()

        if 'location-designator' in context_dict:
            context.location_designator = context_dict['location-designator']

        return context

    def _configure(self, robot, config):
        self._robot = robot

        self.semantics = self._parse_semantics(config.semantics)
        self.context = self._parse_context(config.context)

        if self.context.location_designator is None:
            # Request navigation action
            self._config_result.required_context = {'action': 'navigate-to'}
            if self.semantics.target_location is not None:
                self._config_result.required_context['target-location'] = config.semantics['target-location']
            return
        # We can now assume we arrived at the place to take the picture from.

        self._config_result.succeeded = True
        return

    def _start(self):
        self._robot.speech.speak("I should be taking a picture and sending it to you now. ")
        self._execute_result.message = " I took a picture at {} "
        self._execute_result.succeeded = True

    def _cancel(self):
        pass


if __name__ == "__main__":
    rospy.init_node('inspect_test')

    import sys
    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = SendPicture()

    config = ConfigurationData({'action': 'inspect',
                                'target-location': {'id': 'cabinet'}})

    action.configure(robot, config)
    action.start()
