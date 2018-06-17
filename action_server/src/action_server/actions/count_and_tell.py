from action import Action, ConfigurationData
from entity_description import resolve_entity_description

import rospy


class CountAndTell(Action):
    ''' The CountAndTell class implements the action to count the number of objects and tell the operator.

    Parameters to pass to the configure() method are:
     - `sentence` (required): The sentence to speak. May be a keyword to tell something more intelligent.
    '''
    def __init__(self):
        Action.__init__(self)
        self._required_skills = ['speech']

    class Semantics:
        def __init__(self):
            self.location = None
            self.object = None

    @staticmethod
    def _parse_semantics(semantics_dict):
        semantics = CountAndTell.Semantics()

        semantics.location = resolve_entity_description(semantics_dict['location'])
        semantics.object = resolve_entity_description(semantics_dict['object'])

        return semantics

    def _configure(self, robot, config):
        self._robot = robot

        semantics = CountAndTell._parse_semantics(config.semantics)

        self._sentence = "I understand you want me to count the number of {} on the {} and tell you. " \
                         "But I can't do that yet. Sorry. Next task please.".format(semantics.object.type,
                                                                                    semantics.location.id)

        self._config_result.succeeded = True
        return

    def _start(self):
        self._robot.speech.speak(self._sentence, block=True)
        self._execute_result.succeeded = True

    def _cancel(self):
        pass


if __name__ == "__main__":
    rospy.init_node('say_test')

    import sys
    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = CountAndTell()

    config = ConfigurationData({'action': 'count-and-tell',
              'location': {'id': 'cabinet'},
              'object': {'type': 'beer'}})

    action.configure(robot, config)
    action.start()
