from action import Action, ConfigurationData
from entity_description import resolve_entity_description

import rospy


class TellNameOfPerson(Action):
    ''' The TellNameOfPerson class implements the action to ask someones name and report it to the operator.

    Parameters to pass to the configure() method are:
     - `sentence` (required): The sentence to speak. May be a keyword to tell something more intelligent.
    '''
    def __init__(self):
        Action.__init__(self)
        self._required_skills = ['speech']

    class Semantics:
        def __init__(self):
            self.location = None

    @staticmethod
    def _parse_semantics(semantics_dict):
        semantics = TellNameOfPerson.Semantics()

        semantics.location = resolve_entity_description(semantics_dict['location'])

        return semantics

    def _configure(self, robot, config):
        self._robot = robot

        semantics = TellNameOfPerson._parse_semantics(config.semantics)

        self._sentence = "I understand you want me to tell you the name of the person at the {}. " \
                         "But I can't do that yet. Sorry. Next task please.".format(semantics.location.id)

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
