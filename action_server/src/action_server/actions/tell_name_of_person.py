from action import Action, ConfigurationData
from find import Find
from entity_description import resolve_entity_description

import rospy
from robocup_knowledge import load_knowledge
import hmi


class TellNameOfPerson(Action):
    """ The TellNameOfPerson class implements the action to ask someones name and report it to the operator.

    Parameters to pass to the configure() method are:
     - `sentence` (required): The sentence to speak. May be a keyword to tell something more intelligent.
    """

    def __init__(self):
        Action.__init__(self)
        self._required_skills = ['speech', 'hmi']
        self._preempt_requested = False

        common_knowledge = load_knowledge('common')

        self._grammar = ""

        for name in common_knowledge.names:
            self._grammar += "\nNAME['%s'] -> %s" % (name, name)

    class Semantics:
        def __init__(self):
            self.location = None

    @staticmethod
    def _parse_semantics(semantics_dict):
        semantics = TellNameOfPerson.Semantics()

        if 'location' in semantics_dict:
            semantics.location = resolve_entity_description(semantics_dict['location'])

        return semantics

    class Context:
        def __init__(self):
            self.object = None

    @staticmethod
    def _parse_context(context_dict):
        context = TellNameOfPerson.Context()

        if 'object' in context_dict:
            context.object = resolve_entity_description(context_dict['object'])

        return context

    def _configure(self, robot, config):
        self._robot = robot

        semantics = TellNameOfPerson._parse_semantics(config.semantics)
        context = TellNameOfPerson._parse_context(config.context)

        # If a person is specified in the task description, we need to go and find that person first
        if semantics.location and not context.object:
            self._config_result.required_context = {
                'action': 'find',
                'object': {'type': 'person'},
                'source-location': config.semantics['location'],
            }
            return

        self._config_result.succeeded = True

    def _start(self):
        self._robot.head.look_at_standing_person()
        self._robot.head.wait_for_motion_done()

        self._robot.speech.speak("Hey sweetie what is your name?")

        tries = 0
        while tries < 3 and not self._preempt_requested:
            try:
                res = self._robot.hmi.query(description="",
                                            grammar=self._grammar,
                                            target='NAME')
            except hmi.TimeoutException:
                self._robot.speech.speak("My ears are not working properly, sorry!")
                self._execute_result.message = "I was unable to hear anything when listening for your name"
                tries += 1
                continue

            if res.semantics:
                self._robot.speech.speak("Hi {name}! Bye {name}".format(name=res.sentence))
                self._execute_result.message = "The person's name was {}".format(res.sentence)
                self._execute_result.succeeded = True
                return
            else:
                if tries < 2:
                    self._robot.speech.speak("Sorry, I did not understand you, try again.")
                else:
                    self._robot.speech.speak("Sorry, I was unable to understand you again. I'll just name you Maple. ")
                    self._execute_result.message = " I did not understand the answer. "
                tries += 1

    def _cancel(self):
        self._preempt_requested = True


if __name__ == "__main__":
    rospy.init_node('tell_name_of_person_test')

    import sys

    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = TellNameOfPerson()

    config = {'action': 'tell_name_of_person'}

    action.configure(robot, config)
    action.start()
