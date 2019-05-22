from action import Action, ConfigurationData
from find import Find
from entity_description import resolve_entity_description

import rospy
from robocup_knowledge import load_knowledge
import challenge_spr
import hmi


class AnswerQuestion(Action):
    """ The AnswerQuestion class implements the action of answering a question

    It requires that the robot to perform this action has a speech and an ears skill. It will ask the user what
    the question is, try to answer it based on the grammar defined in the knowledge defined for the speech and
    person recognition challenge and answer it based on the same knowledge.

    """
    def __init__(self):
        Action.__init__(self)
        self._required_skills = ['speech', 'hmi']
        self._preempt_requested = False

    class Semantics:
        def __init__(self):
            self.target_person = None

    @staticmethod
    def _parse_semantics(semantics_dict):
        semantics = AnswerQuestion.Semantics()

        if 'target-person' in semantics_dict:
            semantics.target_person = resolve_entity_description(semantics_dict['target-person'])

        return semantics

    class Context:
        def __init__(self):
            self.object = None

    @staticmethod
    def _parse_context(context_dict):
        context = AnswerQuestion.Context()

        if 'object' in context_dict:
            context.object = resolve_entity_description(context_dict['object'])

        return context

    def _configure(self, robot, config):
        self._robot = robot

        # ToDo: remove this dependency!
        self._speech_data = load_knowledge('challenge_spr')
        if not self._speech_data:
            rospy.logerr("Failed to load speech data for 'AnswerQuestion' action")
            return

        semantics = AnswerQuestion._parse_semantics(config.semantics)
        context = AnswerQuestion._parse_context(config.context)

        # If a person is specified in the task description, we need to go and find that person first
        if semantics.target_person and not context.object:
            self._config_result.required_context = {
                'action': 'find',
                'object': config.semantics['target-person']
            }
            if semantics.target_person.location:
                self._config_result.required_context['source-location'] = config.semantics['target-person']['location']
                return

        self._config_result.succeeded = True

    def _start(self):
        self._robot.head.look_at_standing_person()
        self._robot.head.wait_for_motion_done()

        self._robot.speech.speak("What is your question?")

        tries = 0
        while tries < 3 and not self._preempt_requested:
            try:
                res = self._robot.hmi.query(description="",
                                            grammar=self._speech_data.grammar,
                                            target=self._speech_data.grammar_target)
            except hmi.TimeoutException:
                self._robot.speech.speak("My ears are not working properly, sorry!")
                self._execute_result.message = " I was unable to hear anything when listening for a question, so I " \
                                               "could not answer it. "
                tries += 1
                continue

            if res.semantics and "actions" in res.semantics:
                try:
                    # TODO: remove this from challenge states, because apparently, this is more generic.
                    challenge_spr.riddle_game.answer(self._robot, res, None)
                    self._execute_result.message = " I answered the question {}".format(res.sentence)
                    self._execute_result.succeeded = True
                    return
                except ValueError:
                    self._robot.speech.speak("I don't know these people you are talking about. ")
                    self._execute_result.message = " I couldn't answer the question about the crowd."
                    tries += 1
                    continue
            else:
                if tries < 2:
                    self._robot.speech.speak("Sorry, I did not understand your question, try another one.")
                else:
                    self._robot.speech.speak("Sorry, I was unable to understand any of your questions. I'll leave you puzzled by them. ")
                    self._execute_result.message = " I did not understand the question. "
                tries += 1

    def _cancel(self):
        self._preempt_requested = True


if __name__ == "__main__":
    rospy.init_node('answer_question_test')

    import sys

    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = AnswerQuestion()

    config = {'action': 'answer_question'}

    action.configure(robot, config)
    action.start()
