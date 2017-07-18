from action import Action

import rospy
from robocup_knowledge import load_knowledge


class AnswerQuestion(Action):
    """ The AnswerQuestion class implements the action of answering a question

    It requires that the robot to perform this action has a speech and an ears skill. It will ask the user what
    the question is, try to answer it based on the grammar defined in the knowledge defined for the speech and
    person recognition challenge and answer it based on the same knowledge.

    """
    def __init__(self):
        Action.__init__(self)
        self._required_skills = ['speech', 'ears']

    def _configure(self, robot, config):
        self._robot = robot

        self._speech_data = load_knowledge('challenge_spr')
        if not self._speech_data:
            rospy.logerr("Failed to load speech data for 'AnswerQuestion' action")
            return

        self._config_result.succeeded = True

    def _start(self):
        self._robot.head.look_at_standing_person()
        self._robot.head.wait_for_motion_done()

        self._robot.speech.speak("What is your question?")

        res = self._robot.hmi.query(description="",
                                    grammar=self._speech_data.grammar,
                                    target=self._speech_data.grammar_target)

        if not res:
            self._robot.speech.speak("My ears are not working properly, sorry!")
            self._execute_result.message = "was unable to hear anything when listening for a challenge"
            return

        print res.semantics

        if res.semantics and "actions" in res.semantics:
            rospy.loginfo("Question was: '%s'?" % res.sentence)
            self._robot.speech.speak("The answer is %s" % res.semantics['actions'][0]['solution'])
            self._execute_result.message = "answered the question {}".format(res.sentence)
            self._execute_result.succeeded = True
        else:
            self._robot.speech.speak("Sorry, I did not understand your question.")
            self._execute_result.message = "did not understand the question"

    def _cancel(self):
        pass


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
