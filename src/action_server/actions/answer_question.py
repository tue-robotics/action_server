from action import Action, ConfigurationData
from find import Find

import rospy
from robocup_knowledge import load_knowledge
import hmi


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

        self._speech_data = load_knowledge('challenge_gpsr')
        if not self._speech_data:
            rospy.logerr("Failed to load speech data for 'AnswerQuestion' action")
            return

        # If a person is specified in the task description, we need to go and find that person first
        if 'target-person' in config.semantics:
            self._find_action = Find()
            location_id = config.semantics['target-person']['loc']
            find_semantics = {'object' : config.semantics['target-person'],
                              'location' : {'id': location_id,
                                            'type': "furniture" if self._knowledge.is_location(location_id)
                                            else "room"}}
            find_config = ConfigurationData(find_semantics, config.knowledge)
            find_config_result = self._find_action.configure(robot, find_config)
            if not find_config_result.succeeded:
                self._config_result = find_config_result
                return
        else:
            self._find_action = None

        self._config_result.succeeded = True

    def _start(self):
        if self._find_action:
            find_exec_res = self._find_action.start()
            if not find_exec_res.succeeded:
                self._execute_result = find_exec_res
                return

        self._robot.head.look_at_standing_person()
        self._robot.head.wait_for_motion_done()

        self._robot.speech.speak("What is your question?")

        tries = 0
        while tries < 3:
            try:
                res = self._robot.hmi.query(description="",
                                            grammar=self._speech_data.question_grammar,
                                            target=self._speech_data.question_grammar_target)
            except hmi.TimeoutException:
                self._robot.speech.speak("My ears are not working properly, sorry!")
                self._execute_result.message = " I was unable to hear anything when listening for a question, so I " \
                                               "could not answer it. "
                tries += 1
                continue

            print res.semantics

            if res.semantics and "actions" in res.semantics:
                rospy.loginfo("Question was: '%s'?" % res.sentence)
                self._robot.speech.speak("The answer is %s" % res.semantics['actions'][0]['solution'])
                self._execute_result.message = " I answered the question {}".format(res.sentence)
                self._execute_result.succeeded = True
                return
            else:
                if tries < 2:
                    self._robot.speech.speak("Sorry, I did not understand your question, try another one.")
                else:
                    self._robot.speech.speak("Sorry, I was unable to understand any of your questions. I'll leave you puzzled by them. ")
                    self._execute_result.message = " I did not understand the question. "
                tries += 1


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
