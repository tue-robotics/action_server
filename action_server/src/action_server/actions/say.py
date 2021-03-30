import random
from datetime import datetime, timedelta

import rospy

from .action import Action, ConfigurationData
from .find import Find


class Say(Action):
    """
    The Say class implements the action to say something.

    Parameters to pass to the configure() method are:
     - `sentence` (required): The sentence to speak. May be a keyword to tell something more intelligent.
    """

    def __init__(self):
        Action.__init__(self)
        self._required_field_prompts = {'sentence': "What would you like me to say?"}
        self._required_skills = ['speech']

    def _configure(self, robot, config):
        self._robot = robot

        self._sentence = config.semantics['sentence']

        # If a person is specified in the task description, we need to go and find that person first
        if 'target-person' in config.semantics:
            self._find_action = Find()
            location_id = config.semantics['target-person']['loc']
            find_semantics = {'object': config.semantics['target-person'],
                              'location': {'id': location_id,
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
        return

    def _start(self):
        if self._find_action:
            find_exec_res = self._find_action.start()
            if not find_exec_res.succeeded:
                self._execute_result = find_exec_res
                return

        self._robot.head.look_at_standing_person()
        self._robot.head.wait_for_motion_done()

        rospy.loginfo('Answering {}'.format(self._sentence))

        # TODO: This knowledge should be somehow returned by the robot object
        if self._sentence == 'time':
            hours = datetime.now().hour
            minutes = datetime.now().minute
            line = "The time is {} {}".format(hours, minutes)
            self._execute_result.message = " I told the time. "
        elif self._sentence == "team_name":
            line = "My team's name is Tech United Eindhoven"
            self._execute_result.message = " I told my team's name. "
        elif self._sentence == "country":
            line = "My team is from the Netherlands, also known as Holland."
            self._execute_result.message = " I told my team's name. "
        elif self._sentence == "team_affiliation":
            line = "My team is affiliated with the University of Technology Eindhoven"
            self._execute_result.message = " I told my team's affiliation. "
        elif self._sentence == "robot_name":
            line = 'My name is {}'.format(self._robot.robot_name)
            self._execute_result.message = " I told my name. "
        elif self._sentence == 'today':
            line = datetime.today().strftime('Today is %A %B %d')
            self._execute_result.message = " I told what day it is. "
        elif self._sentence == 'tomorrow':
            line = (datetime.today() + timedelta(days=1)).strftime('Tomorrow is %A %B %d')
            self._execute_result.message = " I told what day it is tomorrow. "
        elif self._sentence == 'day_of_month':
            line = datetime.now().strftime('It is day %d of the month')
            self._execute_result.message = " I told the day of the month. "
        elif self._sentence == 'day_of_week':
            line = datetime.today().strftime('Today is a %A')
            self._execute_result.message = " I told the day of the week. "
        elif self._sentence == 'dark_side':
            line = " I'll never join you! "
            self._execute_result.message = " I told I'll never join the dark side. "
        elif self._sentence == 'you_shall_not_pass':
            line = " You shall not pass! "
            self._execute_result.message = " I told them I won't let them pass. "
        elif self._sentence == 'party':
            line = "start the party"
            self._execute_result.message = " I will get the party started. "
        elif self._sentence == 'joke':
            line = random.choice([
                "What do you call a fish with no eyes? A fsh.",
                "You don't need a parachute to go skydiving. You need a parachute to go skydiving twice.",
                "What is the difference between a snowman and a snowwoman?       Snowballs.",
                "What is Bruce Lee's favorite drink? Wataaaaah!",
                "A blind man walks into a bar. And a table. And a chair.",
                "It's color is yellow and when you push the button, it turns red?         A chick in the blender"
            ])
            self._execute_result.message = " I told a joke. "
        elif self._sentence == 'something_about_self':
            if self._robot.robot_name == 'amigo':
                line = random.choice([
                    "I once dragged a person across the floor for meters.",
                    "I've been to tournaments in seven countries already, and still counting.",
                    "I once tripped over the border of a Middle Size League soccer field at full speed."
                ])
            elif self._robot.robot_name == 'sergio':
                line = random.choice([
                    "I am very proud of my older brother Amigo. He has been competing in Robocup for a long time.",
                    "I have ankles, knees and hips to reach to the floor as well as the ceiling. "
                    "Although that would be hard here. ",
                    "Sometimes I feel a little tense. But then I touch something metal and it helps me discharge and "
                    "relax."
                ])
            elif self._robot.robot_name == 'hero':
                line = random.choice([
                    "My software is like Frankesteins monster, a raggety combination of Toyota, Tech United "
                    "and Stack Overflow.",
                    "I can't subtly drive over my charging cable. That does not keep me from trying though.",
                    "I have really funny ears, they allow me to hear whatever I want to hear!.",
                    "I think I'm a lot fatter than I am, which makes me too scared to go into a lot of places."
                ])
            self._execute_result.message = " I told something about myself. "
        else:
            line = self._sentence
            self._execute_result.message = " I told something. "

        self._robot.speech.speak(line)
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

    action = Say()

    config = {'action': 'say',
              'sentence': 'JOKE'}

    action.configure(robot, config)
    action.start()
