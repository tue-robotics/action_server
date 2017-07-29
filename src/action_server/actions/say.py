from action import Action, ConfigurationData
from find import Find

import rospy
import random
from datetime import datetime, timedelta

class Say(Action):
    ''' The Say class implements the action to say something.

    Parameters to pass to the configure() method are:
     - `sentence` (required): The sentence to speak. May be a keyword to tell something more intelligent.
    '''
    def __init__(self):
        Action.__init__(self)
        self._required_field_prompts = {'sentence' : "I didn't get what you want me to say."}
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
        if self._sentence == 'TIME':
            hours = datetime.now().hour
            minutes = datetime.now().minute
            line = "The time is {} {}".format(hours, minutes)
            self._execute_result.message = " I told the time. "
        elif self._sentence == "TEAM_NAME":
            line = "My team's name is Tech United Eindhoven"
            self._execute_result.message = " I told my team's name. "
        elif self._sentence == "TEAM_AFFILIATION":
            line = "My team is affiliated with the University of Technology Eindhoven"
            self._execute_result.message = " I told my team's affiliation. "
        elif self._sentence == "ROBOT_NAME":
            line = 'My name is {}'.format(self._robot.robot_name)
            self._execute_result.message = " I told my name. "
        elif self._sentence == 'TODAY':
            line = datetime.today().strftime('Today is %A %B %d')
            self._execute_result.message = " I told what day it is. "
        elif self._sentence == 'TOMORROW':
            line = (datetime.today() + timedelta(days=1)).strftime('Tomorrow is %A %B %d')
            self._execute_result.message = " I told what day it is tomorrow. "
        elif self._sentence == 'DAY_OF_MONTH':
            line = datetime.now().strftime('It is day %d of the month')
            self._execute_result.message = " I told the day of the month. "
        elif self._sentence == 'DAY_OF_WEEK':
            line = datetime.today().strftime('Today is a %A')
            self._execute_result.message = " I told the day of the week. "
        elif self._sentence == 'DARK_SIDE':
            line = " I'll never join you! "
            self._execute_result.message = " I told I'll never join the dark side. "
        elif self._sentence == 'JOKE':
            line = random.choice([
                "What do you call a fish with no eyes? A fsh.",
                "You don't need a parachute to go skydiving. You need a parachute to go skydiving twice.",
                "What is the difference between a snowman and a snowwoman?       Snowballs.",
                "What is Bruce Lee's favorite drink? Wataaaaah!",
                "A blind man walks into a bar. And a table. And a chair.",
                "It's color is yellow and when you push the button, it turns red?         A chick in the blender"
            ])
            self._execute_result.message = " I told a joke. "
        elif self._sentence == 'SOMETHING_ABOUT_SELF':
            line = random.choice([
                "I once dragged a person across the floor for meters.",
                "I am still a bit insecure about my gender with my manly voice and my white dress.",
                "I've been to tournaments in seven countries already, and still counting. However, I feel like I'm getting a little too old for this shit.",
                "I once tripped over the border of a Middle Size League soccer field at full speed."
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
