from action import Action

import rospy
import random
from datetime import datetime, timedelta

class Say(Action):
    def __init__(self):
        Action.__init__(self)
        self._required_field_prompts = {'sentence' : "I didn't get what you want me to say."}

    def _configure(self, robot, config):
        if not hasattr(robot, "speech"):
            rospy.logerr("Robot {} does not have attribute 'speech'".format(robot.robot_name))
            return

        self._robot = robot

        self._sentence = config['sentence']

        self._config_result.succeeded = True
        return

    def _start(self):
        rospy.loginfo('Answering {}'.format(self._sentence))

        # TODO: This knowledge should be somehow returned by the robot object
        if self._sentence == 'TIME':
            hours = datetime.now().hour
            minutes = datetime.now().minute
            line = "The time is {} {}".format(hours, minutes)
        elif self._sentence == "ROBOT_NAME":
            line = 'My name is {}'.format(self._robot.robot_name)
        elif self._sentence == 'TODAY':
            line = datetime.today().strftime('Today is %A %B %d')
        elif self._sentence == 'TOMORROW':
            line = (datetime.today() + timedelta(days=1)).strftime('Tomorrow is %A %B %d')
        elif self._sentence == 'DAY_OF_MONTH':
            line = datetime.now().strftime('It is day %d of the month')
        elif self._sentence == 'DAY_OF_WEEK':
            line = datetime.today().strftime('Today is a %A')
        elif self._sentence == 'DARK_SIDE':
            line = " I'll never join you! "
        elif self._sentence == 'JOKE':
            line = random.choice([
                "What do you call a fish with no eyes? A fsh.",
                "You don't need a parachute to go skydiving. You need a parachute to go skydiving twice.",
                "What is the difference between a snowman and a snowwoman?       Snowballs.",
                "What is Bruce Lee's favorite drink? Wataaaaah!",
                "A blind man walks into a bar. And a table. And a chair.",
                "It's color is yellow and when you push the button, it turns red?         A chick in the blender"
            ])
        else:
            line = self._sentence

        self._robot.speech.speak(line)
        self._execute_result.succeeded = True
        self._execute_result.message = " I told you what you wanted to hear. "

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
