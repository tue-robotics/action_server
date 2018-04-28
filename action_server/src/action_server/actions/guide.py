from action import Action, ConfigurationData
import robot_smach_states as states
import robot_smach_states.util.designators as ds

import rospy


class Guide(Action):
    ''' The Guide class navigates to a target, telling someone to follow the robot and about arriving at the target.

    Parameters to pass to the configure() method are:
     - `object` (required): the id of the entity to navigate to
    '''
    def __init__(self):
        Action.__init__(self)
        self._required_field_prompts = {'object': " What exactly would you like me to find? "}
        self._required_skills = ['head', 'base', 'rightArm', 'speech']

    def _configure(self, robot, config):
        if 'object' in config.semantics and 'id' in config.semantics['object']:
            self.follower_id = config.semantics['object']['id']
        else:
            # TODO: HUUUUGE Robocup hack!
            self.follower_id = "lars"

        if 'target-location' in config.semantics and 'id' in config.semantics['target-location']:
            self.target_location_id = config.semantics['target-location']['id']
        else:
            # TODO: HUUUUGE Robocup hack!
            self.target_location_id = "bar"

        target_location_designator = ds.EntityByIdDesignator(robot, id=self.target_location_id)
        follower_designator = ds.EntityByIdDesignator(robot, id=self.follower_id)

        self._guide_state_machine = states.Guide(robot=robot, target_location=target_location_designator, follower=follower_designator)
        # self._config_result.context['location-designator'] = target_location_designator
        self._config_result.succeeded = True

    def _start(self):
        self._guide_state_machine.execute()
        self._execute_result.succeeded = True
        self._execute_result.message = "I guided "

    def _cancel(self):
        self._guide_state_machine.request_preempt()


if __name__ == "__main__":
    rospy.init_node('navigate_to_test')

    import sys
    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = Guide()

    config = ConfigurationData({'action': 'guide',
              'object': {'id': 'cabinet'}})

    action.configure(robot, config)
    action.start()
