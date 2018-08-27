from action import Action, ConfigurationData
from entity_description import resolve_entity_description

import rospy


class Clear(Action):
    """
    The HandOver class implements the action to hand over an object to a person.

    Parameters to pass to the configure() method are 'source-location' (required), 'target-location' (required) and
    an object to bring (required).
    """
    def __init__(self):
        Action.__init__(self)
        self._required_field_prompts = {'source-location': "What location would you like me to clear?",
                                        'target-location': "Where would you like me to clear the items to?"}
        self._required_skills = ['base']

    class Semantics:
        def __init__(self):
            self.target_location = None
            self.source_location = None

    @staticmethod
    def _parse_semantics(semantics_dict):
        semantics = Clear.Semantics()

        semantics.target_location = resolve_entity_description(semantics_dict['target-location'])
        semantics.source_location = resolve_entity_description(semantics_dict['source-location'])

        return semantics

    def _configure(self, robot, config):
        self._robot = robot

        # Parse semantics and context to a convenient object
        self.semantics = self._parse_semantics(config.semantics)

        # options:
        #   - navigate to target  ->  inspect target  ->  for entity in segmented_objects: place to trashbin
        #   - navigate to target  ->  inspect target  ->  place (random) entity in trashbin  ->  inspect target  -> loop
        #   - make state machine (like demo presentation) that executes whole action
        #   -

    def _test(self):
        self._robot.speech.speak("Test if clear can be called through the action server.", block=False)


    def _start(self):
        # Handover
        self._test()

        self._execute_result.succeeded = True


    def _cancel(self):
        pass


if __name__ == "__main__":
    rospy.init_node('clear_test')

    import sys
    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    elif robot_name == 'hero':
        from robot_skills.hero import Hero as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = Clear()

    config = ConfigurationData({'action': 'clear',
                                'source-location': {'id': 'cabinet'},
                                'target-location': {'id': 'dinner_table'}})

    action.configure(robot, config)
    action.start()


#########################################
# smach.StateMachine.add('SEGMENT', robot_smach_states.SegmentObjects(robot,
#                                                                     e_classifications_des.writeable,
#                                                                     e_des,
#                                                                     source_location),
#                        transitions={'done': "HANDLE_DETECTED_ENTITIES"})
#
# smach.StateMachine.add('HANDLE_DETECTED_ENTITIES', HandleDetectedEntities(robot, e_classifications_des, known_types,
#                                                                           location_id, source_location),
#                        transitions={"done": next_state})