from action import Action, ConfigurationData
from entity_description import resolve_entity_description
from robot_smach_states.util.designators import EdEntityDesignator, VariableDesignator
from robot_smach_states.clear import Clear as ClearSmachState
# from challenge_clear import ClearMachine
import rospy


class Clear(Action):
    """
    The Clear class implements the action to clean up a location.

    Parameters to pass to the configure() method are 'source-location' (required) and 'target-location' (optional).
    """
    def __init__(self):
        Action.__init__(self)
        self._required_field_prompts = {'source-location': "What location would you like me to clear?"}
        self._required_skills = ['base']

    class Semantics:
        def __init__(self):
            self.target_location = None
            self.source_location = None

    @staticmethod
    def _parse_semantics(semantics_dict):
        semantics = Clear.Semantics()

        semantics.source_location = resolve_entity_description(semantics_dict['source-location'])
        semantics.target_location = resolve_entity_description(semantics_dict['target-location'])

        return semantics

    def _configure(self, robot, config):
        self._robot = robot

        # Parse semantics and context to a convenient object
        self.semantics = self._parse_semantics(config.semantics)
        self._source_location = self.semantics.source_location
        self._target_location = self.semantics.target_location

        self._robot.speech.speak("target is {}, source is {}".format(self._target_location.id, self._source_location.id))

        source_location_designator = EdEntityDesignator(self._robot, id=self._source_location.id)
        target_location_designator = EdEntityDesignator(self._robot, id=self._target_location.id)

        source_searchArea = self._knowledge.get_inspect_areas(self._source_location.id)
        source_navArea = self._knowledge.get_inspect_position(self._source_location.id)

        target_placeArea = self._knowledge.get_inspect_areas(self._target_location.id)
        target_navArea = self._knowledge.get_inspect_position(self._target_location.id)

        source_searchAreaDes = VariableDesignator(source_searchArea)
        source_navAreaDes = VariableDesignator(source_navArea)
        target_placeAreaDes = VariableDesignator(target_placeArea)
        target_navAreaDes = VariableDesignator(target_navArea)

        self._clear_sm = ClearSmachState(robot = self._robot,
                                         source_location = source_location_designator,
                                         source_navArea = source_navAreaDes,
                                         source_searchArea = source_searchAreaDes,
                                         target_location = target_location_designator,
                                         target_navArea = target_navAreaDes,
                                         target_placeArea = target_placeAreaDes)
        self._config_result.succeeded = True
        return

    def _start(self):

         outcome = self._clear_sm.execute()
         if outcome == 'done':
             self._execute_result.message = " I cleared the table! Wasn't that awesome? "
             self._execute_result.succeeded = True
         elif outcome == 'failed':
             self._execute_result.message = " Something went wrong. Sorry! "
             self._execute_result.succeeded = False
         return


    def _cancel(self):
        # self._clear_sm.request_preempt()
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
