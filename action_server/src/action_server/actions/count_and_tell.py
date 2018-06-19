from action import Action, ConfigurationData
from entity_description import resolve_entity_description

import rospy
import robot_smach_states as states
import robot_smach_states.util.designators as ds



class CountAndTell(Action):
    ''' The CountAndTell class implements the action to count the number of objects and tell the operator.

    Parameters to pass to the configure() method are:
     - `sentence` (required): The sentence to speak. May be a keyword to tell something more intelligent.
    '''
    def __init__(self):
        Action.__init__(self)
        self._required_skills = ['speech']

    class Semantics:
        def __init__(self):
            self.location = None
            self.object = None

    @staticmethod
    def _parse_semantics(semantics_dict):
        semantics = CountAndTell.Semantics()

        semantics.location = resolve_entity_description(semantics_dict['location'])
        semantics.object = resolve_entity_description(semantics_dict['object'])

        return semantics

    def _configure(self, robot, config):
        self._robot = robot

        semantics = CountAndTell._parse_semantics(config.semantics)

        self._count_designator = ds.VariableDesignator(-1)
        self._count_state_machine = states.CountObjectsOnLocation(robot,
                                                                  location=semantics.location.id,
                                                                  num_objects_designator=self._count_designator.writeable,
                                                                  object_type=semantics.object.type)

        # Here we set up a message that is formatted further later, in self._start
        self._execute_result.message = "I counted {{c}} {t}s".format(t=semantics.object.type)
        self._config_result.succeeded = True
        return

    def _start(self):
        self._count_state_machine.execute()
        self._execute_result.succeeded = True

        # This message is instantiated in _configure but leaves some stuff to be formatted
        self._execute_result.message = self._execute_result.message.format(c=self._count_designator.resolve())

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

    action = CountAndTell()

    config = ConfigurationData({'action': 'count-and-tell',
              'location': {'id': 'counter'},
              'object': {'type': 'apple'}})

    action.configure(robot, config)
    action.start()
