import rospy

import robot_smach_states as states
import robot_smach_states.util.designators as ds
from .action import Action, ConfigurationData
from .entity_description import resolve_entity_description


class CountAndTell(Action):
    """
    The CountAndTell class implements the action to count the number of objects in a specific location and tell the
        operator the amount of objects, the type and the location.

    Parameters to pass to the configure() method are:
     - `config` (required): the ConfigurationData defines the input data structure for configuration of an action i.e
                            a JSON string with for this action a "location" and an " object".
    """

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
        self._where_to_count_designator = ds.EntityByIdDesignator(robot, uuid=semantics.location.id)
        self._what_to_count_designator = ds.Designator(semantics.object.type)
        self._count_state_machine = states.InspectAndCount(robot,
                                                           self._where_to_count_designator,
                                                           self._what_to_count_designator,
                                                           self._count_designator)

        # Here we set up a message that is formatted further later, in self._start
        self._execute_result.message = "I counted {{c}} {t}s on the {l}".format(t=semantics.object.type,
                                                                                l=semantics.location.id)
        self._config_result.succeeded = True
        return

    def _start(self):
        res = self._count_state_machine.execute()
        if res == 'Aborted':
            self._execute_result.succeeded = False
            self._execute_result.message = "I failed to count the objects."
            return

        self._execute_result.succeeded = True

        # This message is instantiated in _configure but leaves some stuff to be formatted
        self._execute_result.message = self._execute_result.message.format(c=self._count_designator.resolve())
        self._robot.speech.speak(self._execute_result.message)

    def _cancel(self):
        pass


if __name__ == "__main__":
    rospy.init_node('say_test')

    from robot_skills import get_robot_from_argv

    robot = get_robot_from_argv(1)

    action = CountAndTell()

    config = ConfigurationData({'action': 'count-and-tell',
                                'location': {'id': 'counter'},
                                'object': {'type': 'apple'}})

    action.configure(robot, config)
    action.start()

    rospy.loginfo(action._execute_result)
