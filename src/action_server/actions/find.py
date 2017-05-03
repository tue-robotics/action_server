from action import Action
from entity_description import resolve_entity_description

import rospy

import robot_smach_states as states
from robot_smach_states.util.designators import EdEntityDesignator, VariableDesignator
from robot_skills.util.entity import Entity

class Find(Action):
    def __init__(self):
        Action.__init__(self)
        # TODO: change this to a python dictionary schema
        self._required_parameters = ['object', 'location']

    def _configure(self, robot, config):
        self._robot = robot
        object = resolve_entity_description(config['object'])
        location = resolve_entity_description(config['location'])

        # Set up designator for location to go and find something
        if not location.id:
            self._config_result.missing_field('location.id')
            return
        self._location_designator = EdEntityDesignator(self._robot, id=location.id)

        # Set up designator for area
        if not 'area' in config['location']:
            self._config_result.missing_field('location.area')
            return
        self._area_designator = VariableDesignator(config['location']['area'])

        # Set up the designator with the object description
        if not object.type:
            self._config_result.missing_field('object.type')
            return
        self._description_designator = VariableDesignator({'type': object.type})

        # Set up designator to be filled with the found entity
        self._found_entity_designator = VariableDesignator(resolve_type=Entity)

        # Set the resulting knowledge of the Find action to this found object designator
        self._config_result.resulting_knowledge['found_entity'] = self._found_entity_designator

        # Set up the Find state machine
        self._fsm = states.Find(robot=self._robot,
                                source_entity_designator=self._location_designator,
                                description_designator=self._description_designator,
                                area_name_designator=self._area_designator,
                                found_entity_designator=self._found_entity_designator)

        self._config_result.succeeded = True

    def _start(self):
        res = self._fsm.execute()
        if res == 'succeeded':
            self._execute_result.succeeded = True

    def _cancel(self):
        pass

if __name__ == "__main__":
    rospy.init_node('find_test')

    import sys
    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = Find()

    config = {'action': 'find',
              'location': {'id': 'cabinet',
                           'area': 'on_top_of'},
              'object': {'type': 'coke'}}

    action.configure(robot, config)
    action.start()

    config = {'action': 'find',
              'location': {'id': 'livingroom',
                           'area': 'in'},
              'object': {'type': 'person'}}

    action.configure(robot, config)
    action.start()

