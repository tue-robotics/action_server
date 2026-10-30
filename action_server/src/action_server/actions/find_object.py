import math

import rospy

from ed.entity import Entity
from robot_smach_states.human_interaction import FindPersonInRoom
from robot_smach_states.navigation import Find as StatesFind, NavigateToWaypoint
from robot_smach_states.util.designators import EdEntityDesignator, VariableDesignator
from action_server.actions.action import Action, ConfigurationData
from action_server.actions.entity_description import resolve_entity_description

class FindObject(Action):
    """
    The FindObject class implements the action to find an object at a specified location.

    Parameters to pass to the configure() method are:
     - `location` (required): location to find the object (room or pre-existing world model entity)
     - `object` (required): the object to find.
    """

    def __init__(self):
        Action.__init__(self)
        self._required_field_prompts = {'object': " What exactly would you like me to find? "}
        self._required_skills = ['head', 'base', 'speech']

    class Semantics:
        def __init__(self):
            self.object = None
            self.source_location = None

        def __repr__(self):
            return str(self.__dict__)

    @staticmethod
    def _parse_semantics(semantics_dict):
        semantics = FindObject.Semantics()

        semantics.object = resolve_entity_description(semantics_dict['object'])

        if 'source-location' in semantics_dict:
            semantics.source_location = resolve_entity_description(semantics_dict['source-location'])

        return semantics

    class Context:
        def __init__(self):
            self.location = None

        def __repr__(self):
            return str(self.__dict__)

    @staticmethod
    def _parse_context(context_dict):
        context = FindObject.Context()

        if 'location' in context_dict:
            context.location = resolve_entity_description(context_dict['location'])

        return context

    def _configure(self, robot, config):
        self._robot = robot

        self._semantics = FindObject._parse_semantics(config.semantics)
        self._context = FindObject._parse_context(config.context)

        # Check if we know the source location
        if self._semantics.source_location:
            pass
        elif self._context.location:
            e = self._context.location.designator.resolve()
            if e:
                self._semantics.source_location = self._context.location
            else:
                if self._semantics.object.id:
                    self._config_result.message = " Where should I look for the {}?".format(self._semantics.object.id)
                else:
                    self._config_result.message = " Where should I look for the {}?".format(self._semantics.object.type)
                self._config_result.missing_field = 'source-location'
                return
        else:
            self._config_result.message = " Where should I look for the {}?".format(self._semantics.object.type)
            self._config_result.missing_field = 'source-location'
            return

        # We can now assume we know the source location
        self._config_result.context['object'] = config.semantics['object']

        # Find all areas to inspect
        self._areas = {}
        self._nav_areas = {}
        if self._semantics.source_location.id in self._knowledge.location_rooms:
            locations = self._knowledge.get_locations(self._semantics.source_location.id, True)
            for location in locations:
                inspect_areas = self._knowledge.get_inspect_areas(location)
                self._areas[location] = inspect_areas
                # Use the first inspect area to determine navigation position
                nav_area = inspect_areas[0] if inspect_areas else "on_top_of"
                nav_position = self._knowledge.get_inspect_position(location, nav_area)
                # Fallback to "on_top_of" if the navigation position doesn't work
                self._nav_areas[location] = nav_position if nav_position else "on_top_of"
        else:
            inspect_areas = self._knowledge.get_inspect_areas(self._semantics.source_location.id)
            self._areas[self._semantics.source_location.id] = inspect_areas
            # Use the first inspect area to determine navigation position  
            nav_area = inspect_areas[0] if inspect_areas else "on_top_of"
            nav_position = self._knowledge.get_inspect_position(self._semantics.source_location.id, nav_area)
            # Fallback to "on_top_of" if the navigation position doesn't work
            self._nav_areas[self._semantics.source_location.id] = nav_position if nav_position else "on_top_of"

        # Set up the designator with the object description
        entity_description = {
            'type': self._semantics.object.type,
            'category': self._semantics.object.category
        }
        description_designator = VariableDesignator(entity_description)

        location_designator = None
        self._find_state_machines = []
        self._found_entity_designator = VariableDesignator(resolve_type=Entity)
        for loc, areas in self._areas.items():
            location_designator = EdEntityDesignator(self._robot, uuid=loc)
            nav_area = self._nav_areas[loc]
            for area in areas:
                area_designator = VariableDesignator(area)
                navigation_area_designator = VariableDesignator(nav_area)

                rospy.loginfo("Setting up state machine with loc = {}, area = {}, nav_area = {}".format(
                    loc, area, nav_area
                ))

                self._find_state_machines.append(
                    StatesFind(
                        robot=self._robot,
                        knowledge=self._knowledge,
                        source_entity_designator=location_designator,
                        description_designator=description_designator,
                        area_name=area_designator,
                        navigation_area=navigation_area_designator,
                        found_entity_designator=self._found_entity_designator
                    )
                )

        self._config_result.context['object'] = {
            'designator': self._found_entity_designator,
            'type': self._semantics.object.type,
            'category': self._semantics.object.category
        }
        self._config_result.context['location'] = {'designator': location_designator}
        if self._semantics.source_location.id:
            self._config_result.context['location']['id'] = self._semantics.source_location.id
        self._config_result.succeeded = True

    def _start(self):
        for fsm in self._find_state_machines:
            res = fsm.execute()

            location = None
            if self._semantics.source_location.id:
                location = self._semantics.source_location.id

            if res in ['succeeded', 'found']:
                self._robot.speech.speak("Hey, I found a {}!".format(
                    self._semantics.object.type if self._semantics.object.type else self._semantics.object.category
                ))
                self._execute_result.message = " I found a {} ".format(
                    self._semantics.object.type if self._semantics.object.type else self._semantics.object.category
                )
                self._execute_result.succeeded = True
                return

            elif res == 'not_found':
                self._robot.speech.speak("I don't see what I am looking for here.")
                self._execute_result.message = " I couldn't find {} {} the {} ".format(
                    self._semantics.object.id if self._semantics.object.id and self._semantics.object.id != "None"
                    else "a " + self._semantics.object.type if self._semantics.object.type
                    else "a " + self._semantics.object.category,
                    "in" if self._semantics.source_location.id in self._knowledge.location_rooms else "at",
                    self._semantics.source_location.id
                )

            else:
                self._robot.speech.speak(" I'm unable to inspect the {} ".format(self._semantics.source_location.id))
                self._execute_result.message = " I was unable to inspect the {} to find {}. ".format(
                    self._semantics.source_location.id,
                    self._semantics.object.id if self._semantics.object.id else
                    "a " + self._semantics.object.type if self._semantics.object.type else
                    "a " + self._semantics.object.category
                )

    def _cancel(self):
        pass


if __name__ == "__main__":
    rospy.init_node('find_object_test')

    from robot_skills import get_robot_from_argv

    robot = get_robot_from_argv(1)

    action = FindObject()

    config = ConfigurationData({
        'action': 'find-object',
        'source-location': {'id': 'floor', 'area': 'on_top_of'},
        'object': {'type': 'coke'}
    })

    action.configure(robot, config)
    action.start()
