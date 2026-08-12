import math

import rospy

from ed.entity import Entity
from robot_smach_states.human_interaction import FindPersonInRoom
from robot_smach_states.navigation import Find as StatesFind, NavigateToWaypoint
from robot_smach_states.util.designators import EdEntityDesignator, VariableDesignator
from action_server.actions.action import Action, ConfigurationData
from action_server.actions.entity_description import resolve_entity_description


class FindPerson(Action):
    def __init__(self):
        super().__init__()
        self._required_field_prompts = {'person': "Who exactly would you like me to find?"}
        self._required_skills = ['head', 'base', 'speech']

    def _point_at_person(self, person):
        pose_base_link = self._robot.tf_buffer.transform(person.pose, self._robot.base_link_frame)

        x = pose_base_link.frame.p.x()
        y = pose_base_link.frame.p.y()

        th = math.atan2(y, x)
        vth = 0.5

        self._robot.head.cancel_goal()
        self._robot.base.force_drive(0, 0, math.copysign(1, th) * vth, abs(th / vth))

        self._robot.speech.speak("I will point at you now.")

        self._robot.head.look_at_ground_in_front_of_robot(distance=100)
        arm = self._robot.get_arm(required_goals=["point_at", "reset"])
        arm.send_joint_goal("point_at")

        self._robot.speech.speak("You're right there!")

        arm.send_joint_goal("reset")

    class Semantics:
        def __init__(self):
            self.person = None
            self.source_location = None

    @staticmethod
    def _parse_semantics(semantics_dict):
        semantics = FindPerson.Semantics()
        semantics.person = resolve_entity_description(semantics_dict['person'])
        if 'source-location' in semantics_dict:
            semantics.source_location = resolve_entity_description(semantics_dict['source-location'])
        return semantics

    class Context:
        def __init__(self):
            self.location = None

    @staticmethod
    def _parse_context(context_dict):
        context = FindPerson.Context()
        if 'location' in context_dict:
            context.location = resolve_entity_description(context_dict['location'])
        return context

    def _configure(self, robot, config):
        self._robot = robot
        self._semantics = FindPerson._parse_semantics(config.semantics)
        self._context = FindPerson._parse_context(config.context)

        # --- location handling ---
        if self._semantics.source_location:
            pass
        elif self._context.location:
            e = self._context.location.designator.resolve()
            if e:
                self._semantics.source_location = self._context.location
            else:
                self._config_result.message = f"Where should I look for {self._semantics.person.id or self._semantics.person.type}?"
                self._config_result.missing_field = 'source-location'
                return
        else:
            self._config_result.message = f"Where should I look for {self._semantics.person.type}?"
            self._config_result.missing_field = 'source-location'
            return

        # --- setup state machines ---
        self._config_result.context['person'] = config.semantics['person']

        if self._semantics.source_location.id not in self._knowledge.location_rooms:
            self._semantics.source_location.id = self._knowledge.get_room(self._semantics.source_location.id)

        discard_other_labels = bool(self._semantics.person.id)
        self._found_entity_designator = VariableDesignator(resolve_type=Entity)
        self._find_state_machines = [
            FindPersonInRoom(robot, self._semantics.source_location.id, self._semantics.person.id or "someone",
                             discard_other_labels, self._found_entity_designator.writeable)
        ]
        self._config_result.succeeded = True

        self._navigation_state_machine = NavigateToWaypoint(
            self._robot,
            waypoint_designator=self._found_entity_designator,
            radius=1.0,
            look_at_designator=self._found_entity_designator
        )

    def _start(self):
        for fsm in self._find_state_machines:
            res = fsm.execute()
            if res in ['succeeded', 'found']:
                location = self._semantics.source_location.id if self._semantics.source_location else None
                msg = f"I found {self._semantics.person.id or 'someone'}"
                if location:
                    msg += f" at the {location}."
                self._execute_result.message = msg
                self._point_at_person(self._found_entity_designator.resolve())
                self._navigation_state_machine.execute()
                self._robot.speech.speak("Hi there!")
                self._execute_result.succeeded = True
                return
            elif res == 'not_found':
                self._robot.speech.speak("I don't see anyone here.")
                self._execute_result.message = f"I couldn't find {self._semantics.person.id or 'someone'}"
            else:
                self._robot.speech.speak(f"I'm unable to inspect the {self._semantics.source_location.id}")
                self._execute_result.message = f"I was unable to inspect the {self._semantics.source_location.id}"

    def _cancel(self):
        pass

if __name__ == "__main__":
    rospy.init_node('find_test')

    from robot_skills import get_robot_from_argv

    robot = get_robot_from_argv(1)

    action = FindPerson()

    config = ConfigurationData({'action': 'find',
                                'source-location': {'id': 'cabinet',
                                                    'area': 'on_top_of'},
                                'person': {'type': 'person'}})

    action.configure(robot, config)
    action.start()
