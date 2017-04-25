from action import Action
from util import entities_from_description

import threading
import rospy

import robot_smach_states
from robot_smach_states.navigation import NavigateToObserve, NavigateToWaypoint


class NavigateTo(Action):

    def __init__(self):
        Action.__init__(self)
        self._fsm = None

    def _configure(self, robot, config):
        if "entity" not in config:
            self._config_result.missing_field = "entity"
            return

        # TODO: this should also check if the given robot is capable of this action.
        self._robot = robot
        self._entity_description = config['entity']

        self._config_result.succeeded = True
        return

    def _start(self):
        (entities, error_msg) = entities_from_description(self._entity_description, self._robot)

        if not entities:
            rospy.loginfo("No knowledge of a " + self._entity_description["id"] + " in the world model")
            self._execute_result.succeeded = False
            self._execute_result.message = "did not have knowledge of a " + self._entity_description["id"]
            return

        e = entities[0]

        if e.is_a("waypoint"):
            self._fsm = NavigateToWaypoint(self._robot,
                                          waypoint_designator=robot_smach_states.util.designators.EdEntityDesignator(self._robot, id=e.id),
                                          radius=0.1)
            rospy.loginfo("Navigating to waypoint")
        else:
            self._fsm = NavigateToObserve(self._robot,
                                         entity_designator=robot_smach_states.util.designators.EdEntityDesignator(self._robot, id=e.id),
                                         radius=.5)
            rospy.loginfo("Navigating to observe")

        self._thread = threading.Thread(name='navigate', target=self._fsm.execute)
        self._thread.start()

        # TODO: implement possibility to cancel action when started, but still block while executing
        self._thread.join()
        self._execute_result.succeeded = True
        self._execute_result.message = "navigated to the " + e.id

    def _cancel(self):
        if self._fsm.is_running:
            self._fsm.request_preempt()

        # Wait until canceled
        self._thread.join()
