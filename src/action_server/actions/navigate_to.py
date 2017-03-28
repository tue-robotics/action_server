from action import Action
from util import entities_from_description

import threading, rospy

import robot_smach_states
from robot_smach_states.navigation import NavigateToObserve, NavigateToWaypoint

class NavigateTo(Action):

    def __init__(self):
        self._nwc = None

    def _configure_(self, robot, config):
        if "entity" not in config:
            self._config_result.missing_field = "entity"
            return

        self._robot = robot # TODO: this should also check if the given robot is capable of this action.
        self._entity_description = config["entity"]

        self._config_result.succeeded = True
        return

    def _start(self):
        (entities, error_msg) = entities_from_description(self._entity_description, self._robot)
        if not entities:
            return error_msg

        e = entities[0]

        if e.is_a("waypoint"):
            self._nwc = NavigateToWaypoint(self._robot,
                                          waypoint_designator=robot_smach_states.util.designators.EdEntityDesignator(self._robot, id=e.id),
                                          radius=0.1)
            rospy.loginfo("Navigating to waypoint")
        else:
            self._nwc = NavigateToObserve(self._robot,
                                         entity_designator=robot_smach_states.util.designators.EdEntityDesignator(self._robot, id=e.id),
                                         radius=.5)
            rospy.loginfo("Navigating to observe")

        self._thread = threading.Thread(name='navigate', target=self._nwc.execute)
        self._thread.start()

    def _cancel(self):
        if self._nwc.is_running:
            self._nwc.request_preempt()

        # Wait until canceled
        self._thread.join()
