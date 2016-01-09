from action import Action
from util import entities_from_description

import threading, rospy

import robot_smach_states
from robot_smach_states.navigation import NavigateToObserve, NavigateToWaypoint

class NavigateTo(Action):

    def __init__(self):
        self._nwc = None

    def _start(self, config, robot):

        if "entity" not in config:
            return "No entity given"

        entity_descr = config["entity"]
        (entities, error_msg) = entities_from_description(entity_descr, robot)
        if not entities:
            return error_msg

        e = entities[0]

        if "waypoint" in e.types:
            self._nwc = NavigateToWaypoint(robot,
                                          waypoint_designator=robot_smach_states.util.designators.EdEntityDesignator(robot, id=e.id),
                                          radius=0.1)
            rospy.logwarn("ACTION_SERVER: Navigating to waypoint")
        else:
            self._nwc = NavigateToObserve(robot,
                                         entity_designator=robot_smach_states.util.designators.EdEntityDesignator(robot, id=e.id),
                                         radius=.5)
            rospy.logwarn("ACTION_SERVER: Navigating to observe")

        self._thread = threading.Thread(name='navigate', target=self._nwc.execute)
        self._thread.start()

    def _cancel(self):
        if self._nwc.is_running:
            self._nwc.request_preempt()

        # Wait until canceled
        self._thread.join()
