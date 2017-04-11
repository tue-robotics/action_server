from action import Action

from util import entities_from_description

import robot_smach_states

from robot_smach_states.util.designators import UnoccupiedArmDesignator, EdEntityDesignator
import rospy
import threading


class PickUp(Action):

    def __init__(self):
        Action.__init__(self)

    def _configure(self, robot, config):
        self._robot = robot

        if not "entity" in config:
            rospy.logwarn("No entity given")
            return

        entity_descr = {"id": config["entity"]}
        (entities, error_msg) = entities_from_description(entity_descr, robot)
        if not entities:
            rospy.logwarn(error_msg)
            return

        # Only filter to entities that can be picked up, e.g not furniture etc
        entities = [e for e in entities if not e.is_a("furniture")]

        if not entities:
            rospy.logwarn("Impossible to grab that object")
            return

        self._entity = entities[0]

        self._side = config['side'] if 'side' in config else 'right'

        self._fsm = robot_smach_states.grab.Grab(self._robot,
                                                 item=EdEntityDesignator(self._robot, id=self._entity.id),
                                                 arm=UnoccupiedArmDesignator(self._robot.arms,
                                                                             self._robot.arms[self._side]))

        self._config_result.succeeded = True

    def _start(self):
        self._thread = threading.Thread(name='pick-up', target=self._fsm.execute)
        self._thread.start()

        # TODO: implement possibility to cancel action when started, but still block while executing
        self._thread.join()
        self._execute_result.succeeded = True
        self._execute_result.message = "picked up the " + self._entity.id

    def _cancel(self):
        if self._fsm.is_running:
            self._fsm.request_preempt()
