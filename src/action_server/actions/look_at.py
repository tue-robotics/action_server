from action import Action
from util import entities_from_description

import robot_skills.util.kdl_conversions as kdl

import threading
import rospy


class LookAt(Action):

    def __init__(self):
        Action.__init__(self)
        self._robot = None
        self._entity = None

    def _configure(self, robot, config):
        self._robot = robot

        if "entity" not in config:
            self._config_result.missing_field = "entity"
            rospy.logwarn("Please specify an entity to look at")
            return

        (entities, error_msg) = entities_from_description(config["entity"], robot)
        if not entities:
            rospy.logwarn(error_msg)
            return

        self._entity = entities[0]

        self._config_result.succeeded = True

    def _start(self):
        self._thread = threading.Thread(name='head-goal', target=self._execute)
        self._thread.start()

        self._thread.join()

    def _execute(self):
        self._robot.head.cancel_goal()

        if self._entity:
            pos = self._entity._pose.p
            self._robot.head.look_at_point(kdl.VectorStamped(vector=pos, frame_id="/map"), timeout=10)

        self._execute_result.succeeded = True

    def _cancel(self):
        self._robot.head.cancel_goal()
