from action import Action
from util import entities_from_description

import robot_skills.util.kdl_conversions as kdl

import threading

class LookAt(Action):

    def __init__(self):
        self._robot = None
        self._entity = None

    def _start(self, config, robot):
        self._robot = robot

        if "entity" not in config:
            return "Please specify an entity to look at"

        (entities, error_msg) = entities_from_description(config["entity"], robot)
        if not entities:
            return error_msg

        self._entity = entities[0]

        self._thread = threading.Thread(name='arm-goal', target=self._execute)
        self._thread.start()

    def _execute(self):
        self._robot.head.cancel_goal()

        if self._entity:
            pos = self._entity._pose
            self._robot.head.look_at_point(kdl.VectorStamped(vector=pos, frame_id="/map"), timeout=10)

    def _cancel(self):
        self._robot.head.cancel_goal()
