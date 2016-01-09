from action import Action
from util import entities_from_description

import robot_smach_states
from robot_smach_states.navigation import NavigateToObserve, NavigateToWaypoint, NavigateToGrasp, NavigateToSymbolic
import robot_skills.util.msg_constructors as msgs

import threading, time, rospy

class Bring(Action):

    def __init__(self):
        self._fsm = None
        self._thread = None

        self._from_entity = None
        self._to_entity = None
        self._entity_description = None

    def _start(self, config, robot):
        self._robot = robot

        if "from" not in config or "to" not in config:
            return "Please specify a from and to entity"

        # Get 'from' location
        (entities, error_msg) = entities_from_description(config["from"], robot)
        if not entities:
            return error_msg
        self._from_entity = entities[0]

        # Get 'to' location
        (entities, error_msg) = entities_from_description(config["to"], robot)
        if not entities:
            return error_msg
        self._to_entity = entities[0]

        if "to_radius" in config:
            self.to_radius = config["to_radius"]
        else:
            self.to_radius = 0.7

        if "from_room" in config:
            self.from_room = config["from_room"]
        else:
            self.from_room = None

        # Get type of entity that should be transported
        if "entity" not in config:
            return "Please specify the entity argument (dictionary)"

        self._entity_description = config["entity"]

        self._thread = threading.Thread(name='bring', target=self._execute)
        self._thread.start()

    def _execute(self):

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # Navigate to the room

        if self.from_room:
            self._fsm = NavigateToSymbolic(
                self._robot,
                 { robot_smach_states.util.designators.EdEntityDesignator(robot, id=self.from_room) : "in" },
                 robot_smach_states.util.designators.EdEntityDesignator(self._robot, id = self._from_entity.id))
            self._fsm.execute()

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # Inspect to find the entities

        found_entities_des = robot_smach_states.util.designators.VariableDesignator([])

        self._fsm = robot_smach_states.world_model.Inspect(
            self._robot,
            entityDes = robot_smach_states.util.designators.EdEntityDesignator(self._robot, id = self._from_entity.id),
            objectIDsDes = found_entities_des)
        self._fsm.execute()

        found_ids = [e.id for e in found_entities_des.resolve()]

        (entities, error_msg) = entities_from_description(self._entity_description, self._robot)

        # Only filter to entities that where detected
        entities = [e for e in entities if e.id in found_ids]

        if not entities:
            rospy.logerr("I could not find the object you are looking for")
            return

        self._fsm = robot_smach_states.grab.SjoerdsGrab(
                        self._robot,
                        item_des = robot_smach_states.util.designators.EdEntityDesignator(self._robot, id = entities[0].id),
                        arm_des = robot_smach_states.util.designators.UnoccupiedArmDesignator(self._robot.arms, self._robot.rightArm))
        self._fsm.execute()

        self._fsm = NavigateToObserve(
            self._robot,
            robot_smach_states.util.designators.EdEntityDesignator(self._robot, id = self._to_entity.id),
            radius = self.to_radius)
        self._fsm.execute()

        # - - - - - - - - - - - - - - - - - - - -
        # Make sure the head looks at the entity

        pos = self._to_entity.pose.position
        self._robot.head.look_at_point(msgs.PointStamped(pos.x, pos.y, pos.z, "/map"), timeout=0)

        # - - - - - - - - - - - - - - - - - - - -
        # Hand over

        arm = self._robot.rightArm

        arm._send_joint_trajectory([[-0.1, 0.4, 0, 1.8, 0, -0.4, 0],[-0.1, 0.8, 0, 1, 0, -0.2, 0]])
        self._robot.speech.speak("Here you go!")
        arm.send_gripper_goal('open', timeout=5)

        time.sleep(1)

        # - - - - - - - - - - - - - - - - - - - -
        # Reset arm

        arm.send_gripper_goal('close', timeout=0)
        arm._send_joint_trajectory([[-0.1, 0.4, 0, 1.8, 0, -0.4, 0]])
        arm.reset()

        # Cancel the head goal
        self._robot.head.cancel_goal()

    def _cancel(self):
        if self._fsm:
            self._fsm.request_preempt()

        # Wait until canceled
        self._thread.join()
