import threading
import voluptuous
import rospy
import robot_skills.util.kdl_conversions as kdl
from action import Action, ConfigurationData
# from util import entities_from_description
from .util.entities_from_description import entities_from_description, EntitySchema

REQUIRED_FIELD_PROMPTS = voluptuous.Schema({
    voluptuous.Required(
        "entity",
        description="What would you like me to look at?"
    ): EntitySchema,
})


class LookAt(Action):
    def __init__(self):
        """
        The LookAt class implements the action to look at a world model entity.

        Parameters to pass to the configure() method are:
         - `entity` (optional): the entity id to look at.
        """
        # required_field_prompts = {'entity': " What would you like me to look at? "}

        # Idea 1: required field prompts becomes a list of NamedTuples(key, schema, question)
        # required_field_prompts = [
        #     RequiredField('entity', EntitySchema, "What would you like me to look at?")
        # ]

        # Idea 2: required field prompts becomes a schema with a specific check method
        # required_field_prompts = voluptuous.Schema({
        #     voluptuous.Required(
        #         "entity",
        #         description="What would you like me to look at?"
        #     ): EntitySchema,
        # })
        required_skills = ['head']
        super(LookAt, self).__init__(
            required_field_prompts=REQUIRED_FIELD_PROMPTS,
            required_passed_knowledge={},
            required_skills=required_skills,
        )

    def _configure(self, robot, config):
        self._robot = robot

        (entities, error_msg) = entities_from_description(config.semantics["entity"], robot)
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

if __name__ == "__main__":
    rospy.init_node('look_at_test')

    import sys
    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = LookAt()

    config = ConfigurationData({'action': 'look_at',
              'entity': {'id': 'cabinet'}})

    action.configure(robot, config)
    action.start()
