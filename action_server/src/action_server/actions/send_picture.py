import rospy
from sensor_msgs.msg import Image

import robot_smach_states as states
from robot_smach_states.util.designators import EdEntityDesignator

from .action import Action, ConfigurationData
from .entity_description import resolve_entity_description


class SendPicture(Action):
    """ The Inspect class implements the action to send a picture over a whatsapp client.

    Parameters to pass to the configure() method are:
     - `target-location` (required): an entity with a segmentation area to inspect
    """

    def __init__(self):
        Action.__init__(self)

    class Semantics:
        def __init__(self):
            self.target_location = None

    @staticmethod
    def _parse_semantics(semantics_dict):
        semantics = SendPicture.Semantics()
        semantics.target_location = resolve_entity_description(semantics_dict['target-location'])

        return semantics

    class Context:
        def __init__(self):
            self.location = None

    @staticmethod
    def _parse_context(context_dict):
        context = SendPicture.Context()

        if 'location' in context_dict:
            context.location = resolve_entity_description(context_dict['location'])

        return context

    def _configure(self, robot, config):
        self._robot = robot

        self.semantics = self._parse_semantics(config.semantics)
        self.context = self._parse_context(config.context)

        if self.context.location is None:
            # Request navigation action
            self._config_result.required_context = {'action': 'navigate-to'}
            if self.semantics.target_location is not None:
                self._config_result.required_context['target-location'] = config.semantics['target-location']
            return
        # We can now assume we arrived at the place to take the picture from.

        # self.detect_face_state_machine = states.DetectFace(self._robot)

        entity_to_inspect = resolve_entity_description(config.semantics['target-location'])
        entity = EdEntityDesignator(self._robot, id=entity_to_inspect.id).resolve()
        self.look_at_sm = states.LookOnTopOfEntity(self._robot, entity)

        self._config_result.succeeded = True
        return

    def _start(self):
        self._robot.speech.speak("I will take a picture and send it to my operator now. ")

        image_pub = rospy.Publisher("/hero/image_from_ros", Image, queue_size=1)
        self.look_at_sm.run()
        rospy.sleep(1)
        image_pub.publish(self._robot.perception.get_image())
        self._robot.head.close()
        result = self.detect_face_state_machine.execute(self._robot)
        self._execute_result.succeeded = True

    def _cancel(self):
        return


if __name__ == "__main__":
    rospy.init_node('send_picture_test')

    from robot_skills import get_robot_from_argv

    robot = get_robot_from_argv(1)

    action = SendPicture()

    config = ConfigurationData({'action': 'send-picture',
                                'target-location': {'id': 'cabinet'}})

    action.configure(robot, config)
    action.start()
