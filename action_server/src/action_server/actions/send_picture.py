from action import Action, ConfigurationData
import rospy
import robot_smach_states as states
from sensor_msgs.msg import Image

from entity_description import resolve_entity_description
from robot_smach_states.util.designators import EdEntityDesignator

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
            self.location_designator = None

    @staticmethod
    def _parse_context(context_dict):
        context = SendPicture.Context()

        if 'location-designator' in context_dict:
            context.location_designator = context_dict['location-designator']

        return context

    def _configure(self, robot, config):
        self._robot = robot

        self.semantics = self._parse_semantics(config.semantics)
        self.context = self._parse_context(config.context)

        if self.context.location_designator is None:
            # Request navigation action
            self._config_result.required_context = {'action': 'navigate-to'}
            if self.semantics.target_location is not None:
                self._config_result.required_context['target-location'] = config.semantics['target-location']
            return
        # We can now assume we arrived at the place to take the picture from.

        # self.detect_face_state_machine = states.DetectFace(self._robot)

        entity_to_inspect = resolve_entity_description(config.semantics['target-location'])
	entity_designator = EdEntityDesignator(self._robot, id=entity_to_inspect.id)
	entity = entity_designator.resolve()
        self.look_at_sm = states.LookOnTopOfEntity(self._robot, entity)

        self._config_result.succeeded = True
        return

    def _start(self):
        self._robot.speech.speak("I will take a picture and send it to my operator now. ")

        image_pub = rospy.Publisher("/hero/image_from_ros", Image, queue_size=1)
        self.look_at_sm.run()

        image_pub.publish(self._robot.perception.get_image())
        self._robot.head.close()

        # if result == 'failed':
        #     self._execute_result.message = " I didn't see anyone. "
        #     self._execute_result.succeeded = False
        # else:
        # self._execute_result.message = " I found someone at the door. "
        self._execute_result.succeeded = True

    def _cancel(self):
	pass

if __name__ == "__main__":
    rospy.init_node('send_picture_test')

    import sys
    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        from robot_skills.mockbot import Mockbot as Robot

    robot = Robot()

    action = SendPicture()

    config = ConfigurationData({'action': 'send-picture',
                                'target-location': {'id': 'cabinet'}})

    action.configure(robot, config)
    action.start()
