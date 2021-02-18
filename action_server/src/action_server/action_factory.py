import rospy
import inspect
import re
from . import actions


def class_name_camelcase_to_dashes(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1-\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1-\2', s1).lower()


class ActionFactory(object):
    """
    The action factory creates Action instances based on an action name.

    The ./actions/ directory is inspected for child classes of Action. These
    classes are registered in the _action_name_to_class dict. The user can
    get the registered action names by calling get_action_names() and an
    instance of an action can be created by calling get_action(<action_name>).
    """

    def __init__(self):
        self._action_name_to_class = {}

        for name, obj in inspect.getmembers(actions):
            if inspect.isclass(obj):
                action_name = class_name_camelcase_to_dashes(name)
                self._register_skill(action_name, obj)

    def _register_skill(self, action_type, skill):
        rospy.loginfo("Registering skill '%s' (%s)" % (action_type, skill))
        self._action_name_to_class[action_type] = skill

    def get_action_names(self):
        """
        Get a list of the action names of the registered actions
        """
        names = self._action_name_to_class.keys()
        return names

    def get_action(self, action_name):
        """
        Get the Action class for the requested action_name
        """
        return self._action_name_to_class[action_name]
