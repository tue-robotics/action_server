#! /usr/bin/python

import rospy
import inspect, re
import actions

def class_name_camelcase_to_dashes(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1-\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1-\2', s1).lower()

class ActionFactory(object):

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
        names = [name for name, action in self._action_name_to_class]
        return names

    def get_action(self, action_name):
        return self._action_name_to_class[action_name]
