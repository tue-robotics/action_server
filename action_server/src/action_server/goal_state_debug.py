import json

import rospy
from std_msgs.msg import String

_DEBUG_PARAM = "/action_server/debug_goal_state"
_publishers = {}


def is_enabled():
    return rospy.get_param(_DEBUG_PARAM, False)


def _get_publisher(topic):
    if topic not in _publishers:
        _publishers[topic] = rospy.Publisher(topic, String, queue_size=10, latch=True)
    return _publishers[topic]


def log_server_state(action_server, task_manager, event, extra=None, publish_topic=None):
    if not is_enabled():
        return

    payload = {
        "role": "server",
        "event": event,
        "server_active": action_server.is_active(),
        "preempt_requested": action_server.is_preempt_requested(),
        "task_manager_done": task_manager.done,
        "next_subtask": task_manager.get_next_action_name(),
    }
    if extra:
        payload.update(extra)

    rospy.loginfo("[goal_debug][server] %s", json.dumps(payload, sort_keys=True))

    if publish_topic:
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        _get_publisher(publish_topic).publish(msg)
