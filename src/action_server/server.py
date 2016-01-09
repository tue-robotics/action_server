import rospy, yaml, uuid

import action_server.srv

class Server(object):

    def __init__(self, name, robot):
        self._name = name
        self._action_type_to_skill = {}
        self._action = None
        self._robot = robot
        self._cl_register = None

        self._add_action_service_name = self._name + '/add_action'
        self._get_action_status_service_name = self._name + '/get_action_status'
        self._srv_add_action = rospy.Service(self._add_action_service_name,
                                            action_server.srv.AddAction,
                                            self.add_action_cb)

    def register_skill(self, action_type, skill):
        rospy.loginfo("Registering skill '%s' (%s)" % (action_type, skill))
        self._action_type_to_skill[action_type] = skill

    def connect(self, register_service_name):
        rospy.loginfo("Waiting for registration service ('%s') ..." % register_service_name)
        rospy.wait_for_service(register_service_name)
        rospy.loginfo("...found")

        self._cl_register = rospy.ServiceProxy(register_service_name,
                                              action_server.srv.RegisterActionServer)

        try:
            resp = self._cl_register(self._add_action_service_name,
                                    self._get_action_status_service_name,
                                    [k for k in self._action_type_to_skill.iterkeys()])
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return False

        if resp.error_msg:
            rospy.logerr("Response from action server: " + resp.error_msg)
            return False

        return True

    def add_action_cb(self, req):
        try:
            action_class = self._action_type_to_skill[req.action]
        except KeyError:
            return action_server.srv.AddActionResponse("", "Action type '%s' not found." % req.action)

        config = yaml.load(req.parameters)

        if self._action:
            self._action.cancel()

        action = action_class()

        try:
            err = action.start(config, self._robot)
            if not err:
                self._action = action
                id = str(uuid.uuid1())
                return action_server.srv.AddActionResponse(id, "")
            else:
                return action_server.srv.AddActionResponse("", "Could not construct action: %s" % err)
        except Exception as err:
            error_msg = "%s\n\n" % err
            import traceback
            error_msg += traceback.format_exc()
            return action_server.srv.AddActionResponse("", "Error while starting action: %s" % error_msg)
