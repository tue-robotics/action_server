#! /usr/bin/python

import rospy
import actionlib
import task_manager.msg
        

class TaskManager(object):
    def __init__(self, robot):
        self._task_string = None
        self._robot = robot
        self._action_factory = ActionFactory()
        self._action_sequence = []

        # Set up actionlib interface for clients to give a task to the robot.
        self._action_name = robot.robot_name + "/task/"
        self._action_server = actionlib.SimpleActionServer(self._action_name, task_manager.msg.TaskAction, execute_cb=self._execute_cb, auto_start=False)
        self._feedback = task_manager.msg.TaskFeedback()
        self._result = task_manager.msg.TaskResult()
        self._action_server.start()

    def _execute_cb(self, task):
        recipe = self._parser.parse(task)
        if self.self._set_up_state_machine(recipe):
            self._execute_state_machine()
            self._action_server.set_succeeded()
        else:
            self._action_server.set_failed()

    def _set_up_state_machine(self, recipe):
        for action_name, config in recipe:
            Action = self._action_factory.get_action(action_name)
            action = Action()
            if action.configure(config)
                self._action_sequence.append(action)
            else:
                return False
        return True

    def _execute_state_machine(self):
        for action in self._action_sequence:
            if not action.execute():
                return False
        return True

    def add_action_cb(self, req):
        try:
            action_class = self._action_name_to_class[req.action]
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

if __name__ == "__main__":
    rospy.init_node('task_manager')

    # Create Robot object based on argv[1]
    if len(sys.argv) < 2:
        print "Please specify a robot name 'amigo/sergio'"
        sys.exit()

    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        print "unknown robot"
        sys.exit()

    robot = Robot()

    task_manager = TaskManager()

    # Register all actions that can be found in the actions module
    

    rospy.spin()


