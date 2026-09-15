import threading
from collections import deque

import action_server_msgs.msg
import action_server_msgs.srv
import actionlib
import rospy
from actionlib.action_client import CommState
from actionlib_msgs.msg import GoalStatus


class TaskOutcome(object):
    RESULT_MISSING_INFORMATION = 0
    RESULT_TASK_EXECUTION_FAILED = 1
    RESULT_UNKNOWN = 2
    RESULT_SUCCEEDED = 3

    def __init__(self, result=RESULT_UNKNOWN, messages=None, missing_field=""):
        if not messages:
            messages = []
        self.result = result
        self.missing_field = missing_field
        self.messages = messages

    @property
    def succeeded(self):
        return self.result == self.RESULT_SUCCEEDED

    @succeeded.setter
    def succeeded(self, value):
        if value:
            self.result = self.RESULT_SUCCEEDED

    def __repr__(self):
        return "TaskOutcome(result={}, messages={}, missing_field='{}')".format(self.result,
                                                                                self.messages,
                                                                                self.missing_field)


def task_outcome_from_result(result):
    """
    Converts action_server_msgs.msg.TaskResult to TaskOutcome class

    :param result: (action_server_msgs.msg.TaskResult) result input
    :return: (TaskOutcome) result output
    """
    if result is None:
        return TaskOutcome(messages=["No result received from action server"])

    # Check result to return the correct outcome
    if result.result == action_server_msgs.msg.TaskResult.RESULT_MISSING_INFORMATION:

        to = TaskOutcome(TaskOutcome.RESULT_MISSING_INFORMATION,
                         result.log_messages)
        to.missing_field = result.missing_field
        return to

    elif result.result == action_server_msgs.msg.TaskResult.RESULT_TASK_EXECUTION_FAILED:
        return TaskOutcome(TaskOutcome.RESULT_TASK_EXECUTION_FAILED,
                           result.log_messages)

    elif result.result == action_server_msgs.msg.TaskResult.RESULT_UNKNOWN:
        return TaskOutcome(TaskOutcome.RESULT_UNKNOWN,
                           result.log_messages)

    elif result.result == action_server_msgs.msg.TaskResult.RESULT_SUCCEEDED:
        return TaskOutcome(TaskOutcome.RESULT_SUCCEEDED,
                           result.log_messages)

    return TaskOutcome(messages=result.log_messages)


class _Task(object):
    """
    Immutable-ish record for a single top-level task goal.

    Each task carries its own generation id and callbacks so that late
    transitions belonging to an older goal can never be mistaken for a newer
    one. All mutable lifecycle fields are only ever touched while holding the
    owning Client's condition variable.
    """
    def __init__(self, generation, semantics, done_cb=None, feedback_cb=None):
        self.generation = generation
        self.semantics = semantics
        self.done_cb = done_cb
        self.feedback_cb = feedback_cb

        self.gh = None
        self.cancel_requested = False
        self.terminal = False
        self.status = None
        self.result = None
        self.outcome = None
        self.done_event = threading.Event()


class Client(object):
    """
    A client for the action server.

    Wraps actionlib's ``ActionClient`` (not ``SimpleActionClient``) so that every
    task goal gets its own ``ClientGoalHandle`` and communication state machine.
    A single worker thread owns all goal submission: it sends one goal at a time,
    waits for its terminal state, and only then delivers the ``done_cb`` and picks
    up the next queued task.

    This design deliberately avoids the Noetic ``SimpleActionClient`` reentrancy
    race in which ``done_cb`` runs before ``simple_state`` becomes ``DONE``:
    here the terminal ``done_cb`` is always dispatched from the worker thread,
    never from an actionlib transport/transition callback, and never while another
    goal is being submitted. See https://github.com/ros/actionlib/issues/193.

    Note: ``feedback_cb`` is delivered from the actionlib transport thread (not the
    worker), so it must be cheap and must not submit or cancel goals.

    Example:
        client = Client('amigo')
        semantics = "{'actions': [{'action': 'say', 'sentence': 'ROBOT_NAME'}]}"
        client.send_task(semantics)
    """
    def __init__(self, robot_name, action_client=None, start_worker=True):
        self._action_name = "/" + robot_name + "/action_server/task"

        if action_client is None:
            self._action_client = actionlib.ActionClient(self._action_name,
                                                         action_server_msgs.msg.TaskAction)
            rospy.loginfo("Waiting for task action server to come online...")
            self._action_client.wait_for_server()
            rospy.loginfo("Connected to task action server")
        else:
            # Injected client (used by unit tests).
            self._action_client = action_client

        self._get_actions_proxy = None

        self._lock = threading.RLock()
        self._cv = threading.Condition(self._lock)
        self._queue = deque()
        self._active = None
        self._generation = 0
        self._closed = False

        self._worker = None
        if start_worker:
            self._worker = threading.Thread(target=self._worker_loop, name="action_client_worker")
            self._worker.daemon = True
            self._worker.start()
            try:
                rospy.on_shutdown(self.close)
            except Exception:
                pass

    def get_actions(self):
        """
        Query the available actions from the action server.

        :return: List of action names as registered with the action server, or an empty list if the service call fails.
        """
        if self._get_actions_proxy is None:
            self._get_actions_proxy = rospy.ServiceProxy('get_actions', action_server_msgs.srv.GetActions)
        try:
            res = self._get_actions_proxy()
        except rospy.ServiceException:
            rospy.logerr("Failed to get actions from the action server.")
            return []

        return res.actions

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #
    def send_async_task(self, semantics, done_cb=None, feedback_cb=None):
        """
        Queue a task and return immediately. Tasks run one at a time, in the order
        they were submitted (FIFO). A task is composed of one or multiple actions.

        :param semantics: A json string with a list of dicts, every dict in the list has at least an 'action' field,
            and depending on the type of action, several parameter fields may be required.
        :param done_cb: (callable) Called (from the worker thread) with a single TaskOutcome when the task reaches a
            terminal state.
        :param feedback_cb: (callable) Called whenever feedback for this goal is received. Takes one parameter: the
            feedback.
        :return: the generation id assigned to this task
        """
        with self._cv:
            self._generation += 1
            task = _Task(self._generation, semantics, done_cb, feedback_cb)
            self._queue.append(task)
            rospy.loginfo("Task queued (generation %d, %d pending)", task.generation, len(self._queue))
            self._cv.notify_all()
        return task.generation

    def send_task(self, semantics) -> TaskOutcome:
        """
        Queue a task and block until it reaches a terminal state.

        Must not be called from within a task callback (that would deadlock the
        worker); use :meth:`send_async_task` there instead.

        :param semantics: A json string with a list of dicts, every dict in the list has at least an 'action' field.
        :return: the TaskOutcome for this task
        """
        if self._worker is not None and threading.current_thread() is self._worker:
            raise RuntimeError("send_task() must not be called from a task callback; use send_async_task()")

        with self._cv:
            self._generation += 1
            task = _Task(self._generation, semantics)
            self._queue.append(task)
            self._cv.notify_all()

        try:
            while not task.done_event.wait(timeout=0.5):
                if self._closed or rospy.is_shutdown():
                    return TaskOutcome(messages=["Client shut down before task completed"])
        except KeyboardInterrupt:
            # Preserve the interactive-console contract: Ctrl+C cancels the task.
            self.cancel_all()
            raise

        return task.outcome if task.outcome is not None else TaskOutcome(messages=["No outcome produced"])

    def cancel_all(self):
        """
        Cancel the active task and all queued tasks, then block until they have
        all reached a terminal state.
        """
        rospy.logdebug("cancelling all goals...")
        self.cancel_all_async()
        with self._cv:
            while (self._active is not None or self._queue) and not self._closed and not rospy.is_shutdown():
                self._cv.wait(timeout=0.5)
        rospy.logdebug("... all goals cancelled!")

    def cancel_all_async(self):
        """
        Request cancellation of the active task and all queued tasks, then return
        immediately.

        Queued tasks are marked (not dropped) so the worker finalizes each with a
        terminal (failed) outcome and fires its done_cb exactly once; they are
        never dispatched to the server. The active task's done_cb fires once the
        server confirms the terminal state.
        """
        rospy.logdebug("cancelling all goals async...")
        gh_to_cancel = None
        with self._cv:
            for task in self._queue:
                task.cancel_requested = True
            active = self._active
            if active is not None:
                active.cancel_requested = True
                if active.gh is not None and not active.terminal:
                    gh_to_cancel = active.gh
            self._cv.notify_all()

        # Call cancel() OUTSIDE the lock. gh.cancel() takes the goal's
        # comm-state-machine mutex, while the actionlib transport thread takes
        # that mutex first and then our lock (in _transition_cb); cancelling
        # under our lock would create an AB-BA deadlock.
        if gh_to_cancel is not None:
            try:
                gh_to_cancel.cancel()
            except Exception as e:
                rospy.logerr("Failed to cancel active task: %s", e)

    def close(self):
        """Stop the worker thread. Intended for shutdown and tests."""
        with self._cv:
            self._closed = True
            self._cv.notify_all()

    # ------------------------------------------------------------------ #
    # Worker thread
    # ------------------------------------------------------------------ #
    def _worker_loop(self):
        while True:
            with self._cv:
                while not self._queue and not self._closed and not rospy.is_shutdown():
                    self._cv.wait(timeout=0.5)
                if self._closed or rospy.is_shutdown():
                    return
                task = self._queue.popleft()
                self._active = task

            self._execute(task)

            with self._cv:
                self._active = None
                self._cv.notify_all()

    def _execute(self, task):
        # Submit the goal while holding the lock, so task.gh is assigned before
        # any transition/feedback callback (which also takes the lock) can run.
        with self._cv:
            if task.cancel_requested:
                self._finalize(task, GoalStatus.PREEMPTED, None,
                               message="Task cancelled before it started")
            else:
                goal = action_server_msgs.msg.TaskGoal(recipe=task.semantics)
                task.gh = self._action_client.send_goal(
                    goal,
                    transition_cb=lambda gh, t=task: self._transition_cb(gh, t),
                    feedback_cb=lambda gh, fb, t=task: self._feedback_cb(gh, fb, t))
                rospy.loginfo("Task sent (generation %d)", task.generation)

        with self._cv:
            while not task.terminal and not self._closed and not rospy.is_shutdown():
                self._cv.wait(timeout=0.5)
            if not task.terminal:
                task.outcome = TaskOutcome(messages=["Client shut down before task completed"])
                task.done_event.set()
                return
            outcome = task.outcome

        # Deliver the outcome OUTSIDE the lock and off the actionlib callback
        # thread. This is the whole point of the design: it is safe to send new
        # goals (via send_async_task) or block from here.
        if task.done_cb is not None:
            try:
                task.done_cb(outcome)
            except Exception as e:
                rospy.logerr("Exception in task done_cb: %s", e)
        task.done_event.set()

    def _finalize(self, task, status, result, message=None):
        """Record a task's terminal state. Caller must hold the lock."""
        task.status = status
        task.result = result
        if result is not None:
            task.outcome = task_outcome_from_result(result)
        else:
            outcome = TaskOutcome(messages=[message] if message else [])
            if status in (GoalStatus.PREEMPTED, GoalStatus.ABORTED, GoalStatus.RECALLED,
                          GoalStatus.REJECTED, GoalStatus.LOST):
                outcome.result = TaskOutcome.RESULT_TASK_EXECUTION_FAILED
            task.outcome = outcome
        task.terminal = True
        self._cv.notify_all()

    def _transition_cb(self, gh, task):
        with self._cv:
            if task.terminal or gh != task.gh:
                return
            if gh.get_comm_state() != CommState.DONE:
                return
            self._finalize(task, gh.get_goal_status(), gh.get_result())

    def _feedback_cb(self, gh, feedback, task):
        with self._cv:
            if task.terminal or gh != task.gh:
                return
            feedback_cb = task.feedback_cb
        if feedback_cb is not None:
            try:
                feedback_cb(feedback)
            except Exception as e:
                rospy.logerr("Exception in task feedback_cb: %s", e)
