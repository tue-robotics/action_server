import threading
import time
import unittest

from actionlib.action_client import CommState
from actionlib_msgs.msg import GoalStatus

import action_server_msgs.msg
from action_server.client import Client, TaskOutcome


class _FakeGoalHandle(object):
    def __init__(self):
        self._comm_state = CommState.PENDING
        self._status = GoalStatus.PENDING
        self._result = None
        self.cancelled = False

    def get_comm_state(self):
        return self._comm_state

    def get_goal_status(self):
        return self._status

    def get_result(self):
        return self._result

    def cancel(self):
        self.cancelled = True


class _FakeActionClient(object):
    """
    Minimal stand-in for actionlib.ActionClient that lets a test drive goal
    transitions deterministically from a chosen thread.
    """
    def __init__(self):
        self._lock = threading.Lock()
        self._sent = []  # list of (goal, transition_cb, feedback_cb, gh)
        self._event = threading.Event()

    def send_goal(self, goal, transition_cb=None, feedback_cb=None):
        gh = _FakeGoalHandle()
        with self._lock:
            self._sent.append((goal, transition_cb, feedback_cb, gh))
        self._event.set()
        return gh

    def wait_for_sent(self, count, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if len(self._sent) >= count:
                    return True
            time.sleep(0.005)
        return False

    def sent_count(self):
        with self._lock:
            return len(self._sent)

    def recipe(self, index):
        with self._lock:
            return self._sent[index][0].recipe

    def finish(self, index, result_code, log_messages=None, status=GoalStatus.SUCCEEDED):
        with self._lock:
            _, transition_cb, _, gh = self._sent[index]
        result = action_server_msgs.msg.TaskResult()
        result.result = result_code
        result.log_messages = log_messages or []
        gh._result = result
        gh._status = status
        gh._comm_state = CommState.DONE
        transition_cb(gh)

    def goal_handle(self, index):
        with self._lock:
            return self._sent[index][3]


class ClientCoordinatorTest(unittest.TestCase):
    def setUp(self):
        self.fake = _FakeActionClient()
        self.client = Client("test_robot", action_client=self.fake, start_worker=True)

    def tearDown(self):
        self.client.close()

    def _collector(self):
        outcomes = []
        threads = []
        event = threading.Event()

        def cb(outcome, _outcomes=outcomes, _threads=threads, _event=event):
            _outcomes.append(outcome)
            _threads.append(threading.current_thread())
            _event.set()

        return outcomes, threads, event, cb

    def test_terminal_outcome_delivered_off_caller_thread(self):
        outcomes, threads, event, cb = self._collector()
        self.client.send_async_task("A", done_cb=cb)

        self.assertTrue(self.fake.wait_for_sent(1))
        finisher_thread = threading.current_thread()
        self.fake.finish(0, action_server_msgs.msg.TaskResult.RESULT_SUCCEEDED)

        self.assertTrue(event.wait(timeout=5.0))
        self.assertEqual(len(outcomes), 1)
        self.assertTrue(outcomes[0].succeeded)
        # The done_cb must run on the worker thread, never on the thread that
        # delivered the actionlib transition.
        self.assertIsNot(threads[0], finisher_thread)

    def test_fifo_order(self):
        order = []
        done = threading.Semaphore(0)

        def make_cb(tag):
            def cb(outcome, _tag=tag):
                order.append(_tag)
                done.release()
            return cb

        self.client.send_async_task("first", done_cb=make_cb("first"))
        self.client.send_async_task("second", done_cb=make_cb("second"))
        self.client.send_async_task("third", done_cb=make_cb("third"))

        # Only the first goal should be in flight; the others wait their turn.
        self.assertTrue(self.fake.wait_for_sent(1))
        time.sleep(0.1)
        self.assertEqual(self.fake.sent_count(), 1)

        self.fake.finish(0, action_server_msgs.msg.TaskResult.RESULT_SUCCEEDED)
        self.assertTrue(done.acquire(timeout=5.0))

        self.assertTrue(self.fake.wait_for_sent(2))
        self.fake.finish(1, action_server_msgs.msg.TaskResult.RESULT_SUCCEEDED)
        self.assertTrue(done.acquire(timeout=5.0))

        self.assertTrue(self.fake.wait_for_sent(3))
        self.fake.finish(2, action_server_msgs.msg.TaskResult.RESULT_SUCCEEDED)
        self.assertTrue(done.acquire(timeout=5.0))

        self.assertEqual(order, ["first", "second", "third"])

    def test_send_from_done_cb_runs_after_current(self):
        order = []
        done = threading.Semaphore(0)

        def second_cb(outcome):
            order.append("second")
            done.release()

        def first_cb(outcome):
            order.append("first")
            # Submitting a new task from within a done_cb is the exact pattern
            # that corrupts SimpleActionClient. It must be safe here.
            self.client.send_async_task("second", done_cb=second_cb)
            done.release()

        self.client.send_async_task("first", done_cb=first_cb)
        self.assertTrue(self.fake.wait_for_sent(1))
        self.fake.finish(0, action_server_msgs.msg.TaskResult.RESULT_SUCCEEDED)
        self.assertTrue(done.acquire(timeout=5.0))  # first

        self.assertTrue(self.fake.wait_for_sent(2))
        self.assertEqual(self.fake.recipe(1), "second")
        self.fake.finish(1, action_server_msgs.msg.TaskResult.RESULT_SUCCEEDED)
        self.assertTrue(done.acquire(timeout=5.0))  # second

        self.assertEqual(order, ["first", "second"])

    def test_cancel_finalizes_active_and_queued_without_dispatching(self):
        outcomes, threads, event, cb = self._collector()
        queued_outcome = {}
        queued_done = threading.Event()

        def queued_cb(outcome):
            queued_outcome["outcome"] = outcome
            queued_done.set()

        self.client.send_async_task("active", done_cb=cb)
        self.client.send_async_task("queued", done_cb=queued_cb)

        self.assertTrue(self.fake.wait_for_sent(1))
        self.client.cancel_all_async()

        # The active goal received a server-side cancel request.
        self.assertTrue(self.fake.goal_handle(0).cancelled)

        # Server confirms the terminal (preempted) state for the active goal.
        self.fake.finish(0, action_server_msgs.msg.TaskResult.RESULT_TASK_EXECUTION_FAILED,
                         status=GoalStatus.PREEMPTED)

        self.assertTrue(event.wait(timeout=5.0))
        self.assertFalse(outcomes[0].succeeded)

        # The queued task must be finalized (failed) and its done_cb delivered
        # exactly once, but it must never be dispatched to the server.
        self.assertTrue(queued_done.wait(timeout=5.0))
        self.assertFalse(queued_outcome["outcome"].succeeded)
        time.sleep(0.2)
        self.assertEqual(self.fake.sent_count(), 1)

    def test_sync_send_task_returns_outcome(self):
        result_holder = {}

        def run():
            result_holder["outcome"] = self.client.send_task("sync")

        t = threading.Thread(target=run)
        t.start()

        self.assertTrue(self.fake.wait_for_sent(1))
        self.fake.finish(0, action_server_msgs.msg.TaskResult.RESULT_SUCCEEDED)

        t.join(timeout=5.0)
        self.assertFalse(t.is_alive())
        self.assertIsInstance(result_holder["outcome"], TaskOutcome)
        self.assertTrue(result_holder["outcome"].succeeded)


if __name__ == "__main__":
    unittest.main()
