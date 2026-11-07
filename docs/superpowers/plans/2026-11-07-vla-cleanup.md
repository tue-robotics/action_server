# VLA Cleanup and Fix Plan (v2 → v2.1)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or
> superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the VLA pick-up correct in general, not tuned to the sim. Reuse what `robot_skills` /
`robot_smach_states` already provide. Remove the sim patches and dead code, and fix the open TODOs that can be
solved in code.

**Architecture:** The layering stays as it is (`PickUp` → `executor` provider → backend → observation
source / action sink). The two backend classes become one: the in-process class cannot run on Python 3.8. The
sink keeps the official AIRoA contract and loses its sim corrections. Completion uses the existing
`ActiveGraspDetector`. Cancel reaches the approach and the rollout.

**Tech Stack:** ROS Noetic, Python 3.8, rospy, `urdf_parser_py`, `robot_smach_states`, nose/pytest with mocks (no
roscore).

**Spec:** the TODO lists of [VLA_INTEGRATION_REPORT_v2.md](../../../VLA_INTEGRATION_REPORT_v2.md) and
[VLA_INFERENCE_REPORT_v2.md](../../../VLA_INFERENCE_REPORT_v2.md), plus the owner's review questions 1–7
(answered below).

## Global Constraints

- Defaults must command exactly what the official AIRoA client commands (golden test, reference-equivalent
  params).
- Reuse `robot_skills` / `robot_smach_states` **as on master**. Don't depend on the unmerged `tue_robocup`
  `feat/vla-grasp` branch.
- Nothing robot-specific hardcoded in code that the robot model or the robot parameters already have.
- No new params unless they replace a removed one. Every removed param also leaves `hero_bringup` `vla.yaml`
  and the golden test's `DEPLOYED` mirror.
- Tests run without a roscore, GPU or checkpoint, under both pytest and nose (`catkin_add_nosetests`).
- No commits or pushes: changes stay in the working trees for review. Commit only on request.

## Answers to the review questions

1. **`ARM_LOWER/ARM_UPPER`.** They are the `hsrb4s.urdf` position limits, copied by hand. So is
   `max_arm_delta`: URDF velocity limit × 0.2 s. They are correct for the HSR, but they must not be in the code.
   The robot model is on the parameter server (`/<robot>/robot_description`). → Task 1 reads both from the
   URDF.
2. **The `_apply_*` methods.**
   - **Arm stream** (one target per 0.1 s on the controller topic) and **gripper position**: `robot_skills`
     on master has no API for either. `Arm._send_joint_trajectory` blocks and is timed by a maximum
     velocity; `Gripper.send_goal` is binary open/close. These two stay; they are the policy's output.
   - **Effort grasp**: already reuses `gripper.send_goal("close", …)`.
   - **Head**: already reuses `robot.head` (its only pan/tilt entry point is the private
     `_setHeadReferenceGoal`).
   - **Base**: publishes on the same topic as `robot.base._cmd_vel`. The non-blocking `Base.send_velocity/stop`
     exists only on `feat/vla-grasp`.
   - Proper fix, upstream: merge `Base.send_velocity/stop` and a public head pan/tilt call into `robot_skills`
     master, then use them here. Not done here (other repo, other PR).
   - Head and base stay off by default. Task 4 removes the `pose_delta` base mode (question (a) is settled:
     velocity).
3. **`_release_policy` is empty.** It was a hook of the in-process `SmolVLALocalBackend`, which the WebSocket
   subclass overrides to close the socket. The in-process path imports LeRobot, which needs Python ≥ 3.10, so
   it cannot run in the Noetic Python 3.8 process. → Task 4 removes it. One backend class remains, and the
   hook goes with it.
4. **`# -- items 5-6-8 …` and the other `items` headers.** They are stale numbering from the v1 design notes and
   mean nothing now. → removed in Task 4. The methods under them (start pose, lift, retract, rollout) are
   needed.
5. **`gripper_holding_object` and friends.**
   - `robot_smach_states.manipulation.ActiveGraspDetector` already does this for HERO with
     `arm._arm.gripper_position_detector`: squeeze the gripper, compare the finger positions. It is used by
     `challenge_serve_breakfast`.
   - `gripper_occupied` / `is_manipulation_done` read `occupied_by`, which is always `None` during a VLA
     pick-up (`UnoccupiedArmDesignator`, and `send_goal("open")` clears it). So that signal is dead.
   - → Task 3 replaces `gripper_holding_object`, `gripper_occupied`, `is_manipulation_done`,
     `get_gripper_position` and `done_confirm_chunks` with `ActiveGraspDetector`.
6. **The gripper rules.** The classic `Grab` only opens and closes. The VLA must follow the policy's continuous
   finger column (pre-shaping), which is why there is a gripper rule at all. The official rule is one line:
   column < 0.5 → effort grasp, else → position. On top of that, v2 added two sim patches:
   - the **latch** (`gripper_open_threshold`: hold a grasp until 0.9);
   - the **empty-grasp release** (a patch for the latch).

   → Task 2 keeps only the official rule.
7. **Things that aren't generic.** Agreed. The patches tuned to the sim are `arm_delta_gain`,
   `arm_hold_threshold`, the gripper latch and empty release, `grasp_position_threshold` + `done_confirm_chunks`,
   and the hardcoded HSR limits. Tasks 1–3 remove them. What each one compensated for, and where the real fix
   is, is under "Patches removed" below.

## Review Focus

1. `robot_description` missing, or without one of the arm joints → the sink still builds; the arm is
   unbounded, with one warning. *(Task 1 test `test_without_robot_description_the_arm_is_unbounded`)*
2. Cancel arrives during the approach (before the VLA starts) → no classic pick-up runs afterwards.
   *(Task 5 test `test_cancel_during_the_approach_skips_the_classic_pick_up`)*
3. The grasp check returns `'failed'` or `'cannot_determine'` → not holding: the gripper opens and the rollout
   goes on. *(Task 3 test `test_failed_grasp_check_opens_the_gripper_and_continues`)*
4. A failed check must not re-trigger a lift every chunk while the policy keeps the hand closed: only a new
   close is checked again. *(Task 3, same test: the second chunk does not check again)*
5. A cancel during the rollout must not run the retract. Stop means stop. *(Task 5 test
   `test_cancel_stops_the_rollout_without_retract`)*

---

### Task 1: Arm bounds from the robot model; sim arm corrections removed (Q1, Q7)

**Files:**
- Modify: `action_server/src/action_server/vla/backend_utils.py` (add `joint_limits`, remove `shape_arm_delta`)
- Modify: `action_server/src/action_server/vla/backends.py` (`HSRActionSink.__init__`, `_send_arm`, remove
  `ARM_LOWER/UPPER`, `_arm_param`, `_arm_target`, `reset`)
- Modify: `action_server/package.xml` (`exec_depend urdfdom_py`)
- Test: `test/test_vla_backend_utils.py`, `test/test_vla_action_sink.py`, `test/test_vla_airoa_golden.py`
- Config: `hero_bringup/parameters/action_server/vla.yaml` (drop `max_arm_delta`, `arm_delta_gain`,
  `arm_hold_threshold`)

**Interfaces:**
- Produces: `joint_limits(urdf_xml: str, joint_names: List[str]) -> Optional[Tuple[np.ndarray, np.ndarray,
  np.ndarray]]`, i.e. (lower, upper, velocity), or `None` if the XML is empty or a joint has no limit.
- Produces: `HSRActionSink.arm_lower`, `HSRActionSink.arm_upper` (`np.ndarray` or `None`). Task 4's `_lift_arm`
  uses them.
- Per-row arm delta cap = `velocity × arm_lead_time` (the same values as the old `max_arm_delta`).

- [x] Test `test_joint_limits_are_read_from_the_urdf`: a 2-joint URDF string → `lower`, `upper`, `velocity`
  arrays in the requested order. `test_missing_joint_gives_no_limits` → `None`.
- [x] Sink tests: `test_arm_target_respects_joint_limits` and `test_arm_delta_is_bounded_by_the_velocity_limit`
  use a URDF fixture passed as `/hero/robot_description`. Add `test_without_robot_description_the_arm_is_unbounded`.
  Delete the gain, hold, reset and wrong-length tests.
- [x] Golden: reference-equivalent = no description (no clipping), with no class-attribute patching. Deployed =
  HSR URDF fixture; ticks 1–3 still deviate only as documented, without the gain.
- [x] Run `PYTHONPATH=src:$PYTHONPATH python3 -m pytest -q -p no:warnings test/test_vla_*.py`. Expected: all
  pass.

### Task 2: Gripper = the official hybrid rule (Q6, Q7)

**Files:** `backend_utils.py` (`gripper_action`), `backends.py` (`_apply_gripper`, `play_chunk`), tests,
`vla.yaml` (drop `gripper_open_threshold`).

**Interfaces:** `gripper_action(value: float, close_threshold: float) -> Optional[float]`: `None` = effort grasp,
else the position clipped to [−0.1, 1.23]. `_apply_gripper(value)` no longer takes the measured position.

- [x] Utils tests: open → position, clip, close → `None`. Delete the latch test.
- [x] Sink: delete the latch, empty-release and release-band tests. Add
  `test_grasp_reopens_when_the_column_rises_above_the_threshold` (rows 0.3, 0.6 → one grasp, then a position of
  0.6).
- [x] Golden: drop `gripper_open_threshold` from `REFERENCE_EQUIVALENT`. In the deployed test, the 0.6 tick now
  equals the reference.
- [x] Run the tests. Expected: pass.

### Task 3: Completion with the existing grasp check (Q5, TODO 3, P2)

**Files:** `backends.py` (backend `_holding_object`, `_rollout`), `backend_utils.py` (remove
`gripper_holding_object`, `gripper_occupied`, `is_manipulation_done`), tests, `vla.yaml`
(`grasp_position_threshold` → `grasp_minimum_position`).

**Interfaces:**
- Consumes: `robot_smach_states.manipulation.active_grasp_detector.ActiveGraspDetector(robot, arm_designator,
  minimum_position=…)`, whose `.execute()` returns `'true' | 'false' | 'failed' | 'cannot_determine'`, and
  `robot_smach_states.util.designators.Designator`.
- Produces: `_holding_object() -> bool`.
  1. Only after an effort grasp older than `grasp_settle_time`.
  2. Then `_lift_arm()`: fingers stalled on the table edge must not read as an object (the P2 false positives).
  3. Then the detector. `'true'` → success. Anything else → `_open_gripper()` and go on.
- Param `grasp_minimum_position`. Unset = the detector's own default −0.82, the real HSR. `vla.yaml` sets the sim
  value 0.3 (P2's proposal). **Sim-only**: see "Patches kept".
- Removed: `done_confirm_chunks`, `grasp_position_threshold`, `HSRObservationSource.get_gripper_position`.

- [x] `test_settled_grasp_is_lifted_then_checked`: close held 2 s, detector `'true'` → succeeded. Call order
  is lift, then detector.
- [x] `test_unsettled_grasp_is_not_checked`: 0.5 s → detector not built.
- [x] `test_failed_grasp_check_opens_the_gripper_and_continues`: `'cannot_determine'` → open called. The rollout
  ends at `max_chunks`, and there is no second check while the gripper is `'open'`.
- [x] `test_grasp_minimum_position_is_passed_to_the_detector`.
- [x] Retract tests keep passing ("no retract after a grasp" uses the detector).

### Task 4: One backend class; stale comments; base velocity only (Q2, Q3, Q4)

**Files:** `backends.py`, `test/test_vla_action_sink.py`, `vla.yaml` (drop `device`, `action_layout`,
`state_indices`, `checkpoint_path`, `base_action_mode`), `hero_bringup/launch/action_server*.launch` (drop the
`checkpoint_path` / `vla_repo_path` plumbing of the in-process path).

**Interfaces:**
- `SmolVLAWebSocketBackend(BaseBackend)` holds the rollout. `local_backend_class` keeps working unchanged.
  `SmolVLALocalBackend` is removed.
- Methods: `_connect()`, `_infer(obs)`, `_close_policy_socket()`, `cancel()` (Task 5).
- `HSRActionSink.get_arm()` becomes public (was `_get_arm`, already used by the backend).
- Base: `enable_base_motion` only; the rows are velocities.

- [x] `make_backend` builds `SmolVLAWebSocketBackend`. Retract tests record `_connect` / `_close_policy_socket`.
- [x] The base clamp/brake and 6-wide tests pass unchanged.

### Task 5: Cancel reaches the approach and the rollout (TODO 4)

**Files:** `executor.py`, `vla/__init__.py`, `backends.py`, `actions/pick_up.py`, tests (new
`test/test_vla_executor.py`).

**Interfaces:**
- `executor.cancel_vla(robot_name: str) -> None` calls `cancel()` on the cached providers of that robot.
  `BaseVLAProvider.cancel()` is a no-op. `LocalBackendVLAProvider.cancel()` forwards to the backend.
- Backend `cancel()` sets a `threading.Event` that is shared with the sink. `play_chunk` returns `False` at the
  next row, and the rollout stops before the next query. No retract after a cancel.
- `PickUp._cancel()`:
  1. sets `self._cancelled`;
  2. preempts the stored `NavigateToGrasp`;
  3. calls `cancel_vla`;
  4. preempts the classic FSM only `if self._fsm.is_running()` (the bound-method bug).

  `_start` doesn't start the classic FSM once cancelled. smach doesn't honour a preempt requested before
  `execute()`, hence the flag.

- [x] `test_cancel_during_the_approach_skips_the_classic_pick_up`, `test_cancel_during_the_vla_skips_the_classic_pick_up`.
- [x] `test_cancel_stops_the_rollout_without_retract`; sink `test_stop_ends_the_chunk_before_the_next_row`.
- [x] `test_cancel_vla_reaches_the_backend_of_that_robot_only`.

### Task 6: Execution modes without the delegation shortcut; place/hand-over hooks removed (TODO 5, 6)

**Files:** `executor.py`, `vla/__init__.py`, `task_manager.py` (back to master), `actions/place.py` and
`actions/hand_over.py` (back to master), `test/test_vla_executor.py`.

**Interfaces:**
- `maybe_run_vla_manipulation(robot, action_name, semantics, context, raw_sentence="") -> VLAOutcome` is the
  single entry point. `maybe_run_vla_action` and `should_delegate_action_to_vla` are removed.
- `execution_mode: vla` = the hybrid flow (Find, `NavigateToGrasp`, VLA) with no classic fallback, i.e. hybrid +
  `strict_mode`. Before this, it skipped Find and the approach (C11).
- The place/hand-over hooks ran before the navigation to their target, so a VLA there acts at the wrong place.
  With no place/hand-over checkpoint and no phase boundary, they go. Re-add them with the phase boundary.

- [x] `test_vla_mode_never_falls_back_to_classic`, `test_hybrid_falls_back_without_strict_mode`.
- [x] `test_import.py` and the whole suite pass.

### Task 7: Policy port (TODO 2, P8)

- [x] `policy_url` default `ws://127.0.0.1:8001` in `backends.py` and in `vla.yaml`. Port 8000 belongs to
  `tue_mobile_ui` (`hero-start`). The container maps `-p 8001:8000`.

### Final verification (run 2026-11-07, from `action_server/action_server`, workspace sourced)

- [x] `PYTHONPATH=src:$PYTHONPATH python3 -m pytest -q -p no:warnings test/test_vla_*.py test/test_import.py`:
  **57 passed** (v2 baseline: 50).
- [x] `PYTHONPATH=src:$PYTHONPATH python3 -m nose test/`: 58 tests, 1 failure. The failure is
  `test_grammar_test`, which needs a running roscore (`Connection refused`). It fails the same way on the
  unmodified `HEAD` `87f5f49`.
- [x] Mutation check (appendix): **8/8 caught**: the 6 v2 mutations plus "URDF velocity bound ignored" and
  "URDF position limits ignored".
- [x] `joint_limits` on `/opt/ros/noetic/share/hsrb_description/robots/hsrb4s.urdf` gives exactly the old
  hardcoded `ARM_LOWER/UPPER`, and velocity × 0.2 s = the old `max_arm_delta`. No behaviour change on the HSR.
- [x] No unused imports in the changed files. flake8 isn't installed; an AST check was used instead. The only
  lines over 120 characters were there before this change.
- [x] `roslaunch --files hero_bringup/launch/action_server.launch` resolves. Every includer passes only
  `robot_name`.
- [x] **Sim, 2026-11-07** (fresh `hero-start` + `hero-free-mode`; policy image `:latest` = `:2026-11-07` on CUDA,
  `state_dim=8`, which also closes P7's GPU start-up check). Runs in `~/vla/compare/runs/`:

  | Run | Code | Base | Valid (slide < 2 cm) | Closest | Gripper column min | Score |
  |---|---|---|---|---|---|---|
  | `20261107-204131_vla_v21-1` | this change | drifting | no (155 cm) | 15.1 cm | 0.84, never closed | 1 |
  | `20261107-204528_vla_v21pin-1` | this change | pinned | **yes (1.4 cm)** | 22.1 cm | 0.70, never closed | 1 |
  | `20261107-204941_vla_headpin-1` | old `HEAD` + old `vla.yaml` (A/B) | pinned | **yes (0.7 cm)** | 7.0 cm, apple pushed 66 cm | 0.68, never closed | 3 |

  - **Mechanics with this change: OK.**
    - The arm limits came from the live `robot_description` (no "No limits" warning).
    - The arm followed the stream: lift 0.20 → 0.66, flex 0 → −2.17.
    - 60 chunks at 10 Hz, no transport or controller error.
    - The retract after the failure ran without a warning.
  - **Grasp check, isolated probe** (`probe_grasp_check.py`, session scratchpad, empty hand closed in the air, reads −0.086):
    `grasp_minimum_position 0.3` → `cannot_determine` (correct). The detector default −0.82 → `true`, a false
    positive in the sim. So the flagged sim value is needed in the sim, and the default is right for the real
    HERO.
  - **The full completion path did not fire in a rollout**: the policy never asked to close (column ≥ 0.68)
    with the base held still. This doesn't come from this change: the old code with all sim corrections did the
    same under the same conditions (A/B). In v2 the closes happened with the base drifting.
  - **Even with the drift**, a fresh Gazebo start still slid 155 cm (P1 step 1: a restart does not fix it).
    Pinning the base, the report's step 2, makes trials valid: `pin_base.py` (session scratchpad, to be copied into `~/vla/compare`). It holds the Gazebo
    pose from the first policy query at 10 Hz and is meant to go into the trial kit as an option.
  - n = 1 per condition, with fresh policy noise per query (C8). The 7 cm vs 22 cm difference is not evidence
    either way (different target apples).

---

## Patches removed: what they compensated for, and the real fix

| Patch | Compensated for | Correct fix (outside this change) |
|---|---|---|
| `arm_delta_gain [1,1,2.5,2,1]` | the sim moves `arm_roll` 0.40× and `wrist_flex` 0.55× of the real HSR per unit delta | the sim joint gains (`hsrb_gazebo_bringup/params/hsrb_pid_gains.yaml`), in a TU/e sim override. The policy is closed-loop, so an under-moving joint gets more deltas, not a wrong result |
| `arm_hold_threshold` wrist 0.005 | the sim `wrist_flex` PID has no integral term and sags | add the integral term in the same sim override |
| gripper latch `gripper_open_threshold 0.9` | sim fingers read ~0.55 on an apple, above the 0.5 threshold | none in code: the official client uses one threshold and lifts 12/20 on the real HSR. If the sim drops a held apple, the sim gripper model is the cause |
| empty-grasp release at `grasp_position_threshold` | a consequence of the latch | gone with the latch |
| `grasp_position_threshold 0.1` + `done_confirm_chunks 2` | no grasp check existed | `ActiveGraspDetector` after the lift (Task 3) |
| hardcoded `ARM_LOWER/UPPER`, `max_arm_delta` | — | URDF (Task 1) |

## Patches kept (flagged), and how to correct them

- **`grasp_minimum_position: 0.3` in the shared `vla.yaml`** is the sim's empty-grasp reading (C16). The real HSR
  uses the detector default −0.82, and so does `challenge_serve_breakfast`. Correct fix: a sim parameter
  override in `hero_bringup` (the same place as the PID override above), or a robot-level `skills` param that
  `ActiveGraspDetector` reads. **Calibrate it**: close on nothing, on the table edge and on an apple, and log
  `hand_motor_joint` (P2).
- **`set_head_joints` negates the tilt** for HERO's `head_ref`. Correct fix: a public joint-space head call in
  `robot_skills.Head`.
- **The sink's own base `Twist` publisher**: correct fix is `Base.send_velocity/stop` on `robot_skills` master.
- **`arm_lead_time 0.2`** (official 0.05) is a documented deviation measured in the sim. Re-measure it on the
  real HSR.
- **`_send_arm_pose` uses `Arm._send_joint_trajectory`** (private). Correct fix: a public joint-positions call
  on `PublicArm`.

## Not solvable in this code (reported, unchanged)

- Sim base drift (P1).
- ED apple pose offset (P3).
- Find flakiness and hangs (P4).
- Localization after a teleport (P5).
- The classic baseline's mesh URL (P6).
- Docker dependency pinning (P7).
- Write access, push and PRs.
- Re-running the trials.
- Commanded-vs-achieved analysis.
- Base frame check on HERO.
- Offline A/B.
- Questions for the checkpoint author.
- Filling `raw_sentence` from the NLP pipeline (the templates stay).
- Real-robot calibration.
- Versioning the trial kit.

## Report statements superseded by this change

- Integration v2 "Gripper" / "Completion" / the parameter table rows: `gripper_open_threshold`,
  `grasp_position_threshold`, `done_confirm_chunks`, `arm_delta_gain`, `arm_hold_threshold`, `max_arm_delta`,
  `base_action_mode`, `checkpoint_path`/`device`/`action_layout`/`state_indices`, `policy_url` 8000.
- "`execution_mode: vla` bypasses the approach guard" (C11, caveat 5): fixed.
- "Cancel doesn't stop `NavigateToGrasp` or the rollout": fixed.
- "49 unit tests" / the Appendix A mutation strings: see the appendix below.

## Appendix: mutation check

This replaces Appendix A of the inference report v2: its strings no longer match the code. Run it from
`action_server/action_server` with the workspace sourced. It execs mutated source in memory only; no file is
changed. Result on 2026-11-07:

```text
unmutated: 4 tests, 0 failures/errors
anchor on the chunk-start state        -> CAUGHT (19 failing checks)
cumulative sum of the deltas           -> CAUGHT (18 failing checks)
deltas played as absolute targets      -> CAUGHT (21 failing checks)
gripper inverted (close above 0.5)     -> CAUGHT (20 failing checks)
base read as pose deltas               -> CAUGHT (10 failing checks)
rows at 30 Hz                          -> CAUGHT (2 failing checks)
URDF velocity bound ignored            -> CAUGHT (1 failing checks)
URDF position limits ignored           -> CAUGHT (2 failing checks)
```

```python
"""Re-run the golden test against deliberately wrong sinks: each mutation must make it fail."""
import sys
import unittest

sys.path[:0] = ["src", "test"]
import action_server.vla.backend_utils as utils  # noqa: E402
import action_server.vla.backends as backends  # noqa: E402
import test_vla_airoa_golden  # noqa: E402

ORIGINAL = {m: open(m.__file__).read() for m in (utils, backends)}
MUTATIONS = {
    "anchor on the chunk-start state": (backends, "                positions = self._joint_states.get()\n",
                                        "                positions = positions or self._joint_states.get()\n"),
    "cumulative sum of the deltas": (
        backends,
        "        current = [positions[name] for name in self._arm_joints]\n"
        "        target = apply_relative(",
        "        current = list(getattr(self, '_last', [positions[name] for name in self._arm_joints]))\n"
        "        target = self._last = apply_relative("),
    "deltas played as absolute targets": (utils, "target = np.asarray(current, dtype=np.float64) + delta",
                                          "target = delta + 0 * np.asarray(current, dtype=np.float64)"),
    "gripper inverted (close above 0.5)": (utils, "if value < close_threshold:", "if value >= close_threshold:"),
    "base read as pose deltas": (backends, "clamp_symmetric(base_row[0], self._max_base_velocity)",
                                 "clamp_symmetric(base_row[0] * self._fps, self._max_base_velocity)"),
    "rows at 30 Hz": (backends, "rospy.Rate(self._fps)", "rospy.Rate(30.0)"),
    "URDF velocity bound ignored": (backends, "self._max_arm_delta = velocity * self._arm_lead_time",
                                    "self._max_arm_delta = None"),
    "URDF position limits ignored": (backends, "self.arm_lower, self.arm_upper, velocity = limits",
                                     "_, _, velocity = limits; self.arm_lower = self.arm_upper = None"),
}


def run():
    suite = unittest.defaultTestLoader.loadTestsFromModule(test_vla_airoa_golden)
    return unittest.TextTestRunner(stream=open("/dev/null", "w"), verbosity=0).run(suite)


def load(module, source):
    exec(compile(source, module.__file__, "exec"), module.__dict__)


baseline = run()
print("unmutated: %d tests, %d failures/errors" % (baseline.testsRun, len(baseline.failures) + len(baseline.errors)))
for name, (module, old, new) in MUTATIONS.items():
    assert ORIGINAL[module].count(old) == 1, name
    load(module, ORIGINAL[module].replace(old, new))
    if module is utils:  # backends imported these helpers by name
        load(backends, ORIGINAL[backends])
    result = run()
    failed = len(result.failures) + len(result.errors)
    print("%-38s -> %s (%d failing checks)" % (name, "CAUGHT" if failed else "MISSED", failed))
    load(utils, ORIGINAL[utils])
    load(backends, ORIGINAL[backends])
```

## Also touched, and left alone

- `README.md` (VLA section): the in-process wording, `checkpoint_path`, port 8001, the mode meanings, and the 8-D
  state (it still said 6-D).
- `hero_bringup` launch files: removed the `checkpoint_path` and `vla_repo_path` plumbing of the in-process path.
- Not changed, now stale:
  - `.github/copilot-instructions.md` (6-D state, port 8000);
  - the `PYTHONPATH` to `per-group-mse-vla` in the workspace `.vscode/launch.json` (harmless; the server process
    no longer imports it);
  - the two v2 reports (the owner's uncommitted edits are in them). See "Report statements superseded" above.
