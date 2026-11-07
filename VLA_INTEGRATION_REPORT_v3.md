# VLA Integration Report v3

Date: 2026-11-07. Companion to [VLA_INFERENCE_REPORT_v3.md](./VLA_INFERENCE_REPORT_v3.md).
Supersedes [VLA_INTEGRATION_REPORT_v2.md](./VLA_INTEGRATION_REPORT_v2.md) (v2). v1 is still the reference for
the layered provider/backend idea and the GPSR start-up notes. v2 is the reference for the official AIRoA client
analysis behind the action contract. This file describes the integration **as the code is now** and what changed
since v2. Someone without the earlier context should be able to pick it up from here.

The work plan behind v3, with the owner's review questions and their answers, is
[docs/superpowers/plans/2026-11-07-vla-cleanup.md](./docs/superpowers/plans/2026-11-07-vla-cleanup.md).

## Scope

- What the integration does now, end to end, and why each piece is the way it is.
- Every code change since v2, per repo and file.
- Which v2 TODOs are resolved, and what remains.
- Parameters: code default vs deployed value.
- How it is tested, how to run it, and what is ready vs not.
- Caveats and TODOs for the next version.

## Why v3

The owner's review of v2 found that the code fixed sim test failures with sim-tuned patches instead of making the
VLA work in general, and that it re-implemented things that `robot_skills` / `robot_smach_states` already have.
v3 therefore:
- removes the sim patches;
- reuses the existing robot code;
- removes dead code;
- fixes the v2 TODOs that can be fixed in code (cancel, `vla` mode, port).

Any patch that remains is flagged, together with how to correct it properly.

## What changed since v2, in one table

| Area | v2 | v3 |
|---|---|---|
| Arm bounds | hardcoded HSR limits `ARM_LOWER/UPPER` and `max_arm_delta` | read from `/<robot>/robot_description`: the position limits, and velocity limit × `arm_lead_time` per row. The same values on the HSR |
| Sim arm corrections | `arm_delta_gain`, `arm_hold_threshold` (+ the remembered target) | **removed**. They tuned the sim PID, not the policy |
| Gripper rule | official rule + latch (`gripper_open_threshold` 0.9) + empty-grasp release | **only the official rule**: column < 0.5 → effort grasp, else position |
| Completion | own helper: fingers above `grasp_position_threshold` 0.1, on 2 chunks (`done_confirm_chunks`) | effort grasp held ≥ 1 s → **lift** → the classic **`ActiveGraspDetector`** (robot_smach_states). Not holding → open, the policy continues |
| Backend classes | `SmolVLALocalBackend` (in-process, cannot run on Python 3.8) + `SmolVLAWebSocketBackend` subclass | **one class**, `SmolVLAWebSocketBackend(BaseBackend)` |
| Base columns | `base_action_mode` `velocity` / `pose_delta` / `disabled` | velocity only (question (a) is settled) |
| Cancel | no stop; `PickUp._cancel` used a bound method as a bool | `PickUp._cancel` preempts `NavigateToGrasp`, stops the rollout at the next row (no retract) and never starts the classic pick-up afterwards |
| `execution_mode: vla` | delegated the whole action (`_VLADelegatedAction`): no Find, no approach | = hybrid flow **without the classic fallback** (like `strict_mode`). `task_manager.py` is back to master |
| Place / hand-over hooks | present, disabled by `enabled_actions` | **removed** (they ran at the wrong place); `place.py` / `hand_over.py` back to master |
| Policy port | code and yaml default 8000 (taken by `tue_mobile_ui`) | 8001 in the code and in `vla.yaml` |
| Launch | `checkpoint_path` / `vla_repo_path` plumbing for the in-process path | removed |
| Tests | 49 VLA tests | 56 VLA tests; golden test 4/4; mutation check 8/8 (6 + 2 new) |
| Sim | 20+ trials, all with the base drifting | fresh stack; first **valid** trials (base held still); an A/B against the v2 code |
| `backends.py` | 864 lines | 734 lines |

---

## Goals (unchanged, with the status now)

1. VLA for **manipulation only**; navigation stays classic. Holds in every mode now: `vla` mode also finds and
   approaches classically. Only `pick-up` is handed to the VLA.
2. Backend **replaceable** without touching challenge or action code. Holds: `BaseBackend` (`execute`, `cancel`)
   + `local_backend_class`.
3. **Regression to classic** always possible: `execution_mode: classic`, automatic fallback in `hybrid`, a retract
   before the fallback.
4. **Separate runtimes**: ROS Noetic, Python 3.8 (robot I/O) ↔ Python 3.12 Docker container (LeRobot
   inference) over WebSocket. The in-process path is gone, so this is now the only path.

---

## Architecture

```
Challenge (e.g. GPSR) --recipe--> action_server TaskManager (as on master)
  PickUp (classic Action)
    configure(): classic Find result, arm designator, classic Grab FSM
    _start():
      is_vla_enabled_for_action("pick-up")?           # execution_mode hybrid|vla and enabled_actions
        and _approach_for_vla():                      # NavigateToGrasp(...) == 'arrived', arm resolved
          maybe_run_vla_manipulation(...)             # executor.py, the single entry point
            LocalBackendVLAProvider (cached per robot)
              SmolVLAWebSocketBackend.execute()
                connect; _move_to_start_pose()
                loop max_chunks:
                  stop requested? -> cancelled
                  HSRObservationSource.get_observation()   head RGB, hand RGB, 8-D state, instruction
                  _infer() -> WebSocket/msgpack -> policy server -> (50, 11) chunk
                  HSRActionSink.play_chunk(chunk[:10])    10 Hz: arm (URDF-bounded), gripper, (base); then (head)
                  _holding_object()?                       effort grasp >= 1 s -> lift -> ActiveGraspDetector
                finally: stop base; retract if a failed pick-up moved the arm (not after a cancel);
                         release head; close socket
          success  -> _hold_vla_grasped_object()    occupied_by, identity pose, ED remove_pose, back off, carry
          failure  -> hybrid: classic Grab FSM | vla / strict_mode: report failure
      cancelled -> stop; never start the classic Grab
      otherwise -> classic Grab FSM
    _cancel(): flag; preempt NavigateToGrasp; cancel_vla(robot); preempt the Grab FSM if it runs
```

Runtime boundary:

```text
ROS Noetic, Python 3.8                     Docker smolvla-policy-server, Python 3.12
  action_server (VS Code debugpy)  --ws:8001-->  policy_server.py (baked into :2026-11-07)
  cameras + 8-D joint state                     LeRobot 0.5.1 + SmolVLA, CUDA, ~340 ms/query
  arm/gripper/base topics        <--(50,11)---   checkpoint per-group-mse-smolvla
```

Host port 8000 is taken by `hero-start`'s `tue_mobile_ui` webserver, so the policy is published on 8001, which is
now the default.

---

## Repos, branches and commits

| Repo | Branch | State |
|---|---|---|
| `action_server` | `feature/vla` | pushed: `3744c4f` "[VLA] Reuse the robot model and the classic grasp check, drop the sim patches, add cancel" (code, tests, README, `package.xml`), then the v3 reports and the plan in the commit after it. On top of `87f5f49` (v2) |
| `hero_bringup` | `feature/LUT_for_configuring_yolo_filter` (unrelated) | pushed: `a779184` "[VLA] Parameters for the v3 client …" (`vla.yaml`, `action_server.launch`, `action_server_vla_params.launch`). On top of `6003046` |
| `per-group-mse-vla` | `feature/vla_implementation` | unchanged since v2: 3 commits ahead of origin. The push was refused again on 2026-11-07 (`Permission … denied to PanosBn`, over SSH); see `VLA_PR_STATUS.md` in the workspace root |
| `tue_robocup` | `feat/vla-grasp` | unchanged and not used by v3. v3 only uses `robot_skills` / `robot_smach_states` code that is on master (`ActiveGraspDetector` since 2022, HERO's `gripper_position_detector`, `Designator`) |
| trial kit | `~/vla/compare` | **not under git**, unchanged. The base-pin and grasp-probe scripts of this session are in [the inference report v3](./VLA_INFERENCE_REPORT_v3.md), Appendices B and C |

---

## The verified contract (what the checkpoint expects and returns)

Unchanged since v2. The v2 report has the sources and the analysis of the official AIRoA client
(`airoa-org/airoa-evaluation-ICRA` @ `47b4df5`).

**Input**

| Key | Content |
|---|---|
| `head_rgb` | uint8 (480, 640, 3) **RGB** from `robot.perception.get_image()` (`rgb8`) |
| `hand_rgb` | uint8 (480, 640, 3) RGB from `/hero/hand_camera/image_raw` (needs `use_hand_camera:=true`) |
| `state` | float32 (8,) `[arm_lift, arm_flex, arm_roll, wrist_flex, wrist_roll, hand_motor, head_pan, head_tilt]` |
| `instruction` | e.g. `"pick up the apple"` |

**Output**: (50, 11) = the dataset's `action.relative`.

| Cols | Meaning | Unit | How v3 plays it |
|---|---|---|---|
| 0–4 arm | command − measured joint state at the same frame | m, rad | latest measured joints + row, re-read every tick, clipped to the URDF limits, the per-row delta capped at URDF velocity × `arm_lead_time` |
| 5 gripper | **absolute** `hand_motor_joint`: ≈ +1.24 open, ≈ −0.8 closed on nothing (real HSR) | rad | official hybrid rule: < 0.5 → one effort grasp (0.018 Nm), else position clipped to [−0.1, 1.23] |
| 6–7 head | delta w.r.t. the measured head joints, ~0 in training | rad | off by default; else measured + delta once per chunk |
| 8–10 base | body-frame velocity | m/s, rad/s | off by default; else a clamped Twist per row, braked after the prefix |

**Timing:** one row per 0.1 s (10 Hz), 10 rows per inference, like the official client.

**HERO specifics:**
- `head_ref`'s PAN_TILT goal negates tilt;
- grasp effort = `max_torque` × 0.07 m;
- the URDF in `/hero/robot_description` gives the arm limits (`lower`, `upper`, `velocity`, in the `hsrb4s.urdf`
  values).

---

## Files changed: action_server (since v2, `87f5f49` → `3744c4f` + the reports commit)

### `action_server/src/action_server/vla/backends.py` (864 → 734 lines)

1. **`HSRActionSink`**
   - **Arm bounds from the robot model.** At construction, `joint_limits(robot_description, arm joints)` gives
     `arm_lower`, `arm_upper` and the velocity limits. The per-row delta cap is `velocity × arm_lead_time`; with
     0.2 s that is `[0.04, 0.24, 0.4, 0.3, 0.3]`, exactly the old `max_arm_delta`. Without a description, the arm
     is unbounded and one warning is logged.
   - **Removed:** `ARM_LOWER/ARM_UPPER`, `max_arm_delta`, `arm_delta_gain`, `arm_hold_threshold`, the remembered
     target, `reset()`, `_arm_param()`, `gripper_open_threshold`, the empty-grasp release, `base_action_mode`.
   - **Gripper:** `_apply_gripper(value)` applies the official rule only. One effort grasp
     (`gripper.send_goal("close", timeout=0.0, max_torque=gripper_max_torque)`) while the column stays below 0.5;
     otherwise a position on `gripper_command_topic`.
   - **Stop:** an optional shared `threading.Event`. `play_chunk` returns `False` at the next row once it is set,
     and the base is braked anyway.
   - `get_arm()` (was `_get_arm`) and `arm_positions()` are public, for the backend.
2. **`SmolVLAWebSocketBackend(BaseBackend)`**, now the only backend.
   - **Removed:** `SmolVLALocalBackend` and its in-process `_load_policy` / `_infer` / empty `_release_policy`
     hook; the params `checkpoint_path`, `device`, `action_layout` and `state_indices` (read but ignored in v2);
     the stale `# -- items …` section headers.
   - `execute()`:
     1. clear the stop flag, connect (`_connect`);
     2. `_rollout`;
     3. `finally`: stop the base, retract a failed pick-up after policy motion **unless cancelled**, release the
        head, close the socket.
   - `_rollout()`: per chunk, check the stop flag, observe, infer, play the prefix, log, publish debug, then
     `_holding_object()` for a pick-up.
   - `_holding_object()`:
     - only after an effort grasp older than `grasp_settle_time`;
     - `_lift_arm()` by `post_grasp_lift` (URDF-clipped);
     - then `ActiveGraspDetector(robot, Designator(arm), **grasp_detector_args).execute()`;
     - `'true'` → success; anything else → `_open_gripper()` and continue.
   - `cancel()` sets the stop flag.
   - Kept unchanged: start pose (stop base, open, lift first, arm pose, head pose), prompt templates, the
     per-chunk log, the debug topic, one socket per episode.
3. **`HSRObservationSource`**: `get_gripper_position()` removed (the detector reads the gripper).
4. **`BaseBackend`**: new no-op `cancel()`, part of the backend contract.

### `action_server/src/action_server/vla/backend_utils.py` (162 → 108 lines)

| Helper | Status |
|---|---|
| `joint_limits(urdf_xml, joint_names)` | **new**: (lower, upper, velocity) via `urdf_parser_py`, `None` if missing |
| `gripper_action(value, close_threshold)` | the official rule only (no `grasping`, no `open_threshold`) |
| `apply_relative`, `clamp_symmetric`, `as_rgb_uint8`, `pack/unpack_image_binary`, `query_debug_message` | kept |
| `shape_arm_delta`, `gripper_holding_object`, `gripper_occupied`, `is_manipulation_done` | **removed** |

### `action_server/src/action_server/vla/executor.py`, `__init__.py`

- `maybe_run_vla_manipulation(robot, action_name, semantics, context, raw_sentence="")` is the **single entry
  point**. `maybe_run_vla_action` and `should_delegate_action_to_vla` are removed.
- `strict = strict_mode or execution_mode == "vla"`: in `vla` mode, a VLA failure is the outcome.
- `cancel_vla(robot_name)` calls `cancel()` on the cached providers of that robot. `BaseVLAProvider.cancel()` is a
  no-op, and `LocalBackendVLAProvider.cancel()` forwards to the backend.
- `__init__.py` exports `cancel_vla`, `is_vla_enabled_for_action`, `maybe_run_vla_manipulation`.

### `action_server/src/action_server/actions/pick_up.py`

- `__init__`: `self._approach = None`, `self._cancelled = False`.
- `_approach_for_vla()` keeps the `NavigateToGrasp` instance in `self._approach` so that it can be preempted.
- `_start()`: once cancelled, it doesn't start the classic Grab FSM (message " I stopped picking up. ").
- `_cancel()`:
  1. sets the flag;
  2. preempts `self._approach`;
  3. calls `cancel_vla(robot_name)`;
  4. preempts the Grab FSM **only if `self._fsm.is_running()`**.

  v2 tested the bound method `is_running` without calling it, so the condition was always true.
- `_hold_vla_grasped_object()` (bookkeeping after a VLA success) is unchanged.

### `task_manager.py`, `actions/place.py`, `actions/hand_over.py`

Identical to master again. `_VLADelegatedAction` is gone, and so are the place/hand-over VLA hooks.

### Other files

- `package.xml`: `exec_depend urdfdom_py` (provides `urdf_parser_py`).
- `README.md`, VLA section:
  - WebSocket instead of in-process;
  - no `checkpoint_path`;
  - port 8001;
  - what the modes mean;
  - the 8-D state (it said 6-D).
- Tests:

  | File | Tests | v2 |
  |---|---|---|
  | `test_vla_action_sink.py` | 28 (12 sink, 5 grasp completion, 8 retract + cancel, 2 debug topic, 1 log) | 24 |
  | `test_vla_backend_utils.py` | 10 | 16 |
  | `test_vla_airoa_golden.py` | 4 | 4 |
  | `test_vla_pick_up.py` | 8 (5 + 3 cancel) | 5 |
  | `test_vla_executor.py` (new) | 6 (modes, cancel) | — |

## Files changed: hero_bringup (`a779184`)

- `parameters/action_server/vla.yaml`:
  - **removed**: `device`, `action_layout`, `state_indices`, `checkpoint_path`, `max_arm_delta`,
    `arm_delta_gain`, `arm_hold_threshold`, `gripper_open_threshold`, `base_action_mode`,
    `grasp_position_threshold`;
  - **changed**: `policy_url` → `ws://127.0.0.1:8001`;
  - **added**: `grasp_minimum_position: 0.3`, marked SIM ONLY;
  - the comments describe the v3 behaviour.
- `launch/action_server_vla_params.launch`, `launch/action_server.launch`: the `checkpoint_path` arg/param and the
  `vla_repo_path` `PYTHONPATH` of the in-process path are removed. Every includer (`free_mode*`, `challenge_gpsr`,
  `demo_*`) passes only `robot_name`, and `roslaunch --files` resolves.

---

## Flow of commands (runtime, v3)

```
1. GPSR (or trial.py) sends: find apple @ dinner_table -> pick-up apple.
2. Find: classic navigation + ED segmentation -> object designator.
3. PickUp.configure(): arm designator (UnoccupiedArmDesignator), classic Grab FSM prepared.
4. PickUp._start():
     VLA enabled for pick-up (hybrid|vla mode, enabled_actions)?  no -> classic Grab.
     NavigateToGrasp == 'arrived' and arm resolved?                no -> classic Grab (unless cancelled).
5. SmolVLAWebSocketBackend.execute():
     connect ws (policy_url :8001) -> start pose (stop base, open, lift, arm pose, head tilt -0.6)
     for i in range(max_chunks=60):
        cancelled? -> stop (no retract)
        obs  = head RGB + hand RGB + 8-D state + "pick up the apple"
        chunk = server.infer(obs)                       # (50, 11), ~340 ms
        for row in chunk[:10] at 10 Hz:                 # 1 s
            arm     -> measured + delta, URDF-bounded -> /hero/arm_trajectory_controller/command (0.2 s lead)
            gripper -> column < 0.5: effort grasp once | else position -> /hero/gripper_controller/command
            base    -> Twist (off by default)        -> /hero/base/references, brake after the prefix
        head -> once (off by default)
        effort grasp held >= 1 s? -> lift 0.1 m -> ActiveGraspDetector: 'true' -> success
                                                                      else   -> open, continue
     finally: stop base; retract on a failed pick-up after policy motion (not after a cancel); release head;
              close socket.
6. success -> occupied_by, identity in-hand pose, ED remove_pose, back off 0.25 m, carrying_pose.
   failure -> hybrid: classic Grab runs | vla or strict_mode: the action fails.
```

One cycle takes about 1.4 s: 0.34 s inference, ~0.05 s observation, 1.0 s playing the prefix. A grasp check adds
the lift and one squeeze; the squeeze returned at once in the sim probe.

---

## Key ROS parameters

All under `/<robot>/action_server/vla`. "Code" is the default when the param is absent; "Deployed" is
`hero_bringup` `vla.yaml` (v3, `a779184`).

| Param | Code | Deployed | Meaning |
|---|---|---|---|
| `execution_mode` | `classic` | `hybrid` (launch default too) | `classic` / `hybrid` (VLA, classic fallback) / `vla` (VLA, no fallback) |
| `provider` | `none` | `local` | provider registry key |
| `local_backend_class` | `""` | `action_server.vla.backends:SmolVLAWebSocketBackend` | backend class |
| `enabled_actions` | `[pick-up]` | `[pick-up]` | actions the VLA may run |
| `strict_mode` | `false` | unset | true: no classic fallback (implied by `vla` mode) |
| `output_mode`, `action_dim` | `joints`, 11 | same | executor checks; `action_dim` only trims the logged rows |
| `policy_url` | `ws://127.0.0.1:8001` | same | policy server |
| `policy_timeout` | 30 | 30 | socket timeout [s] |
| `image_timeout` | 5 | 5 | camera wait [s] |
| `policy_fps` | 10 | 10 | row rate [Hz] |
| `chunk_prefix_steps` | 10 | 10 | rows played per inference |
| `max_chunks` | 60 | 60 | inference budget per rollout |
| `start_arm_pose` | `[0.2, 0, 0, -1.57, 0]` | same | `[]` keeps the arm |
| `start_head_pose` | `[0, -0.6]` | same | joint pan/tilt; `[]` keeps the head |
| `arm_lead_time` | 0.2 | 0.2 | `time_from_start` of each arm target; also scales the URDF velocity cap (official: 0.05; sim-measured) |
| `gripper_close_threshold` | 0.5 | 0.5 | below: effort grasp (official) |
| `gripper_max_torque` | 0.26 | 0.26 | ≈ the official 0.018 Nm |
| `grasp_settle_time` | 1.0 | 1.0 | effort grasp age before it is checked [s] |
| `post_grasp_lift` | 0.1 | 0.1 | lift before the check, and in the retract [m] |
| `grasp_minimum_position` | unset → detector default **−0.82** (real HERO) | **0.3 (SIM ONLY)** | `ActiveGraspDetector` `minimum_position`: below it the hand is closed on nothing |
| `enable_head_motion` / `head_vel` | false / 1.0 | same | head columns |
| `enable_base_motion` | false | false | base columns (velocity) |
| `max_base_velocity` / `max_base_yaw_velocity` | 0.2 / 0.4 | same | clamps (no collision avoidance) |
| `instruction_templates` | `{pick-up: "pick up the {object}"}` | unset | prompt templates |
| `publish_debug` | false | unset | debug topic (`trial.py` turns it on) |
| `arm_command_topic`, `gripper_command_topic`, `base_command_topic`, `joint_states_topic`, `hand_camera_topic` | HERO topics | unset | remap per robot |
| `state_joint_groups`, `state_*_joints` | arm, gripper, head | unset | state layout and arm joint names |

Read from the robot, not the VLA namespace: `/<robot>/robot_description` (the arm limits).

**Removed in v3**: `max_arm_delta`, `arm_delta_gain`, `arm_hold_threshold`, `gripper_open_threshold`,
`grasp_position_threshold`, `done_confirm_chunks`, `base_action_mode`, `checkpoint_path`, `device`,
`action_layout`, `state_indices`.

Server side (Docker env): `POLICY_CHECKPOINT_PATH`, `POLICY_STATE_INDICES` (`0,1,2,3,4,5,6,7`, the image
default), `POLICY_ACTION_LAYOUT` (`hsr11`).

---

## v2 TODOs: what v3 resolved

| v2 TODO (integration §TODOs) | v3 |
|---|---|
| 1. Push and PRs | **in part**: v2 and v3 are pushed in `action_server` and `hero_bringup`. Open: `per-group-mse-vla` (no write access, refused again) and the PRs |
| 2. Policy image: GPU check, pin dependencies, settle the port | GPU: **done in part.** `:latest` = `:2026-11-07` started on CUDA with `state_dim=8` and served 180 queries; that ran with the kit's bind mount, so the baked server and the 6-index refusal are still unchecked. Port: **done** (8001). Pinning the dependencies: open |
| 3. Completion: calibrate the threshold, post-lift confirmation | **redesigned**: lift, then `ActiveGraspDetector`. Calibration: empty hand in the air measured (−0.086); table edge and apple still to measure |
| 4. Stop and cancel | **done in code**, unit-tested; not yet run live |
| 5. Approach guard for full `vla` mode | **done**: `vla` = hybrid without the fallback; the delegation is removed |
| 6. Phase boundary for place / hand-over | hooks **removed**; the phase boundary itself is open |
| 7. Refactor targets | **in part**: the in-process path and the duplicate entry points are gone; one backend class. `ObservationAdapter` / `PolicyClient` / `ActionAdapter` / `CompletionPolicy` and a versioned policy protocol are open |
| 8. Fill `raw_sentence` | open (the templates stand in) |
| 9. Real HSR | open, plus: remove `grasp_minimum_position` there |
| 10. Version the trial kit | open |

---

## Status: ready vs not ready

### Ready (implemented, unit-tested, ran in Gazebo on 2026-11-07)

- The action contract: arm deltas on the live state, absolute gripper with the official rule, 10 Hz. The golden
  test is exact to 1e-9 against the official client **with the code defaults** (no URDF), and 8/8 mutations are
  caught.
- Arm bounds from the live `/hero/robot_description` (no warning in the sim runs).
- Start pose, prompt templates, approach guard, retract, one socket per episode, per-chunk log, debug topic.
- The grasp check in isolation in the sim. An empty hand closed in the air reads −0.086:
  - with `grasp_minimum_position` 0.3 the result is `cannot_determine` (correct);
  - with the detector default −0.82 it is `true`, a false positive in the sim.

  See the inference report v3 §4.4.
- 180 inferences over 3 rollouts with no transport or controller error; the retract ran after each failure.

### Implemented and unit-tested, not yet run live

- The full completion path in a rollout: lift → `true` → success. The policy never closed the hand in the
  v3 trials, so nothing triggered it.
- Cancel through the action server (approach, rollout, no classic afterwards).

### Not ready

- **The policy does not close the hand in the sim with the base held still.** This happened with v3 and also with
  the v2 code (A/B), so it is not the cleanup. It is the main blocker; see the inference report v3, P10.
- **The sim base drifts** (P1): 155 cm in a rollout on a fresh Gazebo start. Trials are only valid with the base
  pinned (a script, not yet in the kit).
- **Unstaged approach** usually fails in this sim (P3); Find is flaky (P4).
- **Base frame and sign on HERO** are not verified; base and head stay disabled.
- **Real robot**: nothing has run on hardware. `grasp_minimum_position` and `arm_lead_time` are sim values.
- `place` / `hand-over` through the VLA: removed until a phase boundary and a model exist.
- `raw_sentence` is never filled by the pipeline; the templates stand in.

---

## Testing

| What | Command (from `action_server/action_server`, workspace sourced) | Result 2026-11-07 |
|---|---|---|
| VLA unit + golden tests (pytest) | `PYTHONPATH=src:$PYTHONPATH python3 -m pytest -q -p no:warnings test/test_vla_*.py test/test_import.py` | **57 passed** |
| All tests under nose (`catkin_add_nosetests`) | `PYTHONPATH=src:$PYTHONPATH python3 -m nose test/` | 58 run, 1 failure: `test_grammar_test` needs a roscore (fails the same way on `87f5f49`) |
| Mutation check | script in the inference report v3, Appendix A | **8/8 caught** |
| URDF limits on the real model | `joint_limits` on `hsrb_description/robots/hsrb4s.urdf` | equal to the old hardcoded values |
| Sim trials | `~/vla/compare/run_trials.sh 1 NAME --staged` + the pin script | mechanics OK; first valid trials; no grasp (see the inference report v3) |

None of the unit tests needs a roscore, GPU or checkpoint.

What the tests pin, in one sentence each:
- **sink:**
  - arm = measured + delta, within the URDF limits and the velocity cap, unbounded without a description;
  - the gripper follows the official rule and re-opens above 0.5;
  - the base is clamped and braked, off by default;
  - the head is relative, once per chunk;
  - 6-wide chunks are safe;
  - the stop event ends the chunk.
- **completion:** a settled grasp is lifted and then checked; an unsettled one is not checked; a failed check
  opens and continues without re-checking; `grasp_minimum_position` reaches the detector.
- **rollout:**
  - debug topic opt-in;
  - retract only after policy motion on a failed pick-up, never masking the outcome, never after a cancel;
  - cancel is reset for the next episode.
- **pick-up:**
  - the VLA only after `arrived` with an arm; every failure path → classic;
  - a cancel during the approach or the VLA skips the classic pick-up;
  - the FSM is preempted only while it runs.
- **executor:** hybrid falls back; `vla` and `strict_mode` don't; `cancel_vla` reaches the backend of that robot
  only.
- **golden:** equal to the official client with the defaults; with the deployed params, deviations only where
  documented (lead 0.2 s, URDF cap and limits, torque rounding, no head/base); the mirror equals `vla.yaml`.

When you change `HSRActionSink` or `backend_utils`, run the golden test **and** the mutation script. When you change
`vla.yaml`, update `DEPLOYED` in `test_vla_airoa_golden.py`.

---

## Running it

### In Gazebo with the action server (GPSR or a manual command)

```bash
hero-start                                    # Gazebo + robot; also the mobile UI on :8000
hero-free-mode                                # nav, ED, perception; loads vla.yaml (policy_url :8001)
~/vla/compare/start_policy_server.sh          # smolvla-compare on :8001, waits for "Policy server ready"
# optional for evaluation: rosparam set /hero/action_server/vla/execution_mode vla   (no classic fallback)
# then start "(debugpy) Launch Action Server" in VS Code (restart it after any param change)
```

The `rosparam set … policy_url ws://127.0.0.1:8001` step of v2 is no longer needed. The manual container command
is in v2 ("Running it"). Since the 2026-11-07 image rebuild it needs neither the bind mount nor
`POLICY_STATE_INDICES`.

### Recorded trials, no action-server process

See [VLA_INFERENCE_REPORT_v3.md](./VLA_INFERENCE_REPORT_v3.md) §4 and §7. A trial only counts with the base held
still: run the pin script (Appendix B there) next to `run_trials.sh`.

### Stopping a rollout

Cancel the action-server goal: the rollout stops at the next row, the base brakes, the arm holds its last target,
and no classic pick-up follows. Killing the process (`rosnode kill /action_server`) still works as a last resort.

---

## Design notes (Q&A)

**Why read the limits from the URDF?**
They are a property of the robot, and the robot model is already on the parameter server. On the HSR they give
exactly the old numbers, so nothing changes there. On another robot, or after a URDF change, the bounds follow
without a code change. The per-row cap is "the URDF velocity limit times `arm_lead_time`": the arm speed is about
delta / lead, so the cap bounds the commanded speed at the joint's velocity limit.

**Why only the official gripper rule?**
The classic `Grab` only opens and closes, but the policy outputs a continuous finger position (pre-shaping), so a
mapping is needed. The official client's one-threshold rule lifts 12/20 on the real HSR. The v2 latch existed
because the *sim* fingers read ~0.55 on an apple, above 0.5; the empty-grasp release only existed to repair the
latch. Both compensated the sim gripper and are gone.

**Why `ActiveGraspDetector`, and why lift first?**
It is the existing HERO check (robot_smach_states, used by `challenge_serve_breakfast`): squeeze the gripper and
compare the finger positions. v2's own threshold helper did the same without the squeeze. Lifting first answers
the v2 false positives (fingers stalled on the table edge): off the table, empty fingers close further. The
`occupied_by` signal that v2 combined with it is always empty during a VLA pick-up, so it was dead.

**What is `grasp_minimum_position`, and why is it sim-only?**
It is the detector's `minimum_position`: a `hand_motor_joint` reading below it means "closed on nothing", so the
check returns `cannot_determine` instead of squeezing.
- The real HERO closes to about −0.8; the detector's default −0.82 is meant for it.
- The sim fingers close only to about −0.09 on nothing (measured), so with the default the sim reports an empty
  hand as holding.
- 0.3 sits between the sim's empty reading and its ~0.55 on an apple. It must still be calibrated on the table
  edge and on an apple.
- Remove it for the real robot. Better: move it into a sim override.

**Why does a failed check open the gripper?**
The hand holds nothing, and otherwise the next chunk would lift and check again while the policy keeps it closed.
After opening, only a new grasp of the policy is checked. The policy keeps its retries within `max_chunks`.

**Why no retract after a cancel?**
A cancel means "stop". The retract opens the gripper and moves the arm, which is the opposite. The base is braked
and the arm holds its last target (0.2 s lead).

**Why does `PickUp` keep its own cancelled flag?**
smach only logs a warning for a preempt requested before `execute()`; it does not honour it. A cancel during the
approach or the VLA would otherwise start the classic Grab afterwards.

**Why is `vla` mode now "hybrid without fallback"?**
The policy is a grasp policy trained from within reach, so delegating the whole action (no Find, no approach)
could only run at the wrong place (v2 C11). Hybrid + `strict_mode` already meant "VLA only for the grasp"; `vla`
mode now means the same, and `task_manager.py` no longer needs VLA code.

**Why were the place/hand-over hooks removed?**
They ran before the navigation to their target, so a VLA there acts at the wrong place, and no place or hand-over
checkpoint exists. They come back with the phase boundary (v1 audit) and a model.

**Why one backend class?**
The in-process class imported LeRobot, which needs Python ≥ 3.10, so it could never run in the Python 3.8
process. Its empty `_release_policy` hook and its four ignored params went with it. `BaseBackend` stays the
extension point for another VLA.

Still valid from v2:
- **Re-read the joint state every tick.** The label is "command minus measured state at the same frame".
- **`arm_lead_time` 0.2 s**: it tracks the data's speed in the sim; re-measure it on the HSR.
- **Approach first.** The episodes start within reach.
- **Retract after a failed pick-up.** The classic Grab that follows would collide otherwise.
- **Identity in-hand pose.** The VLA doesn't know the grasp offset.
- **One WebSocket per episode.** Keepalive pings go unanswered between actions.
- **Only pick-up.**

---

## Caveats for future versions (integration level)

The full list, including the simulator and test tooling, is in
[VLA_INFERENCE_REPORT_v3.md](./VLA_INFERENCE_REPORT_v3.md) §8. The ones that bite when changing the code:

1. **The backend reads its params once per process** (provider cache). Restart the action server after a param
   change. `strict_mode` and `publish_debug` are not in `vla.yaml`.
2. **`rosparam load` never deletes keys.** Loading an older `vla.yaml` (an A/B) leaves its removed params on the
   server. v3 ignores them, but delete them afterwards to keep the server honest.
3. **The golden test mirrors `vla.yaml`** on purpose: change both together.
4. **`grasp_minimum_position` 0.3 is a sim value** in the shared `vla.yaml`. Remove it for the real robot.
5. **The base backs off blindly** 0.25 m after a VLA success (`force_drive`), and the base Twists bypass collision
   avoidance. Keep base motion off on hardware until it has been tested on blocks.
6. **Uses of private `robot_skills` API remain:**
   - `Arm._send_joint_trajectory` (start pose, lift, retract);
   - `Head._setHeadReferenceGoal` (tilt negated for HERO);
   - the sink's own base `Twist` publisher.

   The proper fix is in `robot_skills` (TODO 5).
7. **`urdf_parser_py` prints "Unknown tag" warnings** once per process when it parses the robot description.
   Harmless.
8. **`tue_robocup` `feat/vla-grasp` contains the old VLAGrasp path** (`f1194f226`); `grasp_mode` must stay `off`.
9. **The trial kit is outside git**, and the pin script only exists in the inference report v3 appendix.

---

## TODOs (integration), now

1. **PRs and access** (v3 is committed and pushed):
   - decide the `hero_bringup` branch (v2 and v3 sit on the unrelated `feature/LUT_for_configuring_yolo_filter`);
   - get write access and push `per-group-mse-vla`;
   - open the PRs.
2. **Sim parameter override** in `hero_bringup`: move `grasp_minimum_position` (and any future sim-only value) out
   of the shared `vla.yaml`.
3. **Live checks**: the full completion path once the policy grasps (inference report v3, TODO 1), and a cancel
   through the action server.
4. **Calibrate `grasp_minimum_position`** in the sim: on the table edge and on an apple (the empty hand in the air
   is done).
5. **Upstream `robot_skills`**:
   - merge `Base.send_velocity/stop` (on `feat/vla-grasp`);
   - add a public joint-space head call and a public joint-positions call on `PublicArm`;
   - then drop the private uses (caveat 6).
6. **Policy image**: check the baked server without the bind mount (log `state_dim=8`, `smoke_test.py`, 6 indices
   refused); pin the Dockerfile dependencies.
7. **Phase boundary for place / hand-over**, then re-add them with a model.
8. **Refactor targets** still open: `ObservationAdapter`, `PolicyClient`, `ActionAdapter`, `CompletionPolicy`; a
   versioned policy protocol validated at connect.
9. **Fill `raw_sentence`** from the NLP pipeline, or keep the templates (documented).
10. **Real HSR**:
    - remove `grasp_minimum_position`;
    - re-measure `arm_lead_time`, the gripper readings and the head tilt sign;
    - keep base and head off;
    - dry-run with the e-stop at hand;
    - decide `start_arm_pose`.
11. **Docs**: `.github/copilot-instructions.md` still describes the 6-D state and port 8000.

Evaluation TODOs (the grasp blocker, base pinning in the kit, batches, the base frame check, the offline A/B,
questions for the checkpoint author) are in the inference report v3 §9.

---

## Superseded statements in v2 (VLA_INTEGRATION_REPORT_v2.md)

- **"What changed since v2" table, rows Gripper, Completion, Sim corrections, `execution_mode`**: see the v3
  table.
- **Files changed, `HSRActionSink`:** "bounded by `max_arm_delta` and the URDF limits (`ARM_LOWER/UPPER`)" → read
  from `robot_description`. `shape_arm_delta`, the latch, the empty release and `pose_delta` are removed.
- **`SmolVLALocalBackend` "the shared rollout"**: removed; the rollout is in `SmolVLAWebSocketBackend`.
- **`backend_utils` table**: `shape_arm_delta`, `gripper_holding_object`, `gripper_occupied`,
  `is_manipulation_done` are removed; `gripper_action` lost two arguments; `joint_limits` is new.
- **Key ROS parameters**: the rows listed under "Removed in v3"; `policy_url` 8000 → 8001;
  `grasp_position_threshold` → `grasp_minimum_position`.
- **"`execution_mode: vla` … skips `NavigateToGrasp`"** and caveat 5: fixed.
- **"Cancel doesn't stop `NavigateToGrasp` or the rollout … The only stop is killing the action-server process"**:
  fixed in code.
- **"Running it": `rosparam set … policy_url ws://127.0.0.1:8001`**: no longer needed.
- **Design notes "Why are the sim corrections params…" and "Why hold an effort grasp until 0.9…"**: the
  mechanisms are removed.
- **Testing "50 passed / 49 OK", mutation "6/6"**: now 57 passed, 8/8.
