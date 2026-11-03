# VLA Integration Report v2

Date: 2026-11-03. Companion to [VLA_INFERENCE_REPORT_v2.md](./VLA_INFERENCE_REPORT_v2.md).
Supersedes [VLA_INTEGRATION_REPORT.md](./VLA_INTEGRATION_REPORT.md) (v1). v1 is still the reference for
the parts that didn't change: the layered provider/backend architecture, the 6-DoF training layouts and
the GPSR start-up notes. This file describes the integration **as the code is now** and what changed
since v1. Someone without the earlier context should be able to pick it up from here.

## Scope

- What the integration does now, end to end, and why each piece is the way it is.
- Every code change since v1, per repo and file.
- The action/observation contract the checkpoint actually uses (verified).
- Parameters: code default vs deployed value.
- How it is tested, how to run it, and what is ready vs not.
- Caveats and TODOs for the next version.

## What changed since v1, in one table

| Area | v1 | v2 |
|---|---|---|
| Arm execution | `_send_joint_trajectory` with the row as an **absolute** target, timed by `max_joint_vel`, blocking | one row every 0.1 s: target = **latest measured joints + delta**, published to `arm_trajectory_controller/command`, 0.2 s lead, non-blocking |
| Gripper | binary open/close from the last row, **inverted** (closed when ≥ 0.5) | per row, official "hybrid" mode: < 0.5 effort grasp (0.018 Nm), else position; latched grasp; empty grasp released |
| Head | last row as an absolute pan/tilt | off by default; when on: measured + delta once per chunk, tilt negated for HERO's `head_ref` |
| Base | `force_drive` after the arm, 0.1 s per row | Twist per row in the same 10 Hz loop, clamped, braked after the prefix; off by default |
| Body groups | played one after another | one synchronised loop (arm + gripper + base per tick) |
| State | 6-D sent by the server | 8-D, and the server rejects wrong indices at startup |
| Prompt | literal `pick-up` | `"pick up the {object}"` from the semantics (unless `raw_sentence` is set) |
| Where the VLA runs | at the find pose, before any navigation | only after `NavigateToGrasp` **arrived** with a resolved arm |
| Start state | HERO reset/carrying pose (`arm_roll` −1.57, −7σ off training) | start pose: lift to 0.2 first, then `[0.2, 0, 0, −1.57, 0]`; head at joint tilt −0.6; gripper open |
| Completion | could never fire | effort grasp settled 1 s **and** fingers stopped above `grasp_position_threshold`, on 2 consecutive chunks; then lift |
| After success | nothing knew the arm held the object | `occupied_by`, identity in-hand pose, ED pose removed, back off, carrying pose (as `Grab` does) |
| After failure | arm left low over the table | retract: open, lift, start pose; then the classic fallback (hybrid) or failure (strict) |
| VLA for place / hand-over | enabled | disabled (their hooks run before navigation to their target) |
| WebSocket | cached socket, died between actions | one connection per episode |
| Sim corrections | — | `arm_delta_gain`, `arm_hold_threshold` (neutral by default) |
| Diagnostics | base-check log only | per-chunk log line, opt-in debug topic with every query |
| Tests | none | 49 unit tests incl. a golden test against the official AIRoA client; 11 server tests |

---

## Goals (unchanged, with the status now)

1. VLA for **manipulation only**; navigation stays classic. Now enforced: the VLA runs only after
   `NavigateToGrasp`, and only for `pick-up`.
2. Backend **replaceable** without touching challenge or action code. This still holds: `BaseBackend` +
   `local_backend_class`.
3. **Regression to classic** always possible: `execution_mode: classic`, automatic fallback in hybrid,
   and now a retract before the fallback.
4. **Separate runtimes**: ROS Noetic, Python 3.8 (robot I/O) ↔ Python 3.12 Docker container (LeRobot
   inference) over WebSocket. Unchanged.

---

## Architecture

```
Challenge (e.g. GPSR) --recipe--> action_server TaskManager
  PickUp (classic Action)
    configure(): classic Find result, arm designator
    _start():
      is_vla_enabled_for_action("pick-up")?        # execution_mode hybrid|vla and enabled_actions
        and _approach_for_vla():                    # NavigateToGrasp(...) == 'arrived', arm resolved
          maybe_run_vla_manipulation(...)           # executor.py
            LocalBackendVLAProvider (cached per robot)
              SmolVLAWebSocketBackend.execute()
                _move_to_start_pose(); sink.reset()
                loop max_chunks:
                  HSRObservationSource.get_observation()   head RGB, hand RGB, 8-D state, instruction
                  _infer() -> WebSocket/msgpack -> policy server -> (50, 11) chunk
                  HSRActionSink.play_chunk(chunk[:10])    10 Hz: arm, gripper, (base); then (head)
                  _is_episode_done()?  -> lift, succeed
                finally: stop base, retract if a failed pick-up moved the arm, release head, close socket
          success  -> _hold_vla_grasped_object()    occupied_by, identity pose, ED remove_pose, back off, carry
          failure  -> strict: report failure | hybrid: classic Grab FSM
      otherwise -> classic Grab FSM
```

Runtime boundary (unchanged), now with ports:

```text
ROS Noetic, Python 3.8                     Docker smolvla-policy-server, Python 3.12
  action_server (VS Code debugpy)  --ws:8001-->  policy_server.py (bind-mounted from the repo)
  cameras + 8-D joint state                     LeRobot 0.5.1 + SmolVLA, CUDA, ~340 ms/query
  arm/gripper/base topics        <--(50,11)---   checkpoint per-group-mse-smolvla
```

Host port 8000 is taken by `hero-start`'s `tue_mobile_ui` webserver, so the policy is published on 8001.

---

## Repos, branches and commits

Nothing is pushed. The branches are ahead of their remotes: `action_server` by 8 commits (including a
merge from master), `per-group-mse-vla` by 3.

| Repo | Branch | Commits / state |
|---|---|---|
| `action_server` | `feature/vla` | `2517279` "Stream relative actions at 10 Hz, 8-D state, grasp completion and start pose" (baseline of the earlier uncommitted work); `05e4ab4` "Golden test of the action sink against the official AIRoA HSR client"; `e87e410` "Opt-in debug topic with every policy query"; `02e5c81` "Sim arm corrections, empty-grasp release, retract and approach guards". On top of `b57d409` (merge of master, includes the "received DONE twice" fix in `client.py`) |
| `per-group-mse-vla` | `feature/vla_implementation` | `8b58877` "Serve the 8-D state the per-group-mse-smolvla checkpoint was trained on"; `b3200ec` "Validate state_indices against the normalisation statistics"; `4fde5ee` "Drop the state length check that startup validation makes redundant" |
| `hero_bringup` | `feature/LUT_for_configuring_yolo_filter` (unrelated) | **uncommitted**: `parameters/action_server/vla.yaml` (all new params), `launch/action_server.launch` (`execution_mode` default `hybrid`, the owner's edit) |
| `tue_robocup` | `feat/vla-grasp` | not part of this integration. It has uncommitted changes from an older VLAGrasp path, gated by `/hero/skills/vla/grasp_mode` (default `off`) |
| trial kit | `~/vla/compare` | **not under git**: `trial.py`, `run_trials.sh`, `score_runs.py`, `report.py`, `start_policy_server.sh`, `README.md` |

---

## The verified contract (what the checkpoint expects and returns)

Sources:
- the checkpoint's normalisation statistics and `state_proj` weights;
- LeRobot 0.5.1;
- AIRoA's dataset converter;
- the **official AIRoA HSR client** `airoa-org/airoa-evaluation-ICRA` @ `47b4df5`,
  `deploy/hsr_policy_client/scripts/hsr_policy.py`. It is public, despite older docs saying otherwise.

**Input**

| Key | Content |
|---|---|
| `head_rgb` | uint8 (480, 640, 3) **RGB** from `robot.perception.get_image()` (`rgb8`). Training data is RGB; the official client's BGR slice is a no-op |
| `hand_rgb` | uint8 (480, 640, 3) RGB from `/hero/hand_camera/image_raw` (needs `use_hand_camera:=true`) |
| `state` | float32 (8,) `[arm_lift, arm_flex, arm_roll, wrist_flex, wrist_roll, hand_motor, head_pan, head_tilt]`. The `[6]` in `config.json` is stale |
| `instruction` | e.g. `"pick up the apple"` |

**Output**: (50, 11) = the dataset's `action.relative`, bit-identical statistics.

| Cols | Meaning | Unit |
|---|---|---|
| 0–4 arm | command − measured joint state at the same frame. Play row k as the **latest** joint state + row, re-read every tick. Do not sum cumulatively, and do not anchor on the chunk-start state | m, rad |
| 5 gripper | **absolute** `hand_motor_joint`: ≈ +1.24 open, ≈ −0.8 closed on nothing (real HSR), ≈ 0.5–0.55 on an apple | rad |
| 6–7 head | delta w.r.t. the measured head joints, ~0 in training | rad |
| 8–10 base | body-frame velocity: the `/hsrb/command_velocity` Twist (AIRoA docs call it "delta") | m/s, rad/s |

**Timing:** one row per 0.1 s (10 Hz). The official client plays 10 rows per inference (`update_freq`
10, `adopted_action_chunks` 10). This is not confirmed from the private fine-tuning dataset.

**HERO specifics:**
- `head_ref`'s PAN_TILT goal negates tilt, so joint tilt −0.7 = goal tilt +0.7;
- `hero_bridge` has a 0.05 rad head deadband;
- grasp effort = `max_torque` × 0.07 m, so 0.26 ≈ the official 0.018 Nm;
- `robot_skills` `_send_joint_trajectory` times every waypoint from the start pose.

---

## Files changed: action_server (since v1, `770b5cd` → `02e5c81`)

### `action_server/src/action_server/vla/backends.py` (about 580 changed lines)

1. **`HSRActionSink`** (rewritten). It streams a chunk prefix one row per `1/policy_fps` s.
   - **Arm:** `current` = the latest `/hero/joint_states` from a persistent `JointStateCache`.
     - The delta is shaped by `shape_arm_delta(delta, current, arm_delta_gain, arm_hold_threshold,
       previous_target)`, then bounded by `max_arm_delta` and the URDF limits (`ARM_LOWER/UPPER`) in
       `apply_relative`.
     - The result is published as a one-point `JointTrajectory` on `arm_command_topic` (default
       `/hero/arm_trajectory_controller/command`) with `time_from_start = arm_lead_time`. The zero
       stamp replaces the previous trajectory.
     - The target is remembered for the hold; `reset()` forgets it.
   - **Gripper:**
     - `gripper_action(value, grasping, close, open)` returns `None` for an effort grasp. The grasp is
       sent once via `gripper.send_goal("close", timeout=0.0, max_torque=gripper_max_torque)`.
     - Otherwise it returns a position, clipped to [−0.1, 1.23] and published on
       `gripper_command_topic` (`/hero/gripper_controller/command`, tfs 1.0 s).
     - A grasp is held while the column stays below `gripper_open_threshold` (0.9), because a hand
       holding an apple reads ~0.55. If the measured `hand_motor_joint` ≤ `grasp_position_threshold`
       (fingers closed on nothing), it reopens at the close threshold like the official client.
   - **Base** (when `enable_base_motion`): a Twist per row on `base_command_topic`
     (`/hero/base/references`). `pose_delta` rows are multiplied by `policy_fps`, clamped by
     `max_base_velocity` / `max_base_yaw_velocity`, and **always braked** after the prefix (`finally`).
   - **Head** (when `enable_head_motion`): once per chunk, measured + delta of the last row, via
     `set_head_joints` (tilt negated). `head_ref` cancels any in-flight goal, so a per-row stream would
     abort every move.
   - **Parameter checks:** arm params are broadcast to length 5, so a wrong-length param fails when the
     sink is built, before the robot moves.
2. **`SmolVLALocalBackend`** (the shared rollout; `SmolVLAWebSocketBackend` only swaps transport).
   - `_move_to_start_pose()`: `force_drive(0, 0, 0, 0)` cancels a planner goal; then open the gripper;
     **lift first** to the start lift (clears the table edge); then `start_arm_pose`, then
     `start_head_pose`.
   - `_instruction()`: `raw_sentence` if set, else `instruction_templates[action]` formatted with the
     object name (`"pick up the {object}"`), else the action name.
   - `_rollout()`, up to `max_chunks` times:
     - observe (hand image required) and infer;
     - play `chunk[:chunk_prefix_steps]`;
     - `_log_chunk` (state, per-joint |delta| max, **signed** delta sum, gripper range and command,
       base |v| max);
     - optionally publish the debug topic;
     - check completion.
   - Completion: `gripper_holding_object(...)` means an effort grasp ≥ `grasp_settle_time` old and the
     fingers above `grasp_position_threshold`. It must hold on `done_confirm_chunks` (2) consecutive
     chunks. Then `_lift_arm()` (`post_grasp_lift`) and success.
   - `execute()` `finally`:
     - `stop_base()`;
     - **`_retract()`** (open, lift, start pose, guarded by try/except so it never masks the outcome),
       only for a pick-up that failed **after the policy moved**;
     - `robot.head.cancel_goal()` (the start pose leaves a head goal that never expires);
     - `_release_policy()`.
   - `publish_debug`: every query as JSON (iteration, instruction, state, full chunk, prefix length) on
     `/hero/action_server/vla/debug/query`.
3. **`SmolVLAWebSocketBackend`**: msgpack request (binary-framed images, state, instruction). The socket
   is closed on any error and at the end of each episode. Between actions nobody answers the server's
   keepalive pings, so a cached socket was dead by the next action.
4. **`HSRObservationSource`**: unchanged layout (configurable `state_joint_groups`, default 8-D). Added
   `get_gripper_position()`. Images are `rgb8`.
5. **`set_head_joints(robot, pan, tilt, vel, timeout)`**: one place for the HERO tilt negation.

### `action_server/src/action_server/vla/backend_utils.py`

New pure helpers, each unit-tested:

| Helper | What it does |
|---|---|
| `clamp_symmetric` | clamp to [−limit, limit], 0 = no clamp |
| `apply_relative(current, delta, lower, upper, max_delta)` | target = current + clip(delta), clipped to the limits |
| `shape_arm_delta(delta, current, gain, hold_threshold, previous_target)` | scale the deltas; a joint whose \|delta\| < threshold re-sends its previous target. Defaults are identity |
| `gripper_action(value, grasping, close, open)` | the hybrid gripper rule |
| `gripper_holding_object(command, since, position, threshold, settle)` | physical grasp signal |
| `query_debug_message(...)` | JSON for the debug topic |

Kept from v1: `as_rgb_uint8`, `pack_image_binary`/`unpack_image_binary`, `gripper_occupied`,
`is_manipulation_done`.

### `action_server/src/action_server/vla/executor.py`, `__init__.py`

- `enabled_actions` default is now `["pick-up"]`. The place/hand-over hooks run before the classic
  navigation to their target location, so a VLA there would act at the wrong place.
- New public `is_vla_enabled_for_action(robot_name, action_name)`, so `PickUp` only approaches for the
  VLA when the VLA would actually run.
- Fallback semantics (unchanged, made explicit here):
  - VLA failure with `strict_mode` false → `used=False` → the classic FSM runs;
  - with `strict_mode` true → the action fails.

### `action_server/src/action_server/actions/pick_up.py`

- `_start()`: VLA only if `is_vla_enabled_for_action(...)` **and** `_approach_for_vla()`.
- `_approach_for_vla()`:
  - the arm designator resolves to None → log a warning and use the classic pick-up;
  - otherwise run `NavigateToGrasp(robot, arm, object)`;
  - anything but `'arrived'` → log a warning and use the classic pick-up.
- `_hold_vla_grasped_object()`, after a VLA success:
  - `entity.pose = FrameStamped.identity(now, "map")`. `Place` composes its place pose with the object's
    in-hand pose, and the VLA doesn't know the grasp offset; `Grab` would store a measured one;
  - `arm.gripper.occupied_by = entity`;
  - `ed.update_entity(remove_pose=True)`;
  - `force_drive(−0.125, 0, 0, 2.0)` (0.25 m back, **blind**);
  - `carrying_pose`.

### Other files

- `package.xml`: `exec_depend python3-yaml`; `test_depend python_orocos_kdl`, `python3-rospkg`.
- `.github/copilot-instructions.md`: `--state-indices 0,1,2,3,4,5,6,7`.
- `VLA_INFERENCE_REPORT.md`: §7 added (follow-up of 2026-10-30, plus the v2 fixes table rows).
- Tests (new):

  | File | Tests |
  |---|---|
  | `test_vla_action_sink.py` | 24 |
  | `test_vla_backend_utils.py` | 16 |
  | `test_vla_airoa_golden.py` | 4 |
  | `test_vla_pick_up.py` | 5 |

  All are registered through the existing `catkin_add_nosetests(test)`. Details in the inference report
  v2 §3.

## Files changed: per-group-mse-vla

- `inference/policy_server.py`:
  - `8b58877`: default state indices 0..7; the state dim is sized from them.
  - `b3200ec`:
    - `_state_stats_dim(stats)` reads the state dim from the normalisation statistics (`mean`/`q01`, or
      legacy `state_mean`/`state_q01`);
    - `validate_state_indices(indices, stats_dim)` rejects empty, negative, or a length different from
      the statistics;
    - `Server._apply_state_indices()` runs this **before** `_align_processor_state_statistics()` trims
      the stats, on both the processor and the legacy path.

    Before this, 6 indices on the 8-D checkpoint were silently accepted, and the model got the mean head
    pose.
  - `4fde5ee`: removed the now-dead length check in `_select_state`.
- `inference/Dockerfile`: `POLICY_STATE_INDICES=0,1,2,3,4,5,6,7` (the image is **not rebuilt yet**; see
  the caveats).
- `inference/local_smoke_test.py`: 8-D.
- `inference/test_policy_server.py` (new): 11 offline tests with a fake LeRobot factory.

## Files changed: hero_bringup (uncommitted)

`parameters/action_server/vla.yaml` is the full deployed configuration (see the parameter table). Its
comments explain each value. `launch/action_server.launch` defaults `execution_mode` to `hybrid`
(`$(optenv ACTION_SERVER_EXECUTION_MODE hybrid)`). The node itself stays commented out, because VS Code's
debugpy launch owns the process.

---

## Flow of commands (runtime, v2)

```
1. GPSR (or trial.py) sends: find apple @ dinner_table -> pick-up apple.
2. Find: classic navigation + ED segmentation -> object designator.
3. PickUp.configure(): arm designator (UnoccupiedArmDesignator), classic Grab FSM prepared.
4. PickUp._start():
     VLA enabled for pick-up (hybrid|vla mode, enabled_actions)?  no -> classic Grab.
     NavigateToGrasp == 'arrived' and arm resolved?                no -> classic Grab.
5. SmolVLAWebSocketBackend.execute():
     connect ws (policy_url) -> start pose (stop base, open, lift, arm pose, head tilt -0.6) -> sink.reset()
     for i in range(max_chunks=60):
        obs  = head RGB + hand RGB + 8-D state + "pick up the apple"
        chunk = server.infer(obs)                       # (50, 11), ~340 ms
        for row in chunk[:10] at 10 Hz:                 # 1 s
            arm     -> measured + shaped delta  -> /hero/arm_trajectory_controller/command (0.2 s lead)
            gripper -> effort grasp (<0.5, latched) | position -> /hero/gripper_controller/command
            base    -> Twist (off by default)       -> /hero/base/references, brake after the prefix
        head -> once (off by default)
        holding (effort grasp >= 1 s, fingers > threshold) on 2 chunks? -> lift 0.1 m -> success
     finally: stop base; retract on a failed pick-up after policy motion; release head; close socket.
6. success -> occupied_by, identity in-hand pose, ED remove_pose, back off 0.25 m, carrying_pose.
   failure -> strict_mode: action fails | hybrid: classic Grab runs (does its own NavigateToGrasp).
```

One cycle takes about 1.4 s: 0.34 s inference, ~0.05 s observation, 1.0 s playing the prefix.

---

## Key ROS parameters

All under `/<robot>/action_server/vla`. "Code" is the default when the param is absent; "Deployed" is
`hero_bringup` `vla.yaml`.

| Param | Code | Deployed | Meaning |
|---|---|---|---|
| `execution_mode` | `classic` | `hybrid` (launch default too) | `classic` / `hybrid` / `vla` |
| `provider` | `none` | `local` | provider registry key |
| `local_backend_class` | `""` | `action_server.vla.backends:SmolVLAWebSocketBackend` | backend class |
| `enabled_actions` | `[pick-up]` | `[pick-up]` | actions the VLA may run |
| `strict_mode` | `false` | unset | true: VLA failure fails the action (no classic fallback) |
| `policy_url` | `ws://127.0.0.1:8000` | `ws://127.0.0.1:8000` | **use 8001 in sim** (8000 = mobile UI) |
| `policy_timeout` | 30 | 30 | socket timeout [s] |
| `image_timeout` | 5 | 5 | camera wait [s] |
| `policy_fps` | 10 | 10 | row rate [Hz] |
| `chunk_prefix_steps` | 10 | 10 | rows played per inference |
| `max_chunks` | 60 | 60 | inference budget per rollout |
| `start_arm_pose` | `[0.2, 0, 0, -1.57, 0]` | same | `[]` keeps the arm |
| `start_head_pose` | `[0, -0.6]` | same | joint pan/tilt; `[]` keeps the head |
| `arm_lead_time` | 0.2 | 0.2 | `time_from_start` of each arm target (official: 0.05) |
| `max_arm_delta` | `[0.04, 0.24, 0.4, 0.3, 0.3]` | same | per-row delta cap = URDF velocity × 0.2 s; don't raise it |
| `arm_delta_gain` | 1 | `[1, 1, 2.5, 2, 1]` | **sim** correction: arm_roll/wrist_flex under-move |
| `arm_hold_threshold` | 0 | `[0, 0, 0, 0.005, 0]` | **sim** correction: wrist_flex sag |
| `gripper_close_threshold` | 0.5 | 0.5 | below: effort grasp (official 0.5; 0.7 closed too early) |
| `gripper_open_threshold` | 0.9 | 0.9 | release of a grasp on an object |
| `gripper_max_torque` | 0.26 | 0.26 | ≈ 0.018 Nm |
| `grasp_position_threshold` | unset (no completion) | 0.1 | fingers above = holding. **0.1 gives false positives; ~0.3 proposed** |
| `grasp_settle_time` | 1.0 | 1.0 | effort grasp age before it counts [s] |
| `done_confirm_chunks` | 2 | unset | consecutive chunks the signal must hold |
| `post_grasp_lift` | 0.1 | 0.1 | lift after a grasp, and in the retract [m] |
| `enable_head_motion` | false | false | head columns |
| `enable_base_motion` | false | false | base columns |
| `base_action_mode` | `velocity` | `velocity` | `velocity` / `pose_delta` / `disabled` |
| `max_base_velocity` / `max_base_yaw_velocity` | 0.2 / 0.4 | same | clamps (no collision avoidance) |
| `head_vel` | 1.0 | 1.0 | head reference speed |
| `instruction_templates` | `{pick-up: "pick up the {object}"}` | unset | prompt templates |
| `publish_debug` | false | unset | debug topic |
| `arm_command_topic`, `gripper_command_topic`, `base_command_topic`, `joint_states_topic`, `hand_camera_topic` | HERO topics | unset | remap per robot |
| `state_joint_groups`, `state_*_joints` | arm, gripper, head | unset | state layout |
| `action_layout`, `action_dim`, `state_indices`, `checkpoint_path`, `device` | — | `hsr11`, 11, 0..7, launch arg, `cuda` | read but **ignored** by the WebSocket backend; the server's own args decide |

Server side (Docker env): `POLICY_CHECKPOINT_PATH`, `POLICY_STATE_INDICES` (must be `0,1,2,3,4,5,6,7`),
`POLICY_ACTION_LAYOUT` (`hsr11`).

---

## Status: ready vs not ready

### Ready (implemented, unit-tested, ran in Gazebo)

- Action contract (arm/head deltas on the live state, absolute gripper, base velocity, 10 Hz). Locked by
  the golden test (exact to 1e-9 against the official client with reference-equivalent params) plus a
  mutation check (6/6 bugs caught).
- 8-D state, validated by the server at startup.
- Start pose, prompt templates, approach guard, retract, completion signal (mechanics), post-grasp
  bookkeeping, one socket per episode, per-chunk log and debug topic.
- Ran in Gazebo end to end: 693 inferences across 20+ trials, no transport or controller error from the
  VLA path.

### Not ready

- **Grasp quality is unmeasured**: the simulated base drifts by itself (inference report v2 P1).
- **Completion threshold** gives false positives at 0.1 (P2).
- **Unstaged approach** usually fails in this sim (ED apple poses about 0.2 m off): the VLA rarely runs
  in the normal GPSR flow (P3).
- **Base frame and sign on HERO** not verified; base and head stay disabled.
- **`execution_mode: vla`** (full delegation) skips `NavigateToGrasp`, so the VLA runs wherever the robot
  stands. Use `hybrid`.
- **Cancel** doesn't stop `NavigateToGrasp` or the rollout. `PickUp._cancel` checks `self._fsm.is_running`,
  a bound method that is always truthy. The only stop is killing the action-server process.
- **Real robot**: nothing has run on hardware. The sim corrections (`arm_delta_gain`,
  `arm_hold_threshold`) are sim-specific.
- `place` / `hand-over` through the VLA: disabled until a phase boundary exists (see the v1 audit).
- `raw_sentence` is never filled by the pipeline; the templates stand in.

---

## Testing

| What | Command | Result 2026-11-03 |
|---|---|---|
| VLA unit + golden tests (pytest) | `cd action_server/action_server && PYTHONPATH=src:$PYTHONPATH python3 -m pytest -q -p no:warnings test/test_vla_*.py test/test_import.py` | 50 passed |
| Same under nose (what `catkin_add_nosetests` uses) | `PYTHONPATH=src:$PYTHONPATH python3 -m nose test/test_vla_*.py` | 49 OK |
| Policy server | `cd per-group-mse-vla/inference && ~/vla/.venv/bin/python -m unittest test_policy_server` | 11 OK |
| Mutation check | script in inference report v2, Appendix A | 6/6 caught |
| Sim trials | `~/vla/compare/run_trials.sh 10 NAME --staged` | runs end to end; scores invalid (base drift) |

Source the workspace first (`source ~/ros/noetic/system/devel/setup.bash`). None of the unit tests needs
a roscore, GPU or checkpoint.

What the tests pin, in one sentence each:
- **sink:** arm = measured + delta, within limits; gripper hybrid with latch and empty-grasp release;
  base clamped and braked, off by default; head relative, once per chunk; 6-wide chunks safe; gain
  applied before the cap; hold and reset; wrong-length params fail early.
- **rollout:** debug topic opt-in; retract only after policy motion on a failed pick-up, never masking the
  outcome.
- **pick-up:** VLA only after `arrived` with an arm; every failure path → classic.
- **golden:** equal to the official client; deployed deviations only where documented; the test's mirror
  equals `vla.yaml`.
- **server:** state indices validated against the statistics before trimming.

When you change `HSRActionSink` or `backend_utils`, run the golden test **and** the mutation script. When
you change `vla.yaml`, update `DEPLOYED` in `test_vla_airoa_golden.py`, because
`test_deployed_mirror_matches_vla_yaml` will fail otherwise. That failure is intended.

---

## Running it

### In Gazebo with the action server (GPSR or a manual command)

```bash
hero-start                                    # Gazebo + robot; also the mobile UI on :8000
hero-free-mode                                # nav, ED, perception; loads vla.yaml into /hero/action_server/vla
~/vla/compare/start_policy_server.sh          # or `docker start smolvla-compare` if it exists
rosparam set /hero/action_server/vla/policy_url ws://127.0.0.1:8001
# optional for evaluation: rosparam set /hero/action_server/vla/strict_mode true
# then start "(debugpy) Launch Action Server" in VS Code (restart it after any param change)
```

The equivalent manual container, if the script is not available:

```bash
docker run --rm --gpus all \
  -v /home/amigo/.cache/huggingface:/root/.cache/huggingface \
  -v /home/amigo/.cache/huggingface/hub/models--PauMontagut--per-group-mse-smolvla:/model-cache:ro \
  -v /home/amigo/ros/noetic/repos/github.com/tue-robotics/per-group-mse-vla/inference/policy_server.py:/opt/policy/policy_server.py:ro \
  -e POLICY_CHECKPOINT_PATH=/model-cache/snapshots/cb72ca6a3a58e724a3ca8579bea3811f1810be96 \
  -e POLICY_STATE_INDICES=0,1,2,3,4,5,6,7 \
  -p 8001:8000 smolvla-policy-server
```

Check the server log for `state_dim=8`. The bind mount is needed until the image is rebuilt: the current
image bakes an older `policy_server.py` and the 6-D default.

### Recorded trials, no action-server process

See [VLA_INFERENCE_REPORT_v2.md](./VLA_INFERENCE_REPORT_v2.md) §4.1 and §7. `trial.py` runs Find and
PickUp itself with the action server's classes, and sets and restores the params it needs.

### Stopping a rollout

There is no stop button. Stop the action server in VS Code, or run `rosnode kill /action_server`. The
base stops 0.5 s after the last Twist.

---

## Design notes (Q&A, new in v2)

**Why re-read the joint state every tick instead of summing the deltas?**
The training label is "command minus measured state at the same frame". So the target for row k is the
state at tick k plus row k. Summing the deltas would integrate the lead over and over. Anchoring on the
chunk-start state would ignore what the arm actually did. The golden test feeds a drifting joint state
so that both mistakes fail.

**Why `arm_lead_time` 0.2 s when the official client uses 0.05 s?**
The arm's speed is about delta / lead. In the sim, 0.2 s tracks the teleop data's speed for lift, flex
and wrist roll. Do not copy the official 100 Hz spline / 5 ms / EMA scheme; it isn't needed at 10 Hz.
This is a documented deviation in the golden test.

**Why are the sim corrections params with neutral defaults?**
They correct the simulator, not the policy:
- the sim `wrist_flex` PID has no integral term, so re-anchoring on the measured joint lets the wrist sag
  about 0.014 rad per chunk;
- `arm_roll` / `wrist_flex` move 0.40× / 0.55× of the real HSR per unit delta.

Neutral defaults keep the code equal to the official client for the real robot. The golden test checks
that.

**Why hold an effort grasp until 0.9 but release an empty one at 0.5?**
While the hand holds an apple, the policy's gripper column stays near the apple's width (~0.55), above
the 0.5 close threshold; the official rule would drop the apple. An empty grasp (fingers ≤ threshold)
has nothing to keep, so it follows the official rule and the policy can retry.

**Why approach with `NavigateToGrasp` first?**
The training episodes start within reach of the object, and Find leaves the robot 0.9–1.4 m away. The
VLA is the grasp, not the navigation.

**Why retract after a failed VLA pick-up?**
The fingers may be on the object and the arm low over the table. The classic `Grab` that follows in
hybrid mode starts by moving the arm and would collide.

**Why an identity in-hand pose?**
`Place` composes its place pose with the object's pose in the hand. The VLA doesn't measure a grasp
offset, so "object at the gripper" is the honest assumption. The stale map pose would put it in the
wrong place.

**Why one WebSocket per episode?**
Between actions nobody services the server's keepalive pings, so a cached socket is dead by the next
action.

**Why only pick-up?**
The place/hand-over hooks run before their classic navigation to the target location, so the VLA would
act at the wrong place. This needs the phase boundary described in v1's audit.

---

## Caveats for future versions (integration level)

The full list, including the simulator and test-tooling caveats, is in
[VLA_INFERENCE_REPORT_v2.md](./VLA_INFERENCE_REPORT_v2.md) §8. The ones that bite when changing the
code:

1. **The backend reads its params once per process** (provider cache). The VS Code debug launch loads
   none, `rosparam load vla.yaml` resets `policy_url` to 8000, and `strict_mode` isn't in the yaml.
2. **Docker image stale**: it silently serves 6-D without the bind mount. Rebuild it.
3. **Port 8000** is the mobile UI's.
4. **The golden test mirrors `vla.yaml`** on purpose: change both together.
5. **`execution_mode: vla` bypasses the approach guard** (`_VLADelegatedAction`). Add the guard there or
   don't use the mode.
6. **The base backs off blindly** 0.25 m after a VLA success (`force_drive`). The base Twists bypass
   collision avoidance. Keep base motion off on hardware until it has been tested on blocks.
7. **Sim-only values** (`arm_delta_gain`, `arm_hold_threshold`, `grasp_position_threshold`) must be
   re-measured on the real HSR. An empty real grasp reads about −0.8, not −0.07…0.17.
8. **`tue_robocup` has uncommitted runtime changes** (old VLAGrasp path); `grasp_mode` must stay `off`.
9. **The trial kit is outside git**, so `code.diff` per run is the only record of what ran.

---

## TODOs (integration)

1. **Commit and push**:
   - `vla.yaml` and the launch default on the right `hero_bringup` branch (the owner decides which);
   - push `action_server` `feature/vla` and `per-group-mse-vla` `feature/vla_implementation`;
   - open PRs.
2. **Rebuild** `smolvla-policy-server` (bakes the 8-D default and the validated server). Settle the
   policy port (8001 in sim) in `vla.yaml` or in the docs.
3. **Completion**: calibrate `grasp_position_threshold` (~0.3) and add a post-lift confirmation.
4. **Stop and cancel**:
   - a per-tick stop flag in the rollout (zero Twist, hold the arm);
   - make `PickUp._cancel` preempt `NavigateToGrasp` and the rollout;
   - fix `self._fsm.is_running` (bound method, always truthy).
5. **Approach guard for full `vla` mode**, or drop that mode for pick-up.
6. **Phase boundary for place / hand-over** (v1 audit), then re-enable them.
7. **Refactor targets from v1**, still open: `ObservationAdapter`, `PolicyClient`, `ActionAdapter`,
   `CompletionPolicy` and `RolloutRunner`; a versioned policy protocol validated at connect; remove
   `maybe_run_vla_manipulation` and the in-process local path.
8. **Fill `raw_sentence`** from the NLP pipeline, or keep the templates and document it.
9. **Real HSR**:
   - re-measure the arm response, the gripper readings and the head tilt sign;
   - keep base and head off;
   - dry-run with the e-stop at hand;
   - decide on `start_arm_pose`: current `[0.2, 0, 0, −1.57, 0]` vs the training median
     `[0.02, −0.29, 0.02, −1.73, −0.04]`.
10. **Version the trial kit** (`~/vla/compare`): move it into a repo, e.g. `per-group-mse-vla/eval/sim`
    or `action_server/tools`.

Evaluation TODOs (base drift, re-run batches, base frame check, offline A/B, questions for the
checkpoint author) are in the inference report v2 §9.

---

## Superseded statements in v1 (VLA_INTEGRATION_REPORT.md)

- "Flow of commands" step 7: `arm._send_joint_trajectory` / `gripper.send_goal(open/close)` per prefix
  is replaced by the 10 Hz stream described above. `_send_joint_trajectory` is now only used for the
  start, lift and retract poses.
- "Head and gripper last-row application is intentional": the gripper is now per row (hybrid). The head
  is still once per chunk, but relative.
- "Layer 3 code path" table: `_play_arm`, `_apply_base` via `force_drive`, and
  `head._setHeadReferenceGoal(1, …)` with absolute values are all gone; see `HSRActionSink` above.
- "Current checkpoint loader": "The released checkpoint consumes six state values …
  `state_indices: [0, 1, 2, 3, 4, 5]`" is **wrong**: it is 8-D, `0..7`.
- "Docker deployment": `POLICY_STATE_INDICES` "defaults to `0,1,2,3,4,5`" is the stale image. The repo
  default is now `0..7`, and `-p 8000:8000` conflicts with the mobile UI.
- "Plain `hero-free-mode` defaults to classic mode": `action_server.launch` now defaults to `hybrid`
  (uncommitted owner edit).
- "Key ROS parameters": `max_chunks` 20 → 60; `enabled_actions` default pick-up only; many new params
  (table above).
- "Current rollout audit": "hybrid hooks run before classic navigation" is fixed for pick-up
  (`NavigateToGrasp` first). It is still true for place / hand-over, which are disabled.
- "Things to check" §1: base convention = velocity (settled). §3 "calibrate grasp_position_threshold":
  done for sim (0.1), now shown to be too low.
