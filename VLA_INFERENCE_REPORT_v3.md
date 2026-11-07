# VLA Inference Report v3

Date: 2026-11-07. Companion to [VLA_INTEGRATION_REPORT_v3.md](./VLA_INTEGRATION_REPORT_v3.md).
Supersedes [VLA_INFERENCE_REPORT_v2.md](./VLA_INFERENCE_REPORT_v2.md) (v2). Read this file first. Go to v2 for
the 2026-11-03 batches and the base-drift investigation, and to v1 for the original logs.

This report is written so that someone without the earlier conversations can continue the work. It covers:
- where the three questions of the run stand after the v3 cleanup;
- what changed since v2 and why;
- how it was tested, at code level and in the sim;
- what went wrong, and what remains.

**The three questions of the run**

- (a) Does the per-group-mse SmolVLA checkpoint emit base **velocity** or **pose delta**?
- (b) Are all 11 output DOF used, i.e. produced **and** executed correctly?
- (c) Does the integration work end to end?

**TL;DR**

- **(a) Velocity. Settled** (unchanged since v2). The base frame and sign on HERO are still unchecked; base
  motion stays disabled.
- **(b) All 11 columns are produced. By default 6 are executed (arm 5 + gripper 1), at 10 Hz, exactly like the
  official AIRoA client.**
  - The golden test now matches the official client with the **code defaults**: one gripper threshold, and no
    extra bounds without a robot model.
  - The arm bounds come from the robot's URDF; on the HSR they are the same numbers v2 had hardcoded.
  - The mutation check catches 8/8 injected bugs (v2: 6/6).
- **(c) The mechanics work end to end on a fresh stack. The grasp does not happen yet.**
  - Find → staged → 60-chunk rollout → retract ran with 180 inferences, no transport or controller error, and a
    median latency of 340 ms.
  - With the simulated base **held still** (pinned), the first **valid** trials of this sim were recorded: base
    slide 1.4 cm and 0.7 cm.
  - **The policy reached toward the apple (7–22 cm) but never asked to close the hand.** An A/B with the v2 code
    and all its sim corrections did the same, so this does not come from the cleanup. It is the new main blocker
    (P10).
  - The v3 completion check (lift, then the classic `ActiveGraspDetector`) is verified in isolation in the sim.
    It never fired in a rollout, because nothing was grasped.
- The v2 completion false positives (P2) are addressed by design: lift first, then squeeze. The sim needs
  `grasp_minimum_position: 0.3`, because its empty hand closes only to −0.086. With the real-robot default
  (−0.82) the detector reports an empty sim hand as holding.
- The sim base still drifts on a **fresh** Gazebo start (155 cm in one rollout), so a restart does not fix P1.
  Pinning the base does.

**Plain-language picture**

The model outputs 50 rows of 11 numbers; we play the first 10, one every 0.1 s, then ask again. The arm and head
numbers are small nudges from where the joints are now. The gripper number is a finger position (≈ +1.24 open,
−0.8 closed); below 0.5 the hand grasps.

v3 plays these numbers exactly like the official client and keeps the arm inside the limits written in the
robot's own model. It no longer bends the numbers to make the simulator look like the real robot.

In the simulator the arm now follows the policy reliably. With the robot standing still, the policy reaches toward
the apple but never decides to close the hand, with or without the old simulator tweaks. That is the next thing to
understand.

---

## 0. What changed since v2

| Topic | v2 (2026-11-03) | v3 (now) |
|---|---|---|
| Arm bounds | hardcoded HSR limits and `max_arm_delta` | from `/hero/robot_description`: position limits, and velocity × `arm_lead_time` per row (the same values) |
| Sim arm corrections | `arm_delta_gain [1,1,2.5,2,1]`, `arm_hold_threshold` wrist 0.005 | removed |
| Gripper | official rule + latch to 0.9 + empty-grasp release | official rule only |
| Completion | fingers above 0.1 for 2 chunks (false positives: P2) | effort grasp ≥ 1 s → lift 0.1 m → `ActiveGraspDetector` (robot_smach_states); not holding → open and continue |
| Cancel / `vla` mode / port | none / skipped Find and the approach / 8000 | stops the rollout and never starts the classic pick-up / hybrid without fallback / 8001 |
| Policy image on the GPU | not checked (laptop needed a reboot) | `:latest` = `:2026-11-07` started on CUDA, `state_dim=8` |
| Tests | 49 VLA tests, mutation 6/6 | 56 VLA tests, mutation 8/8 |
| Sim evaluation | 2 batches, all with the base drifting (invalid) | fresh stack; base pinned → first valid trials; an A/B against the v2 code; an isolated grasp-check probe |
| Answer to (c) | "works end to end; grasp quality not measurable until the sim base holds still" | "works end to end mechanically; with the base still, the policy doesn't close the hand (also with v2 code)" |

Code revisions this report is based on:

| Repo | Branch | State |
|---|---|---|
| `action_server` | `feature/vla` | `3744c4f` (code) + the reports commit, on top of `87f5f49`; pushed |
| `hero_bringup` | `feature/LUT_for_configuring_yolo_filter` (unrelated branch) | `a779184` (`vla.yaml` + launch files), on top of `6003046`; pushed |
| `per-group-mse-vla` | `feature/vla_implementation` | unchanged: `4fde5ee`, 3 commits not pushed (no write access; refused again on 2026-11-07) |

The A/B trial ran the v2 code (`87f5f49`) from a temporary worktree with the v2 `vla.yaml` (§4.1).

---

## 1. Setup

| Item | Value |
|---|---|
| Checkpoint | `PauMontagut/per-group-mse-smolvla`, snapshot `cb72ca6a3a58e724a3ca8579bea3811f1810be96` (unchanged) |
| Policy server | container `smolvla-compare` from `smolvla-policy-server:latest` = `:2026-11-07` (`8d5b0336a3e7`), started by `~/vla/compare/start_policy_server.sh` (bind-mounts `policy_server.py`, sets `POLICY_STATE_INDICES=0..7`), host port 8001. Startup log: `Policy server ready on cuda with policy=smolvla state_dim=8 action_dim=11` |
| Stack | fresh `hero-start` + `hero-free-mode` on 2026-11-07 (Gazebo had been shut down) |
| Client | `SmolVLAWebSocketBackend` (v3), params from `hero_bringup/parameters/action_server/vla.yaml` (v3, §1.1) |
| World | Gazebo `impuls`, HERO, apples `apple-1` (4.0, 1.2, 0.785) and `apple-2` (4.2, 1.3, 0.785) on `dinner_table` |
| Task | "pick up the apple from the dinner table": Find → PickUp, staged (§4.1) |
| Laptop | RTX 3050 4 GB. Idle before the stack: 24 MiB used |

### 1.1 Client parameters (`vla.yaml` v3, `a779184`)

```yaml
execution_mode: hybrid          # trials set strict_mode true; 'vla' now means the same (no classic fallback)
enabled_actions: [pick-up]
local_backend_class: action_server.vla.backends:SmolVLAWebSocketBackend
policy_url: ws://127.0.0.1:8001 # 8000 is the mobile UI
policy_fps: 10.0                # rows per second
chunk_prefix_steps: 10          # rows played per inference (1 s)
max_chunks: 60                  # 60 s of policy motion
start_arm_pose: [0.2, 0.0, 0.0, -1.57, 0.0]
start_head_pose: [0.0, -0.6]
arm_lead_time: 0.2              # arm bounds come from robot_description (limits, velocity x lead)
gripper_close_threshold: 0.5    # the official rule; no latch
gripper_max_torque: 0.26        # = 0.018 Nm, the official grasp effort
enable_head_motion: false
enable_base_motion: false       # velocity only
grasp_settle_time: 1.0          # effort grasp age before the check
post_grasp_lift: 0.1            # lift before the check
grasp_minimum_position: 0.3     # SIM ONLY: ActiveGraspDetector minimum_position (real HERO: unset -> -0.82)
```

Removed since v2: `max_arm_delta`, `arm_delta_gain`, `arm_hold_threshold`, `gripper_open_threshold`,
`grasp_position_threshold`, `base_action_mode`, `device`, `action_layout`, `state_indices`, `checkpoint_path`.

---

## 2. The three questions

### 2.1 (a) Velocity or pose delta: **velocity**

Unchanged since v2 §2.1: the official client publishes columns 8–10 unscaled as a `Twist`, and the normalisation
statistics have a joystick profile. v3 removed the `pose_delta` interpretation from the code.

**Still open:** the base frame and sign on HERO. This needs a still base, which pinning now provides; see TODO 6.

### 2.2 (b) Are all 11 DOF produced and executed?

| Columns | Meaning (verified) | Produced | Executed by default | How execution is verified |
|---|---|---|---|---|
| 0–4 arm | delta w.r.t. the **measured** joints | yes | **yes**, 10 Hz, bounded by the URDF | golden test: equal to the official client (atol 1e-9) with the code defaults; deployed deviations only where documented; 8/8 mutations caught |
| 5 gripper | **absolute** `hand_motor_joint` [rad] | yes | **yes**, official hybrid rule | golden test (grasp/position decision per tick, 0.018 Nm) |
| 6–7 head | delta w.r.t. the measured head joints | yes | no (disabled by design) | golden test with `enable_head_motion` |
| 8–10 base | body-frame velocity | yes | no (disabled by design) | golden test with `enable_base_motion` |

Answer: **11/11 produced. 6/11 executed by default, by design. All 11 command paths are verified at code level
against the official client.**

In the sim the arm follows the stream. In `v21pin-1`, `arm_lift` went 0.20 → 0.66 m and `arm_flex` 0 → −2.06 rad,
and the URDF bounds held, with no "No limits" warning. The gripper never received a grasp, because the column
never dropped below 0.5 (§4.5). So the grasp branch has run in the sim (v2) but not in a v3 rollout.

### 2.3 (c) Does the integration work end to end?

- **Mechanically: yes**, on a fresh stack:
  - Find, staging, start pose, 60-chunk rollout with fresh head and hand images and the 8-D state, retract after
    the failure;
  - 180 queries over 3 rollouts with no transport, socket or controller error;
  - median latency 340 ms (first 1266 ms, p90 356 ms).
- **Grasp: not yet.** With the base held still, the policy reaches toward the apple and never closes the hand.
  The v2 code with its sim corrections behaves the same (§4.3, §4.5).
- **Completion check:** correct in an isolated probe (§4.4). It was not triggered in a rollout (no grasp).

---

## 3. Code-level verification

### 3.1 Golden test against the official client

`action_server/action_server/test/test_vla_airoa_golden.py`. The reference is the same transcription of
`hsr_policy.py` as in v2 (10 Hz, `adopted_action_chunks 10`, `gripper_mode hybrid`), with a joint state that drifts
a few mrad per tick.

| Test | Asserts |
|---|---|
| `test_reference_equivalent_parameters_command_exactly_the_reference` | With the reference lead 0.05 s, 0.018 Nm, base on, no clamps and **no robot model** (so no URDF bounds), every tick's arm target, `time_from_start`, Twist and gripper decision equals the reference (atol 1e-9). Only documented extra: the base is braked after the prefix. v2 needed `max_arm_delta` 1e3, `gripper_open_threshold` 0.5 and patched class limits for this; v3 needs none |
| `test_head_goal_is_the_reference_head_target_of_the_last_tick` | head goal = the reference target of the last tick, tilt negated for HERO |
| `test_deployed_parameters_deviate_only_where_documented` | With the deployed `vla.yaml` and an HSR URDF the sink differs only in: lead 0.2 s; the velocity cap (tick 1: flex 1.2 rad/s × 0.2 s); the URDF limits (ticks 2, 3); torque rounding; no head/base. The v2 deviations "arm_delta_gain" and "grasp held in the release band" are gone |
| `test_deployed_mirror_matches_vla_yaml` | the test's copy of the deployed sink params equals `hero_bringup/.../vla.yaml` |

### 3.2 Mutation check (2026-11-07)

Eight bugs injected one at a time, in memory; the golden test must fail on each. The script is in Appendix A. It
replaces v2's Appendix A, whose strings no longer match the code.

| Mutation | Result |
|---|---|
| anchor arm targets on the chunk-start state | caught (19) |
| cumulative sum of the deltas | caught (18) |
| deltas played as absolute targets (v1's bug) | caught (21) |
| gripper inverted, close above 0.5 (v1's bug) | caught (20) |
| base read as pose deltas | caught (10) |
| rows at 30 Hz | caught (2) |
| URDF velocity bound ignored (**new**) | caught (1) |
| URDF position limits ignored (**new**) | caught (2) |

### 3.3 Unit tests

| Suite | Count | Covers |
|---|---|---|
| `test_vla_action_sink.py` | 28 | sink (12): live-state delta, URDF limits, velocity cap, unbounded without a description, gripper position/grasp/re-open, base clamp and brake, base off by default, head delta, 6-wide chunk, stop event. Completion (5): lift then check, no check before settle, failed check opens and does not re-check, `grasp_minimum_position` passed, detector default. Rollout (11): debug topic, retract rules, cancel without retract, cancel reset, log line |
| `test_vla_backend_utils.py` | 10 | `apply_relative`, `joint_limits`, `gripper_action`, `query_debug_message` |
| `test_vla_airoa_golden.py` | 4 | §3.1 |
| `test_vla_pick_up.py` | 8 | VLA after `arrived`; unreachable / no arm / failed VLA / VLA disabled → classic; cancel during the approach or the VLA → no classic; FSM preempted only while it runs |
| `test_vla_executor.py` | 6 | hybrid falls back; `strict_mode` and `vla` mode don't; classic doesn't run the VLA; `cancel_vla` reaches only that robot's backend; cancel without a backend is harmless |
| `per-group-mse-vla/inference/test_policy_server.py` | 11 | unchanged since v2 |

Results on 2026-11-07:
- **pytest: 57 passed** (56 VLA + `test_import`).
- **nose over `test/`: 58 run, 1 failure.** The failure is `test_grammar_test`, which needs a roscore and fails the
  same way on `87f5f49`.

### 3.4 URDF limits on the real model

`joint_limits` on `/opt/ros/noetic/share/hsrb_description/robots/hsrb4s.urdf`:
- lower `[0.0, -2.62, -2.09, -1.92, -1.92]`;
- upper `[0.69, 0.0, 3.84, 1.22, 3.67]`;
- velocity × 0.2 s = `[0.04, 0.24, 0.4, 0.3, 0.3]`.

These are exactly v2's hardcoded `ARM_LOWER/UPPER` and `max_arm_delta`. In the sim runs the sink read them from
the live `/hero/robot_description`.

---

## 4. Simulation trials (2026-11-07)

### 4.1 Trial kit and what was added

The kit (`~/vla/compare`, not under git) is unchanged; see v2 §4.1 for `trial.py`, `run_trials.sh`,
`score_runs.py`, `report.py` and the scoring rubric (4 = lifted > 5 cm and held ≥ 2 s; 3 = fingers within 5 cm or
apple moved > 2 cm; 2 = within 15 cm; 1 = otherwise). Added for this session, outside the kit:

- **Base pin** (Appendix B, `pin_base.py`):
  - waits for the first policy query on `/hero/action_server/vla/debug/query`;
  - takes the Gazebo pose of `hsrb` at that moment;
  - re-places the robot there at 10 Hz until no query arrived for 15 s.

  Run it in the background next to `run_trials.sh`. This is v2 P1 fix step 2.
- **Grasp-check probe** (Appendix C, `probe_grasp_check.py`): closes the empty hand in the air and runs
  `ActiveGraspDetector` with the sim value and with the detector default.
- **A/B with the v2 code**, without touching any repo:
  1. `git worktree add --detach <tmp> 87f5f49`;
  2. put `<tmp>/action_server/src` first on `PYTHONPATH`. The catkin devel shim only redirects to the workspace
     source when `action_server` is found in devel, so the worktree wins;
  3. run `trial.main()` with `trial.VLA_YAML` pointed at `git -C hero_bringup show HEAD:…/vla.yaml`;
  4. afterwards delete the old keys from the param server (`rosparam load` never deletes) and reload the v3
     `vla.yaml`.

### 4.2 Chronology (local time)

| Run / step | When | What happened | Lesson |
|---|---|---|---|
| stack start | 20:35–20:38 | `hero-start`, `hero-free-mode`, policy container ready on CUDA (`state_dim=8`); the v3 params are on the server with no stale keys | the rebuilt image starts on the GPU |
| grasp probe | ~20:38 | empty hand closed in the air: −0.086; `grasp_minimum_position` 0.3 → `cannot_determine`, default −0.82 → `true` | §4.4 |
| `20261107-203959_vla_v21-smoke1` | 20:40 | Find failed: the inspect view had the apples at the edge of the frame (a coke can in view), "no objects found". The base slid 156 cm without any VLA motion | P4 again; P1 on a fresh start |
| `20261107-204131_vla_v21-1` | 20:41 | v3, base drifting: 60 chunks, closest 15.1 cm, the gripper column never below 0.84, base slide 155 cm | invalid (P1) |
| `20261107-204528_vla_v21pin-1` | 20:45 | v3, base pinned (973 re-placements): 60 chunks, closest 22.1 cm, column never below 0.70, slide 1.4 cm | **first valid trial**; no close |
| `20261107-204941_vla_headpin-1` | 20:50 | **v2 code + v2 `vla.yaml`**, base pinned (981 re-placements): closest 7.0 cm, apple pushed 66.5 cm, column never below 0.68, slide 0.7 cm | valid; the v2 code doesn't close either |

### 4.3 Results

| Run | Code | Base | Valid (slide < 2 cm) | Score | Closest | Apple moved | Gripper column min | Grasps | Queries | Target |
|---|---|---|---|---|---|---|---|---|---|---|
| `v21-1` | v3 | drifting | no (155.4 cm) | 1 | 15.1 cm | 0 | 0.84 | 0 | 60 | apple-1 |
| `v21pin-1` | v3 | pinned | **yes (1.4 cm)** | 1 | 22.1 cm | 0 | 0.70 | 0 | 60 | apple-2 |
| `headpin-1` | v2 | pinned | **yes (0.7 cm)** | 3 | 7.0 cm | 66.5 cm | 0.68 | 0 | 60 | apple-1 |

All three reported "VLA reached max_chunks without a completion signal" (correct: nothing was held). The
retract after each failure ran without a warning. Bags, `truth.csv`, `meta.json` and `code.diff` are in
`~/vla/compare/runs/` (about 50 MB each).

### 4.4 Grasp-check probe (the v3 completion check, in isolation)

| Step | `hand_motor_joint` | Result |
|---|---|---|
| open | 0.963 | — |
| closed on nothing with the VLA effort (0.26), after 2 s | −0.086 | — |
| `ActiveGraspDetector(minimum_position=0.3)` | −0.075 | `cannot_determine` → not holding (**correct**) |
| `ActiveGraspDetector()` (default −0.82, real HERO) | −0.087 | `true` → **a false positive in the sim** |

The squeeze returned at once in both cases. So:
- the sim needs `grasp_minimum_position`;
- the real robot needs the default;
- the check itself works on HERO's `gripper_position_detector`.

Not measured yet: the table edge and an apple in the hand (TODO 3).

### 4.5 What the policy does with the base still

The same in all three runs, v3 and v2 code alike:
- `arm_lift` rises to ~0.63–0.68 m (near the top, 0.69);
- `arm_flex` goes 0 → about −2.05…−2.17 rad (well past horizontal, toward the table);
- `wrist_flex` rises steadily from −1.57 to between +0.06 and +0.42, on small positive deltas every chunk;
- the gripper column stays at 0.68–1.24 throughout: pre-shaping only, never a grasp.

The fingers get within 7–22 cm of the apple. The v2 code pushed the apple 66 cm, the v3 code didn't touch it; with
n = 1, different target apples and fresh policy noise per query (C8), that difference says nothing.

In the v2 batches, run with the base **drifting**, the policy did close the hand (up to 9 grasp goals in one run).
Those closes may have been driven by the moving scene; they are not evidence that the policy grasps.

### 4.6 Base drift on a fresh start

- `smoke1` slid 156 cm and `v21-1` 155 cm in the pick-up window with no base command, right after a fresh
  `hero-start`. **v2 P1 fix step 1 (restart Gazebo) does not fix it**: the drift is there from the start; the
  stage teleport may still make it worse.
- **Pinning works:** 1.4 cm and 0.7 cm with ~975 re-placements per rollout, and no visible effect on the arm
  stream.

---

## 5. Analysis

1. **The cleanup did not change what is commanded.** The golden test is exact with the defaults, the URDF gives the
   old bounds, and the A/B shows the same policy behaviour with and without the v2 sim corrections. What it
   removed was sim tuning that, with the base still, made no visible difference.
2. **The plumbing is reliable on a fresh stack:**
   - 180 queries without a failure;
   - one socket per episode;
   - the retract after each failure.
3. **The sim can now measure grasp quality** (pinned base), and the first measurements say that **the policy does
   not close the hand** here. That is the question to answer before running batches (P10).
4. **The v3 completion check is sound and doesn't depend on the sim's quirks:**
   - lifting removes the table-edge case;
   - the squeeze is the robot's existing check;
   - the only sim-specific number is the empty-closed reading (`grasp_minimum_position`), now measured once.
5. **The evidence is thin**: n = 1 per condition, and fresh noise per query. Use batches and the decision rule of
   v2 §2.3 once the policy grasps.

---

## 6. Problems and proposed fixes

### P1. The simulated base drifts by itself (blocker for unpinned trials)

- **v3 finding:** it drifts from a fresh start (155 cm per rollout). Step 1 of v2's fix (restart) does not help.
- **Done:** step 2, pinning, as a script (Appendix B); it gives valid trials.
- **Fix, remaining:**
  - put the pin into `trial.py` as an option (`--pin-base`);
  - add the validity gate `base_slide_cm < 2` to `score_runs.py`;
  - step 3: investigate the hsrb Gazebo base plugin, `omni_base_controller` and the PID gains. This also affects
    the classic and unstaged runs.

### P2. False-positive grasp completion: addressed by design

- **v2:** fingers above 0.1 for 2 chunks gave 2 false positives in 11 rollouts (fingers stalled on the table edge
  or the apple's edge).
- **v3:** effort grasp ≥ 1 s → lift → `ActiveGraspDetector`:
  - off the table, empty fingers close further;
  - below `grasp_minimum_position` the hand counts as empty.
- **Remaining:**
  - calibrate `grasp_minimum_position` on the table edge and on an apple (the empty-in-air reading is done:
    −0.086);
  - verify a true positive in a rollout once the policy grasps.

### P3. The unstaged approach fails in this sim

Unchanged since v2: ED places the apples ~0.2 m off and their inflation blocks the approach.

### P4. Find is flaky

Seen again: 1 of 4 Finds failed, because the inspect view had the apples at the edge of the frame. The runner's
retry covers it. The fix is unchanged since v2 (timeouts; and look at the inspect pose).

### P5. Localization after a teleport

Done in the kit since v2. On 2026-11-07 the seeding agreed within 0.02–0.04 m, 0–2°.

### P6. The classic baseline is broken in this scene

Unchanged since v2 (`http://:8080/black_cabinet1.stl is invalid`).

### P7. The Docker image: resolved in part

- **Done:** `:latest` = `:2026-11-07` starts on CUDA with `state_dim=8` and served 180 queries.
- **Left:**
  - check the baked server without the kit's bind mount (`smoke_test.py`; 6 indices must be refused);
  - pin the Dockerfile dependencies;
  - a rebuild from GitHub lacks the 3 unpushed commits.

### P8. Port 8000 conflict: resolved

`policy_url` defaults to 8001 in the code and in `vla.yaml`.

### P9. Policy behaviour in sim

Superseded by P10. With the base still, the "dragging on the table" and "knocking over" of v2 were not the issue;
the policy does not grasp at all.

### P10. The policy does not close the hand with the base still (new, main blocker)

- **Symptom:** §4.5. In 2 valid pinned runs (v3 and v2 code) and 1 drifting v3 run, the gripper column never drops
  below 0.68. The arm reaches high and forward, and the fingers end 7–22 cm from the apple.
- **Not the cause:** the v3 cleanup (A/B), the transport, the gripper rule (the column never asked for a close),
  the state layout (validated 8-D).
- **Candidates, cheapest first.** The first three are param changes only:
  1. **start pose:** `start_arm_pose` `[0.2, 0, 0, −1.57, 0]` vs the training median
     `[0.02, −0.29, 0.02, −1.73, −0.04]`;
  2. **prompt:** `instruction_templates` `"pick up the {object}"` vs the dataset phrasing ("Grasp the apple.");
  3. **head tilt:** `start_head_pose` (training mean −0.61; the view decides what the policy sees);
  4. **sim-to-real images:** rendered Gazebo images vs the real AIRoA images. Check with the fixed-noise offline
     A/B (v2 plan step 7), and ask the checkpoint author.
- **Fix:** run 3–5 pinned trials per variant (1–3), and look at the gripper column first. A variant that closes the
  hand then also exercises the v3 completion path.

### P11. Perception environment (new, pre-existing)

- **Symptoms:**
  - free mode logs `Could not instantiate ObjectRecognizer(...)`: a missing
    `/home/amigo/MEGA/data/impuls/models/image_recognition_tensorflow/output_labels.txt`;
  - the trial logs `/hero/ed/classify not connected timely`.
- Both were already present in the 2026-11-03 free-mode log. Find still worked via segmentation (3 of 4 attempts).
- **Fix:** restore the model files, or confirm that Find does not need `classify`.

---

## 7. Testing procedure we followed (reproducible)

1. **Unit and golden tests**, no ROS master:
   ```bash
   source ~/ros/noetic/system/devel/setup.bash
   cd ~/ros/noetic/repos/github.com/tue-robotics/action_server/action_server
   PYTHONPATH=src:$PYTHONPATH python3 -m pytest -q -p no:warnings test/test_vla_*.py test/test_import.py  # 57 passed
   PYTHONPATH=src:$PYTHONPATH python3 -m nose test/          # 58 run; test_grammar_test needs a roscore
   ```
2. **Mutation check** (Appendix A): all 8 mutations must be caught.
3. **Stack**, once per session:
   ```bash
   hero-start                                  # Gazebo + robot, impuls world (also the mobile UI on :8000)
   hero-free-mode                              # navigation, ED, perception, loads vla.yaml (policy_url :8001)
   ~/vla/compare/start_policy_server.sh        # smolvla-compare on :8001; log must say state_dim=8
   rosparam get /hero/action_server/vla        # v3 keys only; delete stale keys left by an older vla.yaml
   ```
4. **Grasp-check probe** (Appendix C), once per stack: the empty hand must give `cannot_determine` with the sim
   value.
5. **Trials, always with the base pinned**:
   ```bash
   python3 pin_base.py &                       # Appendix B; exits on its own after the rollout
   cd ~/vla/compare && ./run_trials.sh 1 NAME --staged
   ```
   Start a new pin per trial (it exits after each rollout). Watch the `[VLA] chunk N … gripper=[..]` lines: the
   gripper column has to go below 0.5 before anything can be grasped. Look for `[VLA] Grasp check after the lift:`.
6. **Read every run with ground truth.** Check `base_slide_cm` < 2, `vla_ran` and `aborted` before reading the
   score.
7. **A/B against older code**: §4.1 (worktree on `PYTHONPATH` + `trial.VLA_YAML`). Clean the param server
   afterwards.

---

## 8. Caveats and lessons for the next version

**Machine and GPU** (unchanged since v2)
- **C1.** Suspend/resume breaks CUDA; only a reboot fixes it.
- **C2.** No second copy of the model on the CPU next to Gazebo.
- **C3.** The 4 GB GPU is shared (policy ~1.1 GB).
- **C4.** Disk: each trial bag is ~50 MB. Never bag raw images or the unthrottled `/gazebo/model_states`.

**Policy server**
- **C5.** Containers from the old `:2026-10-30` image without the bind mount serve a 6-D state. Always look for
  `state_dim=8`.
- **C6.** Port 8000 is the mobile UI's; 8001 is now the default everywhere.
- **C7.** The checkpoint's `config.json` says state dim 6; that is stale (8-D).
- **C8.** SmolVLA samples fresh noise on every call. Single runs say little; use batches or fixed noise.

**ROS params and processes**
- **C9.** The VS Code launch loads no params; `hero-free-mode` does. The backend reads them once per process.
- **C10.** `strict_mode` and `publish_debug` are not in `vla.yaml`. (v2's "`rosparam load` resets `policy_url` to
  8000" is gone: the yaml says 8001.)
- **C11.** ~~Full `vla` mode skips the approach.~~ Fixed in v3: `vla` = hybrid without fallback.
- **C12.** Publishing on `/hero/initialpose` also teleports the Gazebo robot.
- **C13.** The time bases differ (rospy wall/sim, docker UTC, bags wall).
- **C26 (new).** `rosparam load` never deletes keys. Loading an older `vla.yaml` leaves its removed params on the
  server; delete them afterwards.
- **C27 (new).** `urdf_parser_py` prints "Unknown tag" warnings once per process; harmless.

**Simulator**
- **C14.** The base drifts with no commands, **also on a fresh start**. Pin it in every trial.
- **C15.** The sim `wrist_flex` PID has no integral term, and `arm_roll` / `wrist_flex` move less per unit delta
  than the real HSR. v3 no longer compensates in code; the fix belongs in a sim gain override.
- **C16.** The empty-grasp reading differs: −0.086 in the sim (measured in the air), about −0.8 on the real HSR.
  Hence `grasp_minimum_position` 0.3 (SIM ONLY).
- **C17.** The ED apple offset (P3), Find flakiness (P4), localization lag (P5), the classic grasp (P6), and the
  object recognizer files (P11).
- **C18.** A gripper close threshold of 0.7 closes during the pre-shaping; keep the official 0.5.

**Trial tooling**
- **C19–C22.** Unchanged since v2: SIGINT and blocked service calls, thread-unsafe service proxies,
  `pkill -f` matching itself, localization aborts during the rollout.
- **C28 (new).** The pin and probe scripts exist only in the appendices of this report until they are copied into
  the kit.
- **C29 (new).** For an A/B against older code, verify which `action_server` the trial imports
  (`python3 -c "import action_server.vla.backends as b; print(b.__file__)"` with the same `PYTHONPATH`).

**Repos**
- **C23.** `hero_bringup`'s `vla.yaml` lives on the unrelated branch `feature/LUT_for_configuring_yolo_filter`.
  The only sim-only value left in it is `grasp_minimum_position` (and `arm_lead_time`, measured in the sim).
- **C24.** `tue_robocup` `feat/vla-grasp` carries the old VLAGrasp path; v3 doesn't use it.
- **C25.** `action_server/vla_last_report` holds the owner's notes. Keep it untracked.

---

## 9. TODOs (priority order)

1. **Find out why the policy doesn't grasp** (P10). Pinned trials, 3–5 per variant:
   - `start_arm_pose` = the training median;
   - the prompt "Grasp the apple.";
   - `start_head_pose`.

   Then the fixed-noise offline A/B, and the questions for the author (7).
2. **Make every trial valid by default** (P1):
   - `--pin-base` in `trial.py` (Appendix B);
   - the `base_slide_cm < 2` gate in `score_runs.py`;
   - `~/vla/compare` under git.
3. **Finish verifying v3 in the sim:**
   - the full completion path, once a variant grasps (`Grasp check after the lift: true` → success → carrying
     pose);
   - calibrate `grasp_minimum_position` on the table edge and on an apple;
   - a cancel through the action server.
4. **Batch**: 10 pinned trials with the best variant, read with v2's decision rule (≥ 3/10 lifts and mean ≥ 2.5 =
   works in sim; ≤ 1.5 = doesn't; in between, 10 more).
5. **Commanded vs achieved per group** from the bags (v2 TODO 4).
6. **Base executor check** on HERO (v2 TODO 5), now possible with the base pinned between the pushes.
7. **Ask the checkpoint author:** dataset fps, client settings, BGR/RGB, head camera, arm home pose, trial timer,
   and **whether the policy grasps in their simulator**.
8. **Unstaged flow** after P3 and P4, then GPSR in `vla` mode.
9. **Classic baseline** (P6), then classic vs VLA with `report.py`.
10. **Housekeeping:**
    - decide the `hero_bringup` branch, PRs (v3 is committed and pushed);
    - `per-group-mse-vla` write access;
    - the baked-image check and dependency pinning (P7);
    - the perception model files (P11);
    - the sim parameter override for `grasp_minimum_position`.

### v2 TODOs: status

| v2 TODO (§9) | Status now |
|---|---|
| 1. Neutralize the base drift | restart: done, doesn't help. Pin: done as a script, gives valid trials. Kit integration, validity gate, root cause: open |
| 2. Calibrate the threshold, post-lift confirmation | replaced by lift + `ActiveGraspDetector`; calibration in part (empty in the air) |
| 3. Re-run 10 staged trials with a still base | 2 valid trials; the batch waits for P10 |
| 4. Commanded vs achieved | open |
| 5. Base executor check | open (now possible) |
| 6. Fixed-noise offline A/B | open, now TODO 1 |
| 7. Ask the checkpoint author | open, one more question |
| 8. Unstaged flow | open |
| 9. Classic baseline | open |
| 10. Housekeeping | GPU start-up check: done in part; port: done; the rest open |

Integration-level TODOs (commits, sim override, `robot_skills` upstream, phase boundary, real robot) are in
[VLA_INTEGRATION_REPORT_v3.md](./VLA_INTEGRATION_REPORT_v3.md).

---

## 10. Superseded statements in v2

- TL;DR (b): "golden test … with reference-equivalent params". The defaults are now reference-equivalent.
- TL;DR, §5.4, P2: "the completion signal still gives false positives; about 0.3 is the likely fix". Replaced by
  lift + `ActiveGraspDetector` with `grasp_minimum_position` 0.3.
- §1.1: the parameter block (removed params; `policy_url` 8001).
- §3.1: "documented deviations … `arm_delta_gain` … the grasp held in the release band". Gone.
- §3.2 and Appendix A: 6 mutations whose strings no longer match. See Appendix A (8 mutations).
- §3.3: the test counts.
- P1 "Restart Gazebo … If a fresh start is clean, it is a degraded state". A fresh start is not clean.
- P7 "Still to check: a start of the new image on the GPU". Done (with the kit's bind mount).
- P8: resolved.
- P9: superseded by P10.
- C11: fixed. C10 (`policy_url` reset to 8000): gone.
- C15, C23: the sim corrections are no longer in code or `vla.yaml`.

---

## Appendix A. Golden-test mutation script

Run it from `action_server/action_server` with the workspace sourced. It execs mutated source in memory only; no
file is changed. Result on 2026-11-07: all 8 caught (§3.2).

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

If a string replacement no longer matches after a refactor, the `assert` names the mutation; update its `old` text.

## Appendix B. Base pin (`pin_base.py`)

Run it in the background, with the workspace sourced, before each trial. It exits on its own about 15 s after the
last policy query.

```python
#!/usr/bin/env python3
"""Hold the simulated HERO base still during one VLA rollout (inference report v2, P1 fix step 2).

The Gazebo base drifts by itself (~2-3 cm/s, no commands), so a staged trial measures a moving target. This
waits for the first policy query on the VLA debug topic (trial.py turns publish_debug on), takes the Gazebo pose
of the robot at that moment and re-places the robot there at 10 Hz until no query arrived for 15 s, then exits.
A real HSR's base does not drift, so this is the fair condition for judging the grasp.
"""
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from std_msgs.msg import String

MODEL = "hsrb"
last_query = []

rospy.init_node("pin_base")
rospy.Subscriber("/hero/action_server/vla/debug/query", String, lambda _msg: last_query.append(rospy.Time.now()))
rospy.wait_for_service("/gazebo/set_model_state")
get = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
put = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

while not last_query and not rospy.is_shutdown():
    rospy.sleep(0.05)
pose = get(MODEL, "world").pose
rospy.loginfo("[PIN] holding the base at (%.3f, %.3f) from the first policy query", pose.position.x, pose.position.y)
rate = rospy.Rate(10)
pins = 0
while not rospy.is_shutdown() and (rospy.Time.now() - last_query[-1]).to_sec() < 15.0:
    put(ModelState(model_name=MODEL, pose=pose, reference_frame="world"))
    pins += 1
    rate.sleep()
rospy.loginfo("[PIN] released after %d re-placements, %d policy queries", pins, len(last_query))
```

## Appendix C. Grasp-check probe (`probe_grasp_check.py`)

Run it with the stack up and the arm free (it only moves the gripper). Expected in the sim: `cannot_determine` with
0.3, and `true` with the default.

```python
#!/usr/bin/env python3
"""Isolated check of the VLA completion check in the sim: an empty hand, closed in the air, must not pass.

Closes the empty gripper with the VLA's grasp effort, reads hand_motor_joint, then runs the classic
ActiveGraspDetector with the sim grasp_minimum_position (0.3) and with the detector default (-0.82, real HERO).
"""
import time

import rospy
from robot_skills import get_robot
from robot_smach_states.manipulation.active_grasp_detector import ActiveGraspDetector
from robot_smach_states.util.designators import Designator


def hand(robot):
    return robot.get_joint_states()["hand_motor_joint"]


rospy.init_node("probe_grasp_check")
robot = get_robot("hero")
arm = robot.get_arm()

arm.gripper.send_goal("open", timeout=5.0)
print("open: hand_motor_joint = %.3f" % hand(robot))
arm.gripper.send_goal("close", timeout=0.0, max_torque=0.26)  # the VLA sink's effort grasp
rospy.sleep(2.0)
print("closed on nothing (2 s): hand_motor_joint = %.3f" % hand(robot))

for label, kwargs in (("sim grasp_minimum_position 0.3", {"minimum_position": 0.3}),
                      ("detector default -0.82", {})):
    start = time.time()
    outcome = ActiveGraspDetector(robot, Designator(arm), **kwargs).execute()
    print("%-32s -> %-16s (%.1f s, hand %.3f)" % (label, outcome, time.time() - start, hand(robot)))

arm.gripper.send_goal("open", timeout=5.0)
```

## Appendix D. Base drift test

Unchanged; see [VLA_INFERENCE_REPORT_v2.md](./VLA_INFERENCE_REPORT_v2.md) Appendix B.
