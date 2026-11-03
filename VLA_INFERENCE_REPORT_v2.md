# VLA Inference Report v2

Date: 2026-11-03. Companion to [VLA_INTEGRATION_REPORT_v2.md](./VLA_INTEGRATION_REPORT_v2.md).
Supersedes [VLA_INFERENCE_REPORT.md](./VLA_INFERENCE_REPORT.md) (v1: the 2026-10-09 runs plus its §7
follow-up of 2026-10-30). Read this file first. Go to v1 only for the original logs.

This report is written so that someone without the earlier conversations can continue the work. It
covers four things:
- where the three questions of the run stand;
- what was changed and why;
- how the changes were tested;
- what went wrong while testing, so the next version can avoid it.

**The three questions of the run**

- (a) Does the per-group-mse SmolVLA checkpoint emit base **velocity** or **pose delta**?
- (b) Are all 11 output DOF used, i.e. produced **and** executed correctly?
- (c) Does the integration work end to end?

**TL;DR**

- **(a) Velocity. Settled.** The official AIRoA client publishes columns 8–10 unscaled as a `Twist`.
  Not yet checked on HERO: the base frame and sign (base motion is disabled by default).
- **(b) All 11 columns are produced. By default 6 are executed (arm 5 + gripper 1), at 10 Hz, exactly
  like the official AIRoA client.**
  - A golden test proves the arm and gripper commands equal a transcription of that client to 1e-9, and
    it catches six deliberately injected client bugs.
  - Head (2) and base (3) are disabled by design. Their code paths are verified against the same
    reference.
- **(c) The pipeline runs end to end in Gazebo:**
  - The full chain runs: Find → (staged) approach → 60-chunk VLA rollout → completion check → retract.
  - It made 693 inferences over 20+ trials without one transport error. Median latency is 338 ms.
  - **Whether the VLA can actually grasp is still unmeasured.** The simulated HERO base drifts by
    itself, 0.24–1.8 m per rollout with no base command at all. The policy reaches for an apple that
    keeps moving away.
  - The two 10-trial batches (0 lifts, mean score 1.6–1.9) are therefore **not valid evidence**
    against the policy.
- Most blockers are now in the **simulator and test tooling**, not in the VLA code:
  - the base drift;
  - ED places apples about 0.2 m off;
  - Find is flaky and can hang;
  - localization lags after a teleport;
  - the classic baseline is broken.
- The completion signal still gives **false positives**: `grasp_position_threshold` 0.1 is too low,
  because the sim fingers stall at 0.14–0.17 on nothing. About 0.3 is the likely fix; it needs a
  decision.

**Plain-language picture**

The model outputs 50 rows of 11 numbers each time it is asked; we play the first 10 rows, one every
0.1 s, then ask again.
- For the arm and head, a row is a small nudge relative to where the joint is *right now*.
- The gripper number is a position: +1.24 open, −0.8 closed on nothing.
- The base numbers are a speed.

v1's client misread almost all of this. It treated nudges as destinations, flipped the gripper, played
the body parts one after another and fed the model 6 of the 8 state numbers it was trained on.

That is fixed, and an offline test now locks it to the official client's behaviour. In the simulator the
arm visibly reaches for the apple and closes the hand. We still can't say whether it would pick the
apple up, because the simulated robot slowly drives itself away during every attempt.

---

## 0. What changed since v1

| Topic | v1 (2026-10-09 runs, §1–6) | v1 §7 (2026-10-30) | v2 (now) |
|---|---|---|---|
| Base convention | velocity "most plausible" | velocity, from the official client | velocity, settled. HERO frame and sign still unverified |
| Arm/head columns | assumed absolute targets | identified as deltas w.r.t. the measured joints | deltas, played as live state + delta every tick; **golden-tested** |
| Gripper | binary, inverted | official "hybrid" mode | hybrid, plus release of an empty grasp; golden-tested |
| Row rate | timed by `max_joint_vel`, groups sequential | 10 Hz, one synchronised loop | 10 Hz; golden-tested (`Rate(10)`) |
| State | 6-D (server) | 8-D | 8-D; the server **refuses** wrong `POLICY_STATE_INDICES` at startup |
| Approach | VLA ran at the find pose | `NavigateToGrasp` first | VLA runs only if `NavigateToGrasp` **arrived** with a resolved arm |
| Failed pick-up | arm left over the table | same | retract (open, lift, start pose) before the fallback |
| Sim arm mismatch | not known | found (wrist sags, roll/flex under-move) | `arm_delta_gain` and `arm_hold_threshold` params (neutral by default) |
| Tests | none | sink and helper unit tests | 49 VLA unit tests + golden test + mutation check; 11 policy-server tests |
| Sim evaluation | 2 GPSR runs, eyeballed | 3 controlled rollouts | recorded trial kit (bag + Gazebo ground truth + 1–4 score), staged mode, 2 batches |
| Answer to (c) | "not working" | "works mechanically, no pick" | "works end to end; grasp quality not measurable until the sim base holds still" |

Code revisions this report is based on (none of them is pushed):

| Repo | Branch | Commits since v1 |
|---|---|---|
| `action_server` | `feature/vla` | `2517279` baseline, `05e4ab4` golden test, `e87e410` debug topic, `02e5c81` sim corrections and guards |
| `per-group-mse-vla` | `feature/vla_implementation` | `8b58877` 8-D state, `b3200ec` state-index validation, `4fde5ee` follow-up |
| `hero_bringup` | `feature/LUT_for_configuring_yolo_filter` (unrelated branch) | **uncommitted**: `parameters/action_server/vla.yaml`, `launch/action_server.launch` |

---

## 1. Setup

| Item | Value |
|---|---|
| Checkpoint | `PauMontagut/per-group-mse-smolvla`, snapshot `cb72ca6a3a58e724a3ca8579bea3811f1810be96`, trained on AIRoA ICRA-2026 pick-up data (`local/task6911-v30`) |
| Real-HSR reference (paper) | 12/20 lifts, mean score 3.5/4 |
| Policy server | Docker container `smolvla-compare` (image `smolvla-policy-server`), host port **8001** → container 8000, GPU, current `policy_server.py` bind-mounted, `POLICY_STATE_INDICES=0,1,2,3,4,5,6,7`. Startup log: `Policy server ready on cuda with policy=smolvla state_dim=8 action_dim=11` |
| Why port 8001 | `hero-start` runs the `tue_mobile_ui` webserver on 8000 |
| Client | `SmolVLAWebSocketBackend` in `action_server`, params from `hero_bringup/parameters/action_server/vla.yaml` (section 1.1) |
| World | Gazebo `impuls`, robot HERO (Toyota HSR), two apples on `dinner_table`: `apple-1` (4.0, 1.2, 0.785), `apple-2` (4.2, 1.3, 0.785) |
| Task | "pick up the apple from the dinner table": Find → PickUp |
| Laptop | RTX 3050 4 GB. The policy uses ~1.1 GB; ED, face recognition and HMI nodes use ~1.6 GB |

### 1.1 Client parameters used in the trials (`vla.yaml`, uncommitted)

```yaml
execution_mode: hybrid          # trials set strict_mode true (no classic fallback)
enabled_actions: [pick-up]
policy_fps: 10.0                # rows per second
chunk_prefix_steps: 10          # rows played per inference (1 s)
max_chunks: 60                  # 60 s of policy motion
start_arm_pose: [0.2, 0.0, 0.0, -1.57, 0.0]
start_head_pose: [0.0, -0.6]
arm_lead_time: 0.2
max_arm_delta: [0.04, 0.24, 0.4, 0.3, 0.3]
arm_delta_gain: [1.0, 1.0, 2.5, 2.0, 1.0]       # sim correction
arm_hold_threshold: [0.0, 0.0, 0.0, 0.005, 0.0] # sim correction
gripper_close_threshold: 0.5
gripper_open_threshold: 0.9
gripper_max_torque: 0.26        # = 0.018 Nm, the official client's grasp effort
enable_head_motion: false
enable_base_motion: false
base_action_mode: velocity
grasp_position_threshold: 0.1   # too low, see P2
grasp_settle_time: 1.0
post_grasp_lift: 0.1
policy_url: ws://127.0.0.1:8000 # trials override this to 8001
```

---

## 2. The three questions

### 2.1 (a) Velocity or pose delta: **velocity**

Evidence, strongest first:

1. **Official AIRoA client.** `airoa-org/airoa-evaluation-ICRA` @ `47b4df5`,
   `deploy/hsr_policy_client/scripts/hsr_policy.py:621-625` publishes columns 8–10 unscaled as a
   `Twist` on `/hsrb/command_velocity`. AIRoA's dataset converter also records `/hsrb/command_velocity`.
2. **Normalisation statistics.** y/θ saturate at q01/q99 ≈ ±0.30, and x/y are exactly 0 in ≥ 80% of
   frames. That is the profile of a joystick velocity limit.
3. **Live outputs** fall well within the clamps (0.02–0.17 m/s), as small velocity corrections would.

Row duration is 0.1 s (10 Hz): the official launch file sets `update_freq 10` and
`adopted_action_chunks 10`. This is **not confirmed from the private fine-tuning dataset**; only the
checkpoint author (Pau Montagut) can confirm it.

**Still open:**
- On HERO, the frame and sign of the base columns have not been checked. Base motion is disabled in
  `vla.yaml`; `NavigateToGrasp` does the approach. See TODO 5.
- The sim base drifts on its own (P1). So this check needs the drift fixed first, or has to be
  measured as displacement relative to the drift.

### 2.2 (b) Are all 11 DOF produced and executed?

| Columns | Meaning (verified) | Produced | Executed by default | How execution is verified |
|---|---|---|---|---|
| 0–4 arm | delta w.r.t. the **measured** joints `[arm_lift, arm_flex, arm_roll, wrist_flex, wrist_roll]` | yes | **yes**, streamed at 10 Hz | golden test: equal to the official client (atol 1e-9) with reference-equivalent params; documented deviations only with deployed params; 6/6 mutations caught |
| 5 gripper | **absolute** `hand_motor_joint` [rad], +1.24 open, −0.8 closed on nothing | yes | **yes**, hybrid mode | golden test (grasp/position decisions per tick, effort 0.018 Nm) |
| 6–7 head | delta w.r.t. the measured head joints, ~0 in training (std 0.003–0.006 rad) | yes | no (disabled by design; head held at `start_head_pose`) | golden test with `enable_head_motion`: last-tick head target equals the reference (tilt negated for HERO) |
| 8–10 base | body-frame velocity [m/s, m/s, rad/s] | yes | no (disabled by design; `NavigateToGrasp` does the approach) | golden test with `enable_base_motion`: Twist per tick equals the reference; brake after the prefix |

Answer: **11/11 produced. 6/11 executed by default, by design. All 11 command paths are verified at code
level against the official client.** "Used" doesn't mean "moves a lot": the head and base outputs are
near zero most of the time.

In the simulator the arm visibly follows the policy. In `staged-smoke4`, for example, `arm_lift` went
0.20 → 0.49 m and `arm_flex` 0 → −1.32 rad over 16 chunks, and the gripper closed. A formal per-joint
"commanded vs achieved" analysis from the bags (plan step 8) has **not** been done yet (TODO 4).
`~/vla/compare/report.py` has a section ("Did the arm follow the model's nudges?") that is the starting
point.

### 2.3 (c) Does the integration work end to end?

- **Mechanically: yes.** In Gazebo:
  - Find finds the apple, and the robot goes to the start pose.
  - The rollout streams 10-row prefixes at 10 Hz for up to 60 chunks, with fresh head and hand images and
    the 8-D state on every query.
  - The completion check runs, the arm retracts after a failed pick-up, and the classic fallback (hybrid)
    or a strict failure follows.
  - No transport error, socket error or controller error came from the VLA path in 693 inferences.
- **Grasp quality: unknown.** Every sim rollout so far ran with the base drifting (P1). The two batches
  scored 0 lifts. Measured against the decision rule fixed in advance, though, they say nothing about the
  policy:
  - works in sim: ≥ 3/10 lifts and mean ≥ 2.5;
  - doesn't work: mean ≤ 1.5 while the code-level checks pass.
- **Unstaged end-to-end flow** (Find → `NavigateToGrasp` → VLA) usually doesn't reach the VLA in this
  sim, because the approach fails (P3). The trials therefore teleport the robot to the grasp pose
  ("staged" mode, section 4.1).

---

## 3. Code-level verification

### 3.1 Golden test against the official client

`action_server/action_server/test/test_vla_airoa_golden.py`:
- `AiroaHybridClient` is a line-by-line transcription of the official client's per-tick commands,
  `OpenpiPolicy.act` + `HSREnv.execute_actions`. It cites `hsr_policy.py:452, 602-683, 798-803, 840-842`.
- Configuration: `update_freq 10`, `adopted_action_chunks 10`, `upsample false`, `action_smoothing none`,
  `gripper_mode hybrid`.
- Input: one (50, 11) chunk and a per-tick joint-state sequence that **drifts a few mrad per tick**.
  Without the drift, per-tick re-anchoring and chunk-start anchoring would give the same targets.
- The gripper column runs approach → grasp → latch → re-open → re-grasp.

| Test | Asserts |
|---|---|
| `test_reference_equivalent_parameters_command_exactly_the_reference` | With reference-equivalent params (lead 0.05 s, no caps, close = open = 0.5, 0.018 Nm, base on, no clamps, no URDF clipping), every tick's arm target, `time_from_start`, Twist and gripper decision equals the reference (atol 1e-9). `Rate(10)`, 10 ticks. Only documented extra: the base is braked after the prefix |
| `test_head_goal_is_the_reference_head_target_of_the_last_tick` | Head goal equals the reference target of the last tick, with the tilt negated for HERO's `head_ref` |
| `test_deployed_parameters_deviate_only_where_documented` | With the deployed `vla.yaml` params the sink differs only in: lead 0.2 s, `arm_delta_gain`, `max_arm_delta` cap (tick 1), URDF limits (ticks 2, 3), the grasp held in the release band (column 0.6), torque rounding, and no head/base |
| `test_deployed_mirror_matches_vla_yaml` | The test's copy of the deployed params equals `hero_bringup/.../vla.yaml`, so the yaml can't drift from what is tested |

### 3.2 Mutation check (re-run 2026-11-03)

We injected six bugs into the sink, one at a time, in memory (no file was changed). The golden test
must fail on each:

| Mutation | Result |
|---|---|
| anchor arm targets on the chunk-start state | caught (19 failing sub-checks) |
| cumulative sum of the deltas | caught (18) |
| deltas played as absolute targets (v1's bug) | caught (21) |
| gripper inverted, close above 0.5 (v1's bug) | caught (13) |
| base read as pose deltas | caught (10) |
| rows at 30 Hz | caught (2) |

The script is in Appendix A. Run it again after any change to `HSRActionSink` or `backend_utils`.

### 3.3 Unit tests

| Suite | Count | Covers |
|---|---|---|
| `test_vla_action_sink.py` | 15 sink + 2 debug topic + 6 retract + 1 log | live-state delta, joint limits, gripper position vs grasp, grasp latch, base clamp and brake, base off by default, head delta, 6-wide chunk, gain before the cap, hold, reset, empty-grasp reopen, grasp held in the release band, wrong-length arm param fails before moving, debug topic, retract after a failed pick-up (only if the policy moved, not after a grasp, not for other actions, on exception, never masking the outcome), signed delta sum in the log |
| `test_vla_backend_utils.py` | 16 | `apply_relative`, `gripper_action`, `gripper_holding_object`, `is_manipulation_done`, `query_debug_message` |
| `test_vla_airoa_golden.py` | 4 | section 3.1 |
| `test_vla_pick_up.py` | 5 | VLA after `arrived`; unreachable → classic; no arm → classic; failed VLA → classic; VLA disabled → no approach |
| `per-group-mse-vla/inference/test_policy_server.py` | 11 | `validate_state_indices`, `_state_stats_dim`, server built around a fake LeRobot factory with a stale 6-D `config.json`: 8 indices override it, 6 indices fail **before** the stats are trimmed, no indices = legacy trim |

Results on 2026-11-03: **pytest 50 passed** (49 VLA + `test_import`), **nose 49 OK**, **policy server 11 OK**.
No roscore, GPU or checkpoint is needed for any of them.

---

## 4. Simulation trials

### 4.1 Trial kit (`~/vla/compare`, not under git)

| File | Role |
|---|---|
| `trial.py classic\|vla [--staged] [--name N]` | one recorded Find → PickUp with the action server's own action classes; no action-server process needed |
| `run_trials.sh N NAME [args]` | runs trials one at a time until N reached the pick-up (a failed Find is retried, up to 2N attempts), with a watchdog, then scores them |
| `score_runs.py RUN...` | 1–4 score from Gazebo ground truth, plus diagnostics |
| `report.py RUN...` | HTML timeline comparing up to 3 runs |
| `start_policy_server.sh` | container `smolvla-compare` on :8001 with the 8-D state |

What one trial does:
1. Resets the scene: robot at (3.9, 2.25, −π/2), apples back in place, arm and head reset, ED reset,
   localization seeded with the true pose and **polled until it agrees** (≤ 5 cm / 3°, up to 20 s).
2. Reloads `vla.yaml`, sets `execution_mode` hybrid, `strict_mode` true (vla), `publish_debug` true,
   `policy_url` :8001 and `/hero/skills/vla/grasp_mode` off. Restores them all afterwards.
3. Records a bag: joint states, arm/head/gripper commands and goals, base references and odometry, the
   VLA debug topic (every query's state and full chunk), hand camera and head camera at 3 Hz. Samples
   Gazebo ground truth at 10 Hz into `truth.csv` (robot pose, finger midpoint, both apples). Writes
   `meta.json` and `code.diff` (the git state of the 4 repos).
4. **`--staged`**: after Find, teleports the robot to the pose `NavigateToGrasp` would aim for (the true
   apple at the arm's `base_offset` (0.65, 0.08), facing the table, yaw −π/2). Re-seeds localization
   and stubs `PickUp._approach_for_vla` to `True`.

Scoring rubric (paper):

| Score | Rule |
|---|---|
| 4 | apple lifted > 5 cm and held near the fingers ≥ 2 s |
| 3 | fingers within 5 cm of the apple centre, or apple moved > 2 cm |
| 2 | fingers within 15 cm |
| 1 | otherwise |

Score 3 includes knocking the apple over. `score_runs.py` also reports:
- `vla_ran`, `aborted`, `reported` (the robot's own verdict);
- `closest_cm`, `held_s`, `max_lift_cm`, `apple_moved_cm`, `knocked_off`;
- `base_slide_cm`, `manipulation_s`.

### 4.2 Chronology

| Run(s) | When | What happened | Lesson |
|---|---|---|---|
| `20261031-202000_classic_first`, `-202514_vla_first` | 10-31 | During the approach the robot scraped a table corner while turning in place. Odometry over-counted the rotation by 90–110° and localization never recovered. Classic "arrived" 1.2 m from the apple. VLA: `NavigateToGrasp` never returned; the robot wandered for 2 h (2.2 GB bag) | need staging, a timeout and a localization check |
| `20261103-125441_vla_partB-1` | 11-03 12:54 | CUDA failed everywhere after the laptop resumed from suspend (ED Xid 31, CUDA error 719). Reboot | caveat C1 |
| `staged-smoke-1` | 13:42 | localization still 1 m / 69° off right after the teleport, so `NavigateToGrasp` was unreachable | seed and **poll** localization |
| `staged-smoke2-1` | 13:45 | Find failed: YOLO boxes, but SAM returned 0 masks | retry a failed Find |
| `staged-smoke3-1` | 13:48 | `NavigateToGrasp`: "No connectivity to specified constraint found" | ED apples about 0.2 m off; staged mode now skips the approach |
| `staged-smoke4-1` | 13:53 | **First real rollout**: 16 chunks, arm reached forward and down, gripper closed, apple pushed 29 cm, no lift. Reported **success** (fingers stalled at 0.171 > threshold 0.1) | false positive (P2) |
| `mistake-unstaged-1` | 13:56 | batch launched without `--staged`; killed and renamed | — |
| batch 1 `staged-1..11` | 13:57–14:14 | 6 rollouts, all cut short by the localization-divergence abort; trial 11 hung in Find for over an hour | section 4.3 |
| batch 2 `staged2-1..10` | 15:43–16:20 | 10/10 reached the VLA; base drifted 0.24–1.8 m; 0 lifts | section 4.4 |

### 4.3 Batch 1 (`staged-N`): invalid

| Run | VLA ran | Outcome |
|---|---|---|
| 1 | no | localization did not converge after the teleport (0.29 m); the abort fired at once |
| 2, 5, 9 | no | Find failed |
| 3 | 5.5 s | **false abort: 0.60 m / 97°**. The watcher thread and the truth sampler shared one persistent `get_model_state` proxy, and a robot query got `apple-2`'s reply. The same race corrupted the truth samples |
| 4, 6, 7, 8, 10 | 17–63 s | abort at 0.20–0.21 m (the threshold). The **base really slid** 13–22 cm with zero base commands. Odometry did not see it, so localization fell behind. Run 6 knocked the apple off the table; run 7's "Could not play the action chunk" is the same abort, landing inside `play_chunk` |
| 11 | no | Find hung for over an hour and ignored SIGINT. It was SIGKILLed, which left its `rosbag record` writing (1.9 GB) and the trial's params unrestored |

Fixes to the kit after batch 1:
- a lock around the shared Gazebo proxies;
- no localization abort in staged mode (nothing after staging uses localization);
- `base_slide_cm` and the trial's own `target_apple` in the scorer;
- the runner SIGKILLs a trial that ignores SIGINT for 30 s, kills `/compare_record` and
  `/compare_head_throttle`, and restores the params from a snapshot after every trial.

### 4.4 Batch 2 (`staged2-N`): ran cleanly, but invalid because of the base drift

| Run | Score | Queries | Closest | Apple moved | Base slide | Reported | Note |
|---|---|---|---|---|---|---|---|
| 1 | 1 | 60 | 58.4 cm | 0 | 37 cm | no | 9 grasp goals |
| 2 | 1 | 60 | 15.9 cm | 0 | **181 cm** | no | hand dragged along the table, then toward the floor |
| 3 | 3 | 60 | 13.4 cm | 332 cm | 107 cm | no | apple knocked off the table |
| 4 | 1 | 60 | 16.0 cm | 0 | 137 cm | no | |
| 5 | 3 | 60 | 13.0 cm | 331 cm | 70 cm | no | knocked off |
| 6 | 1 | 60 | 17.0 cm | 0 | 158 cm | no | |
| 7 | 1 | 60 | 20.2 cm | 0 | 157 cm | no | |
| 8 | 1 | 60 | 16.4 cm | 0 | 129 cm | no | |
| 9 | 3 | 30 | 13.0 cm | 267 cm | 24 cm | **yes** | **false positive**: apple on the floor, fingers stalled at 0.14–0.15 > 0.1 |
| 10 | 1 | 60 | 35.2 cm | 0 | 87 cm | no | |

Summary: mean 1.6, 0 lifts, 3 knocks. 9/10 rollouts ran the full 60 chunks (about 92 s). Inference
latency over the whole session (693 queries): first 1094 ms, then median 338 ms, p90 354 ms.

What the policy does, as far as the drift allows us to see:
- it reaches forward and down to apple height (finger z ≈ 0.75–0.79 m) and closes the hand;
- it gets within 13–20 cm in 8/10 runs;
- it pushes the apple off the table in 3/10.

None of this separates policy quality from the drift. v1 §7.3's controlled rollouts (fingers 10–12 cm
from the apple, grasps beside it) were also affected: one of them drifted about 0.6 m with base motion
off.

### 4.5 Base drift investigation

**In the trials** (`staged2-2`, bag + truth):
- The Gazebo robot moved about 1.7 m east at a steady ~2.2 cm/s and turned 50° during the rollout.
- `/hero/base/references` had **no** non-zero message in the pick-up window. Its only publishers were
  `/hero/local_planner` and the trial's own brake.
- `/hero/base/measurements` (`odometry_switcher`, which merges wheel and laser odometry) reported a
  steady twist of about (−0.075, 0.05) m/s; in batch 1 its sign flipped between consecutive messages.
  The odometry pose barely translated (0.3 m) but rotated about 90° while the truth rotated 51°. Odometry
  is wrong as well as the robot moving.
- The drift starts before the policy moves: the robot turned 15° in the ~10 s between the teleport and
  the start of the pick-up.

**Isolated test** (no contact, no commands; Appendix B):
- teleported to (3.92, 1.85, −90°) with no commands: x went 3.92 → 3.06 m in 25 s (≈ 3.4 cm/s), yaw −90
  → −77°;
- the same while streaming a zero `Twist` at 10 Hz on `/hero/base/references`: 0.92 m in 25 s. In sim
  that topic goes to the `/gazebo` plugin (subscribers: `/gazebo`, `/hero/head_ref_client_cmd_vel`,
  `/hero/joke`), so zero velocities don't stop it.

**The controller's state** (`/hero/omni_base_controller/state`, TMC, virtual joints `odom_x`, `odom_y`,
`odom_t`):
- `desired == actual` for the positions, with a non-zero desired velocity (e.g. 0.045, −0.080,
  0.082) whose sign flips between messages;
- a one-point zero-velocity hold at the current position, sent to `/hero/omni_base_controller/command`,
  didn't stop it: the robot still crept about 0.8 cm/s and the desired velocity kept flipping.

**What v1 §7.4 had already found:** the wheels sit at saturated effort (±28 Nm) with no commands, even
right after a fresh simulator start. Wheel odometry read 0.1 m/s while the robot crept about 4 mm/s.

**Working hypothesis (not confirmed):** the sim base loop sustains whatever velocity the base has, and
teleports and contacts inject velocity. A teleport makes it about 10× worse than at a fresh start. Staging
uses two teleports per trial: the stage itself, and the seed, because `/hero/initialpose` also teleports
the Gazebo robot (caveat C12).

**Not tried yet:**
- restarting Gazebo and re-measuring;
- reading the hsrb Gazebo base plugin and controller config (`hsrb_gazebo_bringup`);
- pinning the base in staged trials.

---

## 5. Analysis

1. **The integration contract is right.** Every column means what the official client says it means.
   The rate, anchoring, gripper polarity and state dimension are pinned by tests that fail on v1's bugs.
2. **The end-to-end plumbing is reliable:**
   - 693 queries without a transport failure;
   - one socket per episode, so no dead cached socket;
   - the retract and fallback paths ran as designed.
3. **The sim can't currently measure grasp quality**, because the base drifts (P1). Any score from a
   drifting run is a lower bound at best. Fix the drift, or pin the base, before running more batches.
4. **The completion signal is unsafe at threshold 0.1** (P2). There were two false positives in 11
   rollouts (smoke4 and staged2-9), and both would make a GPSR task continue with an empty hand.
5. **Policy behaviour to watch once the base holds still:**
   - pressing on or dragging along the table top;
   - repeated grasp attempts (up to 9 per run);
   - knocking the apple over (3/10).

   These may be the sim-to-real gap: the model was trained on real AIRoA images. They may also come from
   the start pose: v2 uses `[0.2, 0, 0, −1.57, 0]`, while the training median is about
   `[0.02, −0.29, 0.02, −1.73, −0.04]`.

---

## 6. Problems and proposed fixes

### P1. The simulated base drifts by itself (blocker)

- **Symptoms:** section 4.5.
- **Fix, in order:**
  1. Restart Gazebo (`hero-start`). Re-run the 30 s drift test with no teleport, then once after a
     teleport. If a fresh start is clean, it is a degraded state.
  2. If it still drifts, **pin the base in staged trials**: re-place the robot at the staged pose
     about 10×/s during the rollout. A real HSR's base doesn't drift, so this is the fair condition for
     measuring the grasp.
  3. In parallel, investigate the hsrb sim base (Gazebo plugin + `omni_base_controller` + PID gains
     in `hsrb_gazebo_bringup`). This also affects the classic runs and every unstaged run.
- **Validity gate for every future trial:** `base_slide_cm` < 2 cm during the pick-up. Otherwise the
  trial doesn't count.

### P2. False-positive grasp completion

- **The current rule:** an effort grasp settled ≥ 1 s **and** fingers stopped above
  `grasp_position_threshold` (0.1), confirmed on 2 consecutive chunks.
- **What the fingers read in sim:**
  - an empty grasp stalls at −0.07 at best, but 0.14–0.17 was observed with the fingers on the table
    or the apple's edge;
  - a 9 cm apple holds them at about 0.55.
- **Fix:**
  - Raise the threshold to about 0.3, after a calibration run: close on nothing, on the table edge and
    on the apple, and log `hand_motor_joint`. This is a param change in `vla.yaml`; it needs the owner's
    OK.
  - Better: after the post-grasp lift, confirm that the fingers stay above the threshold. The object
    weight keeps them apart.
  - Never trust `reported` in evaluation: always score from ground truth.

### P3. The unstaged approach fails in this sim

- **What breaks:**
  - ED places segmented apples about 0.2 m north of reality, sometimes duplicated.
  - Their inflation (`robot_radius` 0.24, inflation 0.34) covers the approach side, so the A* planner
    reports "No connectivity to specified constraint found".
  - Localization also degrades during approaches: table scrapes and over-counted odometry.
- **Fix:** investigate the ED apple pose offset (segmentation/projection) and the costmap layers (ED
  occupancy grid + `recent_obstacles` + inflation) before testing the unstaged or GPSR flow.

### P4. Find is flaky and can hang

- **Symptoms:** YOLO boxes but 0 SAM masks (3 of 11 batch-1 attempts). Once Find hung for over an hour.
  SIGINT doesn't interrupt it: rospy turns SIGINT into a shutdown request, which a blocked service call
  never sees.
- **Fix:**
  - add timeouts to the Find path's service calls;
  - until then, the trial runner SIGKILLs after a 30 s grace period.

### P5. Localization after a teleport

- **Symptom:** localization takes several seconds to follow a teleport; it was still 1 m / 69° off 2 s
  after one.
- **Fix (done in the kit):** seed `/hero/initialpose`, then poll until it agrees (≤ 5 cm / 3°).

### P6. The classic baseline is broken in this scene

- **Symptom:** classic grasps fail on an `hsrb_planner` exception, `http://:8080/black_cabinet1.stl is
  invalid` (the mesh server URL has an empty host).
- **Fix:** repair this before any classic-vs-VLA comparison.

### P7. The Docker image is stale

- **Symptom:** `smolvla-policy-server:latest` was built on 2026-10-30, before `8b58877`. Its baked-in
  `policy_server.py` and its default `POLICY_STATE_INDICES=0,1,2,3,4,5` serve a **6-D state without any
  error**.
- **Fix:**
  - rebuild the image;
  - until then, always bind-mount the repo's `policy_server.py` and set `POLICY_STATE_INDICES=0..7`
    (`start_policy_server.sh` does both).

### P8. Port 8000 conflict

- **Symptom:** the `tue_mobile_ui` webserver from `hero-start` holds port 8000, which `vla.yaml`'s
  `policy_url` and v1 §7.5's `docker run -p 8000:8000` both use.
- **Fix:** serve on 8001 and set `/hero/action_server/vla/policy_url ws://127.0.0.1:8001` after every
  `rosparam load`. Or move the default and document it.

### P9. Policy behaviour in sim (after P1)

- **Symptoms:** pressing on and dragging along the table, repeated grasps, knocking the apple.
- **Fix:** once the base is pinned, run a fixed-noise offline A/B (plan step 7) to see which inputs move
  the output:
  - start pose: training median vs current;
  - prompt: "pick up the apple" vs "Grasp the apple.";
  - head tilt;
  - RGB vs BGR.

---

## 7. Testing procedure we followed (reproducible)

1. **Unit and golden tests**, with no ROS master:
   ```bash
   source ~/ros/noetic/system/devel/setup.bash
   cd ~/ros/noetic/repos/github.com/tue-robotics/action_server/action_server
   PYTHONPATH=src:$PYTHONPATH python3 -m pytest -q -p no:warnings test/test_vla_*.py test/test_import.py  # 50 passed
   PYTHONPATH=src:$PYTHONPATH python3 -m nose test/test_vla_*.py                                       # 49 OK
   cd ~/ros/noetic/repos/github.com/tue-robotics/per-group-mse-vla/inference
   ~/vla/.venv/bin/python -m unittest test_policy_server                                               # 11 OK
   ```
2. **Mutation check** (Appendix A): all six mutations must be caught.
3. **Stack**, once per session:
   ```bash
   hero-start                                  # Gazebo + robot, impuls world (also the mobile UI on :8000)
   hero-free-mode                              # navigation, ED, perception, loads vla.yaml
   ~/vla/compare/start_policy_server.sh        # smolvla-compare on :8001, 8-D state
   ~/vla/.venv/bin/python -c "import torch; torch.ones(1, device='cuda')"   # CUDA sanity check
   ```
   Check that the server log says `state_dim=8`.
4. **Single trials** first. Watch the log for `[VLA] chunk N state=... arm|delta|max=... arm_sum=...
   gripper=[..]` lines:
   ```bash
   cd ~/vla/compare && python3 trial.py vla --name smoke --staged
   ```
5. **Batches**:
   ```bash
   cd ~/vla/compare && ./run_trials.sh 10 NAME --staged > logs/batch_NAME.log 2>&1
   python3 score_runs.py runs/*_vla_NAME-*        # also run by the runner
   python3 report.py runs/<run> [runs/<run> ...]  # HTML timeline in reports/
   ```
6. **Read every run with ground truth**, never with the robot's own verdict. Check `base_slide_cm`,
   `vla_ran` and `aborted` before reading the score.
7. **For any surprise**, go to the bag (`/hero/base/references`, `/hero/base/measurements`, the arm and
   gripper commands, `/hero/action_server/vla/debug/query`) and `truth.csv`. Then build an isolated
   reproduction, as with the drift test.

---

## 8. Caveats and lessons for the next version

**Machine and GPU**
- **C1.** Suspend/resume breaks CUDA on this laptop: Xid 31 and CUDA error 719 in every context,
  including the policy container. Only a reboot fixes it. Stop the stack before closing the lid.
- **C2.** Don't run a second copy of the model on the CPU next to Gazebo. RAM spikes and VS Code froze.
- **C3.** The 4 GB GPU is shared. The policy uses about 1.1 GB; ED (0.8 GB), face recognition and the
  HMI nodes use the rest.
- **C4.** Disk:
  - each trial bag is about 150 MB, and `runs/` is already 3.1 GB;
  - an orphaned recorder wrote 1.9 GB in an hour;
  - the disk sits at about 88%;
  - never bag raw images or the unthrottled `/gazebo/model_states`.

**Policy server**
- **C5.** The image is stale and silently serves a 6-D state (P7). Look for `state_dim=8` in the
  startup log.
- **C6.** Port 8000 is taken by the mobile UI (P8).
- **C7.** `config.json` of the checkpoint says state dim 6. That is stale: the normalisation statistics
  and `state_proj` weights are 8-D.
- **C8.** SmolVLA samples fresh noise on every call. Single-run comparisons of chunks are meaningless:
  compare with fixed noise, or over many runs.

**ROS params and processes**
- **C9.** The VS Code "(debugpy) Launch Action Server" loads no params; `hero-free-mode` loads them.
  The backend reads them **once per process** (provider cache), so restart the action server after
  every param change.
- **C10.** `strict_mode` and `publish_debug` are not in `vla.yaml`, and `rosparam load` resets
  `policy_url` to 8000.
- **C11.** Full `execution_mode: vla` delegates the whole action to the VLA (`_VLADelegatedAction`). It
  **skips `NavigateToGrasp`**, so the VLA runs wherever the robot stands. Use `hybrid` (+ `strict_mode`
  for evaluation).
- **C12.** `/hero/gazebo_pose_setter` teleports the Gazebo robot to anything published on
  `/hero/initialpose`, so seeding localization (or an RViz "2D Pose Estimate") moves the simulated
  robot too.
- **C13.** Time bases differ: rospy lines are [wall, sim], docker logs are UTC, bags use wall receive
  time. Align on `[VLA] Sending observation` ↔ `Inference OK`.

**Simulator**
- **C14.** The base drifts with no commands, worse after teleports (P1). Check `base_slide_cm` on every
  trial.
- **C15.** The sim `wrist_flex` PID has no integral term
  (`/opt/ros/noetic/share/hsrb_gazebo_bringup/params/hsrb_pid_gains.yaml`). Per unit delta the sim moves
  `arm_roll` 0.40× and `wrist_flex` 0.55× as far as the real HSR did in the training data, hence
  `arm_delta_gain` / `arm_hold_threshold`. Don't use them on the real robot without re-measuring.
- **C16.** Empty-grasp finger readings differ from the real HSR (−0.07 to 0.17 in sim vs about −0.8
  real) (P2).
- **C17.** ED apple poses are off by about 0.2 m and the approach fails (P3). Find is flaky and can hang
  (P4). Localization lags teleports (P5). The classic grasp is broken (P6).
- **C18.** A gripper close threshold of 0.7 closes during the pre-shaping. Keep the official 0.5.

**Trial tooling** (already fixed in `~/vla/compare`, but keep the lessons)
- **C19.** rospy's SIGINT is only a shutdown request. A blocked service call ignores it.
  - `timeout(1)` sends SIGTERM, which skips Python `finally` blocks.
  - After a SIGKILL, kill the setsid'd recorder (`rosnode kill /compare_record /compare_head_throttle`)
    and restore the params.
- **C20.** A persistent `rospy.ServiceProxy` is not thread-safe. Shared across threads it returned
  crossed replies (a fake 0.6 m / 97° localization error). Lock it, or give each thread its own proxy.
- **C21.** From a shell tool, `pkill -f "<pattern>"` can match the tool's own `bash -c` command line and
  kill it. Kill by PID.
- **C22.** A localization-divergence abort is only meaningful while something uses localization
  (navigation). During the VLA rollout it killed valid trials.

**Repos**
- **C23.** `hero_bringup`'s `vla.yaml` and launch change are uncommitted, on the unrelated branch
  `feature/LUT_for_configuring_yolo_filter`. The `hybrid` default in `action_server.launch` is the
  owner's edit; keep it.
- **C24.** `tue_robocup` (`feat/vla-grasp`) carries uncommitted changes from the older VLAGrasp path
  (`robot_skills` arms/base/hero/mockbot and others). They load at runtime. That path is gated by
  `/hero/skills/vla/grasp_mode`, default `off`. Every trial's `code.diff` records them.
- **C25.** `action_server/vla_last_report` holds the owner's notes. Keep it untracked.

---

## 9. TODOs (priority order)

1. **Neutralize the base drift** (P1): restart test, then pin the base in staged mode, then investigate
   the sim base. Add the `base_slide_cm < 2` validity gate to `score_runs.py`.
2. **Calibrate and raise `grasp_position_threshold`** (P2, owner decision). Add a post-lift
   confirmation.
3. **Re-run 10 staged trials** with a still base. Read them with the rule fixed in advance (≥ 3/10 lifts
   and mean ≥ 2.5 = works in sim; ≤ 1.5 = doesn't; in between: 10 more).
4. **Commanded vs achieved per group** from the bags (plan step 8):
   - arm/head: |actual(t + 0.2 s) − target(t)| < 0.02 rad (0.01 m for lift);
   - gripper: commands follow the column polarity;
   - no safe_pose_changer goals during the rollout.
5. **Base executor check**, for (a) on HERO (plan step 6), with a still base. Push 20 rows of
   (0.1, 0, 0), (0, 0.1, 0) and (0, 0, 0.3). Expect +0.20 m forward, +0.20 m left and +0.60 rad CCW,
   ±20%.
6. **Fixed-noise offline A/B** (plan step 7, P9).
7. **Ask the checkpoint author:**
   - the dataset fps (`scripts/dataset_info.py --show-tasks 50`);
   - the client settings used in their trials;
   - BGR/RGB;
   - head camera;
   - arm home pose;
   - trial timer.
8. **Unstaged flow**, after P3 and P4: Find → `NavigateToGrasp` → VLA. Then the GPSR flow
   ("get the apple from the dinner table") in hybrid + strict.
9. **Classic baseline** (P6), then classic vs VLA with `report.py`.
10. **Housekeeping:**
    - commit `vla.yaml` and the launch default on the right `hero_bringup` branch;
    - push `feature/vla` and `feature/vla_implementation`, then PRs;
    - rebuild the image;
    - settle the policy port;
    - put `~/vla/compare` under version control.

Integration-level TODOs (stop flag, cancel, adapters, place/hand-over) are in
[VLA_INTEGRATION_REPORT_v2.md](./VLA_INTEGRATION_REPORT_v2.md) §11.

---

## 10. Superseded statements in v1

- §1 "`state_dim=6`", and §1's note that the server decides the state: the state is 8-D, and the server
  now validates the indices.
- §4.2 / §5 P1: arm columns are **deltas**, not absolute positions. The "suspects" list is resolved
  (§7.1 of v1, section 2.2 here).
- §5 P1.3: the 25 s first cycle was `_send_joint_trajectory` timing every waypoint from the start pose on
  one big first move. It was not a timeout or goal preemption. The "not tracking goal handle" error is an
  intermittent SimpleActionClient race, also seen in classic runs.
- §5 P1.5: the gripper did change, but inverted.
- §5 P4: about 1.2 s of each old cycle was the sequential base drive (10 × 0.1 s + brake), not the image
  grab (~40 ms).
- §5 P5: the keepalive fix targeted the wrong library. The real issue was a dead cached socket between
  actions; now there is one socket per episode.
- §7.5 step 2's `-p 8000:8000` conflicts with the mobile UI. Use 8001 (P8).
- §7.3's distances were taken with the base drifting, so they are unreliable.

---

## Appendix A. Golden-test mutation script

Run it from `action_server/action_server` with the workspace sourced. It execs mutated source in
memory only; no file is changed.

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
    "anchor on the chunk-start state": (backends, "positions = self._joint_states.get()\n",
                                        "positions = positions or self._joint_states.get()\n"),
    "cumulative sum of the deltas": (backends, "current = [positions[name] for name in self._arm_joints]",
                                     "current = list(self._arm_target) if self._arm_target is not None else "
                                     "[positions[name] for name in self._arm_joints]"),
    "deltas played as absolute targets": (utils, "target = np.asarray(current, dtype=np.float64) + delta",
                                          "target = delta + 0 * np.asarray(current, dtype=np.float64)"),
    "gripper inverted (close above 0.5)": (utils, "if value < close_threshold or",
                                           "if value >= close_threshold or"),
    "base read as pose deltas": (backends, 'if self._base_action_mode == "pose_delta":', "if True:"),
    "rows at 30 Hz": (backends, "rospy.Rate(self._fps)", "rospy.Rate(30.0)"),
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

If a string replacement no longer matches after a refactor, the `assert` names the mutation; update its
`old` text.

## Appendix B. Base drift test

Run it with the stack up and no trial running. The Gazebo model is called `hsrb`.

```python
#!/usr/bin/env python3
"""Does the simulated HERO base drift on its own after a teleport? Two 30 s holds at the staged pose."""
import math

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import Twist

POSE = (3.92, 1.85, -math.pi / 2)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


rospy.init_node("drift_test")
get = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
put = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
cmd = rospy.Publisher("/hero/base/references", Twist, queue_size=1)
rospy.sleep(1.0)
for label, stream_zero in (("no commands", False), ("zero twist at 10 Hz", True)):
    state = ModelState(model_name="hsrb", reference_frame="world")
    state.pose.position.x, state.pose.position.y = POSE[:2]
    state.pose.orientation.z, state.pose.orientation.w = math.sin(POSE[2] / 2), math.cos(POSE[2] / 2)
    put(state)
    start, rate = rospy.Time.now(), rospy.Rate(10)
    print("--", label)
    while (rospy.Time.now() - start).to_sec() < 30 and not rospy.is_shutdown():
        if stream_zero:
            cmd.publish(Twist())
        elapsed = (rospy.Time.now() - start).to_sec()
        if int(elapsed * 10) % 50 == 0:
            p = get("hsrb", "world").pose
            print("t=%4.1f robot=(%.3f, %.3f, %6.1f deg)" % (elapsed, p.position.x, p.position.y,
                                                             math.degrees(yaw_of(p.orientation))))
        rate.sleep()
```

To see the controller side, run `rostopic echo -n1 /hero/omni_base_controller/state` and look at
`desired.velocities` while the robot is supposed to stand still.
