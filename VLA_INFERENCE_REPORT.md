# VLA Inference Report

Date: 2026-10-09. Companion to [VLA_INTEGRATION_REPORT.md](./VLA_INTEGRATION_REPORT.md).

Goal of the two runs: find out (a) whether the per-group-mse SmolVLA checkpoint
emits base **velocity** or **pose delta**, (b) whether all 11 output DOF are used,
and (c) whether the integration actually works end to end.

**TL;DR**

- Base convention: **velocity** is by far the more plausible one. `vla.yaml` was
  set to `pose_delta`; it is now `velocity`.
- All 11 columns are produced by the model (`chunk shape (50, 11)`), but the logs
  show that in practice **the robot barely moves** (observed: no torso/arm_lift
  movement, hand barely moving). The pick did not complete and ended in the
  classic fallback. So the integration is **not working yet**, independent of the
  base convention.
- Several concrete suspects and fixes are listed in
  [Problems and proposed fixes](#problems-and-proposed-fixes).

---

## 1. Setup

| Item | Value |
|---|---|
| Checkpoint | `PauMontagut/per-group-mse-smolvla` (snapshot `cb72ca6a...`) |
| Policy server | Docker, `ws://127.0.0.1:8000`, cuda, `policy=smolvla state_dim=6 action_dim=11` |
| Chunk | `(50, 11)` = arm(5) + gripper(1) + head(2) + base(3) |
| Client config ([vla.yaml](../hero_bringup/parameters/action_server/vla.yaml)) | `execution_mode: vla`, `max_chunks: 20`, `chunk_prefix_steps: 10`, `step_timeout: 10.0`, `enable_head_motion: true`, `enable_base_motion: true`, `base_step_duration: 0.1`, clamps `0.2 m/s`, `0.4 rad/s` |
| Task | `PickUp` of an apple on the dinner table (Gazebo), after navigating to the table |
| Run 1 | `base_action_mode: pose_delta` |
| Run 2 | `base_action_mode: velocity` (only change) |

Note: `state_indices: [0..7]` (8-D) in `vla.yaml` is not used by the WebSocket
backend; the server decides (`state_dim=6`, i.e. arm5 + gripper, no head).

---

## 2. Policy-server log: base convention evidence

Printed once at server start (norm stats of the base columns `[8:11]`):

```
[base-check] ---- Action norm-stats, base columns [8:11] ----
  postprocessor:UnnormalizerProcessorStep max   base=[ 0.2980,  0.3584,  1.7538]
  postprocessor:UnnormalizerProcessorStep mean  base=[ 0.0010,  0.0000, -0.0001]
  postprocessor:UnnormalizerProcessorStep min   base=[-0.4081, -0.4014, -2.0172]
  postprocessor:UnnormalizerProcessorStep q01   base=[-0.1327, -0.2980, -0.3000]
  postprocessor:UnnormalizerProcessorStep q99   base=[ 0.1185,  0.2979,  0.3000]
  postprocessor:UnnormalizerProcessorStep std   base=[ 0.0343,  0.0562,  0.1065]
  (preprocessor:NormalizerProcessorStep: identical values)
[base-check] HINT: |max|<=0.05 -> likely POSE_DELTA | 0.05<|max|<=1.0 -> ambiguous | |max|>1.0 -> almost certainly VELOCITY
```

Reading:

- q01/q99 of y and θ are exactly about ±0.30 (0.298 / 0.300). That looks like a
  saturated command limit (≈0.3 m/s, ≈0.3 rad/s), typical of teleop/joystick
  velocity commands, not of per-step displacements.
- As pose deltas at dt = 0.1 s, 0.3 would mean 3 m/s or 3 rad/s: not possible on
  an HSR base.
- mean ≈ 0, symmetric: forward/backward velocity commands.
- Absolute max (θ = 1.75, min −2.02) is above 1.0, hitting the server's own
  "almost certainly VELOCITY" heuristic; these are rare outliers (q99 = 0.3).

### 2.1 Run 1 server log (trimmed)

20 inferences. First one is slow (1.5 s warm-up), then ≈400–480 ms.

```
17:54:22 policy_server Policy server ready on cuda with policy=smolvla state_dim=6 action_dim=11
17:56:54 Client connected from ('172.17.0.1', 53064)
17:56:56 Inference OK, chunk shape (50, 11), 1518.9 ms
17:56:56 [base-check] chunk T=50 |max|=(x=0.1603 y=0.1398 th=0.1178) |mean|=(x=0.0386 y=0.0303 th=0.0260) if POSE_DELTA dt=0.1 -> implied peak=(1.603, 1.398, 1.178)
17:57:21 Inference OK, 467.5 ms   |max|=(x=0.0751 y=0.0616 th=0.4725)  |mean|=(x=0.0076 y=0.0070 th=0.0573)
17:57:25 Inference OK, 429.9 ms   |max|=(x=0.2096 y=0.0424 th=0.1515)  |mean|=(x=0.0185 y=0.0044 th=0.0360)
17:57:29 Inference OK, 477.6 ms   |max|=(x=0.0710 y=0.0061 th=0.1654)  |mean|=(x=0.0030 y=0.0008 th=0.0160)
17:57:32 Inference OK, 473.4 ms   |max|=(x=0.2716 y=0.0775 th=0.2188)  |mean|=(x=0.0199 y=0.0049 th=0.0234)
   ... (13 more, similar) ...
17:57:55 Inference OK, 459.6 ms   |max|=(x=0.3364 y=0.0112 th=0.1634)
17:57:58 Inference OK, 402.1 ms   |max|=(x=0.0439 y=0.3615 th=0.3859)
   ...
17:58:19 Inference OK, 456.8 ms   |max|=(x=0.0784 y=0.0036 th=0.1526)
17:59:04 websockets.server connection closed
17:59:04 [ERROR] websockets.server connection handler failed
TimeoutError: timed out while closing connection
  ... websockets.exceptions.ConnectionClosedError: sent 1011 (internal error) keepalive ping timeout; no close frame received
```

### 2.2 Run 2 server log (trimmed)

20 inferences. No warm-up spike this time (503 ms first), steady 360–445 ms. No
keepalive error in the portion shared.

```
18:10:19 Client connected from ('172.17.0.1', 35588)
18:10:20 Inference OK, 502.7 ms  |max|=(x=0.0567 y=0.0201 th=0.2582)  |mean|=(x=0.0035 y=0.0018 th=0.0554)
18:10:45 Inference OK, 445.0 ms  |max|=(x=0.0396 y=0.0340 th=0.1696)  |mean|=(x=0.0027 y=0.0027 th=0.0164)
18:10:49 Inference OK, 394.6 ms  |max|=(x=0.1069 y=0.0631 th=0.2089)
18:10:54 Inference OK, 384.7 ms  |max|=(x=0.1043 y=0.0700 th=0.3797)
   ... (14 more, similar) ...
18:11:35 Inference OK, 388.0 ms  |max|=(x=0.0191 y=0.0082 th=0.4733)
18:11:44 Inference OK, 391.3 ms  |max|=(x=0.1639 y=0.0316 th=0.3622)
18:11:48 Inference OK, 391.0 ms  |max|=(x=0.0906 y=0.0927 th=0.1844)
```

### 2.3 Server-side comparison (full 50-row chunk)

| | Run 1 | Run 2 |
|---|---|---|
| Inferences | 20 | 20 |
| First inference | 1519 ms | 503 ms |
| Steady latency | 400–480 ms | 360–445 ms |
| |max| x | 0.044–0.336 | 0.004–0.176 |
| |max| y | 0.003–0.362 | 0.008–0.139 |
| |max| θ | 0.118–0.473 | 0.154–0.473 |
| |mean| (typical) | x ≲ 0.02, y ≲ 0.01, θ ≈ 0.02–0.06 | same |
| Server error | keepalive ping timeout at end | none seen |

The chunk statistics are statistically the same in both runs, as expected: the
policy output does not depend on the client's `base_action_mode`. What changes
is how the client *interprets* it (section 3).

---

## 3. Action-server (ROS client) log

### 3.1 Run 1 (`mode=pose_delta`), trimmed

Navigation to the table, segmentation (5 objects: 4 apples, a dining table),
then:

```
[INFO] [90.14]  State machine terminating 'EXECUTE_PLAN':'succeeded':'arrived'
[INFO] [95.98]  >> Segmented 5 objects!  (apple x4, dining table)
[INFO] [96.19]  'Hey, I found a apple!'
[INFO] [98.55]  Starting executing of action PickUp.
Value(False)
[INFO] [104.70] [VLA] Connected to policy server at ws://127.0.0.1:8000
[INFO] [104.77] lazy subscribe to /hero/head_rgbd_sensor/rgb/image_raw
[INFO] [105.08] [VLA] Sending observation head=(480, 640, 3) hand=(480, 640, 3) payload=1843368 bytes
[INFO] [129.72] [VLA-base-check] mode=pose_delta dt=0.100s T=10 raw|max|=(x=0.1603 y=0.1398 th=0.0974) raw|mean|=(x=0.0914 y=0.0516 th=0.0361)
                if VELOCITY -> peak speed=(0.160 m/s, 0.140 m/s, 0.097 rad/s)
                if POSE_DELTA -> implied peak=(1.603, 1.398, 0.974)  [safe clamp vxy<=0.20 vth<=0.40]
[INFO] [129.73] HINT: magnitudes consistent with VELOCITY (delta/dt would be too fast)
[INFO] [133.71] ... raw|max|=(x=0.0020 y=0.0042 th=0.2604)  -> POSE_DELTA implied th=2.604 rad/s
[INFO] [137.21] ... raw|max|=(x=0.2096 y=0.0035 th=0.1515)  HINT: consistent with VELOCITY
   ... (10 more, similar) ...
[INFO] [166.29] ... raw|max|=(x=0.3364 y=0.0076 th=0.0772)  -> POSE_DELTA implied x=3.364 m/s  HINT: VELOCITY
[ERROR][161.86] Got a transition callback on a goal handle that we're not tracking
[INFO] [169.17] ... raw|max|=(x=0.0103 y=0.0025 th=0.0456)  HINT: both plausible
```

13 `[VLA-base-check]` lines were shared (the server served 20 inferences, so the
tail of the client log is missing). The HINT output was `VELOCITY` for 7 of 13
chunks and "both plausible" for the others. Never "pose delta only".

Important side effect: in `pose_delta` mode the executor divides by dt, so
0.16 m/s of "real" velocity becomes a 1.6 m/s command clamped to 0.2 m/s. The
clamp hid a 10x error.

### 3.2 Run 2 (`mode=velocity`), trimmed

```
[INFO] [915.73] [VLA-base-check] mode=velocity dt=0.100s T=10 raw|max|=(x=0.0232 y=0.0201 th=0.1314) raw|mean|=(x=0.0068 y=0.0054 th=0.0604)
                if VELOCITY -> peak speed=(0.023 m/s, 0.020 m/s, 0.131 rad/s)
                if POSE_DELTA -> implied peak=(0.232, 0.201, 1.314)  [safe clamp vxy<=0.20 vth<=0.40]
[INFO] [920.12] ... raw|max|=(x=0.0014 y=0.0010 th=0.0705)
[INFO] [924.49] ... raw|max|=(x=0.0132 y=0.0020 th=0.0186)
[INFO] [929.22] ... raw|max|=(x=0.0060 y=0.0634 th=0.0332)
[INFO] [933.68] ... raw|max|=(x=0.0807 y=0.0079 th=0.1277)
   ... (11 more, similar: x<=0.10, y<=0.06, th<=0.32) ...
[ERROR][967.46] Got a transition callback on a goal handle that we're not tracking
[INFO] [974.55] ... raw|max|=(x=0.0056 y=0.0135 th=0.3161)
[INFO] [980.93] ... raw|max|=(x=0.0906 y=0.0927 th=0.0544)
[WARN] [982.07] [VLA] Provider returned failure, using classic fallback: VLA reached max_chunks without a completion signal
```

### 3.3 Client-side comparison (first 10 rows, the played prefix)

| | Run 1 (`pose_delta`) | Run 2 (`velocity`) |
|---|---|---|
| Chunks logged | 13 (log incomplete) | 20 |
| Interpreted peak speed | up to 3.4 m/s (clamped to 0.2) | ≤ 0.10 m/s, ≤ 0.32 rad/s |
| Clamp reached? | constantly | never |
| Typical θ mean | 0.02–0.09 | 0.005–0.10 |
| "Not tracking" goal handle ERROR | yes (t = 161.9) | yes (t = 967.5), same point |
| Outcome | log ends before result | `max_chunks` reached, **classic fallback** |

Server rows and client rows line up (for example run 2's last client chunk
x = 0.0906, y = 0.0927 equals the last server chunk's peak, which falls inside
the first 10 rows), so the transport is consistent.

---

## 4. Analysis

### 4.1 Pose delta or velocity?

**Velocity.**

1. Norm-stats saturate at about ±0.30 (a velocity limit), see section 2.
2. Policy outputs have mean θ ≈ 0.03–0.06 per row. As pose deltas this is
   ≈ 2.5 rad of rotation per 50-row chunk and x up to ≈ 1 m/s. Neither is
   sensible for a robot standing at a table.
3. As velocities everything lands comfortably within the clamp (peaks 0.02–0.17
   m/s) and looks like small visual-servoing corrections.

Caveat: we have not directly observed the base moving in the sim. A final check
is to compare odom displacement per chunk to `sum(vel) * dt`.

### 4.2 Are all 11 DOF used?

- **Produced**: yes, every chunk is `(50, 11)`.
- **Executed**: not demonstrated by the logs. Only the base check prints. User
  observation: the **torso (`arm_lift_joint`) did not move** and the **hand barely
  moved**, so the manipulation part is effectively not working.
- The base is commanded for ≈1 s of each 3–4.5 s cycle and with tiny values, so
  it is also nearly a no-op.

### 4.3 What happened in the run?

The VLA ran the full `max_chunks = 20` budget without any completion signal and
then fell back to the classic pick. So any success afterwards is classic, not VLA.

---

## 5. Problems and proposed fixes

### P1. Arm / torso / hand (gripper) barely moves

Symptoms: no `arm_lift` movement, little hand movement, no completion.

Suspects, in the order I would check them:

1. **Execution of the arm prefix.** `_play_arm` sends the 10-row prefix via
   `_send_joint_trajectory` (in `robot_skills/arm/arms.py`) with
   `max_joint_vel` and `timeout=step_timeout`. With 5 references it prepends the
   torso joint names, which is correct, but check:
   - Are the commanded values **absolute joint positions** (what the code assumes)
     or **deltas / normalised values**? Compare actual `/joint_states` against
     `chunk[:, :5]` and see how far apart they are. If the checkpoint predicts
     deltas, the "absolute" interpretation produces tiny or wrong moves (same
     problem as the base convention, but for the arm).
   - Check the joint order against training: `arm_lift, arm_flex, arm_roll,
     wrist_flex, wrist_roll`. A permuted order would move the wrong joints
     (e.g. lift gets a wrist value).
   - Check the unnormalised ranges of the arm columns (mean/std/min/max from the
     server, like the base-check). If `arm_lift` range is ~0.0–0.69 m and the
     predictions sit near the current value, motion is expected to be small.
2. **Prefix too short for the action rate.** 10 rows at the dataset's fps cover
   only ≈ 1 s (at 10 Hz) or less, and the trajectory time spacing is set by
   `max_joint_vel`, not by the dataset dt. Tiny target differences give tiny
   motions. Consider: larger `chunk_prefix_steps`, or send each row at the
   dataset's timestep with `time_from_start = i / fps`.
3. **Trajectory timeouts and overlap.** The recurring
   `Got a transition callback on a goal handle that we're not tracking` error
   (both runs, at the same stage) suggests a goal being cancelled or replaced
   while a result arrives. The first cycle takes ≈ 25 s, so it likely hits
   `step_timeout × len(prefix)` before a result. A new chunk may preempt the
   previous one before it completes, so joints can never reach their targets.
   Fix: wait for the result (not merely a timeout) before re-querying, and log
   the result state of every trajectory.
4. **State mismatch.** The server uses `state_dim=6` (arm5 + gripper), the client
   config advertises 8-D. Verify which values are actually sent and that the
   gripper value is scaled like the training data. A wrongly scaled state skews
   all outputs toward "stay put".
5. **Gripper.** Only a binary open/close, thresholded at
   `gripper_close_threshold = 0.5`, applied from the *last* row of the prefix.
   If the policy outputs gripper values in a different range or sign, the gripper
   would never change. Print the gripper column per chunk.
6. **Sim vs real data gap.** The checkpoint was trained on real AIRoA-style data.
   In Gazebo the camera images look different; the policy may simply output
   "hover" actions (small deltas). Check if the arm columns are near the current
   state (no intent) or far from it (intent that the executor drops).

Recommended diagnostic (cheap, do first): log per chunk, for the arm and gripper
columns, `current state`, `chunk[0]`, `chunk[9]`, and the `/joint_states` after
execution, plus the result state of each trajectory goal.

### P2. Base moves very little

- Only the first 10 of 50 rows are played (≈ 1 s) and the cycle is 3–4.5 s, so the
  base is driven for ≈ 25 % of the time at ≈ 0.01–0.1 m/s.
- Fix options: lengthen the prefix (e.g. 25–50 rows) or replan faster; keep
  `max_base_velocity` clamps. Keep `base_action_mode: velocity`.
- Also reconsider whether the base should move at all for a table pick: it uses
  `force_drive` (no collision avoidance).

### P3. No completion signal, so the VLA always ends in the fallback

- `grasp_position_threshold` is unset. Calibrate the gripper position when
  closed on an apple and set it (`grasp_position_direction: above` is already
  there), or detect a lift of the object.
- Until then, treat `classic fallback` outcomes as VLA failure in evaluation.

### P4. Latency and cycle time

- First chunk: 25 s between the first and second inference in both runs. Add
  timing logs around observation capture, inference, arm/head/gripper/base
  execution to see which part it is (probably arm trajectory waiting on
  `step_timeout`).
- Later cycles: 3–4.5 s of which ≈ 0.4 s is inference; ≈ 1.2 s is the image
  grab (re-subscribe every time). Keep the camera subscribers alive between
  chunks instead of re-subscribing.

### P5. WebSocket keepalive

- Run 1 ended with `1011 keepalive ping timeout` on the server. The client
  blocks for long periods without servicing pings. Fix on the client:
  `websockets` sync client `ping_interval=None` (or large `ping_timeout`), or
  service the connection in a thread. Not seen in run 2's shared log.

### P6. Base convention

- Set `base_action_mode: velocity` (done in `vla.yaml`).
- Update the comment in `vla.yaml` ("Set to pose_delta for AIRoA MoMa") and the
  `HINT` heuristics in `backends.py`, which compare the *peak of a 10-row
  prefix*; that is noisy and printed "both plausible" for small chunks.
  Prefer the norm-stats-based verdict printed by the server.
- Confirm the training dataset fps so that `base_step_duration` (0.1 s) matches.

### P7. Hand image

- `get_hand_rgb` subscribes directly to the hand camera topic (separate from the
  head image), and the payload log shows both `(480, 640, 3)`. Still check in
  Gazebo that the hand topic is really publishing a distinct hand image
  (`use_hand_camera:=true`), because the log only shows a "lazy subscribe" for
  the head topic.

### P8. Minor

- `Value(False)` printed before the VLA connect: unexplained, find the source.
- `torch.from_numpy` "array is not writable" warning: copy the array
  (`np.array(img, dtype=np.uint8)`) in `policy_server.py`.
- Client log is incomplete in run 1 (13 base checks vs 20 inferences). Capture the
  whole run next time.

---

## 6. Suggested next steps (priority order)

1. Add diagnostics (P1): per chunk, log state vs. `chunk[0]`/`chunk[9]` for arm +
   gripper, print unnormalised arm column stats on the server, and log each
   trajectory goal's result state.
2. Determine arm convention (absolute vs delta) from the arm norm-stats, then fix
   `_play_arm` accordingly.
3. Fix the goal-handle/preemption problem: wait for each trajectory result before
   the next query.
4. Extend the executed prefix and keep camera subscribers alive.
5. Calibrate `grasp_position_threshold` to get a real completion signal.
6. Fix client websocket keepalive.
7. Re-run in Gazebo with `velocity`, record rosbag + full logs, and compare to the
   two runs here.
