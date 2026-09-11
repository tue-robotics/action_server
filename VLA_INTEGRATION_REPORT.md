# VLA Integration Report

## Scope

This report documents the full Vision-Language-Action (VLA) integration into the
`action_server`, together with the supporting changes in the `per-group-mse-vla`
policy repository. It covers:

- The architecture and why it is layered the way it is.
- Every file changed, and why.
- The end-to-end flow of commands at runtime.
- What is production-ready vs. what is intentionally left as skeleton.
- The 6-DoF (arm + gripper) support added for both training and inference.
- How the design is modular, with an example of adding a new VLA policy.
- Design notes and answers to the questions raised during integration.

## Goals

1. Use the VLA for **manipulation only** (pick / place / hand-over). Navigation,
   "go to", "drive", etc. stay on the classic pipeline.
2. Keep the VLA backend **fully replaceable** — swapping in a different policy
   must not touch challenge or action code.
3. Keep **regression to the classic pipeline** always possible (config switch,
   plus automatic fallback in hybrid mode).
4. Keep ROS Noetic and LeRobot in separate runtimes: ROS owns robot I/O while
  the Python 3.12 policy container owns VLA inference.

---

## Architecture

The integration is deliberately split into independent layers so that data,
transport, and model concerns never leak into each other.

```
Challenge (e.g. GPSR) --recipe--> action_server
  TaskManager                         # task-level delegation (full "vla" mode)
    Action (pick_up / place / hand_over)   # action-level hooks (hybrid mode)
      vla.executor                     # provider interface + config + fallback
        LocalBackendVLAProvider        # provider selection and cache
          backends.BaseBackend         # MODEL interface (replaceable)
            SmolVLAWebSocketBackend    # ROS-side remote policy client
              HSRObservationSource     # robot_skills: cameras + joint state
              WebSocket/msgpack         # observation and action-chunk transport
              HSRActionSink            # robot_skills: arm/gripper/head/base
```

The runtime boundary is:

```text
ROS Python 3.8                         Python 3.12 Docker image
  cameras + joint state --WebSocket-->  LeRobot + SmolVLA
  arm/gripper actuation <--actions-----  checkpoint inference
```

Two interfaces are kept separate on purpose:

- **`BaseVLAProvider`** (in `executor.py`) is the *provider / transport* interface
  used by the action_server. It knows nothing about any specific model.
- **`BaseBackend`** (in `backends.py`) is the *model* interface. A new VLA is one
  new `BaseBackend` subclass. It never touches the provider or challenge code.

Execution modes (`execution_mode` ROS param):

| Mode      | Where VLA is invoked                              | Fallback to classic |
|-----------|---------------------------------------------------|---------------------|
| `classic` | never (VLA disabled)                              | n/a (always classic)|
| `hybrid`  | inside the manipulation action's `_start()`       | yes, automatic      |
| `vla`     | task manager delegates the whole action to VLA    | only if not strict  |

---

## Files Changed — action_server

### Added

1. `action_server/src/action_server/vla/__init__.py`
   - Public module boundary. Exports `maybe_run_vla_action`,
     `maybe_run_vla_manipulation`, `should_delegate_action_to_vla`.

2. `action_server/src/action_server/vla/executor.py`
   - Provider interface and runtime config from ROS params.
   - `VLAConfig`, `ManipulationRequest`, `ManipulationResponse`, `VLAOutcome`.
   - `BaseVLAProvider`, `NullVLAProvider`, `LocalBackendVLAProvider`.
   - `load_vla_config`, `maybe_run_vla_action`, `maybe_run_vla_manipulation`,
     `should_delegate_action_to_vla`.
   - **Threads the live `robot` object** through to the provider/backend so the
     backend can use the `robot_skills` interfaces (cameras, joints, controllers).
   - **Provider caching** (`_PROVIDER_CACHE`) so the VLA model loads **once per
     robot**, not on every action.
   - Carries `raw_sentence` (the original utterance) into `ManipulationRequest`
     so the backend can use it directly as the VLA language prompt.

3. `action_server/src/action_server/vla/backends.py`  ← the model layer
   - `BaseBackend` — abstract model interface (`__init__(robot)`, `execute(request)`).
   - `HSRObservationSource` — reads head camera + 8-D joint state via `robot_skills`.
   - `HSRActionSink` — plays a 6- or 11-wide action chunk on arm/gripper/head/base.
   - `SmolVLALocalBackend` — concrete backend: lazy model load (reusing the
     policy repo's `Server`), closed-loop control, response mapping.

### Modified

4. `action_server/setup.py`
   - Registers the `action_server.vla` subpackage (plus `actions`, `actions.util`).

5. `action_server/src/action_server/task_manager.py`
   - `_VLADelegatedAction` + `_instantiate_action`: in full `vla` mode, actions are
     delegated to the VLA backend (supports new custom action names too).
   - Passes `raw_sentence` from semantics into the delegated action.

6. `action_server/src/action_server/actions/pick_up.py`
7. `action_server/src/action_server/actions/place.py`
8. `action_server/src/action_server/actions/hand_over.py`
   - Each gained an optional VLA-first path in `_start()` (hybrid mode), with the
     classic FSM preserved as fallback. Each passes `raw_sentence` extracted from
     the raw semantics to the VLA.

---

## Files Changed — per-group-mse-vla

### Modified

9. `inference/policy_server.py`
   - `ACTION_LAYOUT_TO_DIM = {"hsr11": 11, "arm6": 6}` (arm5 removed).
   - `resolve_action_dim`, `--action-layout`, `--action-dim`.
   - At inference the layout **slices** the checkpoint output: `arm6` returns the
     first 6 columns (arm + gripper); `hsr11` returns all 11.

10. `eval/per_group_mse.py`
    - `ACTION_LAYOUT_TO_GROUPS` now has `hsr11` (arm/gripper/head/base) and
      `arm6` (arm/gripper). arm5 removed.

11. `training/train_smolvla_generalist.py`
12. `training/train_smolvla_task.py`
    - `ACTION_LAYOUT_TO_DIM = {"hsr11": 11, "arm6": 6}` (arm5 removed).
    - **`build_layout_overrides` now emits a real action-dim override** for reduced
      layouts instead of only warning:
      - `hsr11` → `[]` (native 11-D; retrain-from-checkpoint path unchanged).
      - `arm6`  → `--policy.action_dim=6` (train a 6-D head from scratch).
    - New `--action-dim-key` flag makes the override key adaptable across LeRobot
      versions; `--policy-override` passthrough retained.

---

## Flow of Commands (runtime)

```
1. Challenge sends a recipe (list of action dicts) to the action_server.
2. TaskManager.set_up_state_machine iterates the recipe.
3. For each action, _instantiate_action:
      - full "vla" mode  -> _VLADelegatedAction (skips classic FSM)
      - otherwise        -> classic Action (PickUp / Place / HandOver / ...)
4. Action.configure(...) runs classic perception/planning (find object, pick arm).
5. Action._start():
      hybrid mode -> maybe_run_vla_manipulation(robot, "pick-up", semantics,
                                                 context, raw_sentence)
6. executor.maybe_run_vla_action:
      - load_vla_config(robot_name)                # ROS params
      - action enabled + mode hybrid/vla? else return used=False -> classic
       - _provider_for(robot, ...)                  # CACHED per robot
         -> LocalBackendVLAProvider(robot, config)
           -> backend = SmolVLAWebSocketBackend(robot=robot)
      - provider.execute_manipulation(request)
7. SmolVLAWebSocketBackend.execute(request):
  a. connect to policy_url (the container loads the model once)
      b. loop up to max_chunks:
           - obs = HSRObservationSource.get_observation(instruction)
                head_rgb = robot.perception.get_image()
                hand_rgb = /<robot>/hand_camera/image_raw
                state    = robot.get_joint_states() -> 8-D vector
           - msgpack/WebSocket request -> action chunk (T, action_dim)
           - HSRActionSink.play_chunk(prefix):
                arm     -> arm._send_joint_trajectory
                gripper -> arm.gripper.send_goal(open/close)   # arm6 & hsr11
                head    -> disabled by default
                base    -> disabled by default
           - _is_episode_done()  -> gripper occupancy or calibrated threshold
      c. return {succeeded, actions, message}
8. VLAOutcome -> action._execute_result.
      - VLA used & ok -> done.
      - VLA not used / failed (hybrid) -> classic FSM runs as fallback.
```

---

## Status: Ready vs. Skeleton

### Ready (validated, compiles, wired end-to-end)
- Layered provider/backend architecture; `robot` object and args threaded correctly.
- Provider caching (the WebSocket client loads once per robot).
- Observation via `robot_skills` (`perception.get_image`, `get_joint_states`).
- 8-D state assembly in trained joint order (matches model `STATE_DIM = 8`).
- Actuation via `robot_skills` (`_send_joint_trajectory`, `gripper.send_goal`,
  `head._setHeadReferenceGoal`, `base.force_drive`).
- Variable-width chunk handling: 6 (arm+gripper) and 11 (full) both supported.
- Hybrid fallback to classic on any VLA failure/exception.
- Inference `arm6`/`hsr11` slicing.
- Training `build_layout_overrides` emits a real action-dim override (arm6 vs hsr11).

### Skeleton / placeholder (marked in code with comments)
- **`HSRActionSink._apply_base`** assumes base output is a body-frame velocity;
  disabled by default and unverified against the checkpoint convention.
- **Training a native 6-D model** also needs the **dataset action sliced to 6 dims**;
  the exact LeRobot dataset key is version-specific and must be passed via
  `--policy-override`. The `--policy.action_dim=6` override is emitted, but the
  dataset-side slice is left to the operator.

### Recently completed (previously listed as skeleton)
- **`HSRObservationSource.get_hand_rgb`** now subscribes directly to the HSR hand
  camera topic (`/<robot>/hand_camera/image_raw` by default, overridable via the
  `hand_camera_topic` ROS param) and returns the latest frame, so `hand_rgb` is no
  longer always `None`. `SmolVLALocalBackend.execute` now requests the observation
  with `require_hand=True`, so a missing/late hand camera cleanly falls back to
  classic instead of raising a `KeyError` inside `Server.infer()`. Still requires
  `use_hand_camera:=true` in the robot bring-up.
- **`SmolVLALocalBackend._is_episode_done`** now uses `arm.gripper.occupied_by` to
  detect success: `pick-up` succeeds when the gripper transitions from empty to
  occupied, `place`/`hand-over` succeed when it transitions from occupied to
  empty. Other actions still have no success signal and rely on `max_chunks`.

### Config prerequisites (not code bugs)
- ROS-side `msgpack` and `websocket-client` installed in the Noetic Python 3.8
  environment.
- The policy image is running and reachable at `policy_url`.
- The checkpoint is mounted into the policy container at `/checkpoint`.
- `cv_bridge`, the hand camera, and the ROS robot skills are available.

---

## 6-DoF Support (arm + gripper)

Only two layouts are supported: `arm6` (6) and `hsr11` (11). `arm5` was removed
(it dropped the gripper, so it could not grasp).

### Inference
- `action_layout: arm6` makes `Server.infer()` return `(T, 6)` = arm(5) + gripper(1).
  The checkpoint still computes 11 internally; the first 6 columns are unnormalized
  and returned. `HSRActionSink` actuates arm + gripper and skips head/base.
- `action_layout: hsr11` returns all 11 (head/base actuation is off by default,
  so effectively arm + gripper unless explicitly enabled).

### Training
- `--action-layout hsr11` → native 11-D. Use with `--resume`/`--generalist` to
  **retrain from an existing checkpoint**.
- `--action-layout arm6` → emits `--policy.action_dim=6` to **train a 6-D head from
  scratch**. Also slice the dataset action to 6 dims via `--policy-override` (key is
  LeRobot-version specific). Use `--action-dim-key` to change the override key name.

### Actuation column map (`HSRActionSink`)
```
arm(5)  : arm_lift, arm_flex, arm_roll, wrist_flex, wrist_roll   -> arm6 & hsr11
grip(1) : gripper open/close scalar                              -> arm6 & hsr11
head(2) : head_pan, head_tilt                                    -> hsr11 (off by default)
base(3) : base_x, base_y, base_theta                            -> hsr11 (off by default)
```
The arm(5) slice maps 1:1 onto `_send_joint_trajectory`, which auto-prepends the
torso (`arm_lift`) joint when given 5 references.

---

## Modularity: adding a new VLA policy

A new policy is one file: a `BaseBackend` subclass. Nothing else changes.

```python
# action_server/src/action_server/vla/backends.py (or a separate module)

class MyCoolVLABackend(BaseBackend):
    def __init__(self, robot):
        BaseBackend.__init__(self, robot)
        self._obs = HSRObservationSource(robot)   # reuse observation layer
        self._sink = HSRActionSink(robot)         # reuse actuation layer
        # load your model here (or lazily on first execute)

    def execute(self, request):
        obs = self._obs.get_observation(request.raw_sentence or request.action_name)
        if obs is None:
            return {"succeeded": False, "actions": [], "message": "no observation"}
        chunk = my_model.predict(obs)             # (T, 6) or (T, 11)
        ok = self._sink.play_chunk(chunk[: 10], step_timeout=10.0)
        return {"succeeded": ok, "actions": chunk.tolist(), "message": ""}
```

Then point one ROS param at it — no code in challenges, actions, or the task
manager is touched:

```
/<robot>/action_server/vla/local_backend_class: action_server.vla.backends:MyCoolVLABackend
```

Because the provider layer is transport-only and the observation/actuation layers
(`HSRObservationSource`, `HSRActionSink`) are reusable, most new policies only
implement `execute()` and the model load. To regress to classic entirely, set
`execution_mode: classic` or `provider: none`.

---

## Key ROS parameters

All under `/<robot>/action_server/vla`:

| Param                    | Default        | Meaning                                            |
|--------------------------|----------------|----------------------------------------------------|
| `execution_mode`         | `classic`      | `classic` / `hybrid` / `vla`                       |
| `provider`               | `none`         | `none` / `local`                                   |
| `enabled_actions`        | pick/place/handover | which actions may use the VLA                 |
| `local_backend_class`    | `""`           | dotted path to the `BaseBackend` subclass          |
| `checkpoint_path`        | `""`           | VLA checkpoint directory                           |
| `device`                 | `cuda`         | inference device                                   |
| `action_layout`          | `hsr11`        | `arm6` (arm+gripper) or `hsr11`                    |
| `max_chunks`             | `20`           | max closed-loop re-query iterations                |
| `chunk_prefix_steps`     | `10`           | steps of each chunk executed before re-querying    |
| `enable_head_motion`     | `False`        | actuate head from VLA output                        |
| `enable_base_motion`     | `False`        | actuate base from VLA output                        |
| `strict_mode`            | `False`        | if true, VLA failure aborts instead of falling back |

---

## Design Notes & Discussion (Q&A)

**What is an action chunk?**
A VLA predicts a short *sequence* of future actions per forward pass, shape
`(T, action_dim)` with `T ≈ 50`. The robot plays a prefix, then re-queries. This
is standard for diffusion/flow policies — predicting a horizon is smoother and
more robust than single-step prediction.

**Why is the state input 8-D but the action output 11-D? Doesn't every output
need to be an input?**
No — input and output spaces are chosen independently:
- **Input** = what helps the policy *decide*: 2 camera images + 8-D proprioception
  (`5 arm + 1 gripper + 2 head`) + the instruction.
- **Output** = what the policy *commands*: `11 = arm5 + gripper1 + head2 + base3`.
The base is **not** in the state because base odometry (drifting, arbitrary origin)
carries no manipulation-relevant signal, whereas arm/gripper/head joint angles are
essential proprioception. The base can still be *commanded* because base motion is
**visually driven** (relative velocity/delta commands from the cameras), so it needs
no base-state input. The "pass all outputs back as inputs" intuition comes from
autoregressive models; this is a feedforward visuomotor policy where the two spaces
are independent by design.

**Do we make the VLA use only 6 DoF (arm+gripper) and not 11?**
Yes for our use case. The model still *predicts* 11 internally, but we only use the
first 6 columns (arm+gripper); head is pre-positioned and base/navigation are owned
by the classic pipeline. `arm5` was a mistake (no gripper → cannot grasp) and was
removed.

**Training on 11-D but using only 6 at inference — does that make sense?**
It works and is a reasonable shortcut when you already have an 11-D checkpoint: the
layout is fixed by column position, so slicing to 6 always yields arm+gripper. The
downside is some model capacity and the per-group loss went into head/base you then
discard. For an optimal 6-D model, train natively with `--action-layout arm6`.

**Hybrid vs. full VLA mode?**
- *Hybrid*: classic pipeline does perception/planning; the VLA only replaces motor
  execution inside pick/place/hand-over, with automatic fallback to the classic FSM.
- *Full (`vla`)*: the task manager delegates the whole action to the VLA (also allows
  new custom action names), no classic configuration.
Hybrid is recommended for on-robot rollout; full is for research/eval.

**Why pass `raw_sentence`?**
The VLA is conditioned on a natural-language instruction. The NLP pipeline already
produces the parsed semantics *from* the spoken sentence, so we thread the original
sentence through to the backend and use it directly as the prompt — no prompt
reconstruction needed.

**Why does the backend need the live `robot` object?**
So all I/O goes through `robot_skills` (cameras, joint states, arm/gripper/head/base
controllers) rather than raw ROS subscribers — consistent with the rest of
tue_robocup and avoiding duplicate subscribers/action clients.

**Runtime status.**
The `local` provider name is retained for configuration compatibility, but its
backend is `SmolVLAWebSocketBackend`. The ROS process does not load the model;
the Docker policy server performs inference.

## Current checkpoint loader

`per-group-mse-vla/inference/policy_server.py` now loads modern LeRobot
checkpoints from their own `config.json`, `policy_preprocessor.json`, and
`policy_postprocessor.json`. This lets the checkpoint define its policy type,
state/action dimensions, image resizing, empty-camera handling, tokenization,
normalization, and action unnormalization. Older checkpoints without processor
files retain the legacy normalization-statistics fallback.

The released `PauMontagut/per-group-mse-smolvla` checkpoint consumes six state
values. The ROS configuration selects the first five arm joints and gripper
through `state_indices: [0, 1, 2, 3, 4, 5]`. Other checkpoints can use a
different state contract by changing that parameter without changing the
loader.

## Startup and GPSR validation

The actionlib endpoint is created by `action_server/src/action_server/server.py`:

```text
/<robot_name>/action_server/task
```

For Hero this is `/hero/action_server/task`. GPSR waits for this endpoint while
constructing its action client. A message such as
`Waiting for task action server to come online...` means that the action-server
node is absent or died during startup; the client and server action names match.

`hero_bringup/launch/action_server.launch` is the reusable startup surface. It
loads `hero_bringup/parameters/action_server/vla.yaml`, applies
`checkpoint_path` and `execution_mode`, and starts `action_server/main.py` with
screen output. `free_mode.launch` includes this launch file.

Verify the endpoint before starting GPSR:

```bash
rosnode list | grep action_server
rosnode info /hero/action_server
rostopic list | grep /hero/action_server/task
rosparam get /hero/action_server/vla/execution_mode
```

Plain `hero-free-mode` defaults to classic mode. For hybrid validation, start
the policy container and export `ACTION_SERVER_EXECUTION_MODE=hybrid` plus
`ACTION_SERVER_CHECKPOINT_PATH` before running `hero-free-mode`. Only start
GPSR after the action endpoint is visible.

The ROS backend no longer imports LeRobot. It imports only the lightweight
WebSocket client dependencies and sends observations to the policy container.
The container runs Python 3.12, LeRobot 0.5.1, PyTorch 2.4.1 CUDA 12.1,
Transformers 4.46.3, and torchvision 0.19.1.

## Docker deployment

The policy image is built from `per-group-mse-vla/inference/Dockerfile`:

```bash
cd /home/amigo/ros/noetic/repos/github.com/tue-robotics/per-group-mse-vla
docker build --progress=plain -f inference/Dockerfile \
  -t smolvla-policy-server .
```

The verified image tag is `smolvla-policy-server:latest`. It contains no
checkpoint; mount the downloaded Hugging Face snapshot read-only:

```bash
docker run --rm --gpus all \
  -v /home/amigo/.cache/huggingface/hub/models--PauMontagut--per-group-mse-smolvla/snapshots/cb72ca6a3a58e724a3ca8579bea3811f1810be96:/checkpoint:ro \
  -p 8000:8000 \
  smolvla-policy-server
```

The container listens on `ws://127.0.0.1:8000` from the ROS host. It receives
head/hand RGB images, the six selected state values, and the raw instruction;
it returns an 11-column action chunk. `POLICY_STATE_INDICES` defaults to
`0,1,2,3,4,5` and can be overridden for another checkpoint.

In a second terminal, install the ROS-side client dependencies once:

```bash
python -m pip install -r \
  /home/amigo/ros/noetic/repos/github.com/tue-robotics/action_server/action_server/requirements-vla.txt
```

Then start the simulator, ROS stack, container, and hybrid action server:

```bash
hero-start
export ACTION_SERVER_EXECUTION_MODE=hybrid
export ACTION_SERVER_CHECKPOINT_PATH=/home/amigo/.cache/huggingface/hub/models--PauMontagut--per-group-mse-smolvla/snapshots/cb72ca6a3a58e724a3ca8579bea3811f1810be96
hero-free-mode
```

The action server must be verified before GPSR:

```bash
rosnode info /hero/action_server
rostopic list | grep /hero/action_server/task
rosparam get /hero/action_server/vla/execution_mode
```

The final image build was validated after removing unused `pynput/evdev`,
avoiding LeRobot's dependency re-resolution of a second Torch version, and
pinning the compatible Transformers/torchvision versions. Failed intermediate
SmolVLA image artifacts were not retained; unrelated Docker images were left
untouched.

### GPSR wait failure: `main.py` not installed

If GPSR waits indefinitely while `roslaunch` logs
`Cannot locate node of type [main.py] in package [action_server]`, the action
server was never started. The ROS package previously installed the Python
modules but omitted `scripts/main.py` from `setup.py`. The script is now
registered and must be rebuilt:

```bash
cd /home/amigo/ros/noetic/system
catkin build action_server --cmake-args -DCMAKE_BUILD_TYPE=Debug
source devel/setup.bash
```

Confirm the installed node exists before retrying:

```bash
test -x devel/.private/action_server/bin/main.py
```
