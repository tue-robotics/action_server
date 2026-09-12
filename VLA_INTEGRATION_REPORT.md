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
   - `SmolVLAWebSocketBackend` — concrete backend: WebSocket policy client,
     closed-loop control, and response mapping. `SmolVLALocalBackend` remains
     an experimental in-process compatibility path.

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
- The complete Hugging Face model cache is mounted into the policy container at
  `/model-cache`, with `POLICY_CHECKPOINT_PATH` pointing at its snapshot.
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

### Adding a new VLA: implementation guide

The preferred integration unit is a policy client, not a new action or a new
GPSR path. A new VLA should consume the stable observation contract and return
the stable named action contract:

```text
ObservationAdapter -> PolicyClient -> ActionAdapter
        instruction + observation -> action chunk
                         CompletionPolicy -> rollout result
```

#### What must change

For a policy that uses the existing HSR observation and action conventions:

1. Create a `BaseBackend` implementation, preferably in a separate module such
   as `action_server/vla/backends/my_policy.py`.
2. Implement `__init__(robot)` and `execute(ManipulationRequest) -> dict`.
3. Load the model lazily and keep it on the backend instance so provider
   caching prevents reloads for every action.
4. Convert the policy output into a `(T, 6)` or `(T, 11)` `float32` chunk in
   the documented units. Do not let the policy client call robot controllers.
5. Reuse `HSRObservationSource`, `HSRActionSink`, and the rollout loop only if
   the new VLA has the same camera, state, action order, units, and completion
   semantics.
6. Add a `local_backend_class` parameter selecting the new backend. No GPSR,
   challenge, or action-file change should be necessary.

For a remote policy, create a small `PolicyClient` transport object instead of
subclassing the SmolVLA backend. It should own connection, serialization,
schema validation, timeout, and response decoding. The shared rollout runner
should own observation acquisition, prefix execution, retries, and fallback.

#### Objects and contracts to define

| Object | Responsibility | Must not contain |
|---|---|---|
| `ObservationAdapter` | Named images, state, instruction, timestamps | Model imports |
| `PolicyClient` | Load/connect to one VLA and predict chunks | ROS action execution |
| `ActionAdapter` | Validate units/capabilities and command the robot | Checkpoint loading |
| `CompletionPolicy` | Decide success from task and robot feedback | Transport details |
| `RolloutRunner` | Closed-loop observation, prefix, retry, fallback | Model-specific preprocessing |

The minimum policy-client contract should declare input names and shapes,
state ordering, image color/encoding, action names and dimensions, units,
latency expectations, and whether actions are absolute positions, deltas, or
velocities. Validate this contract at startup and fail with a readable report
instead of silently slicing columns by position.

#### Configuration-only versus code changes

Configuration is sufficient when the new VLA matches the existing HSR contract:

```yaml
provider: local
local_backend_class: action_server.vla.backends.my_policy:MyPolicyBackend
checkpoint_path: /path/to/checkpoint
policy_url: ws://127.0.0.1:8001       # only for a remote client
action_layout: arm6                   # or hsr11
state_indices: [0, 1, 2, 3, 4, 5]
```

Code changes are required when it changes camera names, state dimensions,
action ordering, normalization, action units, transport, or completion logic.
Those changes belong in the corresponding adapter/client, not in GPSR,
`PickUp`, `Place`, `HandOver`, or `TaskManager`.

#### Test and rollout checklist

1. Run a policy-only smoke test with synthetic observations and require finite
   actions with the declared shape.
2. Run a transport test against the server and verify binary framing, schema,
   response shape, and measured latency.
3. Run the client with recorded observations before commanding hardware.
4. Test the action adapter with motion disabled and inspect units/ranges.
5. Test classic navigation and pre-grasp independently.
6. Test the VLA final-manipulation phase after navigation, with a calibrated
   completion signal and a bounded fallback.
7. Test interruption, timeout, missing camera, malformed response, and policy
   process death. Every failure must close the client and return to classic
   behavior in hybrid mode.

#### Likely bottlenecks and failure modes

- **Model startup:** VLM weights and processor caches can take tens of seconds;
  keep the policy process alive and mount a persistent Hugging Face cache.
- **GPU/runtime:** CUDA, NVIDIA container pass-through, Torch, and checkpoint
  versions must be validated before ROS starts.
- **Serialization:** nested image lists are slow and error-prone; use binary
  frames with explicit shape and dtype, as the current WebSocket protocol does.
- **Inference latency:** synchronous inference must not block WebSocket
  keepalive; run it outside the event loop and set client/server timeouts above
  the measured worst-case latency.
- **Camera readiness:** both camera topics must be live, RGB, and convertible
  to the declared resolution/encoding before the first request.
- **Action semantics:** a 6-column output is not interchangeable with 11
  columns unless the action adapter explicitly maps names, units, and limits.
- **Completion:** gripper occupancy is HSR-specific and may be unavailable to
  a new robot or policy. Provide a policy/task-specific completion adapter.
- **Orchestration:** current hybrid hooks run before classic navigation. Do not
  claim end-to-end task success until the phase-boundary refactor is complete.
- **Fallback safety:** fallback after partial VLA motion needs a known robot
  state and a safe recovery policy; blindly starting a classic FSM can issue
  conflicting goals.

#### Adding a policy without changing the action server

The target workflow is:

```text
new policy package
  -> PolicyClient + manifest/contract
  -> configured backend factory
  -> shared ObservationAdapter/RolloutRunner/ActionAdapter
  -> unchanged GPSR and action definitions
```

If a new VLA requires edits to GPSR or challenge semantics, that is evidence
that a policy-specific concern has leaked across an abstraction boundary and
should be moved back into the policy client or adapter.

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

`hero_bringup/launch/action_server.launch` is the reusable parameter/startup
surface. It loads `hero_bringup/parameters/action_server/vla.yaml` and applies
`checkpoint_path` and `execution_mode`. In the current VS Code workflow its
action-server node is commented out so `(debugpy) Launch Action Server` owns
the process; `free_mode.launch` still includes this launch file for parameters.

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
The container runs Python 3.12, LeRobot 0.5.1, PyTorch 2.7.1 CUDA 11.8,
Transformers 5.3.0, and torchvision 0.22.1.

## Docker deployment

The policy image is built from `per-group-mse-vla/inference/Dockerfile`:

```bash
cd /home/amigo/ros/noetic/repos/github.com/tue-robotics/per-group-mse-vla
docker build --progress=plain -f inference/Dockerfile \
  -t smolvla-policy-server .
```

The verified image tag is `smolvla-policy-server:latest`. It contains no
checkpoint; mount the complete Hugging Face model cache read-only:

```bash
docker run --rm --gpus all \
  -v /home/amigo/.cache/huggingface:/root/.cache/huggingface \
  -v /home/amigo/.cache/huggingface/hub/models--PauMontagut--per-group-mse-smolvla:/model-cache:ro \
  -e POLICY_CHECKPOINT_PATH=/model-cache/snapshots/cb72ca6a3a58e724a3ca8579bea3811f1810be96 \
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

## Current rollout audit and required follow-up

The end-to-end transport and inference path is working, but the first robot
rollout exposed an orchestration limitation that must be treated separately
from model inference:

```text
current hybrid path:  VLA manipulation -> max_chunks -> classic fallback
desired GPSR path:   classic navigation/pre-grasp -> VLA manipulation
```

`PickUp._start()`, `Place._start()`, and `HandOver._start()` currently invoke
the VLA before their classic state machines. `Grab` owns navigation to the
object, arm preparation, and grasping, so a VLA-first pickup can command only
the arm/gripper at the robot's current location. With base motion disabled,
this explains the observed gripper movement before classic navigation begins.

### Required orchestration fix

Keep navigation and pre-grasp setup in the classic pipeline, then invoke the
VLA only for the final manipulation segment. This should be implemented behind
an explicit action-phase interface, not by duplicating `Grab` internals inside
the VLA backend. The phase contract should provide:

- resolved object and arm designators;
- the post-navigation/pre-grasp robot state;
- the action instruction and semantic context;
- a classic fallback for the final manipulation phase.

Until that phase boundary exists, `hybrid` is useful for transport and closed-
loop actuation tests but is not a valid end-to-end GPSR pickup evaluator.

### Redundancy audit

The following parts are intentionally retained for compatibility, but should
be simplified in a follow-up refactor:

- `SmolVLALocalBackend` contains the in-process LeRobot path while
  `SmolVLAWebSocketBackend` subclasses it only to replace `_load_policy()` and
  `_infer()`. Once the Docker transport is the supported deployment, split the
  shared rollout loop into a `ClosedLoopManipulationRunner` and make local and
  WebSocket inference small policy clients, or remove the local client behind
  an explicit experimental flag.
- Image conversion, state conversion, and msgpack framing are currently spread
  between `HSRObservationSource`, `SmolVLAWebSocketBackend`, and the policy
  server. Keep one transport codec with explicit `uint8` RGB and shape fields;
  do not add more ad-hoc `tolist()`/dtype conversions.
- The `maybe_run_vla_manipulation()` wrapper is retained for older action call
  sites. New code should call `maybe_run_vla_action()` directly; remove the
  wrapper after all downstream users migrate.
- The repeated active-source and repository-source copies are a workspace
  deployment constraint, not two implementations. Keep them synchronized via
  the build/link workflow rather than making independent edits.

### VLA-specific assumptions to remove

These assumptions currently limit policy swapping:

- `HSRObservationSource` hardcodes HSR joint names, two camera roles, and an
  eight-value state vector. Move this into a robot observation adapter selected
  by a parameter or backend factory.
- `HSRActionSink` hardcodes the 5+1+2+3 HSR column layout and calls private
  robot-skills methods such as `_send_joint_trajectory`. Introduce an action
  adapter with named outputs, unit conversion, limits, and capability checks.
- `SmolVLAWebSocketBackend` hardcodes the msgpack schema and the HSR image/state
  contract. Define a versioned policy protocol (`schema`, image names, state
  names, action names, and dimensions) and negotiate or validate it at connect.
- `policy_server.py` assumes LeRobot processors, SmolVLA-compatible processor
  keys, and the `hsr11` action layout. Move checkpoint-specific adaptation into
  a policy adapter selected from checkpoint metadata.
- Completion is HSR/gripper-specific: `occupied_by`, gripper position
  thresholds, and action names `pick-up`, `place`, `hand-over` are embedded in
  the generic rollout. Move completion into a policy/task result adapter.
- `device: cuda`, Python 3.12, LeRobot, and the checkpoint state indices are
  deployment-specific. Keep them in backend/container configuration and fail
  with a capability report when a replacement VLA does not match them.

### Recommended target abstraction

The stable interfaces should be:

```text
ObservationAdapter  -> named RGB/state observation
PolicyClient        -> instruction + observation -> named action chunk
ActionAdapter       -> named action chunk -> robot commands
CompletionPolicy    -> action request + robot state -> success/failure
RolloutRunner       -> closed-loop orchestration and fallback
```

With those boundaries, replacing SmolVLA changes only `PolicyClient` and its
configuration. Replacing the HSR or robot skills changes only the adapters;
GPSR, task management, and action definitions remain unchanged.
```

### Things to check:
how to enable all the DOF to control also the movement of the head and wheels ?
def execute at backends.py line 414 - 422 is the gripper_occupied_at_start = gripper_occupied_at_start or grasped_at_start correct ?
