# Action Server Runbook

## Workspace

- Action-server repository: `/home/amigo/ros/noetic/repos/github.com/tue-robotics/action_server`
- ROS workspace: `/home/amigo/ros/noetic/system`
- Active action-server source: `/home/amigo/ros/noetic/system/src/action_server`
- Robot bring-up: `/home/amigo/ros/noetic/repos/github.com/tue-robotics/hero_bringup`
- VLA policy repository: `/home/amigo/ros/noetic/repos/github.com/tue-robotics/per-group-mse-vla`
- GPSR challenge: `/home/amigo/ros/noetic/repos/github.com/tue-robotics/tue_robocup/challenge_gpsr`

Check that the active ROS source path points to this checkout before debugging a source change.

## Startup

Start the simulator and core robot stack:

```bash
hero-start
```

Start free mode, navigation, world model, and the action server:

```bash
hero-free-mode
```

Optional RViz:

```bash
hero-rviz
```

`hero-free-mode` starts `hero_bringup/launch/action_server.launch`. That launch file owns the action-server runtime arguments, loads the VLA YAML, and starts `action_server/main.py` with visible output. Plain free mode uses `execution_mode:=classic`.

## VLA configuration

The shared launch owner is:

```text
hero_bringup/launch/action_server.launch
```

The parameter-only child is:

```text
hero_bringup/launch/action_server_vla_params.launch
```

The YAML is:

```text
hero_bringup/parameters/action_server/vla.yaml
```

Classic mode:

```bash
roslaunch hero_bringup action_server.launch \
  robot_name:=hero execution_mode:=classic
```

Hybrid mode with the released checkpoint:

Start the policy container first:

```bash
docker run --rm --gpus all \
  -v /home/amigo/.cache/huggingface:/root/.cache/huggingface \
  -v /home/amigo/.cache/huggingface/hub/models--PauMontagut--per-group-mse-smolvla:/model-cache:ro \
  -e POLICY_CHECKPOINT_PATH=/model-cache/snapshots/cb72ca6a3a58e724a3ca8579bea3811f1810be96 \
  -p 8000:8000 \
  smolvla-policy-server
```

Then start ROS and hybrid free mode in another terminal:

```bash
hero-start
export ACTION_SERVER_EXECUTION_MODE=hybrid
export ACTION_SERVER_CHECKPOINT_PATH=/home/amigo/.cache/huggingface/hub/models--PauMontagut--per-group-mse-smolvla/snapshots/cb72ca6a3a58e724a3ca8579bea3811f1810be96
hero-free-mode
```

Do not start `action_server.launch` separately after `hero-free-mode`; free mode already starts it.

The released checkpoint uses `state_indices: [0, 1, 2, 3, 4, 5]`, representing five arm joints plus gripper. The model loader reads modern LeRobot preprocessor and postprocessor files from the checkpoint and retains a legacy fallback for older checkpoints.

For the Docker deployment, install the ROS-side client dependencies:

```bash
python -m pip install -r action_server/requirements-vla.txt
```

Build and run the policy server from the `per-group-mse-vla` repository:

```bash
docker build -f inference/Dockerfile -t smolvla-policy-server .
docker run --rm --runtime=nvidia --gpus all \
  -v /absolute/path/to/huggingface-cache:/model-cache:ro \
  -e POLICY_CHECKPOINT_PATH=/model-cache/snapshots/<snapshot-id> \
  -p 8000:8000 \
  smolvla-policy-server
```

Then select hybrid mode through the shared launch owner's environment variables:

```bash
export ACTION_SERVER_EXECUTION_MODE=hybrid
export ACTION_SERVER_CHECKPOINT_PATH=/absolute/path/to/pretrained_model
hero-free-mode
```

The ROS backend connects to `ws://127.0.0.1:8000` and keeps robot
observation/actuation in the ROS process. The verified image uses Python 3.12,
PyTorch 2.7.1 CUDA 11.8, LeRobot 0.5.1, Transformers 5.3.0, and torchvision
0.22.1. The complete Hugging Face cache is mounted read-only at `/model-cache`.

## Endpoint checks

The actionlib endpoint is:

```text
/hero/action_server/task
```

Its type is `action_server_msgs/TaskAction`. Before starting GPSR:

```bash
rosnode list | grep action_server
rosnode info /hero/action_server
rostopic list | grep /hero/action_server/task
rosparam get /hero/action_server/vla/execution_mode
```

If GPSR prints `Waiting for task action server to come online...`, inspect the action-server terminal first. The usual causes are a missing installed `main.py`, a failed robot initialization, or a missing Python dependency. Do not debug the VLA until `/hero/action_server/task` exists.

## GPSR test

```bash
rosrun challenge_gpsr gpsr.py _robot_name:=hero _test_mode:=true _skip:=true
rosnode kill /hero/hmi/random_answerer
```

## Runtime connections

- `action_server/src/action_server/server.py` creates the actionlib endpoint.
- `action_server/src/action_server/client.py` is the client wrapper used by GPSR.
- `action_server/src/action_server/task_manager.py` delegates recipes to actions.
- `action_server/src/action_server/vla/executor.py` selects classic, hybrid, or VLA behavior.
- `action_server/src/action_server/vla/backends.py` reads HSR observations and sends commands.
- `per-group-mse-vla/inference/policy_server.py` loads and runs the checkpoint in the container.

The ROS action-server Python process only needs the lightweight WebSocket
client dependencies; it must not import LeRobot:

```bash
python -c "import rospy, robot_skills, msgpack, websocket"
```

Test checkpoint loading without ROS or GPSR:

```bash
PYTHONPATH=/home/amigo/ros/noetic/repos/github.com/tue-robotics/per-group-mse-vla:$PYTHONPATH \
python /home/amigo/ros/noetic/repos/github.com/tue-robotics/per-group-mse-vla/inference/local_smoke_test.py \
  --checkpoint /absolute/path/to/per-group-mse-smolvla \
  --device cuda \
  --state-indices 0,1,2,3,4,5
```

The current ROS node uses Python 3.8. If the policy container is unavailable,
the WebSocket backend fails and hybrid mode falls back to classic execution.

Validate the policy image independently:

```bash
docker run --rm smolvla-policy-server \
  python -c "import torch, lerobot, transformers, msgpack, websockets; print(torch.__version__)"
```

## Build and validation

After changing `action_server/setup.py` or installed scripts:

```bash
cd /home/amigo/ros/noetic/system
catkin build action_server --cmake-args -DCMAKE_BUILD_TYPE=Debug
source devel/setup.bash
test -x devel/.private/action_server/bin/main.py
```

Validation order:

1. Check Python syntax and launch XML/YAML.
2. Start `hero-start` and `hero-free-mode`.
3. Verify `/hero/action_server/task`.
4. Test classic mode.
5. Test hybrid mode with the checkpoint.
6. Check camera topics and joint states only after the action server is alive.
