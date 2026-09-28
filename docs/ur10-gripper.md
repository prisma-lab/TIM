# Step 2: a gripper primitive in Gazebo, executed by SEED

This exercise uses the **UR10 and Robotiq 2F-85** from the CRF assembly scene.
The two primitives are `open_gripper` and `close_gripper`. Closing means reaching
a finger-joint position; detecting and maintaining an object grasp is a later step.

## Build and start the simulation container

From the TIM repository on the host:

```bash
./docker_sim_build.sh
./docker_sim_run.sh
```

The build script expects the scene package at
`../use_case_crf/ros2_ws/src/use_case_sim`. You can supply a different path:

```bash
./docker_sim_build.sh /path/to/ros2_ws/src/use_case_sim
```

`Dockerfile.sim` extends the existing `tim_img`, installs Gazebo Classic and the
robot descriptions, and copies the complete `use_case_sim` package into the image.
The copied `assembly_task.launch.py` is the simulation entry point. The Robotiq
description revision matches the submodule recorded in the CRF repository.
Rebuild this image to pick up changes to the external scene package.

The run script creates or starts `tim_ur10` and opens a shell inside it. It uses
the desktop's X11 authentication for Gazebo/RViz. The host needs `xauth` and an
active graphical session. TIM's `src` folder is mounted for development; build
and install directories stay inside the container.
Both `docker_sim_run.sh` and `docker_attach.sh` refresh the mounted display cookie
for simulation containers after a new desktop login. See the
[GUI troubleshooting steps](pick-place-design.md#if-gazebo-or-rviz-does-not-open)
if an older attached shell reports an invalid X11 cookie.

**Run every terminal below inside `tim_ur10`.** ROS Humble, both workspaces, and
`ROS_DOMAIN_ID=119` are configured automatically. Other TIM/CRF containers may use
different domains and are not part of this exercise.

## First control the gripper directly

In terminal 1, inside the container:

```bash
ros2 launch use_case_sim assembly_task.launch.py
```

Wait for Gazebo to load and for the log to show that
`robotiq_gripper_controller` is configured and active.

In a second host terminal, attach and start the primitive manager:

```bash
./docker_attach.sh tim_ur10
ros2 launch ur10_primitives gripper.launch.py
```

In a third host terminal, attach and send a command:

```bash
./docker_attach.sh tim_ur10
ros2 topic pub --once /ur10/gripper/command std_msgs/msg/String \
  "{data: close_gripper}"
```

Watch the fingers close. Terminal 2 reports `accepted`, `running`, then
`succeeded`. Open them again with:

```bash
ros2 topic pub --once /ur10/gripper/command std_msgs/msg/String \
  "{data: open_gripper}"
```

An already satisfied target produces `succeeded` without a new trajectory.
The manager also latches completed commands to suppress repeated SEED requests;
use `reset` to intentionally repeat the same command (see below).
You can also watch machine-readable results in another attached terminal:

```bash
ros2 topic echo /ur10/gripper/status
```

The measured finger joint is `robotiq_85_left_knuckle_joint`. The default targets
are **0.0 rad open** and **0.75 rad closed**, with a 0.02 rad tolerance. These are
joint angles, not gripper widths in meters. Parameters are in
[`gripper.yaml`](../src/ur10_primitives/config/gripper.yaml).

## Let SEED execute the same primitives

Keep the scene and primitive manager running. In another host terminal:

```bash
./docker_attach.sh tim_ur10
ros2 run seed seed ur10
```

At the SEED console, type:

```text
gripper_demo
```

SEED executes `hardSequence([close_gripper,open_gripper])`. It waits for the closed
state before starting the open task, then reports `sequence accomplished!`.
Type `listing` to inspect working memory.

You can instead activate the task from an attached ROS terminal:

```bash
ros2 topic pub --once /seed_ur10/stream std_msgs/msg/String \
  "{data: gripper_demo}"
```

To repeat a completed demo, first type `forget(gripper_demo)` at the SEED console,
then type `gripper_demo` again. Type `q` to exit SEED.

## What each piece does

| Interface | Purpose |
| --- | --- |
| `/seed_ur10/stream` | Requests a SEED task such as `gripper_demo`. |
| `/ur10/gripper/command` | Accepts `open_gripper`, `close_gripper`, `cancel`, `stop`, or `reset` as `std_msgs/String`. |
| `/robotiq_gripper_controller/follow_joint_trajectory` | The Gazebo controller action used to execute a finger trajectory. |
| `/joint_states` | Supplies the measured finger position. |
| `/ur10/gripper/status` | Publishes JSON status messages for diagnosis. |
| `/seed_ur10/state` | Publishes observed goals (`gripper.open`, `gripper.closed`), the failure latch, and execution facts such as `succeeded(close_gripper)`. A leading `-` means false. |

The C++ [primitive manager](../src/primitive_manager/src/manager.cpp) loads the
[gripper plugin](../src/ur10_primitives/src/gripper_primitive.cpp) twice: once for
`open_gripper` and once for `close_gripper`. Each instance has its own target in
YAML. After controller success, the plugin checks a new joint-feedback sample
before declaring success. It also observes the initial gripper position, so SEED
can recognize a goal that is already satisfied. See the
[manager/plugin walkthrough](primitive-manager.md) for the code hierarchy and
how to add another primitive.

The task definitions are in
[`seed_ur10_LTM.prolog`](../src/seed/LTM/seed_ur10_LTM.prolog). For example:

```prolog
schema(open_gripper, [
    [rosAct(open_gripper,ur10_gripper,ur10/gripper/command,0.25),0,[-gripper.failed]]
], [gripper.open], []).
```

- `open_gripper` names the task.
- Its child `rosAct` publishes the command while the task is active.
- The releaser `-gripper.failed` inhibits commands after a failure.
- The goal `gripper.open` comes from the plugins' observations.

`rosAct` can repeat its command while waiting. The manager ignores repeats
during an active motion and rejects conflicting commands. Stale feedback clears
the open/closed facts. A controller failure is latched instead of being retried
continuously by SEED.

After resolving a controller failure, clear that latch with:

```bash
ros2 topic pub --once /ur10/gripper/command std_msgs/msg/String "{data: reset}"
```

A repeated completed command returns its previous result. To intentionally run
the same command again, send `reset` first. Switching to a different primitive
also permits a new invocation. The close/open demo therefore repeats normally.

Reset commands do not move the gripper. A timed-out action must finish or confirm
cancellation before another command is accepted. If the action server disappears
without returning a result, recover the controller and restart the manager.

## Checks and lifecycle

The protocol tests use a controllable ROS action server. From an attached shell:

```bash
ROS_DOMAIN_ID=122 python3 -m pytest -q -p no:cacheprovider \
  /home/user/ros2_ws/src/ur10_primitives/test/test_gripper_protocol.py
```

They cover successful motion, duplicate/conflicting commands, controller rejection
and abort, a false success result, unavailable controllers, stale feedback, and
timeouts with a stopped simulation clock. They also check confirmed cancellation,
late goal acceptance, repeat/reset behavior, malformed commands, plugin failures,
and adding another plugin instance through configuration.

The SEED state subscriptions use a queue of 100 messages so a burst containing
multiple distinct facts reaches working memory. The original single-message
queue dropped the earlier facts in these updates.

Closing attached shells leaves the container running. Stop the applications with
Ctrl+C (or `q` for SEED), then stop the container from the host:

```bash
docker stop tim_ur10
```

Use `./docker_sim_run.sh` to restart it and refresh GUI authentication. After an
image rebuild, an existing container still uses its original image; create a new
named container with `./docker_sim_run.sh tim_ur10_new` to try the rebuilt image.

The next exercise adds [parametric pick-and-place plugins](pick-place-design.md)
with a replaceable target-pose publisher.
Here SEED executes a sequence written in its task library. TIM's Fast Downward
task planner is not yet generating that sequence; connecting generated plans to
these primitives is a later exercise.
