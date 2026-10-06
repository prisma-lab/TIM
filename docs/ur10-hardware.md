**UR10 CB3 and Robotiq 2F-140 hardware primitives**

This guide describes the original direct-control backend. If your hardware team
provides motion/gripper services, use the separate
[service adapters guide](hardware-service-primitives.md) and `services.launch.py`.

`ur10_primitives` is the Gazebo UR10/2F-85 demo. The separate
`ur10_hardware_primitives` package implements the physical UR10 CB3 and USB/RS-485
Robotiq 2F-140. Both use the same SEED tasks and primitive commands.

| Primitive | Hardware behavior |
| --- | --- |
| `move_a_b(pick)` / `move_a_b(place)` | MoveIt plans travel from the measured TCP to the target's approach pose. |
| `pick` | Open, descend, close, require detected object contact, attach its collision model, and retreat. |
| `place` | Descend, open, keep the released object in the world collision model, and retreat. |
| `open_gripper` / `close_gripper` | Command the physical gripper and check fresh measured feedback. Closing alone can succeed without an object; `pick` cannot. |

Pick/place require the preceding move to reach the appropriate approach pose.
Their targets are **contact poses of the `grasp_tcp` link**, not object centres
or the bare UR flange. Approach/retreat adds 0.10 m along `base_link` Z by default.
Numeric poses stay outside SEED, as in the simulation.

**Controllers and feedback**

- [`controllers.yaml`](../src/ur10_hardware_primitives/config/controllers.yaml)
  configures the arm's `ur_controllers/ScaledJointTrajectoryController`, the
  gripper's `position_controllers/GripperActionController`, and joint feedback.
  It is merged over the installed official UR controller configuration so the
  driver's status and speed-scaling controllers are retained. CB3 uses 125 Hz.
- [`moveit_controllers.yaml`](../src/ur10_hardware_primitives/config/moveit_controllers.yaml)
  routes arm trajectories to
  `/scaled_joint_trajectory_controller/follow_joint_trajectory`. MoveIt's duration
  monitor is disabled to accommodate UR speed scaling; the primitives retain
  a separate configurable 180-second timeout per motion stage.
- The gripper uses `/robotiq_gripper_controller/gripper_cmd`
  (`control_msgs/action/GripperCommand`). Its position is the driver joint angle:
  0.0 rad open and 0.695 rad closed for this 2F-140 description. Humble's controller
  does not forward per-goal force/speed to this driver; configure their multipliers
  in `cell.yaml` instead.
- `/dynamic_joint_states` must include `finger_joint` interfaces `position`,
  `object_status`, and `gripper_fault`. The `pick` primitive requires object status
  2: contact detected while closing. Full closure without detection is a failed
  pick. Stale feedback, faults, or lost detection during loaded travel cause a
  failure and cancellation request.

One manager owns all five plugins. `/ur10/gripper/command` is an alias for its
main `/ur10/primitives/command` input, so the existing SEED `gripper_demo` also
uses the same execution slot. Status is published on `/ur10/primitives/status`;
SEED facts go to `/seed_ur10/state`. Failure latches both `manipulation.failed`
and `gripper.failed`.

The hardware image pins the [Robotiq ROS driver](https://github.com/robotiq/ros/tree/8d7b8412ad685ffe1db5719da6e8fce6c1896e5e)
and applies a small [feedback-health patch](../src/ur10_hardware_primitives/patches/README.md).
It invalidates cached status during a serial disconnect. A new ROS timestamp
alone does not prove the device supplied a new reading. Retain this patch or
an equivalent health check if using another driver installation.
The arm interface follows the [official UR Humble driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble).

**Prepare the measured configuration**

The combined model is supplied in
[`ur10_2f140.urdf.xacro`](../src/ur10_hardware_primitives/urdf/ur10_2f140.urdf.xacro).
You do not need an existing combined URDF. The arm uses your UR calibration;
the gripper uses the pinned vendor model. The mounting bracket has a configurable
bounding box, and `grasp_tcp` has a configurable fixed transform.

Copy these files inside the container's source workspace:

```bash
cd /home/user/ros2_ws
cp src/ur10_hardware_primitives/config/cell.yaml \
   src/ur10_hardware_primitives/config/cell.local.yaml
cp src/ur10_hardware_primitives/config/targets.yaml \
   src/ur10_hardware_primitives/config/targets.local.yaml
```

Fill `cell.local.yaml` with:

| Setting | Required information |
| --- | --- |
| `robot_ip` | The physical UR10 controller's IP. |
| `kinematics_file` | Absolute in-container path to this robot's extracted UR calibration YAML. |
| `serial_port` | `/dev/robotiq` when using the helper below. |
| `mount_xyz`, `mount_rpy` | Measured `tool0` → mounting bracket transform, in metres/radians. |
| `mount_dimensions`, `mount_box_xyz` | A conservative box covering the bracket, in its frame. |
| `tcp_xyz`, `tcp_rpy` | Calibrated TCP relative to `robotiq_140_base_link`. The vendor model includes a base rotation; inspect this frame before entering values. |
| `object_dimensions`, `object_offset_*` | Workpiece bounding box and its pose relative to `grasp_tcp` while grasped. |
| `obstacles` | Measured table and fixture boxes in `base_link`. |
| Gripper multipliers | Speed and force settings for your workpiece. |

The stock values are placeholders. After entering the measured settings, set
`configured: true`. Launch refuses missing settings and unconfigured geometry.
Configure the pendant's tool/payload settings consistently with the actual tool;
the ROS model does not set those pendant values.

Use the official driver's calibration procedure for `kinematics_file`, rather
than the nominal UR description. The External Control URCap/program must also
be installed and configured for the ROS computer. See the [UR driver setup guide](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/installation/robot_setup.html).

Fill `targets.local.yaml` with measured `pick_pose` and `place_pose` arrays:
`[x, y, z, qx, qy, qz, qw]`, using floating-point values and unit quaternions.
The hardware target publisher has no Gazebo pose defaults. Perception can replace
it by publishing timestamped `geometry_msgs/PoseStamped` messages on
`/ur10/targets/pick` and `/ur10/targets/place`; provide TF for other source frames.

**Build and start**

On the host, from TIM:

```bash
./docker_build.sh               # Skip if tim_img already exists.
./docker_hardware_build.sh      # Builds tim_ur10_hardware_img; fetches dependencies automatically.
./docker_hardware_run.sh /dev/serial/by-id/YOUR_ADAPTER
```

Only TIM needs to be cloned. The hardware image installs the official UR packages
and builds the pinned Robotiq dependency automatically. It is separate from the
simulation image, which uses a different gripper description.

The run helper opens a shell, maps the specified adapter to `/dev/robotiq`, uses
host networking for the UR driver, and selects ROS domain **120**. Every hardware
terminal and external perception node must use that domain. The simulation keeps
domain 119. To open another shell, run `./docker_attach.sh tim_ur10_hardware` on the
host. Rebuilding an image requires a new container to use the new image; supply a
new name as the second argument to the run helper.

Start the measured target publisher inside the hardware container:

```bash
ros2 run ur10_hardware_primitives target_publisher.py --ros-args \
  --params-file /home/user/ros2_ws/src/ur10_hardware_primitives/config/targets.local.yaml
```

In another attached terminal, launch the hardware stack:

```bash
ros2 launch ur10_hardware_primitives hardware.launch.py \
  cell_config:=/home/user/ros2_ws/src/ur10_hardware_primitives/config/cell.local.yaml
```

This starts the official UR driver, physical gripper controller, MoveIt, and
collision-scene setup. The gripper activates during startup and can move its
fingers: start with no object held. The scene setup waits for a fresh pick target,
adds the initial workpiece and measured obstacles, and only then starts the
primitive manager. Run the configured External Control program on the pendant
and confirm the controllers are active:

```bash
ros2 control list_controllers
ros2 topic echo /dynamic_joint_states
```

`scaled_joint_trajectory_controller` and `robotiq_gripper_controller` should be
active. Review the model and workcell in RViz before commanding motion. Add
`launch_rviz:=true` to the launch on a desktop terminal; the Docker helper sets up
X11 forwarding when the host has `DISPLAY` set.

If your combined model, patched gripper driver, and MoveIt are already running,
start only the scene setup and primitives instead:

```bash
ros2 launch ur10_hardware_primitives primitives.launch.py \
  cell_config:=/home/user/ros2_ws/src/ur10_hardware_primitives/config/cell.local.yaml
```

The existing MoveIt instance must use the combined robot/TCP model and the supplied
controller mapping. Do not launch a second driver/controller manager on the same
robot or serial adapter.

**SEED and PDDL execution**

After verifying the physical setup, start `ros2 run seed seed ur10` in another
hardware-container terminal. The same `pick_place_demo` recipe or
[`pddl_to_seed` command](pddl-pick-place.md) can request the primitives. For PDDL,
ensure symbolic objects/locations and initial facts match the physical targets.
The publisher reads the same commands; SEED does not need numeric coordinates.

To stop an invocation through the manager:

```bash
ros2 topic pub --once /ur10/primitives/command std_msgs/msg/String '{data: cancel}'
```

The manager retains its execution slot until the current action ends. Interrupted
grasp/release operations reconcile the collision model from measured gripper
state. If that state is ambiguous or unavailable, the slot stays occupied for
recovery. Resolve the fault and inspect the robot/object/scene before `reset`.
Stopping the primitive-manager process alone is not an arm stop command.

Hardware `object.placed` means release at the target and successful TCP retreat.
There is no camera verification of the object's final resting position yet.
Likewise, gripper detection indicates contact, not object identity. This profile
handles one workpiece; perception and richer object selection can be added later.

**Verification status**

The hardware image builds successfully. All 33 hardware checks and all 47
existing simulation protocol/publisher checks passed. The code was also exercised
with the actual MoveIt model/kinematics. Tests cover grasp detection, missing/stale/faulted feedback,
loss of a carried object, collision updates, command aliases, cancellation, and
configuration validation. The physical UR10 and gripper have not been tested.
Your measured TCP, bracket, collision geometry, and force settings still need
commissioning on the real setup.

The offline model check verified forward/inverse kinematics for `grasp_tcp`,
controller selection, and seven gripper openings without self-collision in the
test posture. In the installed MoveIt 2.5.10 build, `move_group` also logged a
segmentation fault during SIGINT cleanup after these checks. That shutdown issue
remains unresolved; it was not a failure of the model queries or a physical test.
