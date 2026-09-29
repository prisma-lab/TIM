# UR10 simulation primitives

This package is the **Gazebo-only UR10 + Robotiq 2F-85 demonstration**.
It is not configured for the physical UR10 CB3 or the Robotiq 2F-140.

- `move_a_b(target)` uses MoveIt to reach the target's approach pose.
- `pick` descends, closes the simulated fingers, calls Gazebo's demo attachment
  service, and retreats.
- `place` descends, opens the simulated fingers, detaches the Gazebo object,
  and retreats.
- `open_gripper` / `close_gripper` command the simulated 2F-85 finger joint.

The arm motion interface itself is MoveIt, but the package's robot model,
controllers, TCP offset, gripper joint, grasp service, and object observations
belong to the simulation. Changing only `use_sim_time` does not turn these
plugins into hardware primitives.

The real counterparts belong in the separate `ur10_hardware_primitives` package.
The symbolic primitive commands remain the same; the robot/gripper interfaces
and completion evidence differ.

See [the simulation setup](../../README.md#ur10-gazebo-exercises) and
[the pick-and-place guide](../../docs/pick-place-design.md).
