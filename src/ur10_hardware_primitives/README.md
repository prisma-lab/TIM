# UR10 hardware primitives

For hardware primitives supplied as ROS 2 services, use the separate
[service adapter backend](../../docs/hardware-service-primitives.md):
`services.launch.py`, `MoveServicePrimitive`, `PickServicePrimitive`, and
`PlaceServicePrimitive`. This backend tracks `/motion_start` and `/motion_end`
IDs; its pick/place operations only close/open the gripper.

The original direct-control backend below is retained.

Physical UR10 CB3 + Robotiq 2F-140 over USB/RS-485, using the official UR driver
and `scaled_joint_trajectory_controller`. This package has no Gazebo dependency.

Plugins: `move_a_b`, `pick`, `place`, `open_gripper`, and `close_gripper`.
SEED and PDDL use the same symbolic commands as the simulation; targets come from
external `PoseStamped` topics. Pick requires measured gripper object detection.

See the [hardware setup and execution guide](../../docs/ur10-hardware.md).
Configure measured `config/cell.yaml` settings before launching. The software is
tested with mock backends; physical commissioning remains to be done.
