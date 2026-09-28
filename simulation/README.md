# Bundled assembly scene

`use_case_sim` is the minimal scene package used by the UR10 gripper and
pick-and-place demos. `../docker_sim_build.sh` selects it by default, so a fresh
TIM checkout needs no CRF repository or CRF container.

The eight package files are copied unchanged from
`use_case_crf/ros2_ws/src/use_case_sim` at revision
`24d6c7917cd1062be3c647538a37ead4f8269115`. Original package metadata is preserved.
This is a local snapshot: changes in the original repository do not update it.

Included files:

- `launch/assembly_task.launch.py`: starts the UR10, gripper, table scene,
  red connector, blue peg, controllers, and RViz.
- `urdf/assembly_env.urdf.xacro`, `connector.urdf`, and `peg.urdf`: scene and
  object geometry.
- `config/task_controllers.yaml` and `rviz/default.rviz`: controller and display
  configuration.
- `CMakeLists.txt` and `package.xml`: build and install the ROS package.

The unrelated `peg_in_hole` launch/description and `worlds/table_plate.world`
are omitted because the assembly launch does not reference them. The
pick-and-place launch uses TIM's existing
`../src/ur10_sim_support/worlds/assembly_pick_place.world` for the demo grasp
plugin, light, and ground.

`Dockerfile.sim` installs `ur_description` from ROS packages and downloads
`robotiq_description` at the pinned revision specified there. Those robot assets
are not duplicated here. An internet connection is required for the first build.

To update the scene, edit this copy, rebuild with `./docker_sim_build.sh` from
the TIM root, and create a new container from the resulting image. For an
experimental external copy, use `./docker_sim_build.sh /path/to/use_case_sim`.
