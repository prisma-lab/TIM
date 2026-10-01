**Two-object simulation: parameterized pick and place**

This separate experiment places the red connector first, then the blue peg.
It uses the same assembly scene and UR10 with Robotiq 2F-85. The original
`pick_place.launch.py`, single-object plugins, PDDL files, and UR10 LTM are not
changed by this experiment.

The three commands are:

```prolog
move_a_b(Location)
pick(Object,Location)
place(Object,Location)
```

`Object` selects the Gazebo object and its feedback. `Location` selects a ROS pose
topic. SEED knows these symbolic names, but receives no numeric coordinates.
`move_a_b` handles travel; pick/place perform local descent, grasp/release, and
retreat. A pick/place request checks that the arm is already at its approach pose.

**Build and launch**

Use the simulation container described in the [README](../README.md#build-and-run-the-ur10-simulation).
For an existing container, rebuild the affected packages inside it:

```bash
cd /home/user/ros2_ws
source /opt/ros/humble/setup.bash
source /home/user/sim_ws/install/local_setup.bash
source install/local_setup.bash
colcon build --symlink-install --executor sequential \
  --packages-select ur10_sim_support ur10_primitives task_planner
source install/local_setup.bash
```

New simulation images include the experiment through `./docker_sim_build.sh`.
No new dependencies or second repository are needed.

Stop the previous simulation and SEED before starting this experiment: the two
launches share the simulated robot, MoveIt endpoints, controllers, and Gazebo.
Use separate terminals attached to the same simulation container and ROS domain.

Terminal 1:

```bash
ros2 launch ur10_primitives two_objects.launch.py
```

Wait for both objects and all controllers to load. Terminal 2:

```bash
ros2 run seed seed ur10_two_objects
```

Choose one of the following ways to request the task. Do not submit both at once.
Restart the simulation and SEED to restore the initial scene before repeating.

**Option A: named SEED task**

Enter this at the SEED prompt:

```text
two_objects_demo
```

Its complete sequence is:

```prolog
hardSequence([
    move_a_b(red_pick),
    pick(red_connector,red_pick),
    move_a_b(red_place),
    place(red_connector,red_place),
    move_a_b(blue_pick),
    pick(blue_peg,blue_pick),
    move_a_b(blue_place),
    place(blue_peg,blue_place)
])
```

**Option B: Fast Downward**

In terminal 3, preview the PDDL plan:

```bash
cd /home/user/ros2_ws
ros2 run task_planner pddl_to_seed_two_objects \
  --domain src/task_planner/pddl/two_objects_domain.pddl \
  --problem src/task_planner/pddl/two_objects_problem.pddl \
  --mapping src/task_planner/config/two_objects_action_mapping.yaml
```

Add `--execute` to submit that plan to `/seed_ur10_two_objects/stream`.
The original `pddl_to_seed` command retains its original single-object interface.

The new problem includes:

```lisp
(before red_connector blue_peg)
```

The domain's generic pick action requires every predecessor to be `completed`.
The place action sets that predicate. Consequently, Fast Downward must place red
before it can pick blue. The ordering comes from the problem, not from separate
`pick_red` and `pick_blue` action definitions. The goal asks for both objects at
their destination locations and an empty gripper.

The new command defaults to `astar(blind())`. Its small PDDL domain uses a
quantified ordering condition, which Fast Downward translates using axioms;
the original demo's `lmcut()` heuristic does not support those axioms.

The mapping preserves both parameters. For example:

```yaml
actions:
  "(pick red_connector red_pick)": "pick(red_connector,red_pick)"
  "(place blue_peg blue_place)": "place(blue_peg,blue_place)"
```

Every grounded action must have a mapping before anything is sent to SEED.
Changing the order or goal can produce new actions and require new mapping
entries. Changing a PDDL name does not move an object or change a numeric pose.

**How parameter binding works**

The separate [LTM](../src/seed/LTM/seed_ur10_two_objects_LTM.prolog) defines one pick
schema and one place schema for both objects:

```prolog
schema(pick(Object,Location), [
    [rosAct(pick(Object,Location),ur10_two_objects,ur10/two_objects/command,0.25),
     0,[-manipulation.failed]]
], [object.held(Object)], []).

schema(place(Object,Location), [
    [rosAct(place(Object,Location),ur10_two_objects,ur10/two_objects/command,0.25),
     0,[-manipulation.failed]]
], [object.placed(Object,Location)], []).
```

For `pick(blue_peg,blue_pick)`, Prolog binds `Object = blue_peg` and
`Location = blue_pick`. SEED publishes that complete command string and waits
for `object.held(blue_peg)`. A true `object.held(red_connector)` cannot satisfy it.

For `place(blue_peg,blue_place)`, the goal becomes
`object.placed(blue_peg,blue_place)`. Both identity and destination appear in the
completion condition.

The manager parses the command and calls the same plugin class with different
arguments:

```text
pick(red_connector,red_pick) → Pick plugin execute([red_connector, red_pick])
pick(blue_peg,blue_pick)      → Pick plugin execute([blue_peg, blue_pick])
```

The new `ParameterizedPickPrimitive` and `ParameterizedPlacePrimitive` share
`ParameterizedPrimitive`. The original single-object classes remain separate.

**Poses, objects, and feedback**

The [target configuration](../src/ur10_primitives/config/two_object_targets.yaml)
contains four external poses. Each value is `[x,y,z,qx,qy,qz,qw]` in `world`.
They are published as fresh `PoseStamped` messages on:

```text
/ur10/two_objects/targets/red_pick
/ur10/two_objects/targets/red_place
/ur10/two_objects/targets/blue_pick
/ur10/two_objects/targets/blue_place
```

The objects keep their original spawn positions. The blue target tilts the tool
30 degrees toward world +X and sets the grasp TCP 2 cm above the peg center.
This gives clearance from the rear plate during descent across the tested arm
configurations. The two placement
positions are separate open areas of the table. Positions and orientations can
be changed in this file. Each invocation snapshots its pose, so
an update during motion does not redirect the running primitive. Replace the
publisher with perception by launching with `target_publisher:=false` and
publishing the same topics.

The [plugin configuration](../src/ur10_primitives/config/two_objects.yaml) lists
the objects, locations, grasp settings, and carried-object collision boxes.
The separate Gazebo [world](../src/ur10_sim_support/worlds/assembly_two_objects.world)
maps each object ID to its actual model/link. Its attachment plugin uses one
shared grasp slot and rejects attaching a second object or releasing the wrong
object.

For each configured object, the Gazebo plugin provides:

```text
/ur10/two_objects/objects/<Object>/set_grasp    SetBool service
/ur10/two_objects/objects/<Object>/holding      Bool feedback
/ur10/two_objects/objects/<Object>/pose         PoseStamped feedback
```

Grasps still use a simulated fixed attachment; this experiment does not model
friction-based grasp reliability. MoveIt receives the selected carried-object
box during transport. As in the original demo, loose objects are observed in
Gazebo but are not maintained as world collision objects in MoveIt after release.

The manager publishes facts on `/seed_ur10_two_objects/state`, including:

```text
arm.at(red_pick)
object.held(red_connector)
object.placed(red_connector,red_place)
object.held(blue_peg)
object.placed(blue_peg,blue_place)
```

A leading `-` means false. Goal facts are inhibited while any primitive is busy
or a failure is latched. Pick verifies the selected object is attached and lifted;
place verifies release, object position near the selected target, and arm retreat.
Red's placement fact can remain true while blue is subsequently handled.

Inspect execution with:

```bash
ros2 topic echo /ur10/two_objects/status
ros2 topic echo /seed_ur10_two_objects/state
```

A primitive failure latches `manipulation.failed`; there is no automatic PDDL
replanning. The initial PDDL state is supplied by the problem file, not inferred
from Gazebo. See the [PDDL guide](pddl-pick-place.md) for the shared planning and
execution concepts.


**Repeatable simulation check**

Validated in the simulation container: all three affected packages built,
97 planner/protocol tests passed, and both the named-task and Fast Downward
routes completed all eight steps in Gazebo. Both objects finished at their
respective table destinations, with separate placement facts true. The original
single-object PDDL-to-SEED-to-Gazebo demo also passed its regression check.

After sourcing the workspaces, this check starts a fresh scene, runs SEED, and
verifies the eight commands, both object-specific placement facts, and the final
Gazebo object positions. It keeps SEED's learned weights and logs in a temporary
workspace. Use a free ROS domain and Gazebo port:

```bash
cd /home/user/ros2_ws
ROS_DOMAIN_ID=137 GAZEBO_MASTER_URI=http://localhost:11465 \
  xvfb-run -a python3 src/ur10_primitives/test/run_two_objects_gazebo.py
```

The default checks the Fast Downward route. Add `TWO_OBJECT_MODE=named` before
`xvfb-run` to check `two_objects_demo` instead. Xvfb supplies a virtual display;
this test does not open desktop windows. Logs are saved to
`/tmp/two-objects-launch.log` and `/tmp/two-objects-seed.log` in the container.

The planner mapping and command allowlist in
[two_objects_to_seed.py](../src/task_planner/task_planner/two_objects_to_seed.py)
are deliberately limited to this experiment's object and location names. To
extend the experiment, configure the new object/target topics and Gazebo model,
then extend the mapping and allowlist. The parameterized Prolog schemas do not
need separate definitions for each object.
