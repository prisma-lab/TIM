# VLM → PDDL → two-object plan

Build the new code once, then source the workspace:

```bash
cd /home/user/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select task_planner
source install/setup.bash
```

Ollama must serve the selected vision model. Fast Downward must be on PATH or
supplied through `planner_executable`. No build is needed for a new image.

```bash
ros2 launch task_planner vlm_two_objects.launch.py image:=/absolute/path/to/image.png
```

Defaults use the installed Serdar domain, mapping, and descriptions, `full`
generation, and `astar(blind())` planning. The sequence is logged and published
as a JSON result. Add `execute:=true` to submit to an already-running SEED
instance. Submission does not mean completed execution. Start simulation and
SEED separately. Add `output_directory:=/tmp/grounded-scene` to save JSON/PDDL
for debugging; these files are not used to transport the problem.

Launch with `autostart:=false` to start without generating. Request further images:

```bash
ros2 topic pub --once /vlm_grounder/image_path std_msgs/msg/String \
  "{data: '/absolute/path/to/image.png'}"
```

Paths are read on the grounder machine. Startup waits for planner discovery;
manual requests require a connected planner. Each node accepts one job at a
time; busy requests are rejected. Workers keep ROS callbacks responsive.
Shutdown waits for outstanding work, potentially up to the inference timeout.

Subscribe before triggering a request:

```bash
ros2 topic echo /vlm_grounder/status
ros2 topic echo /two_objects_planner/status
ros2 topic echo /two_objects_planner/result
```

The existing `task_planner_msgs/msg/PlanningRequest` transports domain/problem
text on `/two_objects/planning_request`. The legacy `/planning_request` and
`/planner_result` interface is separate. Here, `header.frame_id` carries a
correlation UUID, not a coordinate frame. Status/result JSON carries the same
`request_id`. Recent duplicate IDs are rejected. Volatile topics avoid replaying
old execution requests after restart; they are not durable job queues. No
execution retries are automatic. A SEED acknowledgement timeout means submission
is uncertain: inspect SEED before retrying.

Launch overrides: `domain`, `mapping`, `scene`, `goal`, `predicate_definitions`,
`model`, `endpoint`, `mode`, `context`, `output_directory`, `vlm_timeout`,
`planner_executable`, `planner_build`, `planner_search`, `planner_timeout`,
`autostart`, `execute`, `seed_topic`, `wait_for_seed`, `discovery_timeout`, `planning_topic`.

For scene-only grounding, supply `mode:=init-only context:=/path/to/context.json`
(or PDDL). Context must match domain types/names/predicates. The existing
`serdar_two_objects_problem.pddl` has an undeclared `robot` type and placement
goals different from the current description file; do not use it unchanged as
context. Structural validation does not verify visual accuracy. Unmapped plan
actions fail the entire request before SEED submission. An already-satisfied
goal returns `empty_plan: true` and sends no commands.

This pipeline installs the grounder in `task_planner/vlm_grounder/src` as
`vlm_grounder.src`; existing direct CLI imports remain supported. The separate
workspace `src/vlm_grounder` copy is unchanged. Ollama calls no longer require
writing the prompt into the source tree.
