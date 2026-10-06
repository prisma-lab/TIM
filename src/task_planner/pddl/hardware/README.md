# Hardware PDDL files

Put the hardware team's domain in `domain.pddl` and problem in `problem.pddl`.
The bundled pair is an example that plans the eight requested move/pick/place steps.
Replace its initial state and task rules with the real experiment's description.

Inside the container, this directory is:
`/home/user/ros2_ws/src/task_planner/pddl/hardware`.

Plan without sending commands:

```bash
cd /home/user/ros2_ws
ros2 run task_planner pddl_to_seed_hardware \
  --domain src/task_planner/pddl/hardware/domain.pddl \
  --problem src/task_planner/pddl/hardware/problem.pddl
```

The supported PDDL actions are `(move FROM TO)`, `(pick LOCATION)`, and
`(place LOCATION)`. They translate directly to `move(FROM, TO)`,
`pick(LOCATION)`, and `place(LOCATION)` in SEED.
Location names must start with a letter and use letters, digits or underscores.
No mapping YAML is needed for this vocabulary. Unsupported actions or argument
counts reject the entire plan before submission.

With the hardware service manager, target publisher, provider, and SEED
`ur10_services` profile running, add `--execute` to submit the plan to
`/seed_ur10_services/stream`. Submission is not confirmation of completed motion.
PDDL edits take effect the next time this command runs; they do not change an
already running sequence. Initial facts must describe the actual starting state.
