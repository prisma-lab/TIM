# TIM
TIM (Task Inversion and quality Monitoring) Framework Version 2.0 (second prototype)

## About TIM

The TIM framework represents a novel approach to robotic task planning and execution, specifically designed to handle structured and invertible tasks in dynamic environments. Developed within the INVERSE project, the framework integrates flexible execution control, adaptive planning, and real-time quality monitoring into a coherent and modular architecture.

At the core of the TIM framework lie three tightly integrated components: the Executive System, the Task Planning Module, and the Quality Monitoring Module. These internal subsystems operate in close coordination to ensure that robots can not only carry out complex tasks but also adapt and recover in case of failure or unforeseen events.

The Executive System, inspired by attentional control models, manages the dynamic execution of tasks. It decomposes high-level plans into executable behaviors, regulates the activation and scheduling of concurrent processes, and orchestrates sensorimotor activity. Central to this system are two memory structures: the Long-Term Memory (LTM), which stores task definitions and schemas, and the Working Memory (WM), which maintains the current task hierarchy and execution state. A Behavior-Based System (BBS) handles low-level actions, dynamically prioritizing behaviors based on a combination of top-down goals and bottom-up stimuli. This design allows the robot to reactively shift focus, collaborate with humans, and resolve resource conflicts intelligently during task execution.

The Task Planning Module supports both forward and inverse task generation. When an inverse task is requested, such as disassembling an object or undoing a prior sequence, the planner follows a staged strategy. It first attempts symbolic inversion using traditional planning methods. If this fails, the system can engage learning-based methods to fill gaps or correct errors, and ultimately defer to human guidance through demonstration if needed. 

Complementing these systems is the Quality Monitoring Module, which oversees execution fidelity. It continuously evaluates whether task progress aligns with expected outcomes. When deviations are detected, the system can proactively trigger replanning, engage learning mechanisms, or prompt user intervention. This ensures robustness, especially in uncertain or collaborative settings.

In addition to these core components, TIM interfaces with several external systems that supply essential input data. A Scene Understanding Module provides symbolic and quantitative descriptions of the robot’s environment. A Knowledge Base contains information about available actions and robot capabilities. Moreover, TIM supports integration with Self-Learning and Teaching Modules, enabling the robot to expand its skillset autonomously or via user instruction. These external systems are accessed through ROS2-based interfaces, ensuring modularity and interoperability.

## Installation and execution via Docker

The whole architecture is wrapped into a series of dockerized ROS2 packages. Please check the [Docker](https://docs.docker.com/get-started/) and [ROS2](https://docs.ros.org/en/humble/index.html) guides for further details.

### Prerequisites (Docker installation)

Before to start, docker must be installed on your OS. For Ubuntu users you may refer to the following procedure:
```
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

# Install the Docker packages
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Add docker user
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```


### Create Docker Image for TIM

For the **UR10 Gazebo pick-and-place demo**, follow the complete
[simulation setup below](#build-and-run-the-ur10-simulation). It uses both
Dockerfiles: the base TIM image first, then the simulation image.

Build the TIM image from the repository directory. The first build downloads ROS
and system dependencies and compiles the workspace; allow several gigabytes of
disk space and several minutes for the build.
```
./docker_build.sh           # defaults to tim_img
# Or choose an image name:
./docker_build.sh tim_img
```
The default build includes SEED, its GUI, the task planner, their message packages,
and the VLM planner package. The LN/HFI bridge requires additional external
dependencies and is not part of this build.
The VLM nodes also require separate Gemini SDK/API-key configuration before use.

Compilation uses two jobs by default. To change this limit, build directly:
```bash
docker build -t tim_img --build-arg USER_ID="$(id -u)" \
  --build-arg GROUP_ID="$(id -g)" --build-arg BUILD_JOBS=4 .
```


### Prolog compatibility

Older SEED code using `SWI-cpp.h` can fail with `Plx_exception`/`PlWrap` errors.
This checkout already uses `SWI-cpp2.h` and builds with SWI-Prolog 10.0.2.
Rebuild from the current sources; no Prolog downgrade is needed.

### Run a container
To run TIM a container for the created image must be started, you can used the provided script:
```
./docker_run.sh [image_name] [container_name]
# example:
./docker_run.sh tim_img tim_cnt
```
The shell will be automatically attached to the container. Notice that if you close this shell, the container will be closed as well.

If you want to attach a new shell to the previously started container you can use the following script:
```
./docker_attach.sh [container_name]
# example:
./docker_attach.sh tim_cnt
```


### Execution
For local development without robot hardware or a display, you can instead start
a persistent container from the repository directory:
```bash
docker run -dit --name tim_cnt \
  --mount "type=bind,source=$(pwd)/src,target=/home/user/ros2_ws/src" \
  tim_img bash
./docker_attach.sh tim_cnt
```
This container stays running when an attached shell closes. Stop it with
`docker stop tim_cnt` and restart it with `docker start tim_cnt`.

Inside the container, start the task planner:
```bash
ros2 run task_planner planner_node
```
In a second attached shell, run SEED's hardware-independent test mode:
```bash
ros2 run seed seed test
```
Type `listing` to inspect working memory and `q` to exit SEED. The default ROS
domain is `101`, using Cyclone DDS. The local container uses Docker's default
network; use the hardware run script above when ROS nodes must share the host's
network and devices.

For the PrismaLab hardware configuration, run the following command from within
the container started by `docker_run.sh`:
```
ros2 launch seed tim.launch.py
```
The current launch file starts the task planner, camera, marker detection, and
coordinate transforms. Its SEED node is commented out, so start SEED separately
with `ros2 run seed seed TIM` when using the lab setup. The launch file selects
RealSense serial `213322074516`. The robotic sensorimotor primitives are defined
for the Blocksworld setup at PrismaLab (IIWA robot with WSG50 gripper).

### Automatically start the container in VS Code

Opening this repository folder in VS Code runs two automatic tasks:

- `TIM: Start container` starts `tim_cnt` for the base TIM setup.
- `TIM: Start UR10 container` starts `tim_ur10` for the Gazebo exercises.

Automatic tasks are enabled in this workspace's settings and require a trusted
workspace. Docker must be running, and the named containers must already exist.
Starting an already running container does not restart it. These tasks start
containers; launch Gazebo and SEED separately inside `tim_ur10`.

Reopen the folder or run **Developer: Reload Window** to activate newly added
tasks. You can also run either task through **Tasks: Run Task**. To open the
simulation shell, run:

```bash
./docker_attach.sh tim_ur10
```

The attach script starts the named container if needed, refreshes desktop
authentication for simulation containers, and opens a shell. Use
`./docker_attach.sh tim_cnt` for the base container. VS Code itself runs on the
host and can remain open even when a container is stopped. Closing VS Code or
an attached shell leaves the container running; use `docker stop tim_ur10` or
`docker stop tim_cnt` to stop it.

## UR10 Gazebo exercises

### Build and run the UR10 simulation

**Yes: the current complete pick-and-place demo needs the image built from
`Dockerfile.sim`.** The two Dockerfiles serve different purposes:

| Dockerfile | Image | What it provides |
| --- | --- | --- |
| `Dockerfile` | `tim_img` | Base ROS 2 Humble environment, SEED, its GUI, and the task planner. |
| `Dockerfile.sim` | `tim_ur10_img` | Everything in `tim_img`, plus Gazebo, MoveIt, the copied CRF assembly scene, the primitive manager, and the UR10 primitive plugins. |

Build the images in this order, from a **host terminal in the TIM directory**:

```bash
./docker_build.sh           # 1. Build tim_img (skip if you already have it)
./docker_sim_build.sh       # 2. Build tim_ur10_img from tim_img
./docker_sim_run.sh         # 3. Create/start tim_ur10 and open its shell
```

You only need the `tim_ur10` container to run this demo; the base `tim_cnt`
container and the separate CRF container do not need to be running. The build
requires Docker Buildx/BuildKit (installed by the prerequisites above). For the
Gazebo and RViz windows, run the simulation script from your Linux desktop
terminal with `DISPLAY` set and `xauth` installed on the host
(`sudo apt-get install xauth` if needed).

The assembly scene is bundled in
[`simulation/use_case_sim`](simulation/use_case_sim). **Only this repository is
needed**: the default build does not use a local CRF checkout. It includes the
original `assembly_task.launch.py` and the files that launch needs. Robot
dependencies are installed automatically during the build, including a pinned
Robotiq description revision; the first build requires internet access.

To try a custom scene, you can optionally supply another package directory:

```bash
./docker_sim_build.sh /path/to/use_case_sim
```

Inside the simulation shell, start the environment:

```bash
ros2 launch ur10_primitives pick_place.launch.py
```

Wait for the robot controllers and objects to spawn. In a **second host
terminal**, open another shell in the same container and start SEED:

```bash
./docker_attach.sh tim_ur10
# Now inside the container:
ros2 run seed seed ur10
```

At the SEED prompt, enter:

```text
pick_place_demo
```

SEED runs `move_a_b(pick)` → `pick` → `move_a_b(place)` → `place`.
Numeric target poses come from the test publisher. Typing `pick_place_demo` uses
a predefined SEED sequence. To generate the sequence from domain and problem
PDDL files with Fast Downward, follow the [PDDL execution guide](docs/pddl-pick-place.md).
Object holding uses the Gazebo attach/detach plugin.

#### After rebuilding an image

Rebuilding `tim_ur10_img` does **not** update an existing container. In particular,
`./docker_sim_run.sh` reuses `tim_ur10` if it already exists. To try the rebuilt
image while keeping your existing container, use a new, unused name:

```bash
./docker_sim_run.sh tim_ur10_updated
# In the second host terminal, use the same name:
./docker_attach.sh tim_ur10_updated
```

Stop any previous demo launch before starting the new one. The VS Code automatic
task starts the name `tim_ur10`; change that task's container name in
`.vscode/tasks.json` if you switch to `tim_ur10_updated`.

The scripts mount TIM's `src` directory into the container, so source edits are
visible immediately, but C++ changes still need compilation. The bundled scene
is copied into the image at build time: after editing `simulation/use_case_sim`,
rerun `./docker_sim_build.sh` and create a new container to pick up the changes.
See the [pick-and-place guide](docs/pick-place-design.md) for
changing targets and running individual primitives.

### Learning guides

Start with [SEED explained using pick and place](docs/seed-explained.md) for a
simple explanation of SEED, primitive plugins, and how PDDL planning fits in.

The first robot integration adds topic-triggered Robotiq gripper plugins, a C++
primitive manager, and SEED task definitions, using a copy of the CRF assembly scene. Follow
[Step 2: gripper primitives in Gazebo](docs/ur10-gripper.md) for the simulation
image, commands, and an explanation of the SEED connection.
The [manager/plugin walkthrough](docs/primitive-manager.md) explains the common
interface, execution lifecycle, configuration, and how to add primitives.
Continue with [Step 3: parametric pick and place](docs/pick-place-design.md) to move
the red connector using externally published poses and three distinct skills:
`move_a_b`, `pick`, and `place`. SEED explicitly sequences the transfers and
local grasp/release operations.
[Step 4: PDDL to execution](docs/pddl-pick-place.md) connects Fast Downward plans
to those same SEED tasks and primitives.

# References
See references of specific packages

# Acknowledgments
Funded by the European Union. Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Health and Digital Executive Agency (HADEA). Neither the European Union nor HADEA can be held responsible for them. 
EU -HE Inverse - Grant Agreement 101136067 

# License
This project is licensed under the MIT License.
You are free to use, modify, and distribute this software with proper attribution. See the LICENSE file for details.
