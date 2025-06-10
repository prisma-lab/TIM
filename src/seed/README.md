# SEED
Version 7.1 (ROS2 version + Docker)

This is a dockerized version of a ROS2 node implementing the SEED functionalities. 

## About SEED
SEED is an **attentional executive system** for robotics capable of online orchestrating and monitoring structured robotic tasks at different levels of abstraction. 
The system is basically composed of 3 modules:
1. Long-Term Memory (**LTM**): is a rocedural memory where hierarchical representation of all the available tasks are stored.
2. Working Memory (**WM**): is a volatile memory containing instantiated tasks from the LTM for possible execution.
3. Bheavior-Based System (**BBS**): is a repository of sensorimotor processes/procedures that can be recalled and executed by the robot.
During the execution, hierarchical tasks may be retrieved from the LTM and instantiated into the WM. The WM expands such tasks following the hierarchical specification loading additional subtasks into the memory. If retreived tasks are associated with a vehvior from the BBS (i.e., concrete tasks) the associated process is recalled.
Notice that SEED specifically allow multiple behaviors to be executed at the same time and exploits attentional regulations and contention scheduling for conflinct management (**competition**).

### Long-Term Memory (LTM)
TODO

### Working Memory (WM)
TODO

### Behavior-Based System (BBS)
TODO

### Competition
TODO


## Installation and execution via Docker (recomanded)

The whole SEED architecture is wrapped into a dockerized ROS2 node. Please check the [Docker]() and [ROS2]() guides for further details.

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


### Create Docker Image for SEED
In order to build the package you have to create an image for seed.
NOTE: the image may take up to 3Gb of memory on your drive.
```
./docker_build.sh [image_name]
# example:
./docker_build.sh seed_img
```


### Run a container
To run SEED a container for the created image must be started, you can used the provided script:
```
./docker_run.sh [image_name] [container_name]
# example:
./docker_run.sh seed_img seed_cnt
```
The shell will be automatically attached to the container. Notice that if you close this shell, the container will be closed as well.

If you want to attach a new shell to the previously started container you can use the following script:
```
./docker_attach.sh [container_name]
# example:
./docker_attach.sh seed_cnt
```


### Execution (test)
To execute SEED simply type the following command into one of the attached shell:
```
ros2 run seed seed test
```
some default nodes will be loaded and expanded in the SEED working menmory.

To check the list of loaded nodes you can type the `listing` command into the shell.

To start the QT5 GUI you can type `gui` into the shell. 

To start the ROS2 semaphore test type `ros2 semaphore` into the shell (demo output is available in the `/semaphore/out` topic).


### Execution (TIM)
To execute SEED in the Task Inversion and Quality Monitoring (TIM) domain, simply type the following command into one of the attached shell:
```
ros2 run seed seed TIM
```

## Installation via apt-get (outdated and not recomanded)

This procedure is not recomanded as docker strongly simplifies installation and running on different platforms.

### Installation
In order to build the SEED package in the traditional way the following additional dependancies must be installed:
```
# standard SEED dependancies
sudo apt install swi-prolog
sudo apt install libgraphviz-dev
sudo apt install libqt5charts5-dev
sudo apt install libespeak-dev
```
Please refer to the **Dockerfile** for detailed installation instructions. 
Notice that SEED makes use of swi-prolog for the LTM functionalities, please refer to the offical [website](https://www.swi-prolog.org/) for further details.

### Execution
To execute SEED simply type the following command while a roscore is on:
```
ros2 run seed seed test
```
some default nodes will be loaded and expanded in the SEED working menmory.

To check the list of loaded nodes you can type the `listing` command into the shell.

To start the QT5 GUI you can type `gui` into the shell.




## Creating your SEED application

You may create a new SEED application by creating a new LTM or by customizing a preexisting one, but also by adding new behaviors to the BBS (add-ons). Remember that SEED is wrapped into a ROS2 node, you can either create your custom ROS2 interfaces (topics, services, etc.) within new behaviors or using bulit-in ones.    

### Create or modify a LTM
The SEED architecture is based on a hierarchical representation of tasks. Each task that can be either abstract (collection of further subtasks) or concrete (actual code). Parametric tasks are defined as prolog schemata collected into a specific LTM file from the LTM/ folder. LTM files must be named as follows:
```
seed_[YOUR_APP_NAME]_LTM.prolog
```
Where [YOUR_APP_NAME] is the name of your SEED application.

Inside the LTM, schemata are represented as follows:
```
seed_[YOUR_APP_NAME]_LTM.prolog
```

Different LTM files can be used to specify the set of tasks in the behavioral repository of the system.

### Add a new behavior to the BBS
TODO

### ROS2 interface (ros_behaviors)
TODO

# References

Main reference.
```
@article{caccavale2022robotic,
  title={A robotic cognitive control framework for collaborative task execution and learning},
  author={Caccavale, Riccardo and Finzi, Alberto},
  journal={Topics in Cognitive Science},
  volume={14},
  number={2},
  pages={327--343},
  year={2022},
  publisher={Wiley Online Library}
}
```

See also.
```
@article{cacace2023combining,
  title={Combining human guidance and structured task execution during physical human--robot collaboration},
  author={Cacace, Jonathan and Caccavale, Riccardo and Finzi, Alberto and Grieco, Riccardo},
  journal={Journal of Intelligent Manufacturing},
  volume={34},
  number={7},
  pages={3053--3067},
  year={2023},
  publisher={Springer}
}

@article{caccavale2019kinesthetic,
  title={Kinesthetic teaching and attentional supervision of structured tasks in human--robot interaction},
  author={Caccavale, Riccardo and Saveriano, Matteo and Finzi, Alberto and Lee, Dongheui},
  journal={Autonomous Robots},
  volume={43},
  pages={1291--1307},
  year={2019},
  publisher={Springer}
}
```

# Acknowledgments
Funded by the European Union. Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Health and Digital Executive Agency (HADEA). Neither the European Union nor HADEA can be held responsible for them. 
EU -HE Inverse - Grant Agreement 101136067 

# License
This project is licensed under the MIT License.
You are free to use, modify, and distribute this software with proper attribution. See the LICENSE file for details.