# TIM
TIM (Task Inversion and quality Monitoring) Framework Version 1.0 (first prototype)

## About TIM
TODO

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
In order to build the package you have to create an image for seed.
NOTE: the image may take up to 3Gb of memory on your drive.
```
./docker_build.sh [image_name]
# example:
./docker_build.sh tim_img
```


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
TODO

# References
See references of specific packages

# Acknowledgments
Funded by the European Union. Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Health and Digital Executive Agency (HADEA). Neither the European Union nor HADEA can be held responsible for them. 
EU -HE Inverse - Grant Agreement 101136067 

# License
This project is licensed under the MIT License.
You are free to use, modify, and distribute this software with proper attribution. See the LICENSE file for details.