FROM ros:humble


#Uncomment the following line if you get the "release file is not valid yet" error during apt-get
#	(solution from: https://stackoverflow.com/questions/63526272/release-file-is-not-valid-yet-docker)
#RUN echo "Acquire::Check-Valid-Until \"false\";\nAcquire::Check-Date \"false\";" | cat > /etc/apt/apt.conf.d/10no--check-valid-until

#Solve ROS2 gpg key issue, from 01-Jun-2025
RUN apt-get install curl -y
RUN rm -f /usr/share/keyrings/ros-archive-keyring.gpg
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

#Install essential
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    apt-get install -y software-properties-common && \
    apt-add-repository ppa:swi-prolog/stable && \
    apt-get update && apt-get install -y \
	swi-prolog \
	libgraphviz-dev \
	libqt5charts5-dev \
	libespeak-dev \
	ros-${ROS_DISTRO}-rviz2 \
	ros-${ROS_DISTRO}-cv-bridge \
	ros-${ROS_DISTRO}-vision-opencv \
	ros-${ROS_DISTRO}-image-transport \
	ros-${ROS_DISTRO}-image-transport-plugins \
	ros-${ROS_DISTRO}-aruco-opencv \
	ros-${ROS_DISTRO}-apriltag-ros \
	ros-${ROS_DISTRO}-librealsense2* \
	ros-${ROS_DISTRO}-realsense2-* \
	# TRY WITH CYCLONEDDS
	ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \ 
	ros-${ROS_DISTRO}-franka-msgs \
	libboost-all-dev \
	# for new json-based seed GUI
	nlohmann-json3-dev \
	python3-pyqt5 \
	#libxcb-randr0-dev libxcb-xtest0-dev libxcb-xinerama0-dev libxcb-shape0-dev libxcb-xkb-dev \
	#libxcb-util-dev \
	libxcb-xinerama0 \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-rqt*

#Environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:0
ENV HOME=/home/user
ENV ROS_DISTRO=$ROS_DISTRO

#Set ROS2 domain (fixed for now)
ENV ROS_DOMAIN_ID=11
# set DDS to cyclone! default version of DDS is bugged!
# DDS cyclone guide: https://docs.ros.org/en/humble/Installation/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

#Add non root user using UID and GID passed as argument
ARG USER_ID
ARG GROUP_ID
RUN addgroup --gid $GROUP_ID user
RUN adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID user
RUN echo "user:user" | chpasswd
RUN echo "user ALL=(ALL:ALL) ALL" >> /etc/sudoers

#get access to video
RUN sudo usermod -a -G video user

USER user

#ROS2 workspace creation and compilation
RUN mkdir -p ${HOME}/ros2_ws/src
WORKDIR ${HOME}/ros2_ws
COPY --chown=user ./src ${HOME}/ros2_ws/src

#RUN git clone -b dmp --single-branch https://github.com/matteodv99tn/mdv_cpp_lib.git src/mdvcpplib

# YIGIT: building downward
ARG BUILD_JOBS=2
RUN cd /home/user/ros2_ws/src/downward && ./build.py -j${BUILD_JOBS}

SHELL ["/bin/bash", "-c"] 
# Install dependencies as root, and stop immediately if installation fails.
# The external LN/HFI bridge requires separately supplied packages.
USER root
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    ROS_HOME=/root/.ros rosdep update --rosdistro ${ROS_DISTRO} && apt-get update && \
    ROS_HOME=/root/.ros rosdep install -i --from-paths src/seed src/seed_gui src/task_planner \
        src/task_planner_msgs src/inverse_msgs src/vlm_planner \
        --rosdistro ${ROS_DISTRO} -y && \
    rm -rf /var/lib/apt/lists/*
USER user
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    MAKEFLAGS="-j${BUILD_JOBS}" CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS} \
    colcon build --symlink-install --executor sequential \
        --packages-select seed seed_gui task_planner task_planner_msgs inverse_msgs vlm_task_planner \
        --cmake-args -DCMAKE_CXX_FLAGS="-w"

#Add script source to .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash;" >>  ${HOME}/.bashrc
RUN echo "source ${HOME}/ros2_ws/install/local_setup.bash;" >>  ${HOME}/.bashrc

#Set env variables
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/usr/lib/swi-prolog/lib/x86_64-linux/"
# YIGIT: added downward to PATH
ENV PATH="${PATH}:${HOME}/ros2_ws/src/downward/"
# YIGIT: added ROS workspace path to use it in downward custom build (in downward/driver/util.py)
ENV ROS_WS="${HOME}/ros2_ws" 
# Keep Python from rewriting bytecode in the bind-mounted source checkout.
ENV PYTHONDONTWRITEBYTECODE=1

#Clean image
USER root
RUN rm -rf /var/lib/apt/lists/*
USER user

# run launch file
#CMD ["ros2", "run", "seed", "seed", "test"]

