#FROM ros:foxy
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
	ros-${ROS_DISTRO}-librealsense2* \
	ros-${ROS_DISTRO}-realsense2-* \
	# TRY WITH CYCLONEDDS
	ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \ 
	libboost-all-dev \
	#libxcb-randr0-dev libxcb-xtest0-dev libxcb-xinerama0-dev libxcb-shape0-dev libxcb-xkb-dev \
	#libxcb-util-dev \
	libxcb-xinerama0 \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-rqt*
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-plansys2-*

#Environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:0
ENV HOME /home/user
#ENV ROS_DISTRO=foxy
ENV ROS_DISTRO=$ROS_DISTRO

#Set ROS2 domain (fixed for now)
ENV ROS_DOMAIN_ID=77
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
SHELL ["/bin/bash", "-c"] 
RUN source /opt/ros/${ROS_DISTRO}/setup.bash; rosdep update; rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y; colcon build --symlink-install

#Add script source to .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash;" >>  ${HOME}/.bashrc
RUN echo "source ${HOME}/ros2_ws/install/local_setup.bash;" >>  ${HOME}/.bashrc

#Set env variables
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/usr/lib/swi-prolog/lib/x86_64-linux/"

#Clean image
USER root
RUN rm -rf /var/lib/apt/lists/*
USER user

# run launch file
#CMD ["ros2", "run", "seed", "seed", "test"]




