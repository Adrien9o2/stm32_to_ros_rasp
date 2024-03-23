# ROS distribution to use
ARG ROS_DISTRO=iron

##############
# Base Image #
##############
#FROM osrf/ros:${ROS_DISTRO}-desktop as base
FROM ros:${ROS_DISTRO} as base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

# Install basic apt packages
RUN apt update 
RUN apt-get install -y --no-install-recommends \
git libcanberra-gtk-module libcanberra-gtk3-module fuse3 libfuse2 libqt5svg5-dev \
 python3-pip python3-opencv python3-tk python3-pyqt5.qtwebengine libserial-dev

# Install additional Python modules
RUN pip3 install matplotlib transforms3d
RUN pip3 install keyboard
RUN sudo apt install debconf-utils
RUN echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections
RUN sudo apt-get install -y -q
COPY keyboardconf.conf keyboardconf.conf
RUN debconf-set-selections < keyboardconf.conf
RUN apt-get install -y kbd
RUN sudo apt-get install -y xterm

# Use Cyclone DDS as middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
 ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Create Colcon workspace with external dependencies
RUN mkdir -p /ros2_ws/src


# Build the base Colcon workspace, installing dependencies first.
WORKDIR /ros2_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install


# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE 1

# Set up the entrypoint
WORKDIR /ros2_ws
COPY ./docker/entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

#######################################
# Overlay Image for stm32_to_ros_rasp #
#######################################
FROM base AS overlay


# Create an overlay Colcon workspace
RUN mkdir -p /overlay_ws/src
WORKDIR /overlay_ws
COPY ./stm32_to_ros_rasp/ ./src/stm32_to_ros_rasp/
COPY ./launch.py ./src/launch.py
COPY ./launchkeyboard.py ./src/launchkeyboard.py 
RUN source /ros2_ws/install/setup.bash \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install

# # Set up the entrypoint
COPY ./docker/entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

#####################
# Development Image #
#####################
FROM overlay as dev


# Install extra tools for development
RUN apt-get update && apt-get install -y --no-install-recommends gdb gdbserver nano

RUN echo "source /entrypoint.sh" >> /root/.bashrc