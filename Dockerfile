ARG ROS_DISTRO=noetic

FROM osrf/ros:${ROS_DISTRO}-desktop-full as deps

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_PYTHON_VERSION=3

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
        python3-rosdep \
        python3-catkin-tools \
        build-essential \
        libyaml-cpp-dev \
        ros-${ROS_DISTRO}-ompl \
        ros-${ROS_DISTRO}-grid-map-core \
        ros-${ROS_DISTRO}-grid-map-ros \
        ros-${ROS_DISTRO}-grid-map-msgs \
        ros-${ROS_DISTRO}-geometry-msgs \
        ros-${ROS_DISTRO}-visualization-msgs \
        ros-${ROS_DISTRO}-std-msgs \
        ros-${ROS_DISTRO}-dynamic-reconfigure \
        ros-${ROS_DISTRO}-tf \
        ros-${ROS_DISTRO}-tf2 \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-urdf \
        ros-${ROS_DISTRO}-eigen-conversions \
        ros-${ROS_DISTRO}-rviz \
        ros-${ROS_DISTRO}-fake-localization \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/catkin_ws

RUN mkdir -p src
COPY . /root/catkin_ws/src/se2_navigation

FROM deps as builder

ARG CMAKE_BUILD_TYPE=Release

WORKDIR /root/catkin_ws

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} && \
    catkin build

RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

CMD ["bash"]
