# Base image: ROS 2 Humble minimal installation
FROM ros:humble-ros-base

# Mandatory setting to use BASH in subsequent RUN, CMD, ENTRYPOINT commands instead of /bin/sh
SHELL ["/bin/bash", "-c"]

# Setting the working directory
WORKDIR /ws

# Installing necessary dependencies
RUN apt update && \
    apt install -y python3-pip python3-rosdep \
    ros-humble-ament-cmake \
    ros-humble-ros-core \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Installing ROS dependencies with rosdep
RUN rosdep update

# Copy the source code to /ws/src/ [cite: 77]
COPY src src
COPY launch launch

# Install missing dependencies (including custom_interfaces)
RUN rosdep install --from-paths src --ignore-src -r -y

RUN . /opt/ros/humble/setup.bash && \
    export AMENT_PREFIX_PATH=/opt/ros/humble:$AMENT_PREFIX_PATH && \
    colcon build

# Setting the ENTRYPOINT [cite: 79]
COPY entrypoint.sh entrypoint.sh
RUN chmod +x entrypoint.sh

ENTRYPOINT ["/ws/entrypoint.sh"]
CMD ["bash"]
