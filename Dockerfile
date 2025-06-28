FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /usr/src/ros2_ws

# System level dependencies
RUN echo "Installing dependencies..." \
    && apt-get update -yq \
    && apt-get install -y \
    pip \
    git \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-diagnostic-updater

RUN pip install -U \
    colcon-common-extensions \
    rosdep

# Copy directories
COPY ./src/roomba_bringup ./src/roomba_bringup
COPY ./scripts ./scripts
COPY ./src/roomba_navigation ./src/roomba_navigation
COPY ./src/create_description ./src/create_description

WORKDIR /usr/src/ros2_ws/src
RUN git clone -b humble https://github.com/autonomylab/create_robot.git
RUN git clone https://github.com/AutonomyLab/libcreate.git
RUN touch create_robot/create_description/AMENT_IGNORE
RUN git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git

# Install ros depedencies
WORKDIR /usr/src/ros2_ws/
RUN . /opt/ros/humble/setup.sh \
    && rosdep install --from-paths src -y --ignore-src

WORKDIR /usr/src/ros2_ws
RUN sudo apt-get install  -y
RUN  . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

CMD [ "bash",  "scripts/start_navigation.bash"]
