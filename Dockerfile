FROM ros:foxy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /usr/src/ros2_ws

# System level dependencies
RUN echo "Installing dependencies..." \
    && apt-get update -yq \
    && apt-get install -y \
    pip \
    git \
    ros-foxy-rmw-cyclonedds-cpp \
    ros-foxy-diagnostic-updater

RUN pip install -U \
    colcon-common-extensions \
    rosdep


# Copy directories
COPY ./src/roomba_bringup ./src/roomba_bringup
COPY ./src/sweep_ros ./src/sweep_ros
RUN git clone https://github.com/autonomylab/create_robot.git
RUN cd create_robot && \
    git checkout foxy
RUN git clone https://github.com/AutonomyLab/libcreate.git
COPY ./scripts ./scripts

# Install ros depedencies
RUN . /opt/ros/foxy/setup.sh \
    && apt-get update --fix-missing \
    && rosdep install --from-paths src -y --ignore-src

# Install sweep-sdk
WORKDIR /
RUN git clone https://github.com/scanse/sweep-sdk
RUN mkdir -p build
RUN cd sweep-sdk/libsweep && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    cmake --build . && \
    cmake --build . --target install

WORKDIR /usr/src/ros2_ws
RUN sudo apt-get install  -y
RUN  . /opt/ros/foxy/setup.sh && \
    colcon build --symlink-install

CMD [ "bash",  "scripts/start_navigation.bash"]