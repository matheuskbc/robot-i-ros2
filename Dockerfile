FROM ros:foxy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /usr/src/ros2_ws

RUN echo "Installing dependencies..." \
    && apt update -yq \
    && apt-get update -yq \
    && sudo apt-get install -y \
    pip \
    ros-foxy-nav2-* \
    ros-foxy-rmw-cyclonedds-cpp

RUN pip install -U \
    colcon-common-extensions

COPY ./src ./src
COPY ./scripts ./scripts
 
RUN     . /opt/ros/foxy/setup.sh && \
        colcon build --symlink-install

CMD [ "bash",  "scripts/start_navigation.bash"]