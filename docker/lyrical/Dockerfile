FROM ros:lyrical-ros-base-resolute

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-dev-tools \
    ros-lyrical-joy \
    && rm -rf /var/lib/apt/lists/*

# docker exec bypasses ros_entrypoint.sh, so interactive shells wouldn't otherwise
# have ros2/colcon on PATH.
RUN echo "source /opt/ros/lyrical/setup.bash" >> /etc/bash.bashrc

WORKDIR /workspace
