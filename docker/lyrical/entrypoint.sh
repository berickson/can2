#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --

# Auto-restart joy_node in the background: the 8BitDo Pro 3 auto-powers-off after
# idle, and joy_node doesn't reopen /dev/input/jsN when the controller reconnects --
# it just holds a dead file descriptor silently (no error, no exit), confirmed
# 2026-08-02. A respawn loop sidesteps that without needing to fix joy_node itself,
# and also covers the container starting before the controller is plugged in.
(
    while true; do
        echo "$(date): starting joy_node" >> /tmp/joy_node.log
        ros2 run joy joy_node --ros-args -p device_id:=0 >> /tmp/joy_node.log 2>&1
        echo "$(date): joy_node exited, restarting in 1s" >> /tmp/joy_node.log
        sleep 1
    done
) &

exec "$@"
