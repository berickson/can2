#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --

# Auto-restart joy_node in the background: the 8BitDo Pro 3 auto-powers-off after
# idle, and joy_node doesn't reopen /dev/input/jsN when the controller reconnects --
# it just holds a dead file descriptor silently (no error, no exit), confirmed
# 2026-08-02. A respawn loop sidesteps that without needing to fix joy_node itself,
# and also covers the container starting before the controller is plugged in.
#
# `|| true` on the wrapped commands is load-bearing, not decoration: this whole
# script runs under `set -e`, and these subshells inherit that. Without `|| true`,
# the FIRST non-zero exit from the wrapped command aborts the subshell right there
# -- it never reaches the "restarting" echo below it, let alone loops back -- so
# the loop silently stops respawning after exactly one failure. Confirmed live
# 2026-08-03: joy_watchdog.py crashed once and never ran again, no trace in the
# log except the one crash, until this was caught.
(
    while true; do
        echo "$(date): starting joy_node" >> /tmp/joy_node.log
        ros2 run joy joy_node --ros-args -p device_id:=0 >> /tmp/joy_node.log 2>&1 || true
        echo "$(date): joy_node exited, restarting in 1s" >> /tmp/joy_node.log
        sleep 1
    done
) &

# The respawn loop above only helps if joy_node actually exits -- confirmed
# 2026-08-03 it doesn't: a real reconnect hands the kernel a NEW input device
# object under the same /dev/input/jsN path (host kernel log showed "input97"
# replaced by "input98" for the same js0), and joy_node's already-open file
# descriptor just goes silent forever instead of erroring. This watchdog kills
# joy_node when /joy goes stale so the loop above restarts it against whatever
# device is current. See joy_watchdog.py for why staleness is a safe signal here.
(
    while true; do
        echo "$(date): starting joy_watchdog" >> /tmp/joy_node.log
        python3 /joy_watchdog.py >> /tmp/joy_node.log 2>&1 || true
        echo "$(date): joy_watchdog exited, restarting in 1s" >> /tmp/joy_node.log
        sleep 1
    done
) &

exec "$@"
