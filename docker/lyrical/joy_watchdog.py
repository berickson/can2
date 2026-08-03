#!/usr/bin/env python3
# Kills joy_node whenever /joy goes stale, so entrypoint.sh's respawn loop can
# restart it and reopen whatever device is currently live.
#
# Needed because a controller reconnect (e.g. after the 8BitDo Pro 3's auto-sleep)
# hands the kernel a NEW input device object under the same /dev/input/jsN path --
# confirmed via host kernel log, e.g. "input97" replaced by "input98" for the same
# js0 path. joy_node's already-open file descriptor doesn't follow that: it goes
# silent forever with no error and no exit (confirmed live, twice), so a plain
# restart-on-exit loop never fires.
#
# joy_node publishes continuously at a fixed rate even with the stick at rest
# (confirmed empirically -- a bare `ros2 topic echo /joy --once` returns instantly
# with no controller input), so message staleness is a reliable "it's stopped
# working" signal here -- unlike polling the raw /dev/input/jsN file directly,
# which is event-driven and looks identical whether idle or actually broken.

import os
import signal
import subprocess
import time

import rclpy
from sensor_msgs.msg import Joy

STALE_SECONDS = 8.0
CHECK_INTERVAL_SECONDS = 3.0

_last_msg_time = [time.monotonic()]


def _on_joy(_msg):
    _last_msg_time[0] = time.monotonic()


def _check(node):
    age = time.monotonic() - _last_msg_time[0]
    if age < STALE_SECONDS:
        return
    # -x: exact comm-name match, so this can't ever match the watchdog's own
    # process (unlike an earlier -f attempt that matched its own command line
    # and killed itself instead of joy_node).
    result = subprocess.run(["pgrep", "-x", "joy_node"], capture_output=True, text=True)
    pids = [int(p) for p in result.stdout.split()]
    for pid in pids:
        node.get_logger().warning(
            "/joy stale for %.1fs, killing joy_node pid %d to force a reopen" % (age, pid))
        os.kill(pid, signal.SIGKILL)
    # Give the respawn loop a moment to bring it back up before checking again,
    # rather than re-killing every check while it's mid-restart.
    _last_msg_time[0] = time.monotonic()


def main():
    rclpy.init()
    node = rclpy.create_node("joy_watchdog")
    node.create_subscription(Joy, "/joy", _on_joy, 10)
    node.create_timer(CHECK_INTERVAL_SECONDS, lambda: _check(node))
    rclpy.spin(node)


if __name__ == "__main__":
    main()
