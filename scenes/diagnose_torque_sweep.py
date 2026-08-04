# Automated torque sweep with real telemetry -- answers "does wheel spin rate
# actually scale with applied torque?" quantitatively, instead of by eye.
#
# Everything before this depended on visual impressions of a small yellow lug,
# which is how we ended up with contradictory readings all session. This holds
# each torque value for HOLD_SECONDS, samples the wheel's true spin rate by
# differencing its orientation quaternion (the one measurement method confirmed
# trustworthy -- get_velocities()'s frame convention proved wrong earlier), and
# prints it. Read the log afterward; no eyeballing.
#
# Uses the two-body write pattern (torque to BOTH front wheels every frame),
# since that's what build_white_crash.py does and it's the pattern confirmed to
# actually deliver torque.
#
# IMPORTANT: this kills build_white_crash.py's teleop subscription first. Both
# scripts writing torque to the same bodies every frame fight each other --
# Isaac buffers these writes and a later call in the same physics step overrides
# an earlier one, so leaving teleop running makes every reading here suspect.
# Re-run build_white_crash.py afterward to get teleop back.
#
# Run after build_white_crash.py + Play.

import math
import time

import omni.usd
import omni.kit.app
from pxr import Gf
from isaacsim.core.experimental.prims import RigidPrim

TORQUES = [0.01, 0.1, 1.0, 10.0, 100.0]  # N*m, 4 orders of magnitude
HOLD_SECONDS = 3.0
SAMPLE_SECONDS = 0.5

# Silence build_white_crash.py's teleop so it isn't also writing torque.
if "_white_crash_joy_teleop_sub" in globals():
    globals()["_white_crash_joy_teleop_sub"] = None
    print("[sweep] build_white_crash.py teleop subscription disabled for this test")

if "_diagnose_torque_sweep_sub" in globals():
    globals()["_diagnose_torque_sweep_sub"] = None

stage = omni.usd.get_context().get_stage()
left_wheel = RigidPrim("/World/WhiteCrash/left_wheel_front")
right_wheel = RigidPrim("/World/WhiteCrash/right_wheel_front")
chassis = RigidPrim("/World/WhiteCrash")

_spin_history = {}


def _spin_rate(name, wheel):
    # Orientation-differencing, same method as build_white_crash.py's
    # _wheel_spin_rate -- projects the measured rotation onto the wheel's own
    # live spin axis so chassis yaw doesn't contaminate the reading.
    now = time.time()
    _, orientation = wheel.get_world_poses()
    w, x, y, z = (float(c) for c in orientation.numpy()[0])
    curr = Gf.Quatd(w, Gf.Vec3d(x, y, z))
    prev = _spin_history.get(name)
    _spin_history[name] = (now, curr)
    if prev is None:
        return 0.0
    prev_time, prev_quat = prev
    dt = now - prev_time
    if dt <= 0.0:
        return 0.0
    rel = Gf.Rotation(curr * prev_quat.GetInverse())
    angle = math.radians(rel.GetAngle())
    axis_world_y = Gf.Vec3d(
        2 * (x * y - w * z),
        1 - 2 * (x * x + z * z),
        2 * (y * z + w * x),
    ).GetNormalized()
    return (angle if Gf.Dot(rel.GetAxis(), axis_world_y) >= 0 else -angle) / dt


_state = {"index": 0, "phase_start": time.time(), "last_sample": 0.0, "done": False}
_accum = {"L": 0.0, "R": 0.0, "n": 0}


def _on_update(e):
    if _state["done"]:
        return
    now = time.time()
    torque = TORQUES[_state["index"]]

    for wheel in (left_wheel, right_wheel):
        wheel.apply_forces_and_torques_at_pos(
            torques=[[0.0, torque, 0.0]], local_frame=True)

    # Sample EVERY frame, not once per print. Quaternion differencing can only
    # resolve rotation up to one revolution per sample, so at a 0.5s sample
    # interval anything above 2*pi/0.5 = 12.57 rad/s aliases -- which is exactly
    # what the first run of this script produced: a hard ceiling at ~12.5 rad/s
    # with randomly flipping sign at every torque level, pure artifact. At frame
    # rate (dt ~= 16ms) the limit is ~376 rad/s instead. Averaging the per-frame
    # samples over the print window then gives a stable number to read.
    _accum["L"] += _spin_rate("L", left_wheel)
    _accum["R"] += _spin_rate("R", right_wheel)
    _accum["n"] += 1

    if now - _state["last_sample"] >= SAMPLE_SECONDS:
        _state["last_sample"] = now
        n = max(1, _accum["n"])
        omega_l = _accum["L"] / n
        omega_r = _accum["R"] / n
        _accum["L"] = _accum["R"] = 0.0
        _accum["n"] = 0
        lin_vel, _ = chassis.get_velocities()
        speed = float((lin_vel.numpy()[0][:2] ** 2).sum() ** 0.5)
        rolling = speed / 0.0375  # rad/s the wheel WOULD turn if not slipping
        print("[sweep] torque=%8.3f N*m  omega_L=%8.2f  omega_R=%8.2f rad/s  "
              "chassis_speed=%6.3f m/s  no-slip_omega=%7.2f rad/s" % (
                  torque, omega_l, omega_r, speed, rolling))

    if now - _state["phase_start"] >= HOLD_SECONDS:
        _state["index"] += 1
        _state["phase_start"] = now
        _spin_history.clear()
        if _state["index"] >= len(TORQUES):
            _state["done"] = True
            print("[sweep] DONE -- if omega stops rising while torque keeps climbing, "
                  "that plateau is the bug; if it scales, torque is fine and the "
                  "problem is upstream in motor_model.")


_diagnose_torque_sweep_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(_on_update)
print("[sweep] starting: %s N*m, %.0fs each, both front wheels" % (TORQUES, HOLD_SECONDS))
