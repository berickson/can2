# Joystick teleop: 8BitDo Pro 3 -> /joy (published by joy_node in the lyrical
# container) -> per-side throttle/drag_brake -> motor_model.py -> wheel torque.
# Run after build_white_crash.py, sim playing.
#
# Axis mapping (confirmed empirically 2026-08-02 against this specific controller +
# joy_node, not from any spec):
#   axes[1] = left stick Y, forward (away from body) = +1.0 -- already matches
#             throttle_percent's own forward-positive convention, no sign flip needed.
#   axes[3] = right stick Y, same convention.
#   axes[4] = R2, rest = +1.0 (not pressed), full pull = -1.0.
#   axes[5] = L2, same convention as R2.
#
# motor_model.py is inlined here since training/ isn't mounted into the container
# (only scenes/ is) -- this is the same logic as training/motor_model.py, just
# copy-pasted for this quick test rather than properly imported.
#
# Simplification: reads angular velocity in the WORLD frame and assumes it stays
# aligned with the wheel's local spin axis (true while the robot's flat/upright,
# not exact if it tips or turns hard) -- fine for a first interactive test, not a
# permanent design.

import rclpy
from sensor_msgs.msg import Joy
import omni.usd
import omni.kit.app
from pxr import UsdPhysics
from isaacsim.core.experimental.prims import RigidPrim

# --- motor_model.py inlined ---
VOLTAGE_C0 = 0.412
VOLTAGE_C1 = 2.600
VOLTAGE_C2 = 0.850
COAST_C0 = 0.951
COAST_C1 = 0.794
BRAKE_C0 = 1.008
BRAKE_C1 = 2.950
CHARACTERIZATION_MASS_KG = 1.315
WHEEL_RADIUS = 0.0365
DEADBAND_V = 0.01  # m/s, "close enough to at-rest" for stiction-deadband purposes


def _signed_resistive_decel(v, c0, c1):
    magnitude = c0 + c1 * abs(v)
    if v > 0:
        return -magnitude
    if v < 0:
        return magnitude
    return 0.0


def _accel_from_throttle(throttle_percent, v, v_bat):
    if throttle_percent == 0:
        return _signed_resistive_decel(v, COAST_C0, COAST_C1)
    v_motor = throttle_percent * v_bat
    a = (v_motor - VOLTAGE_C0 - VOLTAGE_C1 * v) / VOLTAGE_C2
    if abs(v) < DEADBAND_V and a * throttle_percent < 0:
        a = 0.0
    return a


def _accel_from_drag_brake(v):
    return _signed_resistive_decel(v, BRAKE_C0, BRAKE_C1)


def motor_model(throttle_percent, drag_brake_percent, omega_current, v_bat, wheel_radius):
    v = omega_current * wheel_radius
    a_throttle = _accel_from_throttle(throttle_percent, v, v_bat)
    a_brake = _accel_from_drag_brake(v)
    a = (1 - drag_brake_percent) * a_throttle + drag_brake_percent * a_brake
    force = CHARACTERIZATION_MASS_KG * a
    torque = force * wheel_radius
    return torque


# --- disable the leftover velocity-drive on the two reference wheels so it
# doesn't fight our open-loop torque application ---
stage = omni.usd.get_context().get_stage()
for side in ["left", "right"]:
    joint_prim = stage.GetPrimAtPath("/World/WhiteCrash/%s_wheel_front_axle" % side)
    drive_api = UsdPhysics.DriveAPI(joint_prim, "angular")
    drive_api.GetDampingAttr().Set(0.0)
    drive_api.GetStiffnessAttr().Set(0.0)

left_wheel = RigidPrim("/World/WhiteCrash/left_wheel_front")
right_wheel = RigidPrim("/World/WhiteCrash/right_wheel_front")

V_BAT = 7.4  # adjust to match the real battery's nominal voltage

# --- joystick ---
if not rclpy.ok():
    rclpy.init()

joy_node = rclpy.create_node("isaac_joy_teleop")
_latest_joy_axes = [0.0] * 8


def on_joy(msg):
    global _latest_joy_axes
    _latest_joy_axes = msg.axes


joy_sub = joy_node.create_subscription(Joy, "/joy", on_joy, 10)


def drag_brake_from_trigger(axis_value):
    # rest = +1.0 -> 0.0 brake, full pull = -1.0 -> 1.0 brake
    return max(0.0, min(1.0, (1.0 - axis_value) / 2.0))


import time
_last_debug_print = [0.0]


def on_update(e):
    rclpy.spin_once(joy_node, timeout_sec=0)
    axes = _latest_joy_axes
    left_throttle = axes[1]
    right_throttle = axes[3]
    left_drag_brake = drag_brake_from_trigger(axes[5])
    right_drag_brake = drag_brake_from_trigger(axes[4])

    do_print = False
    now = time.time()
    if now - _last_debug_print[0] > 0.3:
        _last_debug_print[0] = now
        do_print = True

    for name, wheel, throttle, drag_brake in (
        ("L", left_wheel, left_throttle, left_drag_brake),
        ("R", right_wheel, right_throttle, right_drag_brake),
    ):
        _, angular_velocities = wheel.get_velocities()
        omega_y = float(angular_velocities.numpy()[0][1])
        v = omega_y * WHEEL_RADIUS
        torque = motor_model(throttle, drag_brake, omega_y, V_BAT, WHEEL_RADIUS)
        wheel.apply_forces_and_torques_at_pos(torques=[[0.0, torque, 0.0]], local_frame=True)
        if do_print:
            print("%s throttle=%.2f brake=%.2f v=%.3f m/s omega=%.2f rad/s torque=%.4f N*m" % (
                name, throttle, drag_brake, v, omega_y, torque))


joy_teleop_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(on_update)
print("Joystick teleop running -- left/right stick = throttle, L2/R2 = drag brake.")
