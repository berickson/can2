# Step-response calibration: does the sim actually accelerate the way the
# characterized motor model says the real robot does?
#
# motor_model.py's coefficients were fit from real driving trials, so the model IS
# the ground truth here -- no fresh robot data is needed for a first pass. This
# applies a constant throttle from rest, logs the sim's real chassis speed, and
# prints the model's own predicted curve beside it. If the two diverge, the sim's
# force accounting / inertia / friction are wrong, not the model.
#
# The reference curve is analytic, not simulated. With constant throttle the model
#     dv/dt = (throttle*v_bat - C0 - C1*v) / C2
# is first-order linear, so
#     v(t) = v_term * (1 - exp(-t/tau)),  v_term = (throttle*v_bat - C0)/C1,
#                                          tau    = C2/C1
# which is exact and needs no integration.
#
# THROTTLE default is deliberately small. PLA tracks on wood give mu ~0.35, a
# traction ceiling of only ~3.4 m/s^2, while the model demands 14.3 m/s^2 at full
# throttle -- so hard throttle measures traction, not the drivetrain. Staying
# gripped needs throttle*V_BAT - C0 < 3.4*C2, i.e. throttle below ~0.26. At 0.2 the
# model wants ~2.6 m/s^2 with headroom to spare, so any mismatch here is genuinely
# force accounting. Raise THROTTLE to test the slip regime only once this matches.
#
# Run after build_white_crash.py + Play, pointed down the longest clear runway.

import math
import time

import omni.usd
import omni.kit.app
from isaacsim.core.experimental.prims import RigidPrim

THROTTLE = 0.2
DURATION_SECONDS = 3.0
SAMPLE_SECONDS = 0.25

V_BAT = 12.6
VOLTAGE_C0, VOLTAGE_C1, VOLTAGE_C2 = 0.412, 2.600, 0.850
WHEEL_RADIUS = 0.0375
MU_STATIC = 0.35
ROBOT_MASS = 1.315

# Silence the teleop loop -- it writes torque to the same wheels every frame and
# would fight this test.
if "_white_crash_joy_teleop_sub" in globals():
    globals()["_white_crash_joy_teleop_sub"] = None
    print("[calib] teleop disabled for this test (re-run build_white_crash.py to restore)")

if "_calibrate_dynamics_sub" in globals():
    globals()["_calibrate_dynamics_sub"] = None

stage = omni.usd.get_context().get_stage()
left_wheel = RigidPrim("/World/WhiteCrash/left_wheel_front")
right_wheel = RigidPrim("/World/WhiteCrash/right_wheel_front")
chassis = RigidPrim("/World/WhiteCrash")

v_motor = THROTTLE * V_BAT
v_terminal = (v_motor - VOLTAGE_C0) / VOLTAGE_C1
tau = VOLTAGE_C2 / VOLTAGE_C1
a_initial = (v_motor - VOLTAGE_C0) / VOLTAGE_C2
a_traction_limit = MU_STATIC * 9.81

print("[calib] throttle=%.2f v_bat=%.1f -> model predicts a0=%.2f m/s^2, "
      "v_term=%.3f m/s, tau=%.3f s" % (THROTTLE, V_BAT, a_initial, v_terminal, tau))
print("[calib] traction ceiling at mu=%.2f is %.2f m/s^2 -- %s" % (
    MU_STATIC, a_traction_limit,
    "no slip expected, mismatch means force accounting is wrong"
    if a_initial <= a_traction_limit else
    "MODEL EXCEEDS TRACTION, expect slip and a shortfall vs the reference"))


def _reference_speed(t):
    return v_terminal * (1.0 - math.exp(-t / tau))


_state = {"start": None, "last_sample": 0.0, "done": False}


def _on_update(e):
    if _state["done"]:
        return
    now = time.time()
    if _state["start"] is None:
        _state["start"] = now
    elapsed = now - _state["start"]

    # Speed must be read EVERY frame, not just when printing: motor_model is
    # back-EMF limited via its omega argument, so feeding it a stale (or zero)
    # speed pins the torque at its stall value and the sim accelerates without
    # ever approaching v_term -- it would diverge from the reference by
    # construction rather than because anything is actually miscalibrated.
    lin_vel, _ = chassis.get_velocities()
    speed = float((lin_vel.numpy()[0][:2] ** 2).sum() ** 0.5)

    # Same torque path the teleop loop uses, so this measures the real thing.
    # omega from chassis speed assumes no slip, which is the regime this test is
    # deliberately set up to stay inside.
    torque = _drive_torque_for(THROTTLE, speed / WHEEL_RADIUS)
    for wheel in (left_wheel, right_wheel):
        wheel.apply_forces_and_torques_at_pos(
            torques=[[0.0, torque, 0.0]], local_frame=True)

    if now - _state["last_sample"] >= SAMPLE_SECONDS:
        _state["last_sample"] = now
        expected = _reference_speed(elapsed)
        err = (speed - expected) / expected * 100.0 if expected > 1e-6 else 0.0
        print("[calib] t=%4.2fs  sim=%6.3f m/s  model=%6.3f m/s  err=%+7.1f%%" % (
            elapsed, speed, expected, err))

    if elapsed >= DURATION_SECONDS:
        _state["done"] = True
        print("[calib] DONE. Sustained negative error = sim under-driven or "
              "over-damped; positive = over-driven. Divergence that grows with "
              "speed points at drag/friction, a constant offset at force scaling.")


def _drive_torque_for(throttle, omega):
    # Mirrors the teleop loop: motor_model output scaled by DRIVE_TORQUE_SCALE.
    # Both are defined by build_white_crash.py in this same namespace, so this
    # tracks whatever that script currently does rather than duplicating it.
    return motor_model(throttle, 0.0, omega, V_BAT, WHEEL_RADIUS) * DRIVE_TORQUE_SCALE


_calibrate_dynamics_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(_on_update)
print("[calib] running %.1fs at throttle %.2f" % (DURATION_SECONDS, THROTTLE))
