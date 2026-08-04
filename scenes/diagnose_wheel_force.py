# Isolation test for "wheels won't move at any torque, even 300x real motor
# torque" -- bypasses the joystick, motor_model, and gear-joint coupling entirely,
# to answer one question: does ANY applied force/torque actually move
# left_wheel_front at all? Run build_white_crash.py first, then Play, then paste
# this into Script Editor and run it.
#
# Applies continuously (once per physics step, not a single one-off call) -- a
# single call is only a one-frame impulse (force * one dt), way too small to
# visibly react even if everything's working. Two checks, run one at a time by
# editing MODE below:
# "force"  -- big upward linear force. If apply_forces_and_torques_at_pos works
#             on this body at all, the wheel should visibly launch off the ground.
# "torque" -- big torque around the wheel's own spin axis (same call shape
#             build_white_crash.py uses for driving). If "force" works but this
#             doesn't, the bug is specific to torque/rotation, not force
#             application in general.

import omni.usd
import omni.kit.app
from isaacsim.core.experimental.prims import RigidPrim

MODE = "force"  # "force" or "torque"

stage = omni.usd.get_context().get_stage()
wheel = RigidPrim("/World/WhiteCrash/left_wheel_front")

if "_diagnose_wheel_force_sub" in globals():
    globals()["_diagnose_wheel_force_sub"] = None


def _on_update(e):
    if MODE == "force":
        wheel.apply_forces_and_torques_at_pos(forces=[[0.0, 0.0, 50.0]], local_frame=False)
    else:
        wheel.apply_forces_and_torques_at_pos(torques=[[0.0, 50.0, 0.0]], local_frame=True)


_diagnose_wheel_force_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(_on_update)
print("Applying %s to left_wheel_front every frame -- watch the wheel. MODE=%s" % (
    "50N straight up (world frame)" if MODE == "force" else "50 N*m torque (local Y)", MODE))
