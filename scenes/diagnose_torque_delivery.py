# A/B test: is the torque actually REACHING the body in the build script's call
# pattern, or is it being silently discarded?
#
# Why suspect that rather than a physics/resistance cause: the front wheel's
# rotational inertia is I = 0.5*m*r^2 = 0.5*0.02*0.0375^2 = 1.4e-5 kg*m^2. The real
# motor torque of 0.707 N*m implies alpha = tau/I ~= 50,000 rad/s^2 -- the wheel
# should hit hundreds of rad/s within ONE physics step. Observed in the build
# script: 0.02-0.7 rad/s. That's ~5 orders of magnitude off, and no friction,
# damping, or velocity clamp can produce that while diagnose_wheel_force.py spins
# the SAME wheel fine with 50 N*m. A torque that's ~5 orders down and completely
# insensitive to a 10,000x scale sweep isn't being resisted -- it isn't arriving.
#
# The structural difference between the two paths: the working diagnostic ONLY
# writes. The build script READS (get_world_poses(), inside _wheel_spin_rate) and
# then writes, for L and then again for R, every frame. Isaac Sim buffers these
# writes until the next physics step; a read can force a refresh that drops
# pending writes -- so R's read may be discarding L's queued torque, and so on.
#
# Run after build_white_crash.py + Play. Set MODE, run, watch the wheel, then
# change MODE and run again. Same torque magnitude in every mode -- ONLY the
# read/write pattern differs, so any difference in behavior isolates the cause.
#
#   "write_only"      -- control. Matches diagnose_wheel_force.py exactly (which
#                        is known to work). Wheel should spin hard.
#   "read_then_write" -- adds a get_world_poses() read before the write, same as
#                        the build script does. If this DOESN'T spin while
#                        write_only does, the read is killing the write.
#   "two_bodies"      -- L read, L write, R read, R write: the build script's
#                        exact per-frame pattern. If write_only and
#                        read_then_write both spin but this doesn't, the problem
#                        is the second body's read clobbering the first's write.

import omni.usd
import omni.kit.app
from isaacsim.core.experimental.prims import RigidPrim

MODE = "write_only"  # "write_only" | "read_then_write" | "two_bodies"
TORQUE = 50.0  # N*m, same magnitude in all modes -- only the pattern varies

stage = omni.usd.get_context().get_stage()
left_wheel = RigidPrim("/World/WhiteCrash/left_wheel_front")
right_wheel = RigidPrim("/World/WhiteCrash/right_wheel_front")

if "_diagnose_torque_delivery_sub" in globals():
    globals()["_diagnose_torque_delivery_sub"] = None


def _on_update(e):
    if MODE == "write_only":
        left_wheel.apply_forces_and_torques_at_pos(
            torques=[[0.0, TORQUE, 0.0]], local_frame=True)
    elif MODE == "read_then_write":
        left_wheel.get_world_poses()
        left_wheel.apply_forces_and_torques_at_pos(
            torques=[[0.0, TORQUE, 0.0]], local_frame=True)
    elif MODE == "two_bodies":
        left_wheel.get_world_poses()
        left_wheel.apply_forces_and_torques_at_pos(
            torques=[[0.0, TORQUE, 0.0]], local_frame=True)
        right_wheel.get_world_poses()
        right_wheel.apply_forces_and_torques_at_pos(
            torques=[[0.0, TORQUE, 0.0]], local_frame=True)


_diagnose_torque_delivery_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(_on_update)
print("[torque-delivery] MODE=%s TORQUE=%.1f N*m -- watch the yellow lug on the front wheel(s)." % (MODE, TORQUE))
