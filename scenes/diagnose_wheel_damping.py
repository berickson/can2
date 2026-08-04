# Something is capping wheel speed at a very low, torque-independent ceiling --
# proportional at tiny torque (0.0001*throttle visibly slower), saturated
# anywhere from 0.1*throttle up to 1000*throttle (same speed regardless), even
# after ruling out maxAngularVelocity/maxLinearVelocity/maxDepenetrationVelocity
# on the wheel's PhysxRigidBodyAPI (all raised, no change) and the joint's
# DriveAPI (damping/stiffness confirmed 0.0, maxForce=inf -- genuinely inert).
# Two more candidates from research, checked here across ALL 5 joints on a side
# (not just the front one -- the bottleneck could be on any gear-coupled joint):
# - PhysxJointAxisAPI:maxJointVelocity (per-axis velocity clamp on the JOINT
#   itself, separate from the rigid body's own cap -- default is huge (~1e6
#   deg/s) so unlikely, but unverified on our actual joints until now).
# - PhysxJointAPI:jointFriction (Coulomb friction resisting joint motion --
#   unlikely to explain a scale-dependent plateau by itself since Coulomb
#   friction is a constant offset, not velocity-dependent, but cheap to rule out).
# Run any time after build_white_crash.py (Play not required).

import omni.usd
from pxr import UsdPhysics, PhysxSchema, Usd

stage = omni.usd.get_context().get_stage()

joint_names = [
    ("front", "/World/WhiteCrash/left_wheel_front_axle"),
    ("rear", "/World/WhiteCrash/left_wheel_rear_axle"),
    ("filler_rear", "/World/WhiteCrash/left_wheel_filler_rear_carrier/left_wheel_filler_rear_axle"),
    ("filler_mid", "/World/WhiteCrash/left_wheel_filler_mid_carrier/left_wheel_filler_mid_axle"),
    ("filler_front", "/World/WhiteCrash/left_wheel_filler_front_carrier/left_wheel_filler_front_axle"),
]

for name, path in joint_names:
    joint_prim = stage.GetPrimAtPath(path)
    print("--- %s (%s) ---" % (name, path))
    if not joint_prim.IsValid():
        print("  PRIM NOT FOUND")
        continue

    drive = UsdPhysics.DriveAPI(joint_prim, "angular")
    print("  DriveAPI: damping=%s stiffness=%s maxForce=%s targetVelocity=%s" % (
        drive.GetDampingAttr().Get(), drive.GetStiffnessAttr().Get(),
        drive.GetMaxForceAttr().Get(), drive.GetTargetVelocityAttr().Get()))

    if joint_prim.HasAPI(PhysxSchema.PhysxJointAPI):
        physx_joint = PhysxSchema.PhysxJointAPI(joint_prim)
        friction_attr = physx_joint.GetJointFrictionAttr()
        print("  PhysxJointAPI: jointFriction=%s" % (friction_attr.Get() if friction_attr else "N/A (attr not authored)"))
    else:
        print("  PhysxJointAPI: not applied on this prim")

    # Multi-apply schema, instance name token varies by axis -- try the ones that
    # could plausibly apply to a Y-axis revolute joint rather than assume one.
    found_axis_api = False
    for instance_name in ("rotY", "angular"):
        try:
            if joint_prim.HasAPI(PhysxSchema.PhysxJointAxisAPI, instance_name):
                axis_api = PhysxSchema.PhysxJointAxisAPI(joint_prim, instance_name)
                print("  PhysxJointAxisAPI[%s]: maxJointVelocity=%s" % (
                    instance_name, axis_api.GetMaxJointVelocityAttr().Get()))
                found_axis_api = True
        except Exception as e:
            print("  PhysxJointAxisAPI[%s] check failed: %s" % (instance_name, e))
    if not found_axis_api:
        print("  PhysxJointAxisAPI: not applied on this prim (any checked instance)")
