import omni.usd
from pxr import PhysxSchema

stage = omni.usd.get_context().get_stage()

for side in ["left", "right"]:
    for suffix in ["rear", "filler_rear", "filler_mid", "filler_front"]:
        gear_path = "/World/WhiteCrash/%s_wheel_%s_axle_gear" % (side, suffix)
        gear_prim = stage.GetPrimAtPath(gear_path)
        if not gear_prim.IsValid():
            print(gear_path, "-> PRIM DOES NOT EXIST")
            continue
        gear = PhysxSchema.PhysxPhysicsGearJoint(gear_prim)
        h0 = gear.GetHinge0Rel().GetTargets()
        h1 = gear.GetHinge1Rel().GetTargets()
        ratio = gear.GetGearRatioAttr().Get()
        excl = gear.GetExcludeFromArticulationAttr().Get()
        print(gear_path, "hinge0=", h0, "hinge1=", h1, "ratio=", ratio, "exclude=", excl)
