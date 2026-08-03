# Find which prims under /World/ground actually have collision geometry -- the
# earlier bbox-based floor alignment used a prim picked by name only, which may not
# be the actual physics collision surface (could be a decorative/visual-only mesh).

import omni.usd
from pxr import UsdGeom, UsdPhysics, Usd

stage = omni.usd.get_context().get_stage()
ground_prim = stage.GetPrimAtPath("/World/ground")
bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

print("--- Prims under /World/ground with CollisionAPI ---")
for prim in Usd.PrimRange(ground_prim):
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        b = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
        print(prim.GetPath(), "->", b)

print("--- Actual chassis position after settling ---")
chassis_prim = stage.GetPrimAtPath("/World/WhiteCrash")
if chassis_prim.IsValid():
    xform_cache = UsdGeom.XformCache()
    world_transform = xform_cache.GetLocalToWorldTransform(chassis_prim)
    print("chassis world translate:", world_transform.ExtractTranslation())
