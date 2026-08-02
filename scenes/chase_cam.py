# Adds a chase camera rigidly mounted behind/above the chassis (child prim, so it
# moves with the robot automatically -- no per-frame scripting needed) and switches
# the active viewport to it. Run after build_white_crash.py.
#
# Camera rotation math: USD cameras always look down local -Z with local +Y as "up",
# regardless of the stage's up-axis. This stage is Z-up (see build_white_crash.py's
# chassis height convention), so getting a horizontal, forward-facing, right-side-up
# camera needs two rotations: RotateX(90) first (realigns local Y-up to world Z-up,
# and incidentally swings local -Z-forward to point at world +Y), then RotateZ(-90)
# (yaws that into world +X, this robot's forward direction per the wheel/TOF layout).
# The downward tilt is folded into the X rotation as 90-pitch_deg instead of a
# separate op, since a bigger X angle here tilts the view up, not down -- confirmed
# by working through the rotation algebra, not guessed; still worth a visual check
# since this project has been burned by camera/rotation direction mistakes before.

import omni.usd
from pxr import UsdGeom, Gf

stage = omni.usd.get_context().get_stage()

chassis_path = "/World/WhiteCrash"
if not stage.GetPrimAtPath(chassis_path).IsValid():
    raise RuntimeError("Build the robot first (build_white_crash.py)")

camera_path = chassis_path + "/ChaseCam"
camera = UsdGeom.Camera.Define(stage, camera_path)

# Offset: behind (-X) and above (+Z) the chassis origin. ~1.5m out -- confirmed by
# hand (2026-08-02) as a good distance for this ~19cm robot; same direction as the
# original 0.36m guess, just scaled up.
offset = Gf.Vec3f(-1.3, 0.0, 0.8)
pitch_down_deg = 20.0

translate_op = camera.AddTranslateOp()
translate_op.Set(offset)
rotate_z_op = camera.AddRotateZOp()
rotate_z_op.Set(-90.0)
rotate_x_op = camera.AddRotateXOp()
rotate_x_op.Set(90.0 - pitch_down_deg)
# xformOpOrder: first-listed = applied last (outermost). We want rotateX (local
# realign+pitch) applied first, then rotateZ (yaw), then translate last.
camera.SetXformOpOrder([translate_op, rotate_z_op, rotate_x_op])

camera.CreateFocalLengthAttr(18.0)  # wide-ish, so the small nearby robot isn't cropped

print("Chase camera created at", camera_path)

# Switch the active viewport to it.
import omni.kit.viewport.utility as vp_utils

viewport = vp_utils.get_active_viewport()
viewport.camera_path = camera_path
print("Active viewport camera set to ChaseCam. Switch back via the viewport's camera dropdown (pick 'Perspective') if needed.")
