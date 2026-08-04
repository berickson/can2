# Self-contained, self-healing build for white-crash: room + chassis + skid-steer
# track approximation + sensor markers + chase camera + joystick teleop, all in one
# script. Deletes and rebuilds its own prims every run, so it's always safe to just
# re-run this after any change -- no separate scripts needed. Paste into Script
# Editor and run (or open scenes/build_white_crash.py directly, since training/ isn't
# mounted into the container).
#
# Physical values are from real measurements (training/PLAN.md). Chassis is still a
# plain box (real OnShape CAD import is a later step), but its dims are now measured
# too, not a placeholder -- exact shape still doesn't matter for physics as long as
# mass/CoM/inertia are right, and visuals are meant to be ugly for now.
#
# Track/wheel approximation matches training/PLAN.md's "Track/wheel modeling" section:
# 2 real end wheels (rigid, no suspension) + 3 driven filler wheels per side on a
# vertical prismatic joint with a *limit* (not a spring -- the real track has slack
# then goes rigid, not linear springiness), limit scaled by a parabola matching the
# measured ~1cm center deflection. All wheels driven at the same rate (one continuous
# belt, not independent idlers). Self-collision disabled within the wheel row via a
# CollisionGroup, since the wheels sit close together approximating one surface.
#
# Driving: motor_model.py's blended throttle/drag-brake model, fed from a joystick
# (8BitDo Pro 3 -> /joy via joy_node in the lyrical container -> rclpy, subscribed
# directly here). motor_model.py is inlined since training/ isn't mounted into the
# container -- same logic as training/motor_model.py, kept in sync by hand.

import math
import time
import omni.usd
import omni.kit.app
import omni.kit.viewport.utility as vp_utils
import omni.timeline
import rclpy
from sensor_msgs.msg import Joy
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, PhysxSchema, Sdf, Gf
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.experimental.prims import RigidPrim

stage = omni.usd.get_context().get_stage()

# Clean up subscriptions/nodes left over from earlier, differently-named scratch
# scripts run in this same Kit session (this script's own update subscription and
# rclpy node are self-healing on repeat runs since they keep consistent names below,
# but anything from before that naming existed -- e.g. old on_update/joy_teleop_sub/
# motor_test_sub/joy_debug_sub from prior scenes/immediate.py iterations -- would
# otherwise sit there forever, still firing every frame alongside this one, since
# reassigning a differently-named global doesn't garbage-collect a different name).
# Sweep by TYPE, not by name. This used to be a hardcoded list of known names,
# which silently failed the moment a new script introduced a new name -- confirmed
# live 2026-08-03: a leftover subscription from scenes/diagnose_torque_delivery.py
# survived a rebuild, kept firing every frame against prims that had since been
# deleted (spamming "physics tensor entity is not valid"), and kept slamming 50 N*m
# into the wheels, which looked exactly like a physics explosion in this script.
# Anything holding an update-event subscription or an rclpy node gets dropped here
# regardless of what it's called, so diagnostics can never poison a rebuild again.
_update_stream = omni.kit.app.get_app().get_update_event_stream()
_probe_sub = _update_stream.create_subscription_to_pop(lambda e: None)
_subscription_type = type(_probe_sub)
_probe_sub = None

for _name in list(globals()):
    if _name.startswith("__") or _name in ("_probe_sub", "_subscription_type"):
        continue
    _obj = globals()[_name]
    if isinstance(_obj, _subscription_type):
        globals()[_name] = None  # RAII handle -- dropping the ref unsubscribes
        print("[build] dropped leftover update subscription: %s" % _name)
    elif hasattr(_obj, "destroy_node") and _obj.__class__.__module__.startswith("rclpy"):
        try:
            _obj.destroy_node()
            print("[build] destroyed leftover rclpy node: %s" % _name)
        except Exception:
            pass
        globals()[_name] = None

# (The type-based sweep above already covers this script's own subscription and
# rclpy node from a previous run. Doing it BEFORE any prim deletion below is the
# important part -- when this teardown lived at the bottom of the script instead,
# a previous run's _on_update kept firing all through ground/chassis deletion,
# hitting stale RigidPrim handles and corrupting the rebuild.)

# Stop the timeline before any destructive prim surgery below -- PhysX explicitly
# warns that removing a CollisionGroup (or other physics prims) while playing is
# undefined behavior, and this bit us directly (2026-08-02) TWICE: timeline.stop()
# is asynchronous (only takes effect on the next app update tick), so deleting prims
# immediately afterward was still racing PhysX's simulation actually being live --
# pump a few app updates so the stop has actually landed before touching anything.
timeline = omni.timeline.get_timeline_interface()
if timeline.is_playing():
    timeline.stop()
    for _ in range(5):
        omni.kit.app.get_app().update()

# --- Ground: always delete + recreate, so a stale/wrong environment never lingers ---
ground_path = "/World/ground"
if stage.GetPrimAtPath(ground_path).IsValid():
    stage.RemovePrim(ground_path)
assets_root_path = get_assets_root_path()
ground_prim = stage_utils.add_reference_to_stage(
    usd_path=assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd",
    path=ground_path,
)
# Unlike Grid (floor at z=0, what the chassis spawn height below assumes), this room
# needs a shift so its floor lands at z=0. First attempt (0.8154) used a bbox on a
# prim named "Floor" -- wrong one: the room's real physics floor is a separate
# infinite /World/ground/GroundPlane/CollisionPlane (bbox is unbounded, can't read
# its height from a bbox at all), which sits ~5cm higher. Recalibrated empirically
# 2026-08-02 by watching where the chassis actually settles after Play, not by
# picking a prim by name and hoping it's the collision surface.
UsdGeom.Xformable(ground_prim).AddTranslateOp().Set(Gf.Vec3f(0, 0, 0.7661))

root_path = "/World/WhiteCrash"
# Re-running with the viewport still looking through last run's ChaseCam (a child
# of root_path, about to be deleted below) left the render surface permanently
# blank/white -- confirmed live 2026-08-03. Deleting a viewport's ACTIVE camera
# out from under it doesn't self-recover just because a new camera prim shows up
# at the same path moments later in this same run. Switch to the always-present
# default perspective camera first if that's what's about to happen.
_active_viewport = vp_utils.get_active_viewport()
if _active_viewport.camera_path and str(_active_viewport.camera_path).startswith(root_path):
    _active_viewport.camera_path = "/OmniverseKit_Persp"
if stage.GetPrimAtPath(root_path).IsValid():
    stage.RemovePrim(root_path)

# --- Chassis ---
# Measured 2026-08-02 (calipers against the real chassis box).
chassis_length = 0.165
chassis_width = 0.14
chassis_height = 0.045
chassis_mass = 1.315  # kg, measured
chassis_com = Gf.Vec3f(0.02, 0, 0)  # 20mm forward of geometric center, measured

# Needed here (not just in the wheel section below) to place the chassis at a
# starting height where the wheels rest ON the ground rather than through it.
wheel_radius = 0.0375  # 75mm tracked-wheel diameter, measured 2026-08-02

# The track pokes past the chassis box asymmetrically (5mm above the top, 20mm below
# the bottom, measured 2026-08-02) -- unlike the old placeholder, the wheel center is
# NOT at the chassis's vertical center anymore. Back-computed from the two overhangs
# (top: -10mm, bottom: -5mm) and split the ~5mm difference between them (measurement
# rounding, not a real asymmetry) -- re-measure directly if this ever matters more
# precisely than it does for a first-pass visual/physics model.
wheel_z_offset = -0.0075

chassis_xform = UsdGeom.Xform.Define(stage, root_path)
# Deliberately NOT an articulation (no ArticulationRootAPI): PxGearJoint is a
# maximal-coordinate constraint, and mixing that with reduced-coordinate
# articulation solving is a documented, known-fragile combination (confirmed via
# NVIDIA forum reports of exactly this symptom) -- not a config mistake on our end.
# Plain maximal-coordinate rigid-body-and-joint simulation is what gear joints are
# actually designed for.
# Small clearance above the wheel-bottom height so wheels don't start exactly
# touching the ground (avoids initial-contact jitter/penetration at frame 0). Wheel
# bottom sits at (wheel_z_offset - wheel_radius) in chassis-local z, so the chassis
# origin needs to start at the negation of that, not just wheel_radius.
# XY offset (-3, 0) clears Simple_Room's table_low prop, which sits at the room's
# origin (x:[-1.6,1.6], y:[-0.8,0.8], measured 2026-08-02) -- spawning at (0,0) would
# land the chassis on top of it instead of on the floor.
spawn_xy = Gf.Vec2f(-3.0, 0.0)
spawn_z = wheel_radius - wheel_z_offset + 0.002
chassis_xform.AddTranslateOp().Set(Gf.Vec3f(spawn_xy[0], spawn_xy[1], spawn_z))

chassis_geom = UsdGeom.Cube.Define(stage, root_path + "/ChassisGeom")
chassis_geom.CreateSizeAttr(1.0)
chassis_geom.AddScaleOp().Set(Gf.Vec3f(chassis_length, chassis_width, chassis_height))

UsdPhysics.RigidBodyAPI.Apply(chassis_xform.GetPrim())
# Max-velocity attrs live on the PhysX-specific extension schema, not base
# UsdPhysics.RigidBodyAPI -- confirmed live 2026-08-03 (AttributeError). Same
# reasoning as the wheels' override below -- see there.
chassis_physx_rigid_body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(chassis_xform.GetPrim())
chassis_physx_rigid_body_api.CreateMaxLinearVelocityAttr(100.0)
# Matches the wheels' raised iteration counts -- the whole coupled system has to
# be solved together, so leaving the heaviest body at the low defaults would
# undo much of the benefit. See the wheel inertia comment for the full rationale.
chassis_physx_rigid_body_api.CreateSolverPositionIterationCountAttr(32)
chassis_physx_rigid_body_api.CreateSolverVelocityIterationCountAttr(8)
UsdPhysics.CollisionAPI.Apply(chassis_geom.GetPrim())

mass_api = UsdPhysics.MassAPI.Apply(chassis_xform.GetPrim())
mass_api.CreateMassAttr(chassis_mass)
mass_api.CreateCenterOfMassAttr(chassis_com)
ixx = chassis_mass * (chassis_width**2 + chassis_height**2) / 12.0
iyy = chassis_mass * (chassis_length**2 + chassis_height**2) / 12.0
izz = chassis_mass * (chassis_length**2 + chassis_width**2) / 12.0
mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(ixx, iyy, izz))

# --- Shared wheel/track constants (all measured, see training/PLAN.md) ---
# wheel_radius defined earlier, above the chassis section.
wheel_width = 0.026    # track width
wheelbase = 0.123      # front-to-back hub spacing per side
half_wheelbase = wheelbase / 2.0
track_y = 0.0985       # half of 197mm overall width
max_center_deflection = 0.01  # measured ~1cm at the very center

# Off for now to isolate a debugging question -- see the track_envelope comment
# below where it's actually used.
ENABLE_TRACK_ENVELOPE = False

# --- Wheel friction material ---
# Nothing set a PhysicsMaterial anywhere before this -- wheels and ground were both
# running on whatever PhysX's implicit default is (0.5/0.5 kinetic/static, blended
# with the ground's own default via PhysX's combine rule). Randomizable per-material
# friction is planned later (training/PLAN.md's domain-randomization step); 1.0/0.9
# is a rough rubber-on-hard-floor guess, not measured.
#
# This got pushed up to 1.0/0.9 while debugging a table-foot climb that turned out
# to actually be a measurement bug (the wheel-spin reading was wrong, not the
# physics -- see _wheel_spin_rate), so high friction never even helped the thing
# it was raised for. Meanwhile it had a real, unrelated cost: skid-steer turns in
# place by slipping the tracks sideways against the ground, and high isotropic
# friction (this engine doesn't model directional/anisotropic friction, so
# "grippy" applies equally to that sideways slip) fights exactly the motion
# turning needs -- confirmed live 2026-08-03, the robot could barely rotate in
# place at 1.0/0.9 and mostly wanted to go straight. Back down to something more
# moderate now that the climb question is separately being chased for real.
wheel_material_path = root_path + "/WheelMaterial"
wheel_material_shade = UsdShade.Material.Define(stage, wheel_material_path)
wheel_material = UsdPhysics.MaterialAPI.Apply(wheel_material_shade.GetPrim())
wheel_material.CreateStaticFrictionAttr(0.6)
wheel_material.CreateDynamicFrictionAttr(0.5)


def bind_wheel_material(prim):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(wheel_material_shade, materialPurpose="physics")


# --- Collision group: wheels in the row shouldn't collide with each other ---
collision_group_path = root_path + "/WheelCollisionGroup"
collision_group = UsdPhysics.CollisionGroup.Define(stage, collision_group_path)
collision_group.CreateInvertFilteredGroupsAttr(False)
collision_group.GetFilteredGroupsRel().AddTarget(collision_group_path)
colliders_api = Usd.CollectionAPI.Apply(collision_group.GetPrim(), "colliders")
colliders_includes_rel = colliders_api.CreateIncludesRel()


def add_to_collision_group(prim):
    colliders_includes_rel.AddTarget(prim.GetPath())


def create_wheel(side_name, side_y, x_pos, is_end_wheel, deflection_limit, name_suffix, reference_joint_path):
    wheel_name = "%s_wheel_%s" % (side_name, name_suffix)

    if is_end_wheel:
        joint_parent_path = root_path
        wheel_body_path = "%s/%s" % (root_path, wheel_name)
        joint_local_pos0 = Gf.Vec3f(x_pos, side_y, wheel_z_offset)
    else:
        # Vertical suspension carrier: chassis -> prismatic (limited, no spring) -> carrier -> wheel
        carrier_path = "%s/%s_carrier" % (root_path, wheel_name)
        carrier_xform = UsdGeom.Xform.Define(stage, carrier_path)
        carrier_xform.AddTranslateOp().Set(Gf.Vec3f(x_pos, side_y, wheel_z_offset))
        UsdPhysics.RigidBodyAPI.Apply(carrier_xform.GetPrim())
        carrier_mass_api = UsdPhysics.MassAPI.Apply(carrier_xform.GetPrim())
        carrier_mass_api.CreateMassAttr(0.01)

        prismatic_path = "%s_suspension" % carrier_path
        prismatic = UsdPhysics.PrismaticJoint.Define(stage, prismatic_path)
        prismatic.CreateBody0Rel().SetTargets([Sdf.Path(root_path)])
        prismatic.CreateBody1Rel().SetTargets([Sdf.Path(carrier_path)])
        prismatic.CreateAxisAttr("Z")
        prismatic.CreateLocalPos0Attr(Gf.Vec3f(x_pos, side_y, wheel_z_offset))
        prismatic.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        # PrismaticJoint's OWN lowerLimit/upperLimit, not the multi-apply
        # UsdPhysics.LimitAPI. This previously used LimitAPI with a "transZ"
        # instance name, which is the generic/D6-joint mechanism -- PhysX does not
        # read it on a PrismaticJoint, so the travel limit was authored but never
        # enforced and the joint was free to slide without bound. It went unnoticed
        # until the wheels finally had enough energy to run all the way out, which
        # looked like the wheels flying off the robot (confirmed live 2026-08-03).
        # Units are linear (metres), matching deflection_limit.
        prismatic.CreateLowerLimitAttr(-deflection_limit)
        prismatic.CreateUpperLimitAttr(deflection_limit)

        joint_parent_path = carrier_path
        wheel_body_path = "%s/%s" % (carrier_path, wheel_name)
        joint_local_pos0 = Gf.Vec3f(0, 0, 0)

    wheel_geom = UsdGeom.Cylinder.Define(stage, wheel_body_path)
    wheel_geom.CreateRadiusAttr(wheel_radius)
    wheel_geom.CreateHeightAttr(wheel_width)
    wheel_geom.CreateAxisAttr("Y")
    # A joint's localPos only defines its anchor point -- it does NOT relocate a body
    # that has no transform of its own. End wheels are direct children of the chassis
    # (unlike filler wheels, which inherit correct position from their carrier), so
    # they need their own explicit translate or they all default to the chassis origin.
    if is_end_wheel:
        wheel_geom.AddTranslateOp().Set(Gf.Vec3f(x_pos, side_y, wheel_z_offset))
    UsdPhysics.RigidBodyAPI.Apply(wheel_geom.GetPrim())
    # Max-velocity attrs live on the PhysX-specific extension schema, not base
    # UsdPhysics.RigidBodyAPI (confirmed live 2026-08-03 via AttributeError).
    wheel_physx_rigid_body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(wheel_geom.GetPrim())
    # *** maxAngularVelocity is in DEGREES per second, not radians. ***
    # This attribute is the single biggest gotcha in this file. USD expresses all
    # angular quantities in degrees, and this attr follows that convention even
    # though the underlying PhysX SDK uses radians. The giveaway is PhysX's own
    # default value, 5729.578 -- that is exactly 100 rad/s written in degrees.
    #
    # An earlier attempt here passed 1000.0 believing it was rad/s. It is 1000
    # deg/s = 17.45 rad/s, which at this wheel radius is a hard 0.65 m/s speed
    # ceiling -- and it was measured live at 17.36 rad/s in a torque sweep, with
    # the wheels rolling essentially slip-free while applied torque was swept
    # across four orders of magnitude (0.01 -> 100 N*m) with no effect above
    # ~0.1 N*m. A velocity clamp accidentally introduced while trying to REMOVE a
    # suspected velocity clamp. Always convert explicitly here.
    wheel_physx_rigid_body_api.CreateMaxAngularVelocityAttr(math.degrees(1000.0))
    # Linear units are plain m/s -- no conversion, unlike the angular attr above.
    wheel_physx_rigid_body_api.CreateMaxLinearVelocityAttr(100.0)
    # maxDepenetrationVelocity (m/s) defaulted to 3.0; raised so a wheel that ends
    # up slightly overlapping the floor isn't also speed-limited while PhysX
    # resolves it.
    wheel_physx_rigid_body_api.CreateMaxDepenetrationVelocityAttr(100.0)
    UsdPhysics.CollisionAPI.Apply(wheel_geom.GetPrim())
    bind_wheel_material(wheel_geom.GetPrim())
    wheel_mass_api = UsdPhysics.MassAPI.Apply(wheel_geom.GetPrim())
    wheel_mass_api.CreateMassAttr(0.02)
    # Explicit rotational inertia, NOT the geometric value PhysX would derive.
    #
    # A bare 0.02kg cylinder has spin inertia 0.5*m*r^2 = 1.4e-5 kg*m^2 -- about
    # 130x less than the vehicle's inertia reflected to the wheel (m*r^2 =
    # 1.85e-3). Against a 1.315kg chassis that is a 65:1 mass ratio, across 5
    # rigidly gear-coupled wheels per side, 3 of them also on prismatic
    # suspension joints. PhysX's iterative maximal-coordinate solver does not
    # converge on near-massless bodies chained to a heavy one: observed live
    # 2026-08-03, wheels reached +-60 rad/s with only 0.0003 N*m applied, which
    # is the solver injecting energy, not anything the control loop did.
    #
    # Physically the right correction rather than a fudge: these wheels are
    # driven through a gearbox, so they carry the motor rotor's REFLECTED
    # inertia, I_rotor * N^2, which for a small heavily-geared robot typically
    # dominates the wheel's own inertia. That adds rotational inertia without
    # adding translational mass, which is exactly what the solver needs.
    # SPIN_INERTIA is the tunable knob here -- at 2e-4 across 10 wheels the
    # drivetrain contributes sum(I)/r^2 ~= 1.4kg of apparent mass, comparable to
    # the robot's own 1.315kg, which is a reasonable regime for a geared tracked
    # vehicle. Raise it if the solver still misbehaves, lower it if the robot
    # feels sluggish to spin up. Not measured against the real robot yet.
    SPIN_INERTIA = 2.0e-4
    # Set ISOTROPICALLY, and that is deliberate. A rigid body's principal moments
    # must satisfy the triangle inequality I1 + I2 >= I3 in every permutation. A
    # first attempt here kept the geometric transverse term (8.16e-6) alongside a
    # raised spin term (2e-4), so transverse + transverse = 1.63e-5 was far less
    # than the spin term -- a physically impossible body. PhysX accepted it without
    # complaint and then behaved wildly: wheels visibly detaching from the chassis
    # on Play, confirmed live 2026-08-03. Reflected gearbox inertia genuinely only
    # acts about the spin axis, so raising all three is an approximation, but it is
    # a VALID one, and transverse inertia has little influence on a wheel that is
    # constrained to a single rotational DOF anyway.
    wheel_mass_api.CreateDiagonalInertiaAttr(
        Gf.Vec3f(SPIN_INERTIA, SPIN_INERTIA, SPIN_INERTIA))
    # More solver iterations for the same reason -- the gear-coupled chain is a
    # harder constraint problem than PhysX's low defaults assume.
    wheel_physx_rigid_body_api.CreateSolverPositionIterationCountAttr(32)
    wheel_physx_rigid_body_api.CreateSolverVelocityIterationCountAttr(8)
    add_to_collision_group(wheel_geom.GetPrim())

    # Purely visual lug so spin is actually visible -- a plain grey cylinder looks
    # identical at any rotation angle, so "is it spinning" was never answerable by
    # eye, only by telemetry. Child of the wheel body (no physics APIs of its own,
    # no collision), so it just rides along rigidly with whatever the wheel does.
    spin_marker = UsdGeom.Cube.Define(stage, wheel_body_path + "/SpinMarker")
    spin_marker.CreateSizeAttr(1.0)
    # Radial offset must clear the wheel's own radius, not just be "close to the
    # rim" -- 0.8*radius put it fully inside the solid wheel body, invisible from
    # any angle since the opaque wheel occluded it completely. Needs to protrude
    # past the surface to actually be seen.
    spin_marker_translate_op = spin_marker.AddTranslateOp()
    spin_marker_translate_op.Set(Gf.Vec3f(wheel_radius + 0.003, 0, 0))
    spin_marker_scale_op = spin_marker.AddScaleOp()
    spin_marker_scale_op.Set(Gf.Vec3f(0.004, wheel_width * 0.9, 0.004))
    spin_marker.SetXformOpOrder([spin_marker_translate_op, spin_marker_scale_op])
    spin_marker.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.85, 0.0)])

    revolute_path = "%s_axle" % wheel_body_path
    revolute = UsdPhysics.RevoluteJoint.Define(stage, revolute_path)
    revolute.CreateBody0Rel().SetTargets([Sdf.Path(joint_parent_path)])
    revolute.CreateBody1Rel().SetTargets([Sdf.Path(wheel_body_path)])
    revolute.CreateAxisAttr("Y")
    revolute.CreateLocalPos0Attr(joint_local_pos0)
    revolute.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))

    if reference_joint_path is None:
        # The one actively-driven wheel per side -- the other 4 are gear-locked to
        # it below, not independently driven. Matches a real track: one sprocket is
        # powered, the belt (which these gear joints approximate) rigidly carries
        # that motion to every other contact point, they're not just coincidentally
        # commanded to agree.
        # Damping/stiffness are zero, not a real holding drive: driving now happens
        # entirely via motor_model.py's open-loop torque (below), not a PhysX-level
        # velocity drive. This DriveAPI is just a required attachment point, not an
        # active controller -- earlier versions used a strong damping placeholder
        # here and then had to zero it out at runtime before applying torque; simpler
        # to just never author a fighting drive in the first place.
        drive_api = UsdPhysics.DriveAPI.Apply(revolute.GetPrim(), "angular")
        drive_api.CreateTypeAttr("velocity")
        drive_api.CreateTargetVelocityAttr(0.0)
        drive_api.CreateDampingAttr(0.0)
        drive_api.CreateStiffnessAttr(0.0)
    else:
        # PhysxPhysicsGearJoint, not NewtonMimicAPI: PhysX is Isaac Sim's default
        # backend (Newton is opt-in, not active here), and only PhysxPhysicsGearJoint
        # is guaranteed to actually be read by it. Its docs require
        # excludeFromArticulation=true when linking two joints that are themselves
        # part of an articulation (ours are, via the chassis's ArticulationRootAPI).
        # PhysX's own docs: "the two bodies of the gear joint [must] rotate only
        # around the twist axis" -- the chassis doesn't rotate around the wheel axis
        # at all (it's the stationary anchor), so body0 must be the actual reference
        # WHEEL (hinge0's child), not the chassis (hinge0's parent) -- using the
        # chassis passed PhysX's config-validity check but didn't actually constrain
        # anything, which is exactly the symptom just observed (no error, but the
        # follower wheels weren't angle-locked in practice).
        reference_wheel_body_path = reference_joint_path[: -len("_axle")]
        gear_path = "%s_gear" % revolute_path
        gear_joint = PhysxSchema.PhysxPhysicsGearJoint.Define(stage, gear_path)
        gear_joint.CreateBody0Rel().SetTargets([Sdf.Path(reference_wheel_body_path)])
        gear_joint.CreateBody1Rel().SetTargets([Sdf.Path(wheel_body_path)])
        gear_joint.CreateHinge0Rel().SetTargets([Sdf.Path(reference_joint_path)])
        gear_joint.CreateHinge1Rel().SetTargets([Sdf.Path(revolute_path)])
        # Negative, not 1.0: PhysX's own docs note gear-linked joints "may
        # otherwise have opposite signs, depending on the orientations of the
        # joint frames" -- confirmed empirically in an isolated 2-wheel test that
        # our joint frames need the sign flipped to actually spin the same way.
        gear_joint.CreateGearRatioAttr(-1.0)
        gear_joint.CreateExcludeFromArticulationAttr(True)

    return revolute_path


filler_offsets_and_names = [
    (-half_wheelbase * 0.5, "filler_rear"),
    (0.0, "filler_mid"),
    (half_wheelbase * 0.5, "filler_front"),
]

for side_name, side_y in [("left", track_y), ("right", -track_y)]:
    # Front end wheel is the one reference/driven wheel per side (reference_joint_path
    # =None); everything else on this side gets gear-locked to its joint.
    reference_joint_path = create_wheel(side_name, side_y, half_wheelbase, True, 0.0, "front", None)
    create_wheel(side_name, side_y, -half_wheelbase, True, 0.0, "rear", reference_joint_path)
    for offset, filler_name in filler_offsets_and_names:
        ratio = offset / half_wheelbase
        deflection_limit = max_center_deflection * (1 - ratio ** 2)
        create_wheel(side_name, side_y, offset, False, deflection_limit, filler_name, reference_joint_path)

    # --- Track envelope: what actually lets this climb like a real track ---
    # 5 independent circular wheels cap the climbable step height at the wheel
    # radius (a wheel can only pivot over a step up to h<=r; past that, the
    # required moment arm sqrt(2*r*h - h^2) has no real solution). A real track
    # climbs taller steps than its own wheel radius because the flat belt run
    # between the end wheels acts as a lever reaching out to the wheelbase length.
    # This capsule -- same radius as the wheels, spanning axle to axle, welded to
    # the chassis (no separate RigidBodyAPI -- it's just another collider on the
    # chassis's existing rigid body) -- approximates that outer belt envelope.
    #
    # Disabled by default for now: with real wheel-spin telemetry fixed (it was
    # badly wrong before -- see _wheel_spin_rate), the actual measured problem
    # turned out to be zero wheel rotation at all against even a short, gentle
    # ramp, with real torque and real friction both present. This capsule is a
    # non-rotating, chassis-welded, high-friction shape -- if it's also touching
    # the obstacle, it would act like a frozen skid no wheel torque could ever
    # overcome, which would fully explain that symptom on its own. Isolating it
    # out to test the plain wheels alone before layering anything else on top.
    if ENABLE_TRACK_ENVELOPE:
        track_envelope_path = "%s/%s_track_envelope" % (root_path, side_name)
        track_envelope = UsdGeom.Capsule.Define(stage, track_envelope_path)
        track_envelope.CreateRadiusAttr(wheel_radius - 0.003)
        track_envelope.CreateHeightAttr(wheelbase)
        track_envelope.CreateAxisAttr("X")
        track_envelope.AddTranslateOp().Set(Gf.Vec3f(0, side_y, wheel_z_offset))
        # Collision-only -- it's meant to approximate the belt's outer envelope for
        # physics, not be seen. Left visible by default, it rendered as one big
        # rounded blob engulfing all 5 wheels on each side instead of the actual
        # cylinders.
        track_envelope.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
        UsdPhysics.CollisionAPI.Apply(track_envelope.GetPrim())
        bind_wheel_material(track_envelope.GetPrim())
        add_to_collision_group(track_envelope.GetPrim())

print("White-crash chassis + 10 wheels + 2 track envelopes created at /World/WhiteCrash")

# --- Sensor mounts ---
# Visual reference markers only (no functional sensing yet -- that's later, when the
# actual training environment does real raycasting/IMU simulation). Xform children of
# the chassis, not physics bodies -- they just need to move rigidly with it.

# TOF (lidar): parallel, forward-facing, laterally offset (measured 2026-08-01).
tof_x = chassis_length / 2.0                 # front face, placeholder chassis length
tof_z = chassis_height / 2.0 - 0.009         # 9mm below the top of the chassis, measured
tof_offsets_and_names = [
    (0.0, "tof_center"),
    (0.0525, "tof_left"),
    (-0.0525, "tof_right"),
]
tof_color = [Gf.Vec3f(0.9, 0.2, 0.1)]  # red-orange, distinct from the grey chassis

# IMU: exact board position not measured; centered as a reasonable placeholder.
imu_position = (0.0, 0.0, chassis_height / 2.0)
imu_color = [Gf.Vec3f(0.1, 0.7, 0.2)]  # green, distinct from the TOF cones


def add_sensor_marker(path, position, shape, color):
    if shape == "cone":
        marker = UsdGeom.Cone.Define(stage, path)
        marker.CreateRadiusAttr(0.012)
        marker.CreateHeightAttr(0.035)
        # axis="X" wasn't actually taking effect (still extending vertically) --
        # working from Cone's real documented default instead: axis=Z, apex at +Z,
        # base at -Z. Ry(-90) maps +Z to -X: apex ends up pointing back toward the
        # chassis (mount point), base/flare ends up pointing +X (forward, the
        # sensing direction) -- tip-at-robot, flare-shows-spread, as intended.
        marker.CreateAxisAttr("Z")
        rotate_op = marker.AddRotateYOp()
        rotate_op.Set(-90.0)
        translate_op = marker.AddTranslateOp()
        translate_op.Set(Gf.Vec3f(*position))
        # xformOpOrder reads like a matrix expression left-to-right: the FIRST-listed
        # op is applied LAST (outermost/world-space), the LAST-listed op is applied
        # FIRST (innermost/local-space) -- confirmed by direct inspection, this was
        # backward before. [translate, rotate] means rotate happens first (spins the
        # cone in local space) and translate happens last (moves the already-rotated
        # cone to its mount point) -- not the other way around.
        marker.SetXformOpOrder([translate_op, rotate_op])
    else:
        marker = UsdGeom.Sphere.Define(stage, path)
        marker.CreateRadiusAttr(0.014)
        marker.AddTranslateOp().Set(Gf.Vec3f(*position))
    marker.CreateDisplayColorAttr(color)
    return marker.GetPrim()


for y_offset, name in tof_offsets_and_names:
    tof_path = "%s/%s" % (root_path, name)
    add_sensor_marker(tof_path, (tof_x, y_offset, tof_z), "cone", tof_color)

imu_path = "%s/imu" % root_path
add_sensor_marker(imu_path, imu_position, "sphere", imu_color)

print("Sensor mounts added: 3x TOF (center, left, right), 1x IMU")

# --- Chase camera ---
# Rigidly mounted child prim (not scripted per-frame) so it follows the chassis for
# free. Camera rotation: USD cameras always look down local -Z with local +Y as "up",
# regardless of stage up-axis; this stage is Z-up, so a horizontal, forward-facing,
# right-side-up camera needs RotateX(90) first (realigns local Y-up to world Z-up,
# incidentally swinging local -Z-forward to world +Y) then RotateZ(-90) (yaws that to
# world +X, this robot's forward direction per the wheel/TOF layout). Downward tilt is
# folded into the X angle as 90-pitch_deg rather than a separate op, since a bigger X
# angle here tilts the view up, not down.
camera_path = root_path + "/ChaseCam"
camera = UsdGeom.Camera.Define(stage, camera_path)
chase_cam_offset = Gf.Vec3f(-1.3, 0.0, 0.8)  # ~1.5m out, confirmed by hand 2026-08-02
chase_cam_pitch_down_deg = 20.0

chase_cam_translate_op = camera.AddTranslateOp()
chase_cam_translate_op.Set(chase_cam_offset)
chase_cam_rotate_z_op = camera.AddRotateZOp()
chase_cam_rotate_z_op.Set(-90.0)
chase_cam_rotate_x_op = camera.AddRotateXOp()
chase_cam_rotate_x_op.Set(90.0 - chase_cam_pitch_down_deg)
camera.SetXformOpOrder([chase_cam_translate_op, chase_cam_rotate_z_op, chase_cam_rotate_x_op])
camera.CreateFocalLengthAttr(18.0)  # wide-ish, so the small nearby robot isn't cropped

print("Chase camera created at %s" % camera_path)

vp_utils.get_active_viewport().camera_path = camera_path
print("Active viewport camera set to ChaseCam. Switch back via the viewport's camera dropdown (pick 'Perspective') if needed.")

# --- Joystick teleop ---
# 8BitDo Pro 3 -> /joy (joy_node in the lyrical container) -> per-side
# throttle/drag_brake -> motor_model.py (inlined) -> wheel torque, applied every
# physics step. Axis mapping confirmed empirically 2026-08-02 against this specific
# controller + joy_node, not from any spec:
#   axes[1] = left stick Y, forward (away from body) = +1.0 -- already matches
#             throttle_percent's own forward-positive convention, no sign flip needed.
#   axes[3] = right stick Y, same convention.
#   axes[4] = R2, rest = +1.0 (not pressed), full pull = -1.0.
#   axes[5] = L2, same convention as R2.
#
# Simplification: reads angular velocity in the WORLD frame and assumes it stays
# aligned with the wheel's local spin axis (true while the robot's flat/upright, not
# exact if it tips or turns hard) -- fine for interactive driving, not a permanent
# design.

# --- motor_model.py inlined ---
VOLTAGE_C0 = 0.412
VOLTAGE_C1 = 2.600
VOLTAGE_C2 = 0.850
COAST_C0 = 0.951
COAST_C1 = 0.794
BRAKE_C0 = 1.008
BRAKE_C1 = 2.950
CHARACTERIZATION_MASS_KG = 1.315
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


def motor_model(throttle_percent, drag_brake_percent, omega_current, v_bat, radius):
    v = omega_current * radius
    a_throttle = _accel_from_throttle(throttle_percent, v, v_bat)
    a_brake = _accel_from_drag_brake(v)
    a = (1 - drag_brake_percent) * a_throttle + drag_brake_percent * a_brake
    force = CHARACTERIZATION_MASS_KG * a
    torque = force * radius
    return 20.0 * torque


left_wheel = RigidPrim(root_path + "/left_wheel_front")
right_wheel = RigidPrim(root_path + "/right_wheel_front")
chassis_rigid = RigidPrim(root_path)

V_BAT = 12.6  # full charge on a 3S LiPo, testing whether a weak battery was the issue

# Scales motor_model's real (measured) torque output -- NOT a real motor property,
# purely a test knob. Back to 1.0 (the actual characterized motor): 5x made zero
# difference to the table-foot stall, proving it wasn't a torque problem at all --
# see the track_envelope comment below for the actual (geometric) cause and fix.
TORQUE_MULTIPLIER = 1.0

# (Previous run's node/subscription already torn down at the top of the script,
# before prim rebuilding -- see there for why.)
if not rclpy.ok():
    rclpy.init()

_white_crash_joy_node = rclpy.create_node("isaac_joy_teleop")
_white_crash_joy_axes = [0.0] * 8


def _on_joy(msg):
    global _white_crash_joy_axes
    _white_crash_joy_axes = msg.axes


_white_crash_joy_sub = _white_crash_joy_node.create_subscription(Joy, "/joy", _on_joy, 10)


def _drag_brake_from_trigger(axis_value):
    # rest = +1.0 -> 0.0 brake, full pull = -1.0 -> 1.0 brake
    return max(0.0, min(1.0, (1.0 - axis_value) / 2.0))


# Per-wheel (name -> (timestamp, Gf.Quatd)) history for _wheel_spin_rate's finite
# difference. Keyed by name string, not id(wheel), since a fresh RigidPrim object
# could in principle compare unequal/hash differently across script re-runs even
# for "the same" prim path -- name is simple and unambiguous here (only ever two
# wheels call this).
_wheel_spin_history = {}


def _wheel_spin_rate(name, wheel):
    # Previous approach projected wheel.get_velocities()'s angular velocity onto
    # the wheel's local spin axis -- confirmed live 2026-08-03 this was flatly
    # wrong: it reported ~0.7-1.2 rad/s continuously while the wheel's own
    # orientation quaternion was DEMONSTRABLY frozen (5th-decimal drift only)
    # across many consecutive 0.3s-apart prints, meaning it wasn't real spin --
    # likely a frame/convention mismatch in what that API's angular velocity
    # actually represents (world vs. local frame isn't documented, was guessed).
    # Sidesteps that entirely: compute spin rate from the CHANGE in orientation
    # between two points in time instead of trusting a velocity vector's meaning.
    # Orientation itself is validated sane (chassis quat sits at ~identity, w~1,
    # for a level robot -- confirmed correct component order).
    now = time.time()
    _, orientation = wheel.get_world_poses()
    w, x, y, z = (float(c) for c in orientation.numpy()[0])
    curr_quat = Gf.Quatd(w, Gf.Vec3d(x, y, z))

    prev = _wheel_spin_history.get(name)
    _wheel_spin_history[name] = (now, curr_quat)
    if prev is None:
        return 0.0
    prev_time, prev_quat = prev
    dt = now - prev_time
    if dt <= 0.0:
        return 0.0

    relative_rotation = Gf.Rotation(curr_quat * prev_quat.GetInverse())
    angle_rad = math.radians(relative_rotation.GetAngle())
    axis = relative_rotation.GetAxis()

    # GetAngle()/GetAxis() only give an unsigned angle and an arbitrarily-oriented
    # axis -- need the SIGNED rate around the wheel's actual current spin axis, so
    # project the rotation axis onto the wheel's live local-Y-in-world direction.
    world_y_axis = Gf.Vec3d(
        2 * (x * y - w * z),
        1 - 2 * (x * x + z * z),
        2 * (y * z + w * x),
    ).GetNormalized()
    signed_angle_rad = angle_rad if Gf.Dot(axis, world_y_axis) >= 0 else -angle_rad
    return signed_angle_rad / dt


_last_debug_print = [0.0]
_last_update_time = [0.0]

# Wheel rotational inertia about its spin axis. Must match the DiagonalInertia
# authored on the wheel prims (SPIN_INERTIA in create_wheel) -- this bounds the
# resistive torque below, and if the two disagree the bound is simply wrong.
WHEEL_INERTIA = 2.0e-4


def _limit_resistive_torque(torque, omega, dt, throttle):
    """Stop a COASTING/BRAKING torque from reversing the wheel within a step.

    motor_model sizes its output to accelerate the whole 1.315 kg VEHICLE
    (force = CHARACTERIZATION_MASS_KG * a), but that torque is applied to a single
    wheel with far less inertia. For a DRIVE torque that's fine and even correct:
    exceeding available traction is just wheelspin, which is real. For a resistive
    torque it is not -- coast resistance alone (~0.047 N*m) can flip the wheel's
    direction outright in one 16ms step rather than slowing it, so the sign
    inverts and it oscillates with growing amplitude. Observed live 2026-08-03 as
    wheels spinning violently with no joystick input at all.

    A resistive force can bring a body to rest but never push it backwards, hence
    the |tau| <= I*|omega|/dt bound.

    Crucially, "resistive" means COAST/BRAKE -- it is NOT the same as "opposes
    current motion". Applying forward throttle while still rolling backwards also
    opposes motion, but that is drive torque and must pass through untouched. A
    first version tested only `torque * omega < 0` and so throttled exactly that
    case, producing a trap: forward torque decelerated the wheel toward zero while
    the bound I*|omega|/dt shrank alongside omega, until at omega ~= -0.001 rad/s
    the ceiling was ~1e-5 N*m and forward input did nothing at all. Confirmed live
    2026-08-03 as "once it starts going backward it ignores forward input".
    So: classify by the driver's INTENT (agreement with the throttle command),
    not by sign against velocity.
    """
    if dt <= 0.0 or omega == 0.0:
        return torque
    driving = throttle != 0.0 and (torque > 0.0) == (throttle > 0.0)
    if driving or torque * omega >= 0:
        return torque
    max_resistive = WHEEL_INERTIA * abs(omega) / dt
    return math.copysign(min(abs(torque), max_resistive), torque)


def _on_update(e):
    # The update-event stream fires every frame regardless of timeline play state,
    # but RigidPrim's physics tensor access is only valid once physics has actually
    # initialized (Play pressed at least once) -- confirmed live 2026-08-03,
    # apply_forces_and_torques_at_pos threw "physics tensor entity is not valid"
    # spam while just sitting on the stage pre-Play. Cheap to skip entirely rather
    # than fix a state that isn't actually ready to be touched yet.
    if not omni.timeline.get_timeline_interface().is_playing():
        return
    rclpy.spin_once(_white_crash_joy_node, timeout_sec=0)
    axes = _white_crash_joy_axes
    left_throttle = axes[1]
    right_throttle = axes[3]
    left_drag_brake = _drag_brake_from_trigger(axes[5])
    right_drag_brake = _drag_brake_from_trigger(axes[4])

    do_print = False
    now = time.time()
    if now - _last_debug_print[0] > 0.3:
        _last_debug_print[0] = now
        do_print = True

    dt = now - _last_update_time[0] if _last_update_time[0] else 0.0
    _last_update_time[0] = now

    for name, wheel, throttle, drag_brake in (
        ("L", left_wheel, left_throttle, left_drag_brake),
        ("R", right_wheel, right_throttle, right_drag_brake),
    ):
        omega = _wheel_spin_rate(name, wheel)
        v = omega * wheel_radius
        raw_torque = motor_model(throttle, drag_brake, omega, V_BAT, wheel_radius) * TORQUE_MULTIPLIER
        torque = _limit_resistive_torque(raw_torque, omega, dt, throttle)
        wheel.apply_forces_and_torques_at_pos(torques=[[0.0, torque, 0.0]], local_frame=True)
        if do_print:
            print("%s throttle=%.2f brake=%.2f v=%.3f m/s omega=%.2f rad/s "
                  "torque=%.4f N*m (raw %.4f)" % (
                      name, throttle, drag_brake, v, omega, torque, raw_torque))
    if do_print:
        chassis_lin_vel, chassis_ang_vel = chassis_rigid.get_velocities()
        chassis_positions, chassis_orientations = chassis_rigid.get_world_poses()
        print("chassis pos=%s orientation(quat)=%s lin_vel=%s ang_vel=%s" % (
            chassis_positions.numpy()[0], chassis_orientations.numpy()[0],
            chassis_lin_vel.numpy()[0], chassis_ang_vel.numpy()[0]))


_white_crash_joy_teleop_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(_on_update)
print("Joystick teleop running -- left/right stick = throttle, L2/R2 = drag brake. Press Play.")
