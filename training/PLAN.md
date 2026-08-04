# PID vs. RL: speed control + approach-and-stop

## Goal

Two things, in order of importance:
1. **Practice with Isaac Lab** — real vectorized RL training, not just Script Editor demos.
2. Find out whether a trained policy can actually beat white-crash's current hand-tuned
   PID controllers, on hardware, for two specific sub-skills.

## Why these two tasks

They're the first two sub-skills of the real Can-Do contest sequence:

1. Spin in place until the can is found *(not in scope here)*
2. **Race toward it at speed** ← task A: velocity tracking
3. **Stop at a fixed distance from it** ← task B: minimum-time approach-and-stop
4. Lower and close the claw *(not in scope here)*
5. Lift the can *(not in scope here)*

"Wall" stands in for "can" for now — same sensing/stopping problem, simpler geometry to
model first. Spin-search and can-approach are near-term *extensions* once the wall
version works (see below), not fully out of scope — just sequenced after. The claw
manipulation sequence (lower, close, lift) is the one genuinely out-of-scope piece for
this whole exercise.

## Hardware / sensors (real robot, ESP32)

- Wheel encoders — speed feedback. Calibrated `meters_per_odometer_tick = 0.000653`
  (real value from `encoder_calibration_test.py`, not a guess).
- 3x point lidar (TOF) rangefinders — **parallel, not fanned**: all three point
  straight ahead (not converging/angled), just laterally offset — center, and
  ±52.5mm left/right. Each has a ~14° detection cone (beam divergence/field of view,
  not a mounting-angle spread). This means recovering distance+angle to a wall is a
  **line fit** through the 3 `(lateral_offset, range)` points, not triangle-solving
  trig — simpler than originally assumed. (Corrected 2026-08-01; originally assumed
  angled/fanned, which was wrong.)
- IMU (BNO055) — gyro xyz, linear accel xyz, orientation, mag
- Battery voltage sensor — already a live control input in firmware, not just telemetry
- GPS (not useful for this — indoor/short-range)

**Physical dimensions** (measured 2026-08-01; CAD also exists on OnShape, linked in the
firmware repo's README, but not yet pulled from there):
- Track (tread) width 26mm, thickness 7mm to the thickest tread point
- Wheel hub spacing (wheelbase, front-to-back per side) 123mm
- Overall width, outside-to-outside of left/right assemblies: 197mm — validates the
  firmware's `track_width = 0.20` estimate to within 1.5%
- Wheel diameter 59mm; effective rolling radius over the track surface is closer to
  wheel radius + tread thickness ≈ 36.5mm, not the bare 29.5mm wheel radius
- 4 wheels total, 2 per side (front + rear hub), track loops around each side's pair —
  this is the real mechanical basis for the main README's "row of hidden wheels"
  skid-steer approximation
- **Mass**: 1.315 kg (2.9 lb), weighed directly — CAD mass properties weren't trusted
  for this (off-the-shelf components like motors/battery/TOF modules don't have
  accurate densities assigned in the OnShape model).
- **Center of mass**: ~20mm forward of the robot's geometric center (front-back balance
  test). Left-right assumed centerline (not separately measured — reasonable given the
  chassis is symmetric left-right).
- **Still missing**: moments of inertia. No direct measurement planned (would need a
  bifilar pendulum test) — approximating geometrically from mass + the dimensions above
  (simple-box approximation) as a first pass, refine later if sim behavior shows it
  matters.

Current hand-tuned PIDs are the baseline to beat, and per the user, "honestly aren't up
to my own standards" — so the bar may not be that high, but the comparison should still
be real (same track, same conditions, measured not eyeballed).

## Track/wheel modeling (PhysX has no native track primitive)

Per the main README, tracks are approximated as a row of wheels along each side's
footprint. Design settled 2026-08-01:

- **Two real end wheels** (matching the actual 123mm-apart axles, ~36.5mm effective
  rolling radius) — simple driven revolute joint, no suspension, positioned exactly
  like the real hardware.
- **3 filler wheels** between them, approximating the track's continuous bottom-run
  contact patch (a real track contacts the ground along its whole span between the two
  wheels, not just at 2 circular points — filler wheels are a simulation-only fiction
  to approximate that). 3 chosen over 1 specifically because turn-in-place scrub — the
  main README's flagged hard-to-get-right behavior — depends on *distributed* contact
  resistance along the track's length, which a single filler wheel under-represents.
- **All wheels (end + filler) driven at the same commanded rate** — a real track is one
  continuous belt moving at one speed, not independently-rolling idlers, so filler
  wheels should match that, not free-spin.
- **Self-collision disabled among the wheel row** via PhysX collision groups/filtered
  pairs — necessary since they sit close together approximating one continuous
  surface; without this they'd physically shove each other apart.
- **Filler wheels get a vertical joint travel limit, not a spring.** The real track
  isn't springy — it has slack, offers ~zero resistance until the slack is used up,
  then is effectively rigid (measured: ~1cm deflection at the very center when pressed).
  A PhysX prismatic joint with upper/lower position limits (no spring) matches this
  directly, and is simpler to set up than tuning a spring constant. This still resolves
  the over-constraint problem a fully-rigid multi-wheel line contact would cause (see
  below) — what fixes that is having an independent DOF per wheel at all, not the
  specific force law within it.
- **Deflection limit is scaled by position along the span, not flat.** The track is
  anchored (~zero slack) right at the two real wheels and sags most at the midpoint —
  same shape as a chain sagging under slack. Modeled as a parabola, zero at both real
  wheels and 1cm at the exact center:
  `deflection_limit(x) = 10mm × (1 − (2x/L − 1)²)`, `x` = filler wheel position along
  the span, `L` = 123mm. A parabola is a standard small-deflection approximation (a
  catenary would be more exact, not needed at ~8%-of-span deflection).
- **Why any of this matters for a rigid multi-point line contact**: rigidly fixing every
  wheel at the same height creates an over-constrained system — more simultaneous
  contact constraints than needed to determine chassis position, and collinear along
  the track's length, which commonly shows up as solver jitter/unrealistic force
  spikes in PhysX. Giving filler wheels an independent (even if mostly-free, hard-limit)
  DOF removes that redundancy.

### Wheel-locking implementation (resolved 2026-08-01, after a long debugging chase)

Original plan called for all wheels driven independently at the same commanded rate.
User correctly pushed back: for a real track, "when one sticks, they all need to
stick, when one moves fast, they all need to move fast" — independent drives with
matching targets can't reproduce that (no bidirectional coupling; a stuck follower
wouldn't drag the driven wheel down too). What actually works, built at
`training/robot/build_white_crash.py`:

- **One reference wheel per side is driven** (`UsdPhysics.DriveAPI`, velocity type);
  the other 4 per side are gear-locked to it via `PhysxSchema.PhysxPhysicsGearJoint`
  (`hinge0`/`hinge1` relationships to the two revolute joints, `gearRatio`) — a real
  bidirectional constraint, not independent driving.
- **The robot is deliberately NOT a PhysX articulation** — no `ArticulationRootAPI`.
  `PxGearJoint` is a maximal-coordinate constraint; mixing it with reduced-coordinate
  articulation solving is a documented, known-fragile combination (confirmed via an
  NVIDIA forum report of exactly this symptom — "gear joints... forcing users to
  exclude most joints from physxArticulation, which severely compromises physics
  simulation accuracy"). We hit three distinct failure modes trying to make it work
  with an articulation before dropping it entirely.
- **Gear joint `body0`/`body1` must be the actual rotating wheels, not the chassis.**
  PhysX's own docs: "the two bodies of the gear joint [must] rotate only around the
  twist axis" — the chassis is the stationary anchor, it doesn't rotate around the
  wheel axis. Using the chassis passes PhysX's config-validity check (misleadingly)
  but doesn't actually constrain anything.
- **`gearRatio` needs to be `-1.0`, not `1.0`.** PhysX's docs note gear-linked joints
  "may otherwise have opposite signs, depending on the orientations of the joint
  frames" — confirmed empirically (isolated 2-wheel test spun opposite directions at
  ratio 1.0, same direction at -1.0). With 4 followers all sign-flipped relative to
  one reference wheel simultaneously, this was very likely *also* the cause of an
  earlier "commanded a real velocity, almost nothing moves" symptom, not a separate
  bug — over-constraining the reference wheel against 4 opposing sign-flipped
  constraints plus real ground friction.
- Considered but rejected: `NewtonMimicAPI` (the officially-recommended in-articulation
  approach) — requires Newton as the active physics backend, which is a separate app
  (`isaacsim.exp.full.newton.kit`) from what we're actually running
  (`isaacsim.exp.full.streaming.kit`); PhysX is our confirmed active backend. Full
  `PhysxVehicleAPI` + `PhysxVehicleTankDifferentialAPI` (the "real" tank-specific PhysX
  vehicle system) — does exactly what we want natively, but requires the whole PhysX
  Vehicle framework (per-wheel suspension/tire APIs) plus a mandatory
  `PhysxVehicleDriveStandardAPI` (engine/gears/clutch simulation) that white-crash has
  no real analog for — bigger rework than was justified once the plain-gear-joint
  route (without articulation) turned out to actually work.

### Drivetrain debugging (2026-08-03) — six real bugs, table foot still open

Started from "the real robot climbs a short table-foot ramp easily, the sim robot
can't, even at absurd torque multipliers." That specific question is **still
unresolved**, but chasing it uncovered six genuine bugs, several of which were
actively producing the misleading evidence the investigation was reasoning from.
Script under test: `training/robot/build_white_crash.py`, mirrored to
`scenes/build_white_crash.py` — `training/` isn't mounted into the container, so
the `scenes/` copy is what Isaac Sim actually runs. **Keep both in sync by hand;
`cp` after every edit.**

**Read this before re-opening the climbing question:** most measurements taken
during this session were contaminated by one or more of the bugs below, so
conclusions like "no amount of torque moves it" and "this is a hard kinematic
lock" were measuring artifacts, not the robot. Re-measure before trusting any of
it.

Bugs found and fixed, roughly in the order they were masking each other:

1. **`maxAngularVelocity` is in DEGREES per second, not radians.** The single
   highest-impact find. An earlier fix in this same session set it to `1000.0`
   intending 1000 rad/s; that is 1000 deg/s = 17.45 rad/s, which at this wheel
   radius is a hard 0.65 m/s speed ceiling. Measured live at 17.36 rad/s in a
   torque sweep, with wheels rolling slip-free while applied torque was swept
   0.01 → 100 N·m with no effect above ~0.1 N·m. The giveaway for the units is
   PhysX's own default, 5729.578, which is exactly 100 rad/s written in degrees.
   This one clamp invalidated the `V_BAT`-to-1000V test, every
   `TORQUE_MULTIPLIER` sweep, and the `throttle * 100.0` experiment — all of them
   were measuring the clamp.
2. **Wheel spin measurement was wrong twice, in different ways.** First a naive
   world-frame-Y read, only correct at zero chassis yaw, under-reporting a real
   ~0.5 rad/s spin as 0.02. Replaced by `_wheel_spin_rate()`, which differences
   the wheel's own orientation quaternion and projects onto its live spin axis.
   Then, in the sweep diagnostic, aliasing: quaternion differencing resolves at
   most one revolution per sample, so sampling every 0.5 s capped readings at
   2π/0.5 = 12.57 rad/s and produced random sign flips that looked like physical
   oscillation. Sample every frame, not per print.
3. **Invalid inertia tensor.** Raising only the spin term while leaving the
   geometric transverse term gave principal moments violating the triangle
   inequality (I₁ + I₂ ≥ I₃). PhysX accepted it silently and then flung the
   wheels off the chassis. Now set isotropically.
4. **Resistive torque could reverse the wheel in one step**, then oscillate with
   growing amplitude — wheels thrashing violently with no joystick input. Coast
   resistance (~0.047 N·m) against a tiny wheel inertia is ~53 rad/s of change per
   16 ms frame. Bounded by `_limit_resistive_torque` to `I*|ω|/dt`. Note the
   follow-on bug: the first version classified "resistive" as any torque opposing
   motion, which also caught forward throttle applied while rolling backwards,
   creating a trap where the bound shrank with ω until forward input did nothing.
   Classify by agreement with the throttle command (intent), not by sign against
   velocity.
5. **Leftover diagnostic subscriptions survived rebuilds.** The script's cleanup
   was a hardcoded list of known names, so a subscription from a diagnostic script
   written later in the session kept firing against deleted prims *and* kept
   applying 50 N·m to the wheels, which read as a physics explosion in the build
   script. Cleanup now sweeps by TYPE, so no future diagnostic can poison a
   rebuild. Related, also fixed: the teardown must happen BEFORE prim deletion,
   and the viewport must be moved off `ChaseCam` before that camera's parent is
   deleted (otherwise the render surface goes permanently blank/white).
6. **Suspension travel limits were never enforced.** `UsdPhysics.LimitAPI` with a
   `"transZ"` instance is the generic/D6 mechanism; PhysX does not read it on a
   `PrismaticJoint`, which has its own `lowerLimit`/`upperLimit`. The filler
   wheels' 0.75–1 cm travel was authored but unbounded, so those six wheels could
   wander arbitrarily far from the contact patch. Beyond the visual, this means
   load distribution and which wheels were actually touching ground were wrong for
   every test before this fix — plausibly including the original table-foot
   behaviour, since a freely-sliding filler wheel cannot push back against a step.

**Known-good after these fixes:** forward/reverse both work, wheels respond to
joystick, robot drives and turns.

**Still wrong / unfinished:**

- **Over-powered.** Wheels break traction and spin up almost instantly. Partly
  real (full throttle asks ~18.9 N at the contact patch against ~7.7 N of
  available friction, so slip is correct), partly modelling: `motor_model` sizes
  its output to accelerate the whole 1.315 kg vehicle
  (`force = CHARACTERIZATION_MASS_KG * a`) but that torque is applied to a single
  wheel.
- **`SPIN_INERTIA = 2e-4` is a stability guess, not a measurement.** It stands in
  for gearbox-reflected rotor inertia (`I_rotor * N²`), which for a small heavily
  geared robot usually dominates the wheel's own inertia. It materially affects
  acceleration, so it must be measured or calibrated before any sim-to-real
  conclusion rests on it. `WHEEL_INERTIA` in the teleop loop must be kept equal to
  it — they are two constants describing one quantity.
- **Wheel friction 0.6/0.5 is also a placeholder**, and was swung between 0.6 and
  1.0 during the session chasing the (bogus) climbing evidence. Note the real
  tradeoff found along the way: high isotropic friction fights skid-steer turning,
  since a tank turn needs the tracks to slip sideways and PhysX has no
  directional/anisotropic friction here.
- **The table foot itself is untested since the fixes.** Retest from a clean
  restart before theorising. The most useful single experiment is still
  `scenes/diagnose_wheel_force.py` in torque mode *with the robot wedged against
  the foot* rather than on open ground — it bypasses joystick, motor model, and
  gear coupling entirely.

**Diagnostic scripts** (all in `scenes/`, all runnable from Script Editor):
`diagnose_wheel_force.py` (raw force/torque straight to one wheel),
`diagnose_torque_sweep.py` (automated torque sweep with per-frame telemetry),
`diagnose_wheel_damping.py` (dumps live joint/rigid-body attributes),
`diagnose_torque_delivery.py` (read/write ordering A/B),
`diagnose_zombie_subs.py` (finds leftover subscriptions),
`diagnose_gear_joints.py` (gear joint wiring).

**Practical note:** Script Editor `print()` output and tracebacks are mirrored into
the Kit log at
`/home/brian/docker/isaac-sim/logs/Kit/Isaac-Sim Streaming/6.0/kit_<timestamp>.log`
(tagged `[py stdout]` / `[py stderr]`), so diagnostics can be read directly from
the log instead of screenshotted. The file is per-session — a Kit restart starts a
new one, so pick the newest. Note also that File → New Stage does NOT reset the
Script Editor's Python namespace; only a full application restart does.

## Key design decisions made so far

- **Geometry stays classical, dynamics get learned.** Convert the 3 raw lidar readings
  to `(perpendicular_distance, approach_angle)` via a line fit (the 3 beams are
  parallel with known lateral offsets, not fanned — see hardware section) before it
  reaches the policy, rather than making the network re-derive geometry it can already
  compute exactly. Isolates the learned part to what's actually uncertain (motor
  response, friction, battery sag) — that's also the more honest test of the
  sim-to-real premise.
- **Voltage is measured, not inferred.** Feed it directly as an observation. Only
  friction and motor/gear wear need to be inferred from response history — that's the
  actual RMA-style part.
- **Physics-only Isaac Sim, RTX rendering off.** No vision component in either task, so
  no need for photorealistic rendering — raycast-based range sensors + physics
  materials for carpet/floor friction, run headless for training throughput. Still
  genuine Isaac Sim/PhysX, just not the rendering half.
- **Isaac Lab, not hand-rolled training loop.** Vectorized parallel environments are
  necessary for real domain-randomization sweeps (friction, motor gain, voltage sag,
  sensor noise) in reasonable time. This is the "revisit once past scene authoring"
  item from the main README — this project is that.
- Stage complexity: **straight-on stopping first**, angled-approach second. Don't solve
  both the control policy and the angle-handling at once.

## RL policy design

**The policy replaces two specific existing firmware pieces, not the whole control
stack.** White-crash's firmware (github.com/berickson/white-crash) already separates
concerns: `stopping_distance()`/`set_approach_twist()` turns distance-to-target into a
`(v_target, a_target)` trajectory (classical geometry/planning — stays as-is, unchanged
by this project). `AccelController` (a PI in acceleration space) plus
`control_from_velocity_and_accel` (physics-model-based PWM/brake mapping) turns that
trajectory into actual motor commands — **that's the part being replaced.** The policy
consumes the same `(v_target, a_target)` signal the existing PI already gets.

**One policy for both tasks, not two.** Task A (speed step-response) and Task B
(approach-and-stop) both reduce to the same problem — track a velocity/acceleration
profile as well as possible given real motor/battery/friction dynamics. They only
differ in what profile gets fed in (step changes vs. a smooth trapezoidal brake curve
from the same trajectory generator). Resolves the "one policy or two" open question
below in favor of one.

**Observation space** (per control step, ~100Hz to match the real control loop, not
the 10Hz telemetry rate):
- `v_target`, `a_target` — from the existing, unchanged trajectory generator
- `v_left`, `v_right` — per-side encoder speed (not averaged — needed so the policy can
  correct for left/right hardware asymmetry, one of the more plausible wins over PID)
- battery voltage — measured, direct input (per the measured-vs-inferred split above)
- `gyro_z` — yaw rate, independent check on whether the robot's actually turning
- `accel_x` — forward IMU acceleration, independent check on whether it's actually
  accelerating vs. wheels spinning without traction (slip signal)
- recent history (for friction / motor-wear inference — no direct sensor for these,
  the actual RMA-style part). Fixed window of recent tracking error vs. a small
  recurrent cell (GRU/LSTM, still tiny, still ESP32-feasible) — not decided, see open
  questions.

**Action space (revised 2026-08-02)**: two continuous outputs per wheel —
`throttle_percent ∈ [-1, 1]` (signed rate, forward positive/reverse negative — same
role as `go()`'s `rate`) and `drag_brake_percent ∈ [0, 1]` (same role as `brake()`'s
`intensity`). Four outputs total (`left_throttle_percent`, `left_drag_brake_percent`,
`right_throttle_percent`, `right_drag_brake_percent`).

**Not mutually exclusive — the two are blended, matching Traxxas-style drag brake
generalized to any throttle level.** Superseded the earlier "brake takes precedence,
ignore drive" rule (which treated drive/brake as mutually-exclusive pin states you
can't blend within a PWM cycle). They can be blended: the real H-bridge can time-share
its PWM period between the drive pin-state and the brake (both-pins-shorted) pin-state,
the same mechanism `brake(intensity)` already uses to blend coast↔full-brake — just
generalized so the "coast" fraction of the period can instead be actively driving.
Firmware doesn't implement that time-sharing yet (`go()`/`brake()` are still mutually
exclusive there); this is now a real prerequisite for sim-to-real transfer, not just a
training nicety. Implemented in `training/motor_model.py`:
`a = (1 - drag_brake_percent) * accel_from_throttle(throttle_percent, v) +
drag_brake_percent * accel_from_brake(v)`. A small training penalty (shaped as
`k * |throttle_percent| * drag_brake_percent`, zero unless both are actually in use
together) discourages gratuitous overlap without forbidding it outright — continuous
and gradient-friendly rather than a hard branch.

Reverse is included (not just forward-to-coast) because the action *interface* is
expensive to change later even though task complexity is cheap to stage — e.g.
overshoot recovery will need it soon regardless of whether the first training
curriculum exercises it much.

**Windup gotcha from the old design is now moot.** The old "brake takes precedence"
rule meant `drive` got zero gradient whenever `brake > 0` (windup, then a jarring snap
back on brake release) — the reason for the auxiliary anti-windup loss described in an
earlier version of this doc. Under the blended action space, throttle always
contributes via its `(1 - drag_brake_percent)` weight, so it always has a live gradient
path; the auxiliary loss is no longer needed.

## Firmware audit findings (2026-08-01)

Read through `main.cpp` and the existing characterization scripts in
github.com/berickson/white-crash before assuming phase 1 needs to be built from
scratch. Findings:

- **`fast_decay` (the drv8833.h parameter originally suspected as a tuning culprit) is
  dead code** — grepped every `.go()` call site, none pass `fast_decay=true` in current
  control paths, despite a stale header comment. Not the cause of tuning difficulty.
- **The real mode-switching culprit**: `control_from_velocity_and_accel` decides
  between PWM-drive and active `brake()` every 10ms based on whether desired
  deceleration exceeds coast-friction alone — a genuine hybrid/switched system, which
  is why a single fixed-gain PID has been hard to tune well across its whole operating
  range. This is the same class of problem the two-output action space above is
  designed to let the policy handle better.
- **Substantial existing infrastructure, not a from-scratch job**: a PI controller in
  acceleration space (not direct PWM) with anti-windup at 100Hz; characterized physics
  models already fit from real data (coast-decel curve, pure-brake curve, voltage
  feedforward model `V_motor = 0.412 + 2.6·v + 0.85·a`); battery voltage already a live
  control input, not just telemetry; a real closed-form stopping-distance trajectory
  generator already driving both `GoToCanMode` and `WallApproachTestMode`; existing
  test harnesses for both tasks (`PIControlTestMode` + `monitor_pi_control.py` for A,
  `WallApproachTestMode` + `run_wall_approach_test.py` for B).
- **Gaps for phase 1, concentrated in analysis + data-completeness, not new firmware**:
  no script computes rise-time/overshoot/settling-time or time-to-stop as scalar
  numbers yet; IMU and left/right TOF aren't captured together with the wall-approach
  trial (only center-TOF, even though IMU fields are already sitting unused in the same
  message); no mechanism to deliberately vary battery voltage in closed-loop trials
  (`virtual_vbat` exists but only wired into manual driving); telemetry publishes at
  10Hz despite the control loop running at 100Hz, which may undersample fast
  transients.
- Practical implication: phase 1 looks like **extending
  `monitor_pi_control.py` and `run_wall_approach_test.py`**, not writing new harnesses.

## Near-term extension: wall → can

Once the wall version works, extend to the real target (a can) rather than treating
wall-approach as the final deliverable. This isn't a drop-in swap — a can breaks the
classical-trig assumption:

- The wall-case line fit relies on all 3 (parallel, laterally-offset) lidar beams
  always hitting the *same plane* (a wall is locally infinite), guaranteeing 3 valid
  points to fit a line through.
- A can is small enough that beams can miss it entirely and instead return the
  distance to whatever's behind it (far wall, floor, nothing). So before any geometry
  is possible, there's a **data-association problem**: which of the 3 beams, if any,
  are actually on-target right now vs. seeing background?
- This reopens the classical-vs-learned question, but scoped to just this part:
  - **Classical-ish**: threshold each beam against expected background range to decide
    which are "on-target," triangulate with however many valid points exist (0/1/2/3),
    with defined fallback behavior for the under-determined cases.
  - **Raw-to-network**: feed the 3 raw readings (with a sentinel for "no valid return")
    straight to the policy and let it learn to interpret partial/ambiguous hits. Unlike
    the wall case, this is actually well-motivated here — there's genuine inference
    involved, not just reimplementing known trig.
- This also blurs into "spin in place until found" — "which beams are on-target" and
  "is the can findable from here at all" are closely related, not cleanly separable
  the way the phased plan below implies. Worth keeping in mind, not necessarily solving
  up front.

Not designing this further until the wall version is proven — recorded here so the
scope decision (extension, not out-of-scope) isn't lost.

## Open questions (not yet decided)

- Isaac Lab: build the image locally vs. adopt a prebuilt base — main README flagged
  this as generally open too.
- Recent-history representation for the inferred (friction/motor-wear) observations —
  fixed window vs. small recurrent cell.
- Reward function shape for task B specifically — time-to-stop vs. overshoot vs.
  collision penalty tradeoff isn't designed yet.
- Exact ESP32 deployment path for the trained weights (hand-rolled C inference vs.
  TFLite Micro) — model will be tiny either way, but haven't picked the toolchain.
- Deliberate battery-voltage variation for closed-loop trials — physically test across
  charge levels (works today, `battery_voltage` already logged) vs. wiring
  `virtual_vbat` into the closed-loop control path (small firmware change, not done).

## Rough phases

1. **Capture real-world baseline** — step-response data for speed control (rise time,
   overshoot, steady-state error) and real approach-and-stop trials (time-to-stop,
   overshoot past target distance) with the *current* PID, on the real robot. This is
   the number RL has to beat, and it also calibrates sim parameters (motor response
   curve, sensor noise/update rate, voltage sag under load) instead of guessing them.
2. **Model the robot** ✅ — chassis + 2 real end wheels + 3 driven filler wheels per
   side (self-collision disabled via CollisionGroup, filler suspension via joint
   *limits* not springs, per "Track/wheel modeling" above) + 3 TOF/IMU sensor mount
   markers, built procedurally at `training/robot/build_white_crash.py`. Chassis
   dimensions still a placeholder box (mass/CoM are real, inertia is a geometric
   approximation from them). Wheel geometry, sensor offsets, and mount height are all
   from real measurements. Not yet: a real motor/actuator model driving the wheels
   (currently placeholder zero-velocity drives) — that's Isaac Lab task code, phase 3+,
   not part of the static model. See [[reference_usd_physics_gotchas]] for USD
   authoring pitfalls hit building this, worth not re-learning next time.
3. **Set up Isaac Lab** — bring it into the docker-compose stack, define the task
   (Isaac Lab task workflow), domain randomization config split into measured
   (voltage — direct observation) vs. inferred (friction, motor gain — needs history)
   variables per the design decision above.
4. **Build the training scene** — flat floor with randomizable friction material, a
   wall/can-proxy target, raycast rangefinder sensors, encoder + IMU + voltage sensor
   models.
5. **Train** — PPO via Isaac Lab, straight-on case first, compare against phase 1's
   baseline numbers throughout, not just at the end.
6. **Sim-to-sim eval** — stress-test the trained policy across randomized conditions in
   sim before ever touching hardware.
7. **Deploy to ESP32, compare against real PID baseline** — the actual sim-to-real
   test. This is where phase 1's real numbers matter most.

## Non-goals (for this exercise)

- Claw manipulation (lower, close, lift) — genuinely out of scope, no plan to extend to it
- Angled-approach handling (until straight-on is proven)
- Photorealistic rendering
- Can detection / spin-search / can-approach — not non-goals, just sequenced after the
  wall version; see "Near-term extension" above
