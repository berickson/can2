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

- Wheel encoders — speed feedback
- 3x point lidar rangefinders, forward-facing, **spread out** (fan arrangement — gives
  approach angle to a wall via triangulation, not just distance)
- IMU
- Battery voltage sensor
- GPS (not useful for this — indoor/short-range)

Current hand-tuned PIDs are the baseline to beat, and per the user, "honestly aren't up
to my own standards" — so the bar may not be that high, but the comparison should still
be real (same track, same conditions, measured not eyeballed).

## Key design decisions made so far

- **Geometry stays classical, dynamics get learned.** Convert the 3 raw lidar readings
  to `(perpendicular_distance, approach_angle)` via known trig before it reaches the
  policy, rather than making the network re-derive geometry it can already compute
  exactly. Isolates the learned part to what's actually uncertain (motor response,
  friction, battery sag) — that's also the more honest test of the sim-to-real premise.
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

## Near-term extension: wall → can

Once the wall version works, extend to the real target (a can) rather than treating
wall-approach as the final deliverable. This isn't a drop-in swap — a can breaks the
classical-trig assumption:

- The wall-case trig relies on all 3 lidar beams always hitting the *same plane* (a
  wall is locally infinite), guaranteeing 3 valid, coplanar points to solve a triangle
  from.
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
- Exact lidar mounting angles/spacing (needed for the trig, and for the sim model).
- One combined policy for both tasks, or two separate small ones? Leaning separate
  (racing vs. decelerating are pretty different regimes) but not decided.
- Reward function shape for task B specifically — time-to-stop vs. overshoot vs.
  collision penalty tradeoff isn't designed yet.
- Exact ESP32 deployment path for the trained weights (hand-rolled C inference vs.
  TFLite Micro) — model will be tiny either way, but haven't picked the toolchain.

## Rough phases

1. **Capture real-world baseline** — step-response data for speed control (rise time,
   overshoot, steady-state error) and real approach-and-stop trials (time-to-stop,
   overshoot past target distance) with the *current* PID, on the real robot. This is
   the number RL has to beat, and it also calibrates sim parameters (motor response
   curve, sensor noise/update rate, voltage sag under load) instead of guessing them.
2. **Model the robot** — USD model of white-crash: chassis, skid-steer wheel
   approximation (per main README), mass properties matched to the real robot, motor
   model informed by phase 1, sensor mounts (3 lidars at real angles, IMU, encoders).
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
