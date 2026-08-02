"""Forward physics model of white-crash's motor + DRV8833 H-bridge, for simulation.

The firmware's control code goes (desired v, a) -> actuator command. This goes the
opposite direction -- actuator command -> resulting torque -- since that's what the
simulated wheel joint actually needs each step. Built from the same characterized
relationships the firmware itself uses (see training/PLAN.md's "Firmware audit
findings" and github.com/berickson/white-crash), not re-derived here.

Treated as one lumped whole-drivetrain black box (motor + gearbox + wheel), matching
how the real robot was characterized: voltage in, resulting vehicle
speed/acceleration out. No separate motor-shaft/gear-ratio model, since we have no
independently-measured data for those in isolation -- inventing a gear ratio to
decompose this further would add an assumption, not remove one.
"""

# Voltage feedforward model: V_motor = c0 + c1*v + c2*a
# (fit from real driving trials -- firmware uses this to convert a desired v/a into
# the motor voltage needed; here we invert it: known voltage + v -> resulting a)
VOLTAGE_C0 = 0.412
VOLTAGE_C1 = 2.600
VOLTAGE_C2 = 0.850

# Coast deceleration magnitude: decel = c0 + c1*|v|  (throttle_percent=0, drag_brake_percent=0)
COAST_C0 = 0.951
COAST_C1 = 0.794

# Pure-brake deceleration magnitude: decel = c0 + c1*|v|  (drag_brake_percent=1.0)
BRAKE_C0 = 1.008
BRAKE_C1 = 2.950

# Below this speed the wheel is treated as "at rest" for deadband purposes -- not a
# measured constant, just a small epsilon.
DEADBAND_V = 0.01  # m/s

# Real robot mass, measured directly (training/PLAN.md). Used to convert the
# characterized model's acceleration into a real force via F=ma -- this is just
# Newton's second law applied to the empirically-measured "a", not an attempt to
# decompose VOLTAGE_C2 into its constituent physical factors. Using the real
# (fixed) characterization mass here, not the sim's current mass, keeps the motor
# model itself mass-independent -- correct behavior under domain-randomized mass
# is then the physics engine's job, not this function's.
CHARACTERIZATION_MASS_KG = 1.315


def _signed_resistive_decel(v, c0, c1):
    """Coast/brake deceleration magnitude, always opposing the current motion
    direction (like kinetic friction) -- not just valid for v >= 0."""
    magnitude = c0 + c1 * abs(v)
    if v > 0:
        return -magnitude
    if v < 0:
        return magnitude
    return 0.0


def _accel_from_throttle(throttle_percent, v, v_bat):
    """Acceleration if the H-bridge spent the whole PWM period driving at
    throttle_percent (no braking blended in)."""
    if throttle_percent == 0:
        # Not driving -- the H-bridge isn't actively pushing the wheel either way
        # (real driver's coast state), NOT the same as asking the voltage
        # feedforward formula to "hold v=0, a=0", which would imply the motor is
        # actively working to resist motion.
        return _signed_resistive_decel(v, COAST_C0, COAST_C1)

    v_motor = throttle_percent * v_bat
    a = (v_motor - VOLTAGE_C0 - VOLTAGE_C1 * v) / VOLTAGE_C2
    # VOLTAGE_C0 is the voltage needed to overcome friction and actually move the
    # wheel, fit from trials where the wheel was already turning (kinetic
    # friction). Near rest, a throttle too weak to reach that threshold implies a
    # negative `a` here -- i.e. running backward -- but a real motor that can't
    # overcome static friction just doesn't move, it doesn't reverse. Only clamp
    # in that narrow near-rest/wrong-direction case: once actually moving, a
    # negative `a` from an under-driven wheel is correct (it's decelerating).
    if abs(v) < DEADBAND_V and a * throttle_percent < 0:
        a = 0.0
    return a


def _accel_from_drag_brake(v):
    """Acceleration if the H-bridge spent the whole PWM period at full brake."""
    return _signed_resistive_decel(v, BRAKE_C0, BRAKE_C1)


def motor_model(throttle_percent, drag_brake_percent, omega_current, v_bat, wheel_radius):
    """One wheel's motor+H-bridge, for one simulation step.

    throttle_percent: [-1, 1] signed PWM rate, forward positive -- same role as
        go()'s `rate` on the real driver.
    drag_brake_percent: [0, 1] proportional brake PWM -- same role as brake()'s
        `intensity`. NOT mutually exclusive with throttle_percent: the two are
        blended (0 = pure throttle, 1 = pure brake), matching a real H-bridge that
        time-shares its PWM period between drive and brake pin-states (the same
        trick as Traxxas-style drag brake, generalized to any throttle level
        rather than just neutral). The real driver doesn't implement that blended
        time-sharing yet -- go()/brake() are still mutually exclusive in firmware.
    omega_current: current wheel angular velocity, rad/s (as reported by the sim's
        wheel joint -- NOT a separately-derived linear velocity).
    v_bat: battery voltage.
    wheel_radius: effective rolling radius, meters.

    Returns: torque to apply to the wheel joint, N*m.
    """
    v = omega_current * wheel_radius

    a_throttle = _accel_from_throttle(throttle_percent, v, v_bat)
    a_brake = _accel_from_drag_brake(v)
    a = (1 - drag_brake_percent) * a_throttle + drag_brake_percent * a_brake

    force = CHARACTERIZATION_MASS_KG * a
    torque = force * wheel_radius
    return torque
