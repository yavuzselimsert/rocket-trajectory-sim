import numpy as np
import matplotlib.pyplot as plt


# Physical constants
GRAVITY = 9.81
AIR_DENSITY_SEA_LEVEL = 1.225
SCALE_HEIGHT = 8500.0

# Rocket parameters
DRY_MASS = 20.0
PROPELLANT_MASS = 10.0
THRUST = 800.0
BURN_TIME = 4.0

DRAG_COEFFICIENT = 0.5
CROSS_SECTIONAL_AREA = 0.03


def air_density(altitude):
    return AIR_DENSITY_SEA_LEVEL * np.exp(-altitude / SCALE_HEIGHT)


def drag_force(velocity, altitude):
    rho = air_density(altitude)
    direction = np.sign(velocity)
    return (
        0.5 * rho * velocity**2 * DRAG_COEFFICIENT * CROSS_SECTIONAL_AREA
        * direction
    )


def rocket_mass(t):
    burn_rate = PROPELLANT_MASS / BURN_TIME
    if t < BURN_TIME:
        return DRY_MASS + PROPELLANT_MASS - burn_rate * t
    else:
        return DRY_MASS


def current_thrust(t):
    return THRUST if t < BURN_TIME else 0.0


def derivatives(t, state):
    altitude, velocity = state
    mass = rocket_mass(t)
    thrust = current_thrust(t)
    weight = mass * GRAVITY
    drag = drag_force(velocity, altitude)
    net_force = thrust - weight - drag
    acceleration = net_force / mass
    return np.array([velocity, acceleration])


def euler_step(t, state, dt):
    return state + dt * derivatives(t, state)


def rk4_step(t, state, dt):
    k1 = derivatives(t, state)
    k2 = derivatives(t + dt / 2, state + dt / 2 * k1)
    k3 = derivatives(t + dt / 2, state + dt / 2 * k2)
    k4 = derivatives(t + dt, state + dt * k3)
    return state + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)


def simulate(step_function, dt, max_time=200.0):
    """
    Run a flight simulation using the given integration step function.
    Uses event detection to align time steps exactly with:
      1. Engine burnout (thrust discontinuity)
      2. Apogee (velocity sign change, which flips drag direction)
    This avoids integration error caused by stepping over
    these discontinuities.
    """
    t = 0.0
    state = np.array([0.0, 0.0])

    max_altitude = 0.0
    burnout_handled = False
    apogee_handled = False

    while t < max_time:

        step_dt = dt

        # Split at burnout: thrust drops from THRUST to 0 at t = BURN_TIME
        if not burnout_handled and t < BURN_TIME < t + dt:
            step_dt = BURN_TIME - t
            burnout_handled = True

        # Split at apogee: velocity crosses zero, flipping drag direction
        if not apogee_handled and state[1] > 0:
            trial_state = step_function(t, state, step_dt)

            if trial_state[1] <= 0:
                # Bisection: find the sub-step where velocity ~ 0
                lo, hi = 0.0, step_dt
                for _ in range(30):
                    mid = (lo + hi) / 2
                    mid_state = step_function(t, state, mid)
                    if mid_state[1] > 0:
                        lo = mid
                    else:
                        hi = mid
                step_dt = hi
                apogee_handled = True

        state = step_function(t, state, step_dt)
        t += step_dt

        altitude, velocity = state

        if altitude > max_altitude:
            max_altitude = altitude

        if altitude <= 0 and velocity < 0 and t > BURN_TIME:
            break

    return max_altitude

def main():
    # Reference: RK4 with a very small time step (treated as "ground truth")
    reference_altitude = simulate(rk4_step, dt=0.001)

    time_steps = [0.001, 0.01, 0.05, 0.1, 0.2, 0.5, 1.0]

    euler_errors = []
    rk4_errors = []

    for dt in time_steps:
        euler_altitude = simulate(euler_step, dt)
        rk4_altitude = simulate(rk4_step, dt)

        euler_error = abs(euler_altitude - reference_altitude) / reference_altitude * 100
        rk4_error = abs(rk4_altitude - reference_altitude) / reference_altitude * 100

        euler_errors.append(euler_error)
        rk4_errors.append(rk4_error)

        print(
            f"dt={dt:>6.3f}s | "
            f"Euler error: {euler_error:6.2f}% | "
            f"RK4 error: {rk4_error:6.2f}%"
        )

    # Plot the comparison
    plt.figure(figsize=(8, 5))
    plt.plot(time_steps, euler_errors, "o-", label="Euler")
    plt.plot(time_steps, rk4_errors, "s-", label="RK4")
    plt.xscale("log")
    plt.yscale("log")
    plt.xlabel("Time step (s)")
    plt.ylabel("Error in max altitude (%)")
    plt.title("Euler vs RK4: Accuracy vs Time Step Size")
    plt.legend()
    plt.grid(True, which="both", alpha=0.3)
    plt.tight_layout()
    plt.savefig("integrator_comparison.png", dpi=150)
    print("\nPlot saved as integrator_comparison.png")


if __name__ == "__main__":
    main()