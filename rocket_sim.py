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

LAUNCH_ANGLE_DEG = 75.0  # measured from horizontal (90 = straight up)

TIME_STEP = 0.01


def air_density(altitude):
    return AIR_DENSITY_SEA_LEVEL * np.exp(-max(altitude, 0) / SCALE_HEIGHT)


def rocket_mass(t):
    burn_rate = PROPELLANT_MASS / BURN_TIME
    if t < BURN_TIME:
        return DRY_MASS + PROPELLANT_MASS - burn_rate * t
    else:
        return DRY_MASS


def current_thrust(t):
    return THRUST if t < BURN_TIME else 0.0


def derivatives(t, state):
    """
    2D equations of motion.
    state = [x, y, vx, vy]
    """
    x, y, vx, vy = state

    mass = rocket_mass(t)
    thrust = current_thrust(t)

    # Thrust direction is fixed at the launch angle (no thrust vectoring)
    angle_rad = np.radians(LAUNCH_ANGLE_DEG)
    thrust_x = thrust * np.cos(angle_rad)
    thrust_y = thrust * np.sin(angle_rad)

    # Drag opposes the velocity vector
    speed = np.sqrt(vx**2 + vy**2)
    rho = air_density(y)
    drag_magnitude = 0.5 * rho * speed**2 * DRAG_COEFFICIENT * CROSS_SECTIONAL_AREA

    if speed > 0:
        drag_x = -drag_magnitude * (vx / speed)
        drag_y = -drag_magnitude * (vy / speed)
    else:
        drag_x = 0.0
        drag_y = 0.0

    weight_y = -mass * GRAVITY

    net_force_x = thrust_x + drag_x
    net_force_y = thrust_y + drag_y + weight_y

    ax = net_force_x / mass
    ay = net_force_y / mass

    return np.array([vx, vy, ax, ay])


def rk4_step(t, state, dt):
    k1 = derivatives(t, state)
    k2 = derivatives(t + dt / 2, state + dt / 2 * k1)
    k3 = derivatives(t + dt / 2, state + dt / 2 * k2)
    k4 = derivatives(t + dt, state + dt * k3)
    return state + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)


def simulate_flight():
    """
    Simulate a 2D rocket launch using RK4 integration.
    Returns arrays of time, x, y, vx, vy.
    """
    t = 0.0
    state = np.array([0.0, 0.0, 0.0, 0.0])  # [x, y, vx, vy]

    burnout_handled = False

    times = [t]
    xs = [state[0]]
    ys = [state[1]]
    vxs = [state[2]]
    vys = [state[3]]

    max_time = 200.0

    while t < max_time:

        step_dt = TIME_STEP

        if not burnout_handled and t < BURN_TIME < t + TIME_STEP:
            step_dt = BURN_TIME - t
            burnout_handled = True

        state = rk4_step(t, state, step_dt)
        t += step_dt

        x, y, vx, vy = state

        if y <= 0 and vy < 0 and t > BURN_TIME:
            y = 0.0
            times.append(t)
            xs.append(x)
            ys.append(y)
            vxs.append(vx)
            vys.append(vy)
            break

        times.append(t)
        xs.append(x)
        ys.append(y)
        vxs.append(vx)
        vys.append(vy)

    return (
        np.array(times),
        np.array(xs),
        np.array(ys),
        np.array(vxs),
        np.array(vys),
    )


def main():
    times, xs, ys, vxs, vys = simulate_flight()

    speeds = np.sqrt(vxs**2 + vys**2)

    max_altitude = np.max(ys)
    max_altitude_time = times[np.argmax(ys)]
    total_range = xs[-1]
    flight_time = times[-1]
    max_speed = np.max(speeds)

    print("=== Rocket Flight Summary (2D) ===")
    print(f"Launch angle:       {LAUNCH_ANGLE_DEG:.1f} deg")
    print(f"Max speed:          {max_speed:.2f} m/s")
    print(f"Max altitude:       {max_altitude:.2f} m")
    print(f"Time to apogee:     {max_altitude_time:.2f} s")
    print(f"Total range:        {total_range:.2f} m")
    print(f"Total flight time:  {flight_time:.2f} s")

    # Dashboard: trajectory, altitude, and speed over time
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    # Trajectory (x vs y)
    axes[0].plot(xs, ys)
    axes[0].set_xlabel("Downrange distance (m)")
    axes[0].set_ylabel("Altitude (m)")
    axes[0].set_title("Trajectory")
    axes[0].grid(True, alpha=0.3)
    axes[0].axis("equal")

    # Altitude vs time
    axes[1].plot(times, ys, color="tab:orange")
    axes[1].axvline(BURN_TIME, color="gray", linestyle="--", alpha=0.5, label="Burnout")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Altitude (m)")
    axes[1].set_title("Altitude vs Time")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    # Speed vs time
    axes[2].plot(times, speeds, color="tab:green")
    axes[2].axvline(BURN_TIME, color="gray", linestyle="--", alpha=0.5, label="Burnout")
    axes[2].set_xlabel("Time (s)")
    axes[2].set_ylabel("Speed (m/s)")
    axes[2].set_title("Speed vs Time")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend()

    fig.suptitle(f"Rocket Flight Dashboard (launch angle = {LAUNCH_ANGLE_DEG}°)")
    plt.tight_layout()
    plt.savefig("flight_dashboard.png", dpi=150)
    print("\nPlot saved as flight_dashboard.png")


if __name__ == "__main__":
    main()