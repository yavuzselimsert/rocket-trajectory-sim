import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from rocket_sim import simulate_flight, BURN_TIME, LAUNCH_ANGLE_DEG


def create_animation():
    times, xs, ys, vxs, vys = simulate_flight()

    # Downsample frames so the GIF isn't huge / slow
    # (simulation has ~2000 points at TIME_STEP=0.01, we don't need all of them)
    frame_skip = 5
    times = times[::frame_skip]
    xs = xs[::frame_skip]
    ys = ys[::frame_skip]

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.set_xlim(0, max(xs) * 1.1)
    ax.set_ylim(0, max(ys) * 1.2)
    ax.set_xlabel("Downrange distance (m)")
    ax.set_ylabel("Altitude (m)")
    ax.set_title(f"Rocket Flight Animation (launch angle = {LAUNCH_ANGLE_DEG}°)")
    ax.grid(True, alpha=0.3)

    # Full trajectory path (faint, drawn once as reference)
    ax.plot(xs, ys, color="lightgray", linewidth=1, zorder=1)

    # Elements that will be updated each frame
    trail, = ax.plot([], [], color="tab:blue", linewidth=2, zorder=2)
    rocket_dot, = ax.plot([], [], "o", color="tab:red", markersize=8, zorder=3)
    time_text = ax.text(0.02, 0.95, "", transform=ax.transAxes)

    def init():
        trail.set_data([], [])
        rocket_dot.set_data([], [])
        time_text.set_text("")
        return trail, rocket_dot, time_text

    def update(frame):
        trail.set_data(xs[:frame + 1], ys[:frame + 1])
        rocket_dot.set_data([xs[frame]], [ys[frame]])

        phase = "Powered ascent" if times[frame] < BURN_TIME else "Coasting"
        time_text.set_text(f"t = {times[frame]:.2f}s  ({phase})")

        return trail, rocket_dot, time_text

    anim = animation.FuncAnimation(
        fig, update, frames=len(times),
        init_func=init, interval=30, blit=True
    )

    anim.save("flight_animation.gif", writer="pillow", fps=30)
    print("Animation saved as flight_animation.gif")


if __name__ == "__main__":
    create_animation()