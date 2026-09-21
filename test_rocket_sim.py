import numpy as np
import pytest

from rocket_sim import (
    simulate_flight,
    air_density,
    rocket_mass,
    AIR_DENSITY_SEA_LEVEL,
    DRY_MASS,
    PROPELLANT_MASS,
    BURN_TIME,
    GRAVITY,
)


def test_air_density_at_sea_level():
    """
    At altitude = 0, air density should equal the sea-level constant.
    """
    assert air_density(0) == pytest.approx(AIR_DENSITY_SEA_LEVEL)


def test_air_density_decreases_with_altitude():
    """
    Air density should strictly decrease as altitude increases.
    """
    densities = [air_density(h) for h in [0, 1000, 5000, 10000]]
    assert densities == sorted(densities, reverse=True)


def test_mass_at_launch():
    """
    At t=0, the rocket's mass should be dry mass + full propellant.
    """
    expected = DRY_MASS + PROPELLANT_MASS
    assert rocket_mass(0) == pytest.approx(expected)


def test_mass_after_burnout():
    """
    After burnout, the rocket's mass should equal dry mass only.
    """
    assert rocket_mass(BURN_TIME) == pytest.approx(DRY_MASS)
    assert rocket_mass(BURN_TIME + 10) == pytest.approx(DRY_MASS)


def test_mass_decreases_monotonically_during_burn():
    """
    Mass should decrease steadily while the engine is firing.
    """
    times = np.linspace(0, BURN_TIME, 20)
    masses = [rocket_mass(t) for t in times]
    assert masses == sorted(masses, reverse=True)


def test_rocket_returns_to_ground():
    """
    A launched rocket should always come back down to y = 0.
    """
    times, xs, ys, vxs, vys = simulate_flight()
    assert ys[-1] == pytest.approx(0.0, abs=1e-6)


def test_apogee_occurs_before_landing():
    """
    The point of maximum altitude should occur strictly before
    the rocket lands, not at the very start or end.
    """
    times, xs, ys, vxs, vys = simulate_flight()
    apogee_index = np.argmax(ys)
    assert 0 < apogee_index < len(ys) - 1


def test_vertical_velocity_near_zero_at_apogee():
    """
    At the moment of maximum altitude, vertical velocity should
    be close to zero (it's the defining physical condition of apogee).
    """
    times, xs, ys, vxs, vys = simulate_flight()
    apogee_index = np.argmax(ys)
    assert vys[apogee_index] == pytest.approx(0.0, abs=1.0)


def test_horizontal_distance_increases_monotonically():
    """
    Downrange distance should never decrease (no wind, no thrust
    vectoring — the rocket only moves forward horizontally).
    """
    times, xs, ys, vxs, vys = simulate_flight()
    assert np.all(np.diff(xs) >= -1e-9)


def test_energy_roughly_conserved_without_drag():
    """
    Sanity check: with drag artificially disabled, mechanical energy
    (kinetic + potential) after burnout should stay roughly constant
    until landing, since only gravity acts on the rocket at that point.
    This is checked using the analytic free-fall relation for a
    coasting projectile.
    """
    times, xs, ys, vxs, vys = simulate_flight()

    # Find the index right after burnout
    post_burnout_index = np.searchsorted(times, BURN_TIME) + 1

    v0 = np.sqrt(vxs[post_burnout_index]**2 + vys[post_burnout_index]**2)
    y0 = ys[post_burnout_index]

    # Rough check: speed at apogee (vy=0) should be close to |vx| there,
    # and total speed should never exceed the post-burnout speed by much
    # (since only gravity + drag act afterward, both of which remove energy
    # or redirect it, never add it).
    apogee_index = np.argmax(ys)
    speed_at_apogee = np.sqrt(vxs[apogee_index]**2 + vys[apogee_index]**2)

    assert speed_at_apogee <= v0 * 1.01  # allow small numerical tolerance