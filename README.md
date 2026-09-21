# Rocket Trajectory Simulator

![CI](https://github.com/yavuzselimsert/rocket-trajectory-sim/actions/workflows/ci.yml/badge.svg)

![Flight Dashboard](flight_dashboard.png)
![Flight Animation](flight_animation.gif)

A Python-based 2D simulation of a rocket launch, modeling thrust, gravity, 
variable mass, and aerodynamic drag using numerical integration.

> A small numerical physics project exploring rocket flight dynamics 
> and the accuracy trade-offs between different integration methods.

## Features

- 🚀 Physically consistent 2D rocket flight model (thrust, gravity, drag, variable mass)
- 🌍 Exponential atmosphere model (air density decreasing with altitude)
- 🧮 Runge-Kutta 4th order (RK4) numerical integration
- ⚡ Event detection for discontinuities (engine burnout, apogee)
- 📊 Euler vs RK4 accuracy comparison across time step sizes
- 📈 Full flight dashboard: trajectory, altitude, and speed over time

## How It Works

The simulator solves the rocket's 2D equations of motion:

```
F_net_x = F_thrust_x + F_drag_x
F_net_y = F_thrust_y + F_drag_y - F_weight
a = F_net / m(t)
```
where mass `m(t)` decreases as propellant burns, thrust is fixed at a 
launch angle (no thrust vectoring), and drag opposes the velocity vector:
F_drag = 0.5 × ρ(h) × v² × Cd × A

with air density `ρ(h)` decreasing exponentially with altitude, and `v` 
being the total speed (`√(vx² + vy²)`).

Because these equations can't be solved analytically in closed form, the 
system is integrated numerically over time using **Runge-Kutta 4th order 
(RK4)**, which evaluates the rate of change at four points per time step 
for significantly higher accuracy than simple Euler integration.

### Handling Discontinuities

The model contains two physical discontinuities:

1. **Engine burnout** — thrust drops instantly from full thrust to zero
2. **Apogee** — drag direction flips as vertical velocity crosses zero

Both are handled with **event detection**: the simulation detects when a 
time step would cross one of these events and splits the step exactly at 
the transition (using bisection search for apogee), preventing artificial 
integration error.

### A Note on the Speed Curve

In the dashboard above, speed dips to a *minimum* at apogee rather than 
reaching zero. This is expected in a 2D simulation: at apogee, vertical 
velocity (vy) is zero, but horizontal velocity (vx) is still nonzero — so 
total speed (`√(vx² + vy²)`) stays above zero even at the highest point 
of the trajectory.

## Numerical Method Comparison

To validate the choice of RK4 over simple Euler integration, both methods 
were tested across a range of time steps, using a high-resolution RK4 run 
(dt = 0.001s) as the reference "ground truth."

![Integrator Comparison](integrator_comparison.png)

At small time steps, RK4 achieves near-zero error (< 0.01%) while Euler's 
error grows steadily — a direct demonstration of RK4's 4th-order convergence 
versus Euler's 1st-order convergence.

At large time steps (approaching the flight's characteristic timescale), 
both methods degrade similarly. This is expected: once the step size 
becomes comparable to the duration of the physical event being resolved, 
no integration method — regardless of theoretical order — can accurately 
capture the dynamics.

| dt (s) | Euler error | RK4 error |
|--------|-------------|-----------|
| 0.001  | 0.01%       | 0.00%     |
| 0.010  | 0.10%       | 0.00%     |
| 0.050  | 0.49%       | 0.00%     |
| 0.100  | 0.98%       | 0.90%     |
| 0.200  | 1.96%       | 1.79%     |
| 0.500  | 4.90%       | 4.44%     |
| 1.000  | 9.77%       | 8.79%     |

## Sample Flight Results

For a launch angle of 75° from horizontal:

| Metric | Value |
|--------|-------|
| Max speed | 88.58 m/s |
| Max altitude | 449.90 m |
| Time to apogee | 11.58 s |
| Total range | 527.50 m |
| Total flight time | 21.53 s |

## Project Structure

```
rocket-trajectory-sim/
├── rocket_sim.py # Main 2D flight simulation
├── compare_integrators.py # Euler vs RK4 accuracy comparison
├── flight_dashboard.png
├── integrator_comparison.png
├── requirements.txt
└── README.md
```


## Installation

```bash
git clone https://github.com/yavuzselimsert/rocket-trajectory-sim.git
cd rocket-trajectory-sim
pip install -r requirements.txt
```

## Usage

**Run the flight simulation:**
```bash
python rocket_sim.py
```
Prints a flight summary and saves `flight_dashboard.png` (trajectory, 
altitude, and speed over time).


**Run the integration method comparison:**
```bash
python compare_integrators.py
```
Prints an error table and saves `integrator_comparison.png`.

## Testing

The project includes automated tests validating physical correctness — 
mass conservation, apogee conditions, monotonicity constraints, and 
energy consistency:

```bash
python -m pytest test_rocket_sim.py -v
```

## Technologies

- **Python**
- **NumPy**
- **Matplotlib**
- **Numerical Integration (Euler, RK4)**

## Motivation

This project explores the numerical methods behind rocket flight 
simulation — a practical introduction to solving differential equations 
computationally, validating numerical accuracy, and handling the kind of 
real-world modeling complications (like discontinuities) that don't show 
up in textbook examples.

---

**Author:** [Yavuz Selim Sert](https://github.com/yavuzselimsert)