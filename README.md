# AUV Depth Control Simulation

A professional Python simulation of Autonomous Underwater Vehicle (AUV) depth control using a PID controller and realistic underwater vehicle dynamics.

## Features

* PID depth control
* Buoyancy force modeling
* Gravity force modeling
* Added mass effects
* Hydrodynamic quadratic drag
* Thruster saturation
* Ocean current disturbances
* Performance metrics:

  * Overshoot
  * Steady-state error
  * Integral Absolute Error (IAE)
  * Root Mean Square Error (RMSE)

## Governing Dynamics

(m + ma) dv/dt = T + B − W − D(v) + Fcurrent

Where:

* T = Thruster force
* B = Buoyancy force
* W = Weight force
* D(v) = Hydrodynamic drag
* Fcurrent = Ocean current disturbance
* ma = Added mass

## Installation

```bash
git clone <repository-url>
cd auv-depth-control

python -m venv .venv
.venv\Scripts\activate

pip install -r requirements.txt
```

## Run

```bash
python main.py
```

## Future Work

* Adaptive PID Control
* LQR Control
* Model Predictive Control
* Monte Carlo Analysis
* Sensor Noise Modeling
* Full 6-DOF AUV Dynamics
