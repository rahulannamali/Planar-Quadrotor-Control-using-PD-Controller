# Planar Quadrotor Control using PD Controller – MATLAB

## 📌 Project Description

This project demonstrates planar flight control of a quadrotor (in the Y-Z plane) using a Proportional-Derivative (PD) controller in MATLAB. It models the nonlinear dynamics of a quadrotor and implements a PD controller through linearization to track trajectories and maintain stable hover.

The system is simulated using MATLAB’s `ode45` solver and visualized in real-time. Control inputs include total thrust and moment, while the controller is responsible for stabilizing position and orientation.

## 🚁 Features

- 2D simulation of quadrotor motion in the Y-Z plane
- PD control with dynamic linearization about hover configuration
- Follows reference trajectories (line, sine wave, hover)
- Visual output and `.gif` generation for demonstration

## 📂 File Structure

- `controller.m` – Implements PD control for y, z, and φ
- `simulation_2d.m` – Simulates quadrotor motion using ode45
- `runsim.m` – Top-level simulation runner
- `trajectories/` – Contains example trajectory functions (`traj_line.m`, `traj_sine.m`, etc.)
- `utils/` – Contains helper functions for plotting and physics
- `evaluate.p`, `submit.m` – Legacy files from original assignment (not needed here)

## ⚙️ How It Works

The nonlinear system is linearized around φ = 0 to design the PD controller. Desired accelerations are calculated using position and velocity errors, which are used to compute thrust and moment inputs.

### Key Control Equations:

```matlab
z̈_des = z̈_ref + Kp_z * (z_ref - z) + Kv_z * (ẋ_ref - ẋ)
ÿ_des = ÿ_ref + Kp_y * (y_ref - y) + Kv_y * (ẏ_ref - ẏ)
φ_des  = - ÿ_des / g
u1     = m * (g + z̈_des)
u2     = Ixx * (Kp_phi * (φ_des - φ) + Kv_phi * (φ̇_des - φ̇))
