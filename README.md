# Maze-Solving Robot Using PID, LQR, MPC, and SMC

## Project Overview
This project presents a comprehensive simulation framework for autonomous navigation of a unicycle robot through complex 2D maze environments. Four distinct controllers—Proportional-Integral-Derivative (PID), Linear Quadratic Regulator (LQR), Model Predictive Control (MPC), and Sliding Mode Control (SMC)—are implemented and systematically evaluated across multiple maze scenarios.

## Objectives
- Design and implement a robust simulation environment to solve maze navigation tasks.
- Evaluate and compare PID, LQR, MPC, and SMC controllers using clearly defined performance metrics:
  - Mean Tracking Error (path-following accuracy)
  - Time to Reach Goal (navigation efficiency)
  - Control Effort (number of control steps)
- Identify optimal controller characteristics for reliable robotic navigation.

## Controllers Implemented

### PID Controller
- Real-time feedback mechanism minimizing tracking error.
- Lightweight and responsive; suitable for rapid control scenarios.
- Includes anti-windup measures and control input constraints for improved stability.

### Linear Quadratic Regulator (LQR)
- Optimal controller minimizing a cost function balancing tracking error and control effort.
- Requires linearization around reference trajectories for optimal performance.
- Employs state-space representation for precise control action.

### Model Predictive Control (MPC)
- Advanced predictive method using a receding horizon approach to optimize future robot behavior.
- Explicitly handles system constraints and solves an optimization problem at each timestep.
- Predictive horizon: 25 steps; Control horizon: 8 steps.

### Sliding Mode Control (SMC)
- Robust nonlinear controller effectively managing uncertainties and disturbances.
- Uses a defined sliding surface to maintain system stability.
- Implements boundary layers to reduce "chattering," enhancing actuator longevity.

## Simulation Environment
- Robot modeled as a unicycle with linear and angular velocity as inputs.
- Maze environments generated using Probabilistic Roadmap (PRM) path planning.
- Simulated in MATLAB with visualization, optional video logging, and detailed performance metrics export.

## Tools and Technologies
- MATLAB
- Robotics System Toolbox

## Course
- ME5659 Control Systems Engineering, April 2025
