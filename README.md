# Quadcopter 6-DOF Simulation using Pybullet

A physics-based quadcopter simulation built usinf Python and Pybullet, featuring:

- 6-DOF rigid body dynamics
- Cascaded PID altitude controller
- Attitude stabilization
- Real-time visualization in Pybullet
- Quadrotor translational and rotational dynamics
- Euler angle kinematics
- Hover stabilization

---

## Features

### Drone Modeling
- Custom quadcopter built using `createMultiBody()`
- Central body, arms, and rotors modeled manually
- Realistic mass and inertia properties

### Flight Dynamics
Implementation of full rigid-body equations of motion:

#### Translational Dynamics
- Position
- Velocity
- Acceleration

#### Rotational Dynamics
- Angular rates
- Moments
- Euler angle kinematics

### Control System

#### Cascaded PID Altitude Controller
Outer Loop:
- Altitude error → desired vertical velocity

Inner Loop:
- Vertical velocity error → thrust command

#### Attitude PID Controllers
- Roll stabilization
- Pitch stabilization
- Yaw stabilization

---

## Technologies Used

- Python
- NumPy
- PyBullet