# Self-Balancing Robot with Ball-on-Head Control
**Modeling, Control, and Observer Design using MATLAB & Simulink**

## Overview
This project studies the modeling, control, and state estimation of a **nonlinear self-balancing wheeled robot** that must balance a **ball on its head** under disturbances and measurement noise.

The goal is to bridge **control theory** with **practical implementation**, exposing real-world challenges such as nonlinear dynamics, noise, partial state measurements, and black-box testing.

All modeling, simulations, and analysis are performed using **MATLAB and Simulink**.

---

## System Description
- **System**: Self-balancing robot with a ball mounted on top  
- **Motion**: 2D planar motion  
- **Assumptions**:
  - No-slip condition
  - Rigid body dynamics
  - Ball rolls without slipping
- **Measured Output**:
  - Robot angular orientation (IMU)
  - Wheel angle (encoder assumed if required)

The system is **open-loop unstable** and nonlinear.

---

## Project Objectives
- Derive nonlinear equations of motion using first principles
- Convert the model to state-space form
- Linearize the system about equilibrium points
- Design and compare different control strategies
- Design state observers under noisy measurements
- Validate results using MATLAB and Simulink
- Quantitatively compare controller and observer performance

---

## Tasks Performed

### 1. System Modeling
- Derived governing differential equations
- Identified equilibrium points
- Linearized the nonlinear model
- Validated behavior through simulation

---

### 2. Controller Design

#### PID Controller
- Derived transfer function
- Designed PID controller using root locus techniques
- Tuned for steady-state and transient performance

#### Full State Feedback Controller
- Designed using pole placement and/or LQR
- Justified design assumptions and constraints
- Achieved desired stability and performance metrics

#### Controller Comparison
- Compared PID vs full-state feedback
- Metrics evaluated:
  - Settling time
  - Overshoot
  - Steady-state error
  - Control effort
- Quantitative performance comparison provided

---

### 3. Observer Design

#### Full-Order Observer
- Designed and implemented in Simulink
- Integrated with state feedback control

#### Reduced-Order Observer
- Designed to meet similar performance requirements
- Compared estimation accuracy and convergence

#### Noise & Signal Processing
- Added measurement noise using Simulink noise blocks
- Performed FFT analysis on noisy signals
- Designed and applied a low-pass filter
- Evaluated observer and controller performance with:
  - Noisy output
  - Filtered output

---

### 4. Blind (Black-Box) Testing
- Controllers and observers tested on a black-box system
- System parameters unknown during testing
- Emphasis on automated gain computation
- User input limited to a maximum of two parameters

---

## Tools Used
- **MATLAB**
  - System modeling and linearization
  - Controller and observer design
  - FFT and signal filtering
- **Simulink**
  - Nonlinear simulations
  - Noise injection
  - Observer implementation
  - Visualization of system response

---

## Learning Outcomes
- Nonlinear system modeling
- Linearization techniques
- PID and state feedback controller design
- Full and reduced-order observer design
- MATLAB programming and scripting
- Simulink-based system visualization
- Frequency-domain noise analysis
- Black-box system testing
- Technical report writing

---

## Repository Structure
