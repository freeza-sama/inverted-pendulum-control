# Self-Balancing Robot with Ball Balancer 🤖⚖️

**Tech Stack:** MATLAB | Simulink | Control Systems | State-Space Analysis

## 📌 Project Overview
This project implements advanced control strategies for a **coupled underactuated system**: a 2-wheeled self-balancing robot that must simultaneously balance a free-rolling ball on its top surface.

The system is modeled in 2D space, deriving governing differential equations and converting them into **State-Space form**.

![System Diagram](images/system_diagram.png)
*(Figure: 2D Model of the Robot-Ball Coupled System)*

## 🧠 Control Architecture
We designed and compared two primary control approaches:
1.  **LQR (Linear Quadratic Regulator):** Optimized for steady-state tracking and energy efficiency.
2.  **PID (Root Locus):** Tuned for robust error correction.

### Key Features
* **State Estimation:** Implemented **Full-Order and Reduced-Order Observers** to estimate unmeasured states.
* **Noise Handling:** Analyzed sensor noise using **FFT** and implemented **Butterworth Low-Pass Filters**.
* **Robustness:** Validated via "Blind Testing" on a black-box system.

## 📊 Results
* **Performance:** The LQR controller achieved stabilization faster than the PID approach.
* **Observer Accuracy:** The Reduced-Order Observer successfully estimated angular velocity with minimal error.

![Simulation Result](images/simulation_result.png)

## 📂 Repository Structure
* `src/`: Contains all `.m` scripts and `.slx` Simulink models.
* `images/`: System diagrams and simulation result plots.
