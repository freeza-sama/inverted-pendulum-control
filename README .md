# Ball and Beam Control System: Modeling, Control, and Estimation

## Overview
This project provides a comprehensive analysis and implementation of a Ball and Beam control system using MATLAB and Simulink. The workflow spans from first-principles mathematical modeling (Lagrangian mechanics) to advanced state estimation and robust signal processing in the presence of sensor noise.

## Project Features
* **Mathematical Modeling:** Non-linear derivation using the Lagrangian formulation and linearization for control design.
* **Controller Design:** Comparative analysis of **PID** (Proportional-Integral-Derivative) and **LQR** (Linear-Quadratic Regulator) controllers.
* **State Estimation:** Implementation of **Full-Order** and **Minimal-Order** observers to estimate system states.
* **Signal Processing:** * Noise injection into sensor outputs.
    * FFT (Fast Fourier Transform) analysis to identify noise frequencies.
    * Low-Pass Filter design for signal restoration.

## Prerequisites
* MATLAB (R202xx recommended)
* Simulink
* Control System Toolbox
* Signal Processing Toolbox
* Symbolic Math Toolbox (for Lagrangian derivation)

## Project Workflow & File Structure

The project is executed in the following chronological phases. 

### Phase 1: Mathematical Modeling
1.  **Derivation:** Equations of motion derived using $\mathcal{L} = T - V$.
2.  **Simulation:** Open-loop verification in MATLAB to ensure physics compliance (e.g., gravity effects on beam tilt).

### Phase 2: Controller Implementation
* **PID Control:** Classical control design tuned for stabilization.
* **LQR Control:** Optimal control design minimizing the cost function $J = \int (x^T Q x + u^T R u) dt$.
* *File:* `controller_design.m` (Calculates gains $K_{pid}$ and $K_{lqr}$).

### Phase 3: Observer Design (Simulink)
Implementation of state observers to reconstruct state vectors from limited sensor data.
* **Full-Order Observer:** Estimates all system states.
* **Minimal-Order Observer:** Estimates only unmeasured states to reduce computational load.
* *File:* `observer_sim.slx`

### Phase 4: Robustness & Signal Processing
1.  **Noise Injection:** Gaussian white noise added to the sensor output channel.
2.  **Spectral Analysis:** FFT performed on noisy signals to characterize the noise profile.
3.  **Filter Design:** Butterworth Low-Pass Filter designed to attenuate high-frequency noise.
4.  **Final Verification:** Closed-loop simulation with Noise + Filter + Observer + Controller.

## How to Run

1.  **Initialize Parameters:** Run the initialization script to load physical constants (mass, length, gravity) and calculate matrices ($A, B, C, D$).
    ```matlab
    >> init_params
    ```

2.  **Calculate Gains:** Run the design script to generate controller gains and observer matrices.
    ```matlab
    >> design_controllers
    ```

3.  **Run Simulation:** Open the Simulink model to view the Ball and Beam behavior.
    ```matlab
    >> open_system('BallBeam_Main.slx')
    ```
    *Select the desired configuration (PID/LQR, Full/Min Observer) using the manual switch blocks inside the model.*

4.  **Analyze Noise:** Run the FFT script to generate frequency domain plots before and after filtering.
    ```matlab
    >> noise_analysis_fft
    ```

## Results
* **Step Response:** Comparison of settling time and overshoot between PID and LQR.
* **Estimation Error:** Convergence plots showing the difference between actual states and observed states.
* **Noise Rejection:** FFT plots demonstrating the efficacy of the Low-Pass Filter.

## Author
[Your Name/Student ID]
[Date]
