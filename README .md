# Control & State Estimation of a Coupled Underactuated System 🤖

**Project Status:** Complete  
**Tools:** MATLAB, Simulink, Control System Toolbox

## 1. 📝 Abstract
This project focuses on the stabilization of a highly unstable, underactuated system: a 2-wheeled self-balancing robot with a free-rolling ball on top. The primary objective was to design a robust control architecture capable of maintaining equilibrium in the presence of sensor noise and unmeasured states.

Key contributions include:
1.  **Mathematical Modeling:** Derivation of State-Space equations.
2.  **Signal Processing:** FFT analysis of sensor noise and Butterworth Low-Pass Filter design.
3.  **State Estimation:** Implementation of Full-Order and Reduced-Order (Min-Order) Observers.
4.  **Control Design:** Comparative analysis of LQR (Optimal Control) vs. PID (Classical Control).

---

## 2. 📐 Mathematical Modeling
The system dynamics were linearized around the upright equilibrium point. We derived the governing differential equations and converted them into **State-Space Representation**:

$$\dot{x} = Ax + Bu$$
$$y = Cx + Du$$

Where the state vector $x$ includes:
* $\theta$: Rod angle
* $\dot{\theta}$: Angular velocity
* $x$: Cart position
* $\dot{x}$: Cart velocity

---

## 3. 📡 Signal Processing & Noise Handling
Real-world sensors (IMUs/Encoders) introduce high-frequency noise. We simulated this environment to ensure robust controller performance.

### 3.1 Noise Injection & FFT Analysis
* **Simulation:** White Gaussian Noise was injected into the sensor feedback loop in Simulink.
* **Analysis:** A Fast Fourier Transform (FFT) was performed on the noisy signal to identify the noise spectrum.
* **Findings:** Significant noise power was observed at frequencies $> 50$ Hz, interfering with the derivative terms in the controller.

### 3.2 Filter Design
To mitigate this, a **2nd-Order Butterworth Low-Pass Filter** was designed.
* **Cutoff Frequency:** Selected based on the FFT results to attenuate noise while preserving system dynamics.
* **Result:** The filtered signal successfully restored stability, preventing high-frequency chatter in the actuators.

---

## 4. 👁️ State Estimation (Observer Design)
Since not all states (specifically angular velocities) were directly measurable, we designed observers to estimate the full state vector $\hat{x}$.

### 4.1 Full-Order Observer (Max-Order)
A Luenberger Observer was designed to estimate all states.
* **Dynamics:** $\dot{\hat{x}} = A\hat{x} + Bu + L(y - C\hat{x})$
* **Pole Placement:** Observer poles were placed $5\times$ to $10\times$ faster than the controller poles to ensure rapid convergence of the error dynamics $e(t) \to 0$.

### 4.2 Reduced-Order Observer (Min-Order)
To reduce computational load, we implemented a Reduced-Order Observer that estimates *only* the unmeasured states.
* **Method:** Partitioned the state vector into measured ($x_m$) and unmeasured ($x_u$) components.
* **Outcome:** The Min-Order observer provided accurate velocity tracking with lower processing overhead compared to the Full-Order variant.

---

## 5. 🎛️ Controller Design & Results

### 5.1 LQR (Linear Quadratic Regulator)
We minimized the quadratic cost function:
$$J = \int_0^\infty (x^T Q x + u^T R u) dt$$
* **Q Matrix:** Tuned to penalize deviations in the "Ball Position" heavily.
* **Performance:** LQR showed superior steady-state performance and energy efficiency compared to PID.

### 5.2 PID Control
Designed using Root Locus techniques to ensure all closed-loop poles reside in the Left Half Plane (LHP). While stable, the PID response showed slightly higher overshoot during transient phases.

### 5.3 Simulation Results
*(Ensure your image filename matches exactly below)*
![Simulation Graphs](images/simulation_result.png)
*Figure 1: Comparison of System Response (LQR vs PID) under impulse disturbance.*

---

## 6. 📂 Project Structure
* `src/`: MATLAB scripts for LQR calculation, Observer matrices, and Simulink `.slx` models.
* `images/`: System block diagrams and result plots.

## 🚀 How to Run
1.  Clone the repository.
2.  Run `src/init_params.m` to load system matrices ($A, B, C, D$) and filter coefficients.
3.  Open `src/main_simulation.slx` and click **Run**.
