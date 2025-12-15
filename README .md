## 🧠 Control & State Estimation
The system is underactuated, meaning we cannot directly measure all states (like the ball's exact velocity). We implemented two advanced observers to estimate these missing variables from noisy IMU data:

### 1. State Observers (Sensor Fusion)
* **Full-Order Observer ("Max Order"):** Reconstructs the entire state vector ($\hat{x}$) using the plant output and estimation error.
* **Reduced-Order Observer ("Min Order"):** Designed to estimate *only* the unmeasured states (angular velocities) to reduce computational load.
* **Validation:** Comparison plots show the Observer error converging to zero within $t < 2s$.

### 2. Signal Processing Pipeline
Real-world sensors are noisy. We simulated this by injecting White Gaussian Noise into the feedback loop and built a cleaning pipeline:
1.  **Noise Injection:** Added random noise to sensor blocks in Simulink.
2.  **FFT Analysis:** Performed Fast Fourier Transform on the noisy signal to identify high-frequency noise components.
3.  **Filter Design:** Designed a **Butterworth Low-Pass Filter** based on the FFT cutoff frequency.
4.  **Result:** The filtered signal restored system stability where the raw noisy signal caused jitter.

## 🎛️ Control Strategies
* **LQR (Linear Quadratic Regulator):** Minimized a cost function $J = \int (x^T Q x + u^T R u) dt$ to balance performance vs. energy.
* **PID (Root Locus):** Tuned via Root Locus to pull closed-loop poles into the stable left-half plane.
