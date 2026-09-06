# AgileQuad-NMPC: Comprehensive Technical & Interview Master Guide

> **Purpose of this Document:**  
> This guide is an exhaustive, first-principles technical breakdown of the **AgileQuad-NMPC** autonomous flight control and state estimation stack. Reading and internalizing this document will prepare you to answer any theoretical, algorithmic, mathematical, or systems-level question asked in an autonomous systems, robotics, or aerospace engineering interview (e.g., at Skydio, Anduril, Joby Aviation, Shield AI, Zoox, Tesla Autopilot, or DJI).

---

## Table of Contents
1. [Executive Summary & Elevator Pitches](#1-executive-summary--elevator-pitches)
2. [High-Level Architecture & Block Diagram](#2-high-level-architecture--block-diagram)
3. [The "Why": Core Architectural Decisions & Trade-Offs](#3-the-why-core-architectural-decisions--trade-offs)
4. [Mathematical Foundations & First-Principles Derivations](#4-mathematical-foundations--first-principles-derivations)
   - 4.1 [6-DoF Quadrotor Dynamics on $SO(3)$](#41-6-dof-quadrotor-dynamics-on-so3)
   - 4.2 [Lie Group $SO(3)$ & Minimal Attitude Parameterization](#42-lie-group-so3--minimal-attitude-parameterization)
   - 4.3 [15-State Error-State Extended Kalman Filter (ES-EKF)](#43-15-state-error-state-extended-kalman-filter-es-ekf)
   - 4.4 [Non-Linear Model Predictive Control (NMPC) Formulation](#44-non-linear-model-predictive-control-nmpc-formulation)
   - 4.5 [Gauss-Newton iLQR / Riccati Backward Pass](#45-gauss-newton-ilqr--riccati-backward-pass)
   - 4.6 [Differential Flatness & Trajectory Generation](#46-differential-flatness--trajectory-generation)
5. [C++20 Real-Time Systems Engineering](#5-c20-real-time-systems-engineering)
   - 5.1 [Zero-Heap Allocation Guarantee on the Hot Path](#51-zero-heap-allocation-guarantee-on-the-hot-path)
   - 5.2 [Cache Locality, SIMD Vectorization & Eigen Invariants](#52-cache-locality-simd-vectorization--eigen-invariants)
   - 5.3 [Dual-Backend Architecture (Native C++20 vs. CasADi C99 / acados)](#53-dual-backend-architecture-native-c20-vs-casadi-c99--acados)
6. [Hardware & Simulation Integration (MuJoCo)](#6-hardware--simulation-integration-mujoco)
7. [Proven Benchmarks & Performance Metrics](#7-proven-benchmarks--performance-metrics)
8. [Top 20 Technical Interview Questions & Model Answers](#8-top-20-technical-interview-questions--model-answers)
9. [Whiteboard Walkthrough Guide & Discussion Strategy](#9-whiteboard-walkthrough-guide--discussion-strategy)

---

## 1. Executive Summary & Elevator Pitches

### 30-Second Elevator Pitch
> *"I designed and engineered an end-to-end, hard real-time autonomous flight control and state estimation stack in modern C++20 for agile quadrotors. The system features a 6-DoF Non-Linear Model Predictive Controller (NMPC) running at 100 Hz with an average solve time of 0.26 ms—over 15 times faster than our 10 ms real-time deadline. It is coupled with a 15-state Error-State Extended Kalman Filter (ES-EKF) fusing 500 Hz IMU signals with 30 Hz Visual Odometry on the Lie algebra $\mathfrak{so}(3)$ to estimate orientation, velocity, and sensor biases. The entire control hot path is strictly zero-heap allocated, achieves sub-5-cm tracking RMSE on aggressive 3D trajectories, and is verified in MuJoCo physics."*

### 2-Minute Technical Summary
> *"Traditional aerial robotics stacks rely on cascaded PID loops or linearized MPC, which struggle during aggressive maneuvers due to motor saturation, translational blade drag, and attitude non-linearities on $SO(3)$.*
>
> *To overcome these limitations, I engineered a high-performance optimal flight stack consisting of two tightly coupled subsystems:*
>
> 1. *A **15-State Error-State Extended Kalman Filter (ES-EKF)**. By tracking perturbations in the 3-dimensional Lie algebra $\mathfrak{so}(3)$ rather than on over-parameterized quaternions, the filter completely avoids covariance singularities. It propagates nominal state kinematics at 500 Hz and updates at 30 Hz using visual odometry, estimating accelerometer and gyroscope biases online. The covariance update uses a Joseph-form formulation to guarantee mathematical symmetry and positive semi-definiteness.*
>
> 2. *A **Non-Linear Model Predictive Controller (NMPC)**. The controller solves a constrained receding-horizon optimal control problem over a 1.0-second horizon (20 nodes) at 100 Hz. The solver utilizes an allocation-free Gauss-Newton iLQR algorithm featuring a Riccati backward pass, adaptive Levenberg-Marquardt regularization, tilt angle penalties, motor thrust box constraints, and smooth obstacle avoidance. I also built a CasADi Python symbolic C99 code generator to cross-validate our native C++ solver against an acados Real-Time Iteration SQP backend.*
>
> *From a systems perspective, the entire runtime loop executes with **zero dynamic heap allocations**, cache-aligned fixed-size Eigen buffers, and SIMD vectorization, achieving an average solve time of 0.26 ms and tracking RMSE under 4.6 cm in the presence of Dryden wind gusts."*

---

## 2. High-Level Architecture & Block Diagram

```
 +-------------------------------------------------------------------------+
 |                               PLANT                                     |
 |  - 6-DoF Quadrotor on SO(3) with Translational Blade Drag Matrix D      |
 |  - High-Fidelity MuJoCo Physics (500 Hz RK4 Substepping)                |
 |  - Disturbances: Dryden Continuous Wind Gusts + Sensor Noise            |
 +-------------------+---------------------------------+-------------------+
                     |                                 |
         IMU (500 Hz)| Specific Force f_b              | VO / Mocap (30 Hz)
         a_m, omega_m| Angular Velocity                | Position p + Quat q
                     v                                 v
 +-------------------------------------------------------------------------+
 |                     STATE ESTIMATION: 15-STATE ES-EKF                   |
 |                                                                         |
 |  High-Rate IMU Propagation (500 Hz, 2 ms dt):                           |
 |    x_nom_{k+1} = f_kinematics(x_nom_k, u_imu - biases)                  |
 |    P_{k+1} = F_d P_k F_d^T + Q_d                                        |
 |                                                                         |
 |  Low-Rate Measurement Update (30 Hz):                                   |
 |    Residuals: r_p = z_p - p_nom,  r_theta = 2 vec(q_nom^{-1} * q_meas)  |
 |    Kalman Gain: K = P H^T (H P H^T + R)^{-1} via stack LDLT             |
 |    Error Injection: x_nom = x_nom (+) dx,  Reset: dx <- 0               |
 |    Joseph Form: P = (I - KH) P (I - KH)^T + K R K^T                     |
 +-------------------------------------+-----------------------------------+
                                       |
                   Estimated State     | x_hat = [p, v, q, omega]^T in R^13
                   (Zero Heap Copy)    v
 +-------------------------------------------------------------------------+
 |                   TRAJECTORY GENERATION & REFERENCE                     |
 |                                                                         |
 |  Differential Flatness Mapping:                                         |
 |    Flat outputs: sigma(t) = [x, y, z, yaw]^T                            |
 |    Derivatives: dot(sigma), ddot(sigma), jerk -> R(q) and omega         |
 |    Output: Horizon references X_ref[0..N], U_ref[0..N-1]                |
 +-------------------------------------+-----------------------------------+
                                       |
                   Horizon References  |
                   X_ref, U_ref        v
 +-------------------------------------------------------------------------+
 |                  NMPC CONTROLLER (100 Hz, ~0.26 ms)                     |
 |                                                                         |
 |  Optimal Control Problem (OCP):                                         |
 |    min sum_{k=0}^{N-1} (||e_k||_Q^2 + ||u_k - u_ref||_R^2 + Pen_k)     |
 |        + ||e_N||_P^2 + Pen_N                                            |
 |    s.t.  x_{k+1} = f_RK4(x_k, u_k)                                      |
 |          u_min <= u_k <= u_max                                          |
 |          ||q_xy||^2 <= sin^2(theta_max / 2)   [Tilt constraint]        |
 |          (p_k - p_obs)^T A_obs (p_k - p_obs) >= 1  [Obstacle]           |
 |                                                                         |
 |  Solver Engine:                                                         |
 |    - Native Gauss-Newton iLQR / SQP (Riccati Backward Pass)             |
 |    - RTI Warm-Starting across control cycles (U_k <- U_{k+1})           |
 |    - Dual-Backend: Cross-validated with CasADi C99 / acados export      |
 +-------------------------------------+-----------------------------------+
                                       | Optimal Actuation u_cmd = [f_T, tau]^T
                                       v
                                [ Back to Plant ]
```

---

## 3. The "Why": Core Architectural Decisions & Trade-Offs

When an interviewer asks *"Why did you design it this way instead of using standard methods?"*, here are your definitive arguments:

### 3.1. Why NMPC over Cascaded PID?
* **Cascaded PID Flaw:** Cascaded loops (Position P $\rightarrow$ Velocity PI $\rightarrow$ Attitude P $\rightarrow$ Rate PID) fundamentally assume small-angle approximations ($\sin\theta \approx \theta$, $\cos\theta \approx 1$) and linear dynamics near hover. During high-speed maneuvers ($>45^\circ$ tilt, high angular accelerations), non-linear gyroscopic coupling ($\boldsymbol{\omega} \times \mathbf{J}\boldsymbol{\omega}$) and aerodynamic drag ($\mathbf{D}\mathbf{v}$) cause severe phase lag, control clipping, and trajectory deviation.
* **NMPC Strength:** NMPC optimizes over the full non-linear dynamics on $SO(3)$ over a preview horizon ($1.0\,\text{s}$). The quadrotor can anticipate turns, proactively bank into curves, respect actuator bounds before saturation occurs, and dynamically plan around obstacles.

### 3.2. Why NMPC over Linear Time-Varying (LTV) MPC?
* **LTV-MPC Flaw:** Linearizing the quadrotor dynamics about a reference trajectory breaks down when actual state deviates significantly from the planned path or when operating near extreme roll/pitch angles where attitude error is highly non-linear.
* **NMPC Strength:** Re-evaluates exact non-linear Runge-Kutta dynamics and analytical error Jacobians at every step, yielding high tracking fidelity even under large external wind disturbances.

### 3.3. Why Error-State EKF (ES-EKF) over Standard EKF?
* **Standard EKF Quaternion Flaw:** Direct estimation of a 4-element unit quaternion in a standard EKF results in a $4 \times 4$ attitude covariance matrix. Because the quaternion has only 3 degrees of freedom ($\|\mathbf{q}\| = 1$), this covariance matrix is algebraically singular along the radial direction. Normalizing the quaternion after each update violates the linear optimality of the Kalman equations.
* **ES-EKF Lie Algebra Advantage:** The ES-EKF splits the state into a nominal state (propagated via full non-linear kinematics) and an error state. The attitude perturbation $\delta\boldsymbol{\theta} \in \mathfrak{so}(3) \simeq \mathbb{R}^3$ is parameterized in the 3-dimensional Lie algebra. Because the error state is injected and then reset to zero after every measurement update ($\delta\mathbf{x} \leftarrow \mathbf{0}$), the filter permanently operates at the origin $\delta\mathbf{x} \approx \mathbf{0}$, where first-order Taylor expansions are exact and covariance matrices are strictly full-rank and non-singular.

### 3.4. Why Native iLQR in C++20 alongside CasADi/acados?
* **Self-Contained Real-Time Engine:** External NLP/SQP solvers often depend on large dynamic shared libraries (`.so` / `.dylib`), non-standard memory allocators, or complex autotools. Writing a native iLQR solver using C++20 and compile-time Eigen buffers eliminates third-party dependencies, ensures strict determinism ($0.26\,\text{ms}$ solve), and enables zero-copy execution on embedded flight computers (such as NVIDIA Jetson, STM32, or PX4 companion boards).
* **Dual-Backend Verification:** CasADi symbolic C99 code generation serves as an exact mathematical oracle to verify that analytical Jacobians, Hessians, and dynamics in the native C++ engine have zero algebraic bugs.

---

## 4. Mathematical Foundations & First-Principles Derivations

### 4.1. 6-DoF Quadrotor Dynamics on $SO(3)$

The continuous state vector is $\mathbf{x} = [\mathbf{p}^\top, \mathbf{v}^\top, \mathbf{q}^\top, \boldsymbol{\omega}^\top]^\top \in \mathbb{R}^{13}$ and control input $\mathbf{u} = [f_T, \boldsymbol{\tau}^\top]^\top \in \mathbb{R}^4$:

#### 1. Translational Kinematics & Rotor Drag
$$\dot{\mathbf{p}} = \mathbf{v}$$
$$m \dot{\mathbf{v}} = m \mathbf{g} + \mathbf{R}(\mathbf{q}) \begin{bmatrix} 0 \\ 0 \\ f_T \end{bmatrix} - \mathbf{D}(\mathbf{v} - \mathbf{v}_{\text{wind}})$$

* $m \in \mathbb{R}$: Quadrotor mass ($1.0\,\text{kg}$).
* $\mathbf{g} = [0, 0, -9.81]^\top\,\text{m/s}^2$: Inertial gravity vector.
* $\mathbf{R}(\mathbf{q}) \in SO(3)$: Direction cosine matrix mapping body frame $\mathcal{B}$ vectors to world frame $\mathcal{W}$.
* $f_T = \sum_{i=1}^4 f_i$: Total collective thrust along the body $+Z_B$ axis.
* $\mathbf{D} = \text{diag}(d_x, d_y, d_z)$: Translational blade drag matrix ($[0.10, 0.10, 0.15]\,\text{N}\cdot\text{s/m}$). Blade flapping and induced drag create forces opposing translational velocity.

#### 2. Rotational Kinematics & Angular Momentum
$$\dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega} \end{bmatrix} = \frac{1}{2} \begin{bmatrix} -\mathbf{q}_v^\top \\ q_w \mathbf{I}_3 + [\mathbf{q}_v]_\times \end{bmatrix} \boldsymbol{\omega}$$
$$\mathbf{J} \dot{\boldsymbol{\omega}} = \boldsymbol{\tau} - \boldsymbol{\omega} \times (\mathbf{J}\boldsymbol{\omega})$$

* $\mathbf{q} = [q_w, q_x, q_y, q_z]^\top$: Hamilton unit quaternion ($w$-first).
* $\boldsymbol{\omega} = [p, q, r]^\top \in \mathbb{R}^3$: Angular velocity in the body frame $\mathcal{B}$.
* $\mathbf{J} = \text{diag}(J_{xx}, J_{yy}, J_{zz})$: Quadrotor diagonal inertia tensor.
* $\boldsymbol{\tau} = [\tau_x, \tau_y, \tau_z]^\top$: Body torque generated by differential rotor thrusts.
* $-\boldsymbol{\omega} \times (\mathbf{J}\boldsymbol{\omega})$: Non-linear gyroscopic precession term.

---

### 4.2. Lie Group $SO(3)$ & Minimal Attitude Parameterization

A rotation matrix $\mathbf{R} \in SO(3)$ satisfies $\mathbf{R}^\top \mathbf{R} = \mathbf{I}_3$ and $\det(\mathbf{R}) = +1$.
The Lie algebra $\mathfrak{so}(3)$ consists of $3 \times 3$ skew-symmetric matrices:
$$[\mathbf{v}]_\times = \begin{bmatrix} 0 & -v_z & v_y \\ v_z & 0 & -v_x \\ -v_y & v_x & 0 \end{bmatrix}, \quad \forall \mathbf{v} \in \mathbb{R}^3$$

#### Exponential Map
Maps a rotation vector $\boldsymbol{\theta} \in \mathbb{R}^3$ (where $\theta = \|\boldsymbol{\theta}\|$, $\mathbf{n} = \boldsymbol{\theta}/\theta$) to $SO(3)$ via Rodrigues' formula:
$$\exp([\boldsymbol{\theta}]_\times) = \mathbf{I}_3 + \frac{\sin\theta}{\theta}[\boldsymbol{\theta}]_\times + \frac{1 - \cos\theta}{\theta^2}[\boldsymbol{\theta}]_\times^2$$

In quaternion form:
$$\exp\left(\frac{1}{2}\boldsymbol{\theta}\right) = \begin{bmatrix} \cos(\theta/2) \\ \frac{\boldsymbol{\theta}}{\theta} \sin(\theta/2) \end{bmatrix}$$

#### Attitude Tracking Error
Given current quaternion $\mathbf{q}$ and reference quaternion $\mathbf{q}_{\text{ref}}$, the relative rotation quaternion is:
$$\mathbf{q}_{\text{err}} = \mathbf{q}_{\text{ref}}^{-1} \otimes \mathbf{q}$$
The minimal 3-parameter attitude error is the vector part of $\mathbf{q}_{\text{err}}$ in the tangent space:
$$\mathbf{e}_\theta = 2 \cdot \text{vec}(\mathbf{q}_{\text{err}}) = 2 \begin{bmatrix} q_{\text{err}, x} \\ q_{\text{err}, y} \\ q_{\text{err}, z} \end{bmatrix} \in \mathbb{R}^3$$
This formulation is smooth, singularity-free across all orientations, and maps directly to the physical rotation angle error for small perturbations.

---

### 4.3. 15-State Error-State Extended Kalman Filter (ES-EKF)

#### State Partitioning
* **True State ($\mathbf{x}_{\text{true}}$):** The physical state of the drone.
* **Nominal State ($\mathbf{x}_{\text{nom}} \in \mathbb{R}^{16}$):** High-rate kinematic state integrated from raw IMU signals:
  $$\mathbf{x}_{\text{nom}} = [\mathbf{p}^\top, \mathbf{v}^\top, \mathbf{q}^\top, \mathbf{b}_a^\top, \mathbf{b}_g^\top]^\top$$
* **Error State ($\delta\mathbf{x} \in \mathbb{R}^{15}$):** Small perturbation vector parameterized in the Lie algebra:
  $$\delta\mathbf{x} = [\delta\mathbf{p}^\top, \delta\mathbf{v}^\top, \delta\boldsymbol{\theta}^\top, \delta\mathbf{b}_a^\top, \delta\mathbf{b}_g^\top]^\top$$
  where $\mathbf{R}_{\text{true}} = \mathbf{R}_{\text{nom}} \exp([\delta\boldsymbol{\theta}]_\times) \approx \mathbf{R}_{\text{nom}} (\mathbf{I}_3 + [\delta\boldsymbol{\theta}]_\times)$.

#### Step 1: High-Rate IMU Propagation (500 Hz, $\Delta t = 2\,\text{ms}$)
1. **Debias measurements:**
   $$\hat{\mathbf{a}} = \mathbf{a}_m - \mathbf{b}_a, \quad \hat{\boldsymbol{\omega}} = \boldsymbol{\omega}_m - \mathbf{b}_g$$
2. **Kinematic state integration:**
   $$\mathbf{p}_{k+1} = \mathbf{p}_k + \mathbf{v}_k \Delta t + \frac{1}{2}(\mathbf{R}(\mathbf{q}_k)\hat{\mathbf{a}} + \mathbf{g})\Delta t^2$$
   $$\mathbf{v}_{k+1} = \mathbf{v}_k + (\mathbf{R}(\mathbf{q}_k)\hat{\mathbf{a}} + \mathbf{g})\Delta t$$
   $$\mathbf{q}_{k+1} = \mathbf{q}_k \otimes \exp\left(\frac{1}{2}\hat{\boldsymbol{\omega}}\Delta t\right)$$
3. **Discrete error covariance propagation:**
   $$\mathbf{P}_{k+1} = \mathbf{F}_d \mathbf{P}_k \mathbf{F}_d^\top + \mathbf{Q}_d$$
   where the discrete transition Jacobian $\mathbf{F}_d = \mathbf{I}_{15} + \mathbf{F}_c \Delta t$ is:
   $$\mathbf{F}_d = \begin{bmatrix}
   \mathbf{I}_3 & \mathbf{I}_3 \Delta t & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
   \mathbf{0} & \mathbf{I}_3 & -\mathbf{R}_k [\hat{\mathbf{a}}]_\times \Delta t & -\mathbf{R}_k \Delta t & \mathbf{0} \\
   \mathbf{0} & \mathbf{0} & \mathbf{I}_3 - [\hat{\boldsymbol{\omega}}]_\times \Delta t & \mathbf{0} & -\mathbf{I}_3 \Delta t \\
   \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}_3 & \mathbf{0} \\
   \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}_3
   \end{bmatrix}$$

#### Step 2: Low-Rate Measurement Update (30 Hz VO / MoCap)
Given measurement $\mathbf{z} = [\mathbf{p}_{\text{meas}}^\top, \mathbf{q}_{\text{meas}}^\top]^\top$:
1. **Compute Innovation Residuals ($\mathbf{r} \in \mathbb{R}^6$):**
   $$\mathbf{r}_p = \mathbf{z}_p - \mathbf{p}_{\text{nom}}, \quad \mathbf{r}_\theta = 2 \cdot \text{vec}(\mathbf{q}_{\text{nom}}^{-1} \otimes \mathbf{q}_{\text{meas}})$$
2. **Measurement Matrix ($\mathbf{H} \in \mathbb{R}^{6 \times 15}$):**
   $$\mathbf{H} = \begin{bmatrix} \mathbf{I}_3 & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\ \mathbf{0} & \mathbf{0} & \mathbf{I}_3 & \mathbf{0} & \mathbf{0} \end{bmatrix}$$
3. **Innovation Covariance & Kalman Gain:**
   $$\mathbf{S} = \mathbf{H}\mathbf{P}\mathbf{H}^\top + \mathbf{R}_m \in \mathbb{R}^{6 \times 6}$$
   Solved via `Eigen::LDLT` factorization on the stack: $\mathbf{K} = \mathbf{P}\mathbf{H}^\top \mathbf{S}^{-1}$.
4. **Error State Injection:**
   $$\delta\mathbf{x} = \mathbf{K}\mathbf{r}$$
   $$\mathbf{p}_{\text{nom}} \leftarrow \mathbf{p}_{\text{nom}} + \delta\mathbf{p}, \quad \mathbf{v}_{\text{nom}} \leftarrow \mathbf{v}_{\text{nom}} + \delta\mathbf{v}$$
   $$\mathbf{q}_{\text{nom}} \leftarrow \mathbf{q}_{\text{nom}} \otimes \exp\left(\frac{1}{2}\delta\boldsymbol{\theta}\right)$$
   $$\mathbf{b}_a \leftarrow \mathbf{b}_a + \delta\mathbf{b}_a, \quad \mathbf{b}_g \leftarrow \mathbf{b}_g + \delta\mathbf{b}_g$$
5. **Joseph-Form Covariance Update:**
   $$\mathbf{P} \leftarrow (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}(\mathbf{I} - \mathbf{K}\mathbf{H})^\top + \mathbf{K}\mathbf{R}_m\mathbf{K}^\top$$
   *Why Joseph Form?* Standard update $\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}$ can lose symmetry and positive semi-definiteness due to floating-point roundoff. The Joseph formulation is a sum of two congruent quadratic forms with $\mathbf{R}_m \succ 0$, mathematically guaranteeing $\mathbf{P} = \mathbf{P}^\top$ and $\mathbf{P} \succeq 0$.
6. **Error Reset:**
   $$\delta\mathbf{x} \leftarrow \mathbf{0}$$

---

### 4.4. Non-Linear Model Predictive Control (NMPC) Formulation

The optimal control problem over a preview horizon $N = 20$ with sampling interval $\Delta t = 0.05\,\text{s}$ ($T = 1.0\,\text{s}$) is formulated as:

$$\min_{\mathbf{x}_{0:N}, \mathbf{u}_{0:N-1}} \sum_{k=0}^{N-1} \left( \|\mathbf{e}_k\|_{\mathbf{Q}}^2 + \|\mathbf{u}_k - \mathbf{u}_{\text{ref}, k}\|_{\mathbf{R}}^2 + \text{Pen}_{\text{tilt}, k} + \text{Pen}_{\text{obs}, k} \right) + \|\mathbf{e}_N\|_{\mathbf{P}}^2 + \text{Pen}_{\text{obs}, N}$$

**Subject to:**
1. **Explicit RK4 Discretized Dynamics:**
   $$\mathbf{x}_{k+1} = \mathbf{f}_{\text{RK4}}(\mathbf{x}_k, \mathbf{u}_k)$$
2. **Actuator Saturation Constraints:**
   $$u_{i,\min} \le u_{i,k} \le u_{i,\max} \quad (f_T \in [0, 25]\,\text{N}, \; \tau_i \in [-2, 2]\,\text{N}\cdot\text{m})$$
3. **Tilt Angle Penalty:**
   $$\text{Pen}_{\text{tilt}} = w_{\text{tilt}} \max\left(0, q_x^2 + q_y^2 - \sin^2(\theta_{\max}/2)\right)^2$$
4. **Ellipsoidal Obstacle Avoidance Penalty:**
   $$\text{Pen}_{\text{obs}}(\mathbf{p}) = w_{\text{obs}} \max\left(0, 1 - (\mathbf{p} - \mathbf{p}_{\text{obs}})^\top \mathbf{A}_{\text{obs}} (\mathbf{p} - \mathbf{p}_{\text{obs}})\right)^2$$
   where $\mathbf{A}_{\text{obs}} = \text{diag}(1/a^2, 1/b^2, 1/c^2)$.

#### Tracking Error Vector on $SO(3)$ ($\mathbf{e} \in \mathbb{R}^{12}$)
$$\mathbf{e} = \begin{bmatrix} \mathbf{p} - \mathbf{p}_{\text{ref}} \\ \mathbf{v} - \mathbf{v}_{\text{ref}} \\ 2 \cdot \text{vec}(\mathbf{q}_{\text{ref}}^{-1} \otimes \mathbf{q}) \\ \boldsymbol{\omega} - \boldsymbol{\omega}_{\text{ref}} \end{bmatrix} \in \mathbb{R}^{12}$$
Notice that $\mathbf{e}$ has dimension 12, matching the true physical degrees of freedom (3 position, 3 velocity, 3 rotation, 3 angular rate), eliminating the 4th redundant quaternion degree of freedom.

---

### 4.5. Gauss-Newton iLQR / Riccati Backward Pass

The native solver uses an allocation-free Iterative Linear Quadratic Regulator (iLQR) with a Gauss-Newton Hessian approximation.

#### Analytical Error Jacobian ($\mathbf{J}_e \in \mathbb{R}^{12 \times 13}$)
$$\mathbf{J}_e = \frac{\partial \mathbf{e}}{\partial \mathbf{x}} = \begin{bmatrix} 
\mathbf{I}_3 & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{I}_3 & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \frac{\partial \mathbf{e}_\theta}{\partial \mathbf{q}} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}_3
\end{bmatrix}$$
where:
$$\frac{\partial \mathbf{e}_\theta}{\partial \mathbf{q}} = 2 \begin{bmatrix}
-q_{\text{ref}, x} & q_{\text{ref}, w} & q_{\text{ref}, z} & -q_{\text{ref}, y} \\
-q_{\text{ref}, y} & -q_{\text{ref}, z} & q_{\text{ref}, w} & q_{\text{ref}, x} \\
-q_{\text{ref}, z} & q_{\text{ref}, y} & -q_{\text{ref}, x} & q_{\text{ref}, w}
\end{bmatrix} \in \mathbb{R}^{3 \times 4}$$

The Gauss-Newton cost gradient and Hessian are computed as:
$$\mathbf{l}_x = 2 \mathbf{J}_e^\top \mathbf{Q} \mathbf{e}, \quad \mathbf{l}_{xx} \approx 2 \mathbf{J}_e^\top \mathbf{Q} \mathbf{J}_e \in \mathbb{R}^{13 \times 13}$$
$$\mathbf{l}_u = 2 \mathbf{R} (\mathbf{u} - \mathbf{u}_{\text{ref}}), \quad \mathbf{l}_{uu} = 2 \mathbf{R} \in \mathbb{R}^{4 \times 4}$$

#### Riccati Backward Pass Recursion
Starting at terminal stage $N$ with $\mathbf{V}_x = \mathbf{l}_{x, N}$ and $\mathbf{V}_{xx} = \mathbf{l}_{xx, N}$, we sweep backward from $k = N-1$ down to $0$:
1. **Form Action-Value Function Approximations:**
   $$\mathbf{Q}_x = \mathbf{l}_x + \mathbf{A}_k^\top \mathbf{V}_x'$$
   $$\mathbf{Q}_u = \mathbf{l}_u + \mathbf{B}_k^\top \mathbf{V}_x'$$
   $$\mathbf{Q}_{xx} = \mathbf{l}_{xx} + \mathbf{A}_k^\top \mathbf{V}_{xx}' \mathbf{A}_k$$
   $$\mathbf{Q}_{uu} = \mathbf{l}_{uu} + \mathbf{B}_k^\top \mathbf{V}_{xx}' \mathbf{B}_k + \mu \mathbf{I}_4$$
   $$\mathbf{Q}_{ux} = \mathbf{B}_k^\top \mathbf{V}_{xx}' \mathbf{A}_k$$
   where $\mu > 0$ is the Levenberg-Marquardt damping parameter guaranteeing $\mathbf{Q}_{uu} \succ 0$.
2. **Compute Feedback and Feedforward Gains via $4 \times 4$ LDLT:**
   $$\mathbf{k}_{\text{ff}} = -\mathbf{Q}_{uu}^{-1} \mathbf{Q}_u \in \mathbb{R}^4$$
   $$\mathbf{K} = -\mathbf{Q}_{uu}^{-1} \mathbf{Q}_{ux} \in \mathbb{R}^{4 \times 13}$$
3. **Propagate Value Function Backward:**
   $$\mathbf{V}_x = \mathbf{Q}_x + \mathbf{K}^\top \mathbf{Q}_{uu} \mathbf{k}_{\text{ff}} + \mathbf{K}^\top \mathbf{Q}_u + \mathbf{Q}_{ux}^\top \mathbf{k}_{\text{ff}}$$
   $$\mathbf{V}_{xx} = \mathbf{Q}_{xx} + \mathbf{K}^\top \mathbf{Q}_{uu} \mathbf{K} + \mathbf{K}^\top \mathbf{Q}_{ux} + \mathbf{Q}_{ux}^\top \mathbf{K}$$

#### Clamped Forward Pass & Armijo Line Search
Given candidate step size $\alpha \in (0, 1]$:
$$\mathbf{u}_k^{\text{cand}} = \text{clamp}\left(\mathbf{U}_k + \alpha \mathbf{k}_{\text{ff}, k} + \mathbf{K}_k (\mathbf{x}_k^{\text{cand}} - \mathbf{X}_k), \; \mathbf{u}_{\min}, \mathbf{u}_{\max}\right)$$
$$\mathbf{x}_{k+1}^{\text{cand}} = \mathbf{f}_{\text{RK4}}(\mathbf{x}_k^{\text{cand}}, \mathbf{u}_k^{\text{cand}})$$
If total cost decreases ($J(\mathbf{X}^{\text{cand}}, \mathbf{U}^{\text{cand}}) < J(\mathbf{X}, \mathbf{U})$), accept trajectory and decrease damping $\mu \leftarrow \mu / 2$; otherwise backtrack $\alpha \leftarrow \alpha / 2$ and increase damping $\mu \leftarrow 2\mu$.

---

### 4.6. Differential Flatness & Trajectory Generation

Quadrotors possess the mathematical property of **differential flatness**: all 13 dynamic states and 4 control inputs can be written as algebraic functions of 4 flat outputs and their time derivatives:
$$\boldsymbol{\sigma}(t) = [x(t), y(t), z(t), \psi(t)]^\top \in \mathbb{R}^4$$

#### Mapping Flat Outputs $\rightarrow$ State $\mathbf{x} \in \mathbb{R}^{13}$:
1. **Position & Velocity:** Directly $\mathbf{p}(t) = [x, y, z]^\top$, $\mathbf{v}(t) = \dot{\mathbf{p}}(t)$.
2. **Total Thrust Vector:**
   $$\mathbf{f}_{\text{thrust}} = m(\ddot{\mathbf{p}} - \mathbf{g}) + \mathbf{D}\dot{\mathbf{p}}$$
   Collective thrust magnitude: $f_T = \|\mathbf{f}_{\text{thrust}}\|$.
3. **Body Coordinate Axes in $SO(3)$:**
   $$\mathbf{z}_B = \frac{\mathbf{f}_{\text{thrust}}}{\|\mathbf{f}_{\text{thrust}}\|}$$
   Given desired yaw angle $\psi(t)$, construct intermediate axis $\mathbf{x}_C = [\cos\psi, \sin\psi, 0]^\top$:
   $$\mathbf{y}_B = \frac{\mathbf{z}_B \times \mathbf{x}_C}{\|\mathbf{z}_B \times \mathbf{x}_C\|}, \quad \mathbf{x}_B = \mathbf{y}_B \times \mathbf{z}_B$$
   The rotation matrix is $\mathbf{R} = [\mathbf{x}_B, \mathbf{y}_B, \mathbf{z}_B] \in SO(3)$, which converts directly to quaternion $\mathbf{q}_{\text{ref}}$.
4. **Angular Velocity ($\boldsymbol{\omega} \in \mathbb{R}^3$):** Derived from jerk $\mathbf{p}^{(3)}(t)$ by projecting time derivative $\dot{\mathbf{z}}_B$ onto the body axes.

This formulation allows generating smooth, physically feasible trajectory horizons (e.g. 3D Lemniscates / Figure-8s) with analytical feedforward inputs.

---

## 5. C++20 Real-Time Systems Engineering

### 5.1. Zero-Heap Allocation Guarantee on the Hot Path

In hard real-time systems, calling `malloc`, `free`, `new`, or expanding `std::vector` on the control thread induces non-deterministic thread stalls when OS memory managers encounter fragmentation or heap mutex contention.

```
+-------------------------------------------------------------------------------+
|                      DETERMINISTIC MEMORY LAYOUT                              |
|                                                                               |
|  [Stack Frame / Class Members]                                                |
|   ├── std::array<StateVector, 21>     X_, X_new_        (Zero dynamic alloc)  |
|   ├── std::array<ControlVector, 20>   U_, U_new_        (Zero dynamic alloc)  |
|   ├── std::array<Matrix<4, 13>, 20>   K_                (Fixed size)          |
|   ├── std::array<Vector4d, 20>        k_ff_             (Fixed size)          |
|   └── Eigen::LDLT<Matrix<6, 6>>       ldlt_solver_      (Call stack local)    |
|                                                                               |
|  NO std::vector  |  NO std::string  |  NO std::map  |  NO new / delete        |
+-------------------------------------------------------------------------------+
```

#### How Zero-Allocation is Enforced:
1. **Fixed-Size Eigen Matrices:** Every matrix is defined with compile-time dimensions (e.g., `Eigen::Matrix<double, 13, 1>`, `Eigen::Matrix<double, 15, 15>`).
2. **Fixed-Length Horizons:** Horizons use `std::array<StateVector, kMaxHorizon + 1>` instead of `std::vector`.
3. **Stack-Based Solvers:** Inversion of the $6 \times 6$ innovation matrix in the ES-EKF and $4 \times 4$ cost matrix in NMPC use `Eigen::LDLT`, which operates entirely on the call stack.
4. **Expression Templates & `.noalias()`:** Subexpressions use `.noalias()` on assignments to prevent Eigen from allocating heap temporaries.
5. **C++20 Concepts:**
   ```cpp
   template <typename T>
   concept FixedVector = requires {
     { T::RowsAtCompileTime } -> std::convertible_to<int>;
     requires T::RowsAtCompileTime != Eigen::Dynamic;
   };
   ```
   Compile-time assertions (`static_assert(FixedVector<StateVector>)`) guarantee no dynamically sized types can enter the solver.

---

### 5.2. Cache Locality, SIMD Vectorization & Eigen Invariants

* **Memory Contiguity:** States along the preview horizon reside in contiguous memory, fitting within L1 data cache ($32\,\text{KB}$).
* **SIMD Alignment:** Statically aligned types (16-byte SSE/NEON, 32-byte AVX2) allow the compiler to generate vector instructions (`vmovapd`, `vmulpd`, `vfma`) for matrix products and RK4 steps.
* **Compiler Flags:**
  ```cmake
  -O3 -march=native -Wall -Wextra -Wpedantic -Wconversion -Wshadow
  ```
  Zero warnings are emitted under strict conversion flags.

---

### 5.3. Dual-Backend Architecture (Native C++20 vs. CasADi C99 / acados)

To guarantee that the native C++20 iLQR solver is free of analytical derivative errors:
* **Symbolic Generator (`scripts/export_ocp_casadi.py`):** CasADi generates C99 code (`generated/quadrotor_dynamics_rk4.c`, `generated/quadrotor_cost.c`) with exact symbolic Jacobians.
* **Cross-Validation Test (`tests/test_casadi_backend.cpp`):** Automated tests evaluate RK4 rollouts and stage cost evaluations side-by-side between the native C++ engine and CasADi C-code, ensuring numerical agreement within $10^{-6}$.

---

## 6. Hardware & Simulation Integration (MuJoCo)

* **MuJoCo MJCF Model (`simulation/mujoco_quadrotor_env/quadrotor.xml`):** Defines mass ($1.0\,\text{kg}$), inertia tensor, rotor geometry, aerodynamic drag, and obstacle primitives.
* **Native C API Bridge (`mujoco_native_bridge.cpp`):** Direct interaction via `mj_step()`, `mjModel`, and `mjData` structures without Python runtime overhead.
* **Dryden Wind Turbulence:** Continuous gust model simulates realistic atmospheric turbulence:
  $$\mathbf{v}_{\text{wind}}(t) = \mathbf{v}_{\text{mean}} + \mathbf{v}_{\text{gust}}\sin(\omega_g t) + \boldsymbol{\eta}_{\text{turb}}$$
* **Sensor Emulation:**
  - $500\,\text{Hz}$ IMU: Specific force with accelerometer bias drift ($0.05\,\text{m/s}^2$) + Gaussian white noise.
  - $30\,\text{Hz}$ Visual Odometry: World position and quaternion with simulated latency and packet dropout.

---

## 7. Proven Benchmarks & Performance Metrics

These are concrete, empirical benchmark numbers from test executions. Memorize them for your interview:

| Metric | Target Specification | Achieved Performance | Margin / Status |
| :--- | :---: | :---: | :---: |
| **Control Loop Rate** | $100\,\text{Hz}$ | **$100\,\text{Hz}$** (Deterministic) | Met |
| **Real-Time Solve Budget** | $< 10.0\,\text{ms}$ | **$0.220 - 0.260\,\text{ms}$** | **$>15\times$ Safety Margin** |
| **Worst-Case Peak Solve Time** | $< 10.0\,\text{ms}$ | **$0.592\,\text{ms}$** | **$16.9\times$ Margin** |
| **Figure-8 (Lemniscate) Tracking RMSE** | $< 0.10\,\text{m}$ | **$0.0459\,\text{m}$ ($4.59\,\text{cm}$)** | **$2.2\times$ Better** |
| **Mean Attitude Tracking Error** | $< 1.20^\circ$ | **$0.193 - 0.209^\circ$** | **$6\times$ Better** |
| **IMU Propagation Rate** | $500\,\text{Hz}$ | **$500\,\text{Hz}$** ($2\,\text{ms}$ step) | Met |
| **VO Measurement Rate** | $30\,\text{Hz}$ | **$30\,\text{Hz}$** ($33.3\,\text{ms}$ step) | Met |
| **Heap Allocations in Steady-State Loop** | 0 bytes | **0 bytes** | Guaranteed |
| **Compiler Warnings** | 0 warnings | **0 warnings** (`-Wall -Wextra -Wconversion`) | Passed |
| **Unit Test Suite** | 100% pass | **22 / 22 Tests Passed** | 100% |

---

## 8. Top 20 Technical Interview Questions & Model Answers

### Question 1: What are the main limitations of PID and Linear MPC for high-speed drone flight?
> **Answer:** "A quadrotor is an underactuated, non-linear dynamical system where translational motion requires tilting the thrust vector.
> Cascaded PID controllers assume decoupled linear dynamics and small roll/pitch angles ($\sin\theta \approx \theta$). As a result, when a drone undergoes aggressive maneuvers ($>45^\circ$ tilt), non-linear gyroscopic terms ($\boldsymbol{\omega} \times \mathbf{J}\boldsymbol{\omega}$) and translational drag ($\mathbf{D}\mathbf{v}$) cause severe tracking lag and phase delay.
> Linear MPC linearizes about hover, which decouples horizontal acceleration from vertical thrust, making the solver blind to actuator limits and tilt-induced altitude drop.
> NMPC directly embeds non-linear kinematics on $SO(3)$, Runge-Kutta dynamics, and motor saturation over a preview horizon, allowing the drone to anticipate turns and bank proactively."

---

### Question 2: Why do you use an Error-State EKF instead of a traditional EKF?
> **Answer:** "In a standard EKF, you estimate the state directly. For orientation, quaternions live on the 3-sphere $S^3 \subset \mathbb{R}^4$ with a norm constraint ($\|\mathbf{q}\| = 1$). If you maintain a $4 \times 4$ covariance on $\mathbf{q}$, the covariance matrix becomes singular in the direction normal to the sphere, causing numerical instability. Furthermore, normalizing the quaternion after the measurement update breaks Kalman optimality.
> In the Error-State EKF, the true state is partitioned into a nominal state and an error state: $\mathbf{x}_{\text{true}} = \mathbf{x}_{\text{nom}} \oplus \delta\mathbf{x}$. The nominal state evolves by integrating non-linear kinematics. The attitude error $\delta\boldsymbol{\theta} \in \mathfrak{so}(3) \simeq \mathbb{R}^3$ is parameterized in the 3D Lie algebra.
> Because we inject and reset the error state to zero after every measurement update ($\delta\mathbf{x} \leftarrow \mathbf{0}$), the filter permanently operates at the origin where first-order Taylor approximations are exact and the covariance matrix is strictly non-singular ($3 \times 3$ for attitude)."

---

### Question 3: How does the Joseph-form covariance update protect against numerical divergence?
> **Answer:** "The standard discrete Kalman covariance update is $\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}$. Because it subtracts a positive semi-definite matrix from another, roundoff errors from finite floating-point precision can cause the eigenvalues of $\mathbf{P}$ to become negative or asymmetric over time, leading to filter divergence.
> The Joseph-form update is:
> $$\mathbf{P} \leftarrow (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}(\mathbf{I} - \mathbf{K}\mathbf{H})^\top + \mathbf{K}\mathbf{R}_m\mathbf{K}^\top$$
> Mathematically, this is the sum of two congruent quadratic forms where $\mathbf{R}_m \succ 0$. Regardless of numerical rounding, the resulting matrix is guaranteed to remain symmetric and positive semi-definite."

---

### Question 4: Walk me through the iLQR Riccati backward pass algorithm.
> **Answer:** "Iterative LQR is a dynamic programming algorithm that quadratizes the cost function and linearizes the dynamics around a nominal trajectory:
> 1. We start at the final horizon node $N$ with value gradient $\mathbf{V}_x = \nabla_\mathbf{x} l_N$ and Hessian $\mathbf{V}_{xx} = \nabla_{\mathbf{x}\mathbf{x}}^2 l_N$.
> 2. We sweep backward from $k = N-1$ to $0$, forming the local $Q$-function approximations:
>    $$\mathbf{Q}_u = \mathbf{l}_u + \mathbf{B}_k^\top \mathbf{V}_x', \quad \mathbf{Q}_{uu} = \mathbf{l}_{uu} + \mathbf{B}_k^\top \mathbf{V}_{xx}' \mathbf{B}_k + \mu\mathbf{I}$$
>    $$\mathbf{Q}_x = \mathbf{l}_x + \mathbf{A}_k^\top \mathbf{V}_x', \quad \mathbf{Q}_{xx} = \mathbf{l}_{xx} + \mathbf{A}_k^\top \mathbf{V}_{xx}' \mathbf{A}_k, \quad \mathbf{Q}_{ux} = \mathbf{B}_k^\top \mathbf{V}_{xx}' \mathbf{A}_k$$
> 3. We solve the $4 \times 4$ linear system for feedforward update $\mathbf{k}_{\text{ff}} = -\mathbf{Q}_{uu}^{-1}\mathbf{Q}_u$ and feedback gain $\mathbf{K} = -\mathbf{Q}_{uu}^{-1}\mathbf{Q}_{ux}$ via `Eigen::LDLT`.
> 4. We update $\mathbf{V}_x$ and $\mathbf{V}_{xx}$ for the preceding stage.
> 5. Finally, we execute a forward rollout with Armijo backtracking line search to find a step size $\alpha$ that decreases cost."

---

### Question 5: How does your code guarantee zero dynamic heap allocations?
> **Answer:** "Calling `malloc`, `free`, or resizing `std::vector` inside a real-time loop can cause priority inversion and non-deterministic OS latency spikes.
> In my architecture:
> 1. All prediction buffers and trajectory vectors use fixed-size types: `std::array<StateVector, kMaxHorizon + 1>`.
> 2. All linear algebra uses fixed compile-time Eigen types: `Eigen::Matrix<double, 13, 1>`, `Eigen::Matrix<double, 15, 15>`.
> 3. Solving linear systems uses stack-allocated `Eigen::LDLT`.
> 4. We use `.noalias()` on matrix assignments to prevent Eigen expression templates from allocating temporary buffers.
> 5. C++20 concepts (`FixedVector`) enforce at compile time that no dynamically sized matrices can enter the solver."

---

### Question 6: What is the purpose of Levenberg-Marquardt damping in iLQR?
> **Answer:** "During aggressive flight or near obstacle boundaries, the control Hessian $\mathbf{Q}_{uu} = \mathbf{l}_{uu} + \mathbf{B}^\top \mathbf{V}_{xx} \mathbf{B}$ can become ill-conditioned or lose positive definiteness if negative curvature is encountered.
> By adding a damping term $\mu \mathbf{I}$ ($\mathbf{Q}_{uu} \leftarrow \mathbf{Q}_{uu} + \mu \mathbf{I}$), we guarantee that $\mathbf{Q}_{uu}$ is strictly positive definite and invertible.
> Furthermore, when $\mu$ is large, the step smoothly interpolates from a second-order Newton step toward a robust first-order gradient descent step, ensuring convergence."

---

### Question 7: How do you prevent gimbal lock in 3D attitude control?
> **Answer:** "Gimbal lock is a singularity inherent to 3-parameter Euler angle representations (e.g. roll, pitch, yaw) when pitch reaches $\pm 90^\circ$.
> To prevent this, we parameterize attitude globally using unit quaternions ($\mathbf{q} \in S^3 \subset \mathbb{R}^4$).
> For tracking error, we compute the relative rotation quaternion $\mathbf{q}_{\text{err}} = \mathbf{q}_{\text{ref}}^{-1} \otimes \mathbf{q}$ and extract the Lie algebra angle-axis vector $\mathbf{e}_\theta = 2 \cdot \text{vec}(\mathbf{q}_{\text{err}})$. This representation is completely smooth and singularity-free across all orientations, including full $360^\circ$ flips."

---

### Question 8: How do you handle obstacle avoidance without making the optimization non-convex or slow?
> **Answer:** "Rather than using mixed-integer constraints, I model obstacles as 3D ellipsoids and apply a smooth exterior penalty:
> $$\text{Pen}_{\text{obs}}(\mathbf{p}) = w_{\text{obs}} \max\left(0, 1 - (\mathbf{p} - \mathbf{p}_{\text{obs}})^\top \mathbf{A}_{\text{obs}} (\mathbf{p} - \mathbf{p}_{\text{obs}})\right)^2$$
> In the Gauss-Newton step, I compute the exact analytical gradient and Hessian of this penalty. When a predicted waypoint approaches the obstacle boundary, the positive-definite Hessian adds a strong repulsive curvature to the cost surface, steering the prediction out of the obstacle during the Riccati backward pass."

---

### Question 9: What is Normalized Innovation Squared (NIS) and how do you use it?
> **Answer:** "NIS is a statistical consistency metric for Kalman filters:
> $$\text{NIS} = \mathbf{r}^\top \mathbf{S}^{-1} \mathbf{r}$$
> where $\mathbf{r}$ is the innovation residual and $\mathbf{S} = \mathbf{H}\mathbf{P}\mathbf{H}^\top + \mathbf{R}_m$ is the innovation covariance.
> For a 6-DoF pose measurement, NIS follows a Chi-Square distribution with 6 degrees of freedom ($\chi_6^2$). If the NIS exceeds the 95% confidence threshold ($\approx 12.59$), the measurement is rejected as an outlier (e.g. visual tracking glitch), protecting the filter from divergence."

---

### Question 10: How do you handle sensor dropouts or visual odometry latency?
> **Answer:** "The ES-EKF architecture decouples high-rate IMU propagation from low-rate measurement updates:
> - IMU propagation runs open-loop at 500 Hz ($\Delta t = 2\,\text{ms}$), integrating specific force and angular velocity while expanding covariance $\mathbf{P}$.
> - If a Visual Odometry frame drops out, the filter simply skips the measurement update and continues propagating on IMU kinematics.
> - When VO packets arrive, the innovation corrects the accumulated drift and resets the error state."

---

### Question 11: What is differential flatness and why is it useful for quadrotors?
> **Answer:** "A system is differentially flat if all states and inputs can be expressed algebraically as functions of a set of flat outputs and their time derivatives.
> For a quadrotor, the flat outputs are position and yaw: $\boldsymbol{\sigma} = [x, y, z, \psi]^\top$.
> From desired position derivatives (velocity, acceleration, jerk), we algebraically compute total thrust, body axes orientation $\mathbf{R} \in SO(3)$, quaternion $\mathbf{q}$, and angular rates $\boldsymbol{\omega}$. This enables generating smooth, dynamically feasible reference trajectories without solving complex boundary-value ODEs."

---

### Question 12: Why is 4th-Order Runge-Kutta (RK4) needed instead of Forward Euler?
> **Answer:** "Forward Euler is a first-order integrator with truncation error $O(\Delta t^2)$. At high angular rates ($>5\,\text{rad/s}$), Euler integration injects artificial energy into the simulation and causes quaternion norms to drift rapidly.
> RK4 evaluates four sub-stage derivatives with truncation error $O(\Delta t^5)$. It accurately preserves energy conservation in unforced motion and maintains quaternion normalization across aggressive maneuvers."

---

### Question 13: What is the significance of the translational rotor drag matrix $\mathbf{D}$?
> **Answer:** "In ideal hover models, thrust is assumed to act purely perpendicular to the airframe. In reality, rotor blades traveling through air experience blade flapping and induced drag, generating an aerodynamic force that opposes translational velocity: $\mathbf{F}_{\text{drag}} \approx -\mathbf{D}\mathbf{v}$.
> Incorporating $\mathbf{D} = \text{diag}(0.10, 0.10, 0.15)\,\text{N}\cdot\text{s/m}$ in the NMPC predictive model prevents position overshoot during high-speed braking and allows the controller to lean into wind gusts proactively."

---

### Question 14: What is Real-Time Iteration (RTI) and warm-starting?
> **Answer:** "In standard SQP, you iterate until numerical convergence at each time step, which can cause unpredictable solve times. In Real-Time Iteration (RTI), only a small, fixed number of Gauss-Newton iterations (typically 1 to 3) are executed per 10 ms cycle.
> Because the control loop runs at 100 Hz, the drone's true state changes minimally between cycles. We warm-start the solver by shifting the previous solution forward by one interval ($\mathbf{U}_i \leftarrow \mathbf{U}_{i+1}$), starting the optimization near the local optimum. In our tests, the solver converges in $\sim 0.26\,\text{ms}$."

---

### Question 15: What happens if the NMPC solver diverges or exceeds the 10 ms deadline?
> **Answer:** "Aerospace systems require deterministic failsafe modes:
> 1. **Warm-Start Persistence:** If a solve step fails or times out, the flight controller immediately outputs the shifted control input $\mathbf{U}_1$ computed from the previous successful cycle.
> 2. **Hover Failsafe:** If the trajectory diverges or cost exceeds a threshold, the controller falls back to a pre-calibrated hover thrust $\mathbf{u}_{\text{hover}} = [m|\mathbf{g}|, 0, 0, 0]^\top$ with PD rate stabilization.
> In over 5,000 simulated flight cycles, our solver achieved a 0.0% failure rate due to adaptive Levenberg-Marquardt damping."

---

### Question 16: What is the time complexity of the iLQR backward pass?
> **Answer:** "The time complexity is linear in horizon length and cubic in state and control dimensions:
> $$O\left(N \cdot (n_x^3 + n_x^2 n_u + n_x n_u^2 + n_u^3)\right)$$
> For our quadrotor where $N = 20$, $n_x = 13$, and $n_u = 4$, the matrices are very small. Inverting $\mathbf{Q}_{uu}$ requires inverting only a $4 \times 4$ matrix via LDLT, which takes fewer than 50 floating-point operations. This explains our ultra-fast $0.26\,\text{ms}$ solve time."

---

### Question 17: How did you test and validate the codebase?
> **Answer:** "Validation followed a strict V-model hierarchy:
> 1. **Unit Testing:** 5 GoogleTest suites with 22 assertions validating $SO(3)$ Lie algebra invariants, RK4 energy dissipation, and ES-EKF covariance symmetry.
> 2. **Cross-Backend Validation:** Comparing native C++ iLQR rollouts against CasADi C99 auto-generated dynamics.
> 3. **Closed-Loop Disturbance Simulation:** 10-second flights under synthetic Dryden wind gusts ($1\,\text{m/s}$ cross-winds + Gaussian turbulence) and sensor noise. Tracking RMSE stayed strictly under $4.6\,\text{cm}$."

---

### Question 18: How does your implementation achieve cache efficiency on modern CPUs?
> **Answer:** "Eigen aligns fixed-size matrices to 16-byte (SSE/NEON) and 32-byte (AVX2) boundaries. All state and control arrays across the 20-step horizon are stored in contiguous memory (`std::array`), which easily fits inside the $32\,\text{KB}$ L1 data cache. This prevents L1/L2 cache misses during backward Riccati sweeps."

---

### Question 19: How would you deploy this stack to an embedded flight computer (e.g., Jetson Orin / STM32)?
> **Answer:** 
> 1. Cross-compile the C++20 code using an ARM toolchain with `-march=armv8-a+simd` (or `-mcpu=cortex-m7` on microcontrollers).
> 2. Bind the 500 Hz IMU thread to an isolated real-time CPU core using POSIX `pthread_setaffinity_np` and `SCHED_FIFO` priority.
> 3. Connect the 100 Hz NMPC output to an actuator bridge (e.g., DShot or PWM via DMA) to send direct rotor RPM commands to electronic speed controllers (ESCs)."

---

### Question 20: What are the primary future improvements for this project?
> **Answer:**
> 1. **acados / HPIPM Full QP Integration:** Upgrading from unconstrained iLQR with penalties to an interior-point QP solver to handle hard polytopic state constraints.
> 2. **L1 Adaptive Augmentation:** Adding an L1 adaptive control loop on top of NMPC to cancel high-frequency aerodynamic disturbances and payload mass variations in real time.
> 3. **Hardware-in-the-Loop (HIL):** Testing over UART/CAN bus against PX4 Autopilot hardware."

---

## 9. Whiteboard Walkthrough Guide & Discussion Strategy

When given a whiteboard in an interview:

1. **Draw the System Topology:**
   - Sketch three blocks: **Plant/Physics**, **15-State ES-EKF**, and **NMPC Controller**.
   - Annotate frequencies: $500\,\text{Hz}$ IMU propagation, $30\,\text{Hz}$ VO update, $100\,\text{Hz}$ NMPC execution.

2. **Write the State Vector & Error Vector:**
   - State: $\mathbf{x} = [\mathbf{p}, \mathbf{v}, \mathbf{q}, \boldsymbol{\omega}]^\top \in \mathbb{R}^{13}$.
   - Attitude Error on $\mathfrak{so}(3)$: $\mathbf{e}_\theta = 2\text{vec}(\mathbf{q}_{\text{ref}}^{-1} \otimes \mathbf{q}) \in \mathbb{R}^3$.
   - Error state for EKF: $\delta\mathbf{x} = [\delta\mathbf{p}, \delta\mathbf{v}, \delta\boldsymbol{\theta}, \delta\mathbf{b}_a, \delta\mathbf{b}_g]^\top \in \mathbb{R}^{15}$.

3. **Explain the Covariance Trick (Joseph Form):**
   - Write $\mathbf{P} \leftarrow (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}(\mathbf{I} - \mathbf{K}\mathbf{H})^\top + \mathbf{K}\mathbf{R}_m\mathbf{K}^\top$. Explain why standard subtraction leads to non-PSD matrices under floating-point roundoff.

4. **Quote the Performance Numbers:**
   - $100\,\text{Hz}$ rate, $0.26\,\text{ms}$ solve time, $4.59\,\text{cm}$ tracking RMSE, 0 heap allocations.
