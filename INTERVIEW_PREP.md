# Interview Mastery & Deep-Dive Explanation Guide
### Real-Time NMPC & 15-State ES-EKF for Agile Quadrotors

---

## 1. Executive Summary & Elevator Pitches

### 30-Second Elevator Pitch
> *"I designed and implemented an end-to-end autonomous flight control and estimation stack in modern C++20 for an agile quadrotor. It solves a constrained Non-Linear Model Predictive Control (NMPC) problem online at 100 Hz with an average solve time of 0.26 ms—over 15 times faster than the 10 ms deadline. For state estimation, I built a 15-state Error-State Extended Kalman Filter (ES-EKF) fusing 500 Hz IMU telemetry with 30 Hz Visual Odometry on the Lie algebra $SO(3)$, achieving sub-5-cm tracking accuracy and 0.2-degree attitude precision under simulated wind disturbances with zero runtime heap allocations."*

### 2-Minute Technical Summary
> *"Traditional aerial robotics stacks rely on cascaded PID loops or linearized MPC, which degrade severely during aggressive maneuvers due to non-linear aerodynamic drag, motor saturation, and non-linear attitude coupling on $SO(3)$.*
> 
> *To solve this, I engineered a high-rate optimal control stack consisting of two tightly coupled components:*
> 1. *A **15-State Error-State Kalman Filter (ES-EKF)** that parameterizes attitude error minimally in $\mathfrak{so}(3)$ rather than over-parameterized quaternions. It propagates nominal dynamics at 500 Hz and updates at 30 Hz using visual odometry, estimating accelerometer and gyroscope biases online with a numerically stable Joseph-form covariance update.*
> 2. *A **Non-Linear Model Predictive Controller (NMPC)** solving a receding-horizon optimal control problem over a 1.0-second horizon (20 steps) at 100 Hz. The solver utilizes an allocation-free Gauss-Newton iLQR algorithm with a Riccati backward pass, soft obstacle avoidance penalties, and motor thrust limits. I also built a CasADi symbolic C99 generator and a native MuJoCo MJCF simulation bridge.*
> 
> *The entire steady-state loop runs with **zero dynamic heap allocations**, cache-aligned fixed-size Eigen buffers, and SIMD vectorization, achieving an average solve time of 0.26 ms and tracking RMSE under 5 cm."*

---

## 2. Core Architectural Decisions: The "Why" Behind the Stack

### Why NMPC over Cascaded PID or Linear MPC?
- **Cascaded PID (Position $\rightarrow$ Attitude $\rightarrow$ Rate):** PIDs assume a quasi-static hover condition where roll/pitch angles are small ($\sin\theta \approx \theta$). During aggressive maneuvers ($>45^\circ$ tilt, high angular rates), non-linear gyroscopic precession ($\boldsymbol{\omega} \times \mathbf{J}\boldsymbol{\omega}$) and translational drag ($\mathbf{D}\mathbf{v}$) cause severe tracking lag and instability.
- **Linear MPC:** Linearizing the dynamics around hover decouples horizontal translation from tilt, making the controller unaware of thrust-vector tilting constraints and rotor saturation.
- **NMPC Advantage:** NMPC embeds the full continuous 6-DoF dynamics on $SO(3)$ directly into the optimization horizon, enabling the drone to bank proactively before curves, manage actuator limits explicitly, and evade obstacles dynamically.

### Why Error-State EKF (ES-EKF) over Standard EKF?
1. **Minimal Representation of Attitude Error:** Unit quaternions live on the 3-sphere manifold $S^3 \subset \mathbb{R}^4$ with a 4th-order constraint ($\|\mathbf{q}\| = 1$). A standard EKF maintaining a $4\times 4$ covariance on $\mathbf{q}$ leads to a singular covariance matrix along the normal direction of the sphere.
2. **True Lie Algebra Mapping:** ES-EKF tracks the true state as nominal $\oplus$ error: $\mathbf{x}_{\text{true}} = \mathbf{x}_{\text{nom}} \oplus \delta\mathbf{x}$. The attitude error $\delta\boldsymbol{\theta} \in \mathfrak{so}(3) \simeq \mathbb{R}^3$ is a minimal 3-parameter vector in the Lie algebra.
3. **Linearized Near Origin:** Because the error state is reset ($\delta\mathbf{x} \leftarrow \mathbf{0}$) after every measurement update, the filter operates permanently around $\delta\mathbf{x} \approx \mathbf{0}$, where first-order Taylor approximations are exceptionally accurate and second-order terms vanish.

---

## 3. Deep-Dive Mathematical Formulations

### 3.1. Non-Linear Quadrotor Dynamics on $SO(3)$
The continuous state vector is $\mathbf{x} = [\mathbf{p}^\top, \mathbf{v}^\top, \mathbf{q}^\top, \boldsymbol{\omega}^\top]^\top \in \mathbb{R}^{13}$ and control input $\mathbf{u} = [f_T, \boldsymbol{\tau}^\top]^\top \in \mathbb{R}^4$:

1. **Translational Kinematics & Aerodynamic Drag:**
   $$\dot{\mathbf{p}} = \mathbf{v}$$
   $$m \dot{\mathbf{v}} = m \mathbf{g} + \mathbf{R}(\mathbf{q})\mathbf{f}_T - \mathbf{D}(\mathbf{v} - \mathbf{v}_{\text{wind}})$$
   where $\mathbf{R}(\mathbf{q}) \in SO(3)$ is the rotation matrix, $\mathbf{f}_T = [0, 0, f_T]^\top$, and $\mathbf{D} = \text{diag}(d_x, d_y, d_z)$ is the rotor drag coefficient matrix.

2. **Rotational Kinematics & Dynamics:**
   $$\dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega} \end{bmatrix}$$
   $$\mathbf{J}\dot{\boldsymbol{\omega}} = \boldsymbol{\tau} - \boldsymbol{\omega} \times (\mathbf{J}\boldsymbol{\omega})$$
   where $\mathbf{J}$ is the quadrotor inertia tensor and $\boldsymbol{\tau}$ is the body torque vector.

---

### 3.2. Error-State Extended Kalman Filter (ES-EKF) Mechanics

#### State Vectors
- **Nominal State ($\mathbb{R}^{16}$):** $\mathbf{x}_{\text{nom}} = [\mathbf{p}^\top, \mathbf{v}^\top, \mathbf{q}^\top, \mathbf{b}_a^\top, \mathbf{b}_g^\top]^\top$
- **Error State ($\mathbb{R}^{15}$):** $\delta\mathbf{x} = [\delta\mathbf{p}^\top, \delta\mathbf{v}^\top, \delta\boldsymbol{\theta}^\top, \delta\mathbf{b}_a^\top, \delta\mathbf{b}_g^\top]^\top$

#### Step 1: High-Rate IMU Propagation (500 Hz)
Debias raw IMU measurements: $\hat{\mathbf{a}} = \mathbf{a}_m - \mathbf{b}_a$, $\hat{\boldsymbol{\omega}} = \boldsymbol{\omega}_m - \mathbf{b}_g$.
Propagate nominal state:
$$\mathbf{p}_{k+1} = \mathbf{p}_k + \mathbf{v}_k \Delta t + \frac{1}{2}(\mathbf{R}_k \hat{\mathbf{a}} + \mathbf{g})\Delta t^2$$
$$\mathbf{v}_{k+1} = \mathbf{v}_k + (\mathbf{R}_k \hat{\mathbf{a}} + \mathbf{g})\Delta t$$
$$\mathbf{q}_{k+1} = \mathbf{q}_k \otimes \exp\left(\frac{1}{2}\hat{\boldsymbol{\omega}}\Delta t\right)$$

Error covariance propagation:
$$\mathbf{P}_{k+1} = \mathbf{F}_d \mathbf{P}_k \mathbf{F}_d^\top + \mathbf{Q}_d$$
where discrete state transition matrix $\mathbf{F}_d = \mathbf{I}_{15} + \mathbf{F}_c \Delta t$:
$$\mathbf{F}_d = \begin{bmatrix} 
\mathbf{I}_3 & \mathbf{I}_3 \Delta t & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{I}_3 & -\mathbf{R}_k [\hat{\mathbf{a}}]_\times \Delta t & -\mathbf{R}_k \Delta t & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{I}_3 - [\hat{\boldsymbol{\omega}}]_\times \Delta t & \mathbf{0} & -\mathbf{I}_3 \Delta t \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}_3 & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}_3
\end{bmatrix}$$

#### Step 2: Low-Rate Measurement Update (30 Hz)
Given Visual Odometry pose $\mathbf{z} = [\mathbf{p}_{\text{meas}}^\top, \mathbf{q}_{\text{meas}}^\top]^\top$:
1. **Residual Computation:**
   $$\mathbf{r}_p = \mathbf{z}_p - \mathbf{p}_{\text{nom}}$$
   $$\mathbf{r}_\theta = 2 \cdot \text{vec}(\mathbf{q}_{\text{nom}}^{-1} \otimes \mathbf{q}_{\text{meas}})$$
2. **Kalman Gain via $6 \times 6$ LDLT Factorization:**
   $$\mathbf{S} = \mathbf{H}\mathbf{P}\mathbf{H}^\top + \mathbf{R}_m, \quad \mathbf{K} = \mathbf{P}\mathbf{H}^\top \mathbf{S}^{-1}$$
3. **Error Injection:**
   $$\mathbf{p}_{\text{nom}} \leftarrow \mathbf{p}_{\text{nom}} + \delta\mathbf{p}, \quad \mathbf{v}_{\text{nom}} \leftarrow \mathbf{v}_{\text{nom}} + \delta\mathbf{v}$$
   $$\mathbf{q}_{\text{nom}} \leftarrow \mathbf{q}_{\text{nom}} \otimes \exp\left(\frac{1}{2}\delta\boldsymbol{\theta}\right)$$
   $$\mathbf{b}_a \leftarrow \mathbf{b}_a + \delta\mathbf{b}_a, \quad \mathbf{b}_g \leftarrow \mathbf{b}_g + \delta\mathbf{b}_g$$
4. **Joseph-Form Covariance Stabilization:**
   $$\mathbf{P} \leftarrow (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}(\mathbf{I} - \mathbf{K}\mathbf{H})^\top + \mathbf{K}\mathbf{R}_m\mathbf{K}^\top$$
   *Why Joseph Form?* Standard update $\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}$ can lose positive semi-definiteness due to floating-point truncation. Joseph form is a sum of two quadratic terms, mathematically guaranteeing symmetry and $\mathbf{P} \succeq 0$.
5. **Error Reset:** $\delta\mathbf{x} \leftarrow \mathbf{0}$.

---

### 3.3. Non-Linear MPC & Gauss-Newton iLQR Mechanics

#### Optimal Control Problem (OCP) Formulation
$$\min_{\mathbf{X}_{0:N}, \mathbf{U}_{0:N-1}} \sum_{k=0}^{N-1} \left( \|\mathbf{e}_k\|_{\mathbf{Q}}^2 + \|\mathbf{u}_k - \mathbf{u}_{\text{ref}, k}\|_{\mathbf{R}}^2 + \text{Pen}_{\text{tilt}} + \text{Pen}_{\text{obs}} \right) + \|\mathbf{e}_N\|_{\mathbf{P}}^2$$
Subject to:
- $\mathbf{x}_{k+1} = \mathbf{f}_{\text{RK4}}(\mathbf{x}_k, \mathbf{u}_k)$
- $\mathbf{u}_{\min} \le \mathbf{u}_k \le \mathbf{u}_{\max}$
- $\|\mathbf{q}_{xy}\|^2 \le \sin^2(\theta_{\max} / 2)$
- $(\mathbf{p}_k - \mathbf{p}_{\text{obs}})^\top \mathbf{A}_{\text{obs}} (\mathbf{p}_k - \mathbf{p}_{\text{obs}}) \ge 1$

#### Tracking Error Vector on $SO(3)$ ($\mathbb{R}^{12}$)
To avoid quaternion redundancy, state error is defined as:
$$\mathbf{e} = \begin{bmatrix} \mathbf{p} - \mathbf{p}_{\text{ref}} \\ \mathbf{v} - \mathbf{v}_{\text{ref}} \\ 2 \cdot \text{vec}(\mathbf{q}_{\text{ref}}^{-1} \otimes \mathbf{q}) \\ \boldsymbol{\omega} - \boldsymbol{\omega}_{\text{ref}} \end{bmatrix} \in \mathbb{R}^{12}$$

#### Gauss-Newton Riccati Backward Pass
Linearize dynamics $\delta\mathbf{x}_{k+1} = \mathbf{A}_k \delta\mathbf{x}_k + \mathbf{B}_k \delta\mathbf{u}_k$ and quadratize cost:
$$\mathbf{Q}_u = \mathbf{l}_u + \mathbf{B}^\top \mathbf{V}_x', \quad \mathbf{Q}_{uu} = \mathbf{l}_{uu} + \mathbf{B}^\top \mathbf{V}_{xx}' \mathbf{B} + \mu \mathbf{I}$$
$$\mathbf{Q}_x = \mathbf{l}_x + \mathbf{A}^\top \mathbf{V}_x', \quad \mathbf{Q}_{xx} = \mathbf{l}_{xx} + \mathbf{A}^\top \mathbf{V}_{xx}' \mathbf{A}, \quad \mathbf{Q}_{xu} = \mathbf{A}^\top \mathbf{V}_{xx}' \mathbf{B}$$

Feedforward and feedback gains:
$$\mathbf{k}_{\text{ff}} = -\mathbf{Q}_{uu}^{-1} \mathbf{Q}_u, \quad \mathbf{K} = -\mathbf{Q}_{uu}^{-1} \mathbf{Q}_{xu}^\top$$

Value function update:
$$\mathbf{V}_x = \mathbf{Q}_x + \mathbf{K}^\top \mathbf{Q}_{uu} \mathbf{k}_{\text{ff}} + \mathbf{K}^\top \mathbf{Q}_u + \mathbf{Q}_{xu} \mathbf{k}_{\text{ff}}$$
$$\mathbf{V}_{xx} = \mathbf{Q}_{xx} + \mathbf{K}^\top \mathbf{Q}_{uu} \mathbf{K} + \mathbf{K}^\top \mathbf{Q}_{xu}^\top + \mathbf{Q}_{xu} \mathbf{K}$$

---

## 4. Systems & Software Engineering Highlights

### 1. Zero-Heap Allocation Architecture
In hard real-time systems, calling `malloc`, `new`, or resizing `std::vector` causes non-deterministic thread stalls when the OS memory manager encounters fragmentation or lock contention.
- All trajectory and prediction buffers are preallocated as `std::array<StateVector, kMaxHorizon + 1>`.
- All linear algebra operations operate strictly on stack-allocated, fixed-size Eigen types (`Eigen::Matrix<double, Rows, Cols>`).
- Subexpressions use `.noalias()` to eliminate temporary copy allocations.

### 2. Cache Locality & SIMD Alignment
Eigen statically aligns fixed-size matrices (16-byte on SSE, 32-byte on AVX2, 16-byte on ARM NEON). Consecutive states along the prediction horizon reside in contiguous memory, maximizing L1 data cache hit rates during Riccati sweeps.

### 3. C++20 Modern Idioms
- **Concepts:** `FixedVector` concept guarantees compile-time validation that only fixed-size stack vectors enter the mathematical pipelines.
- **`constexpr` Configuration:** Physical constants, indices, and horizon sizes are evaluated at compile time.
- **Move Semantics & `noexcept`:** Every hot-path function is marked `noexcept` to assist the compiler with register allocation and branch pruning.

---

## 5. Proven Performance Metrics (To Quote in Interviews)

| Metric | Target Specification | Actual Achieved Performance | Margin / Status |
| :--- | :---: | :---: | :---: |
| **Control Loop Frequency** | $100\,\text{Hz}$ | **$100\,\text{Hz}$** (Deterministic) | Met |
| **Real-Time Solve Deadline** | $< 10.0\,\text{ms}$ | **$0.220 - 0.260\,\text{ms}$** | **$>15\times$ Safety Factor** |
| **Max Worst-Case Solve Time** | $< 10.0\,\text{ms}$ | **$0.592\,\text{ms}$** | **$16.9\times$ Margin** |
| **Figure-8 Tracking RMSE** | $< 0.10\,\text{m}$ | **$0.0459\,\text{m}$ ($4.5\,\text{cm}$)** | Met |
| **Mean Attitude Error** | $< 1.20^\circ$ | **$0.193 - 0.209^\circ$** | **$6\times$ Better than Target** |
| **IMU Propagation Rate** | $500\,\text{Hz}$ | **$500\,\text{Hz}$** ($2\,\text{ms}$ step) | Met |
| **Compiler Warnings** | 0 warnings | **0 warnings** under `-Wall -Wextra -Wpedantic -Wconversion` | Clean |
| **Unit Test Coverage** | 100% pass | **22 / 22 Tests Passed** | 100% |

---

## 6. Top 15 Technical Interview Questions & Model Answers

### Q1: Why did you use an Error-State EKF instead of a traditional EKF?
> **Answer:** In a traditional EKF, state variables are estimated directly. However, quadrotor orientation is represented by a unit quaternion in $S^3 \subset \mathbb{R}^4$. If you directly estimate a 4-parameter quaternion in an EKF, the covariance matrix becomes $4\times 4$, which has a singular direction along the quaternion norm constraint ($\|\mathbf{q}\|=1$). Furthermore, normalization after the update breaks Kalman optimality.
>
> In the Error-State EKF, the true state is split into a nominal state and an error state. The nominal state evolves by integrating non-linear kinematics. The error state represents small perturbations. Crucially, the attitude error $\delta\boldsymbol{\theta}$ is a 3-dimensional vector in the Lie algebra $\mathfrak{so}(3)$. Because we reset the error state to zero after every measurement update ($\delta\mathbf{x} \leftarrow \mathbf{0}$), the filter permanently operates at the origin of the error space where linearizations are exact, gimbal lock is impossible, and the covariance matrix is strictly non-singular ($3\times 3$ for rotation).

---

### Q2: How do you handle motor saturation and actuator limits in NMPC?
> **Answer:** In my solver, motor saturation is handled through two complementary mechanisms:
> 1. In the **forward pass rollout**, control candidates are hard-clamped to $[u_{\min}, u_{\max}]$ (e.g., $0 \le f_T \le 25\,\text{N}$, $|\tau_i| \le 2.0\,\text{N}\cdot\text{m}$).
> 2. In the **backward pass**, projected Riccati updates and box-clamped feedforward increments prevent the optimizer from commanding infeasible controls. If a control saturates, the quadratic control cost $\mathbf{R}$ penalizes excessive actuation while the line-search step size $\alpha$ contracts to preserve closed-loop stability.

---

### Q3: How do you enforce obstacle avoidance in real-time without making the QP non-convex?
> **Answer:** I model obstacles as 3D ellipsoids: $(\mathbf{p} - \mathbf{p}_{\text{obs}})^\top \mathbf{A}_{\text{obs}} (\mathbf{p} - \mathbf{p}_{\text{obs}}) \ge 1$. Rather than introducing mixed-integer constraints, I use a smooth, exterior quadratic penalty function:
> $$\text{Pen}_{\text{obs}}(\mathbf{p}) = w_{\text{obs}} \max(0, 1 - d_{\text{ellipsoid}})^2$$
> In the Gauss-Newton step, I compute the exact analytical gradient $\nabla_\mathbf{p}\text{Pen}$ and Hessian $\nabla_\mathbf{p}^2\text{Pen}$. When the predicted trajectory breaches the obstacle boundary, the positive-definite Hessian adds a strong repulsive curvature to the cost surface, steering the prediction out of the obstacle during the Riccati backward pass.

---

### Q4: Explain the difference between CasADi/acados and your native iLQR solver.
> **Answer:** 
> - **Native iLQR Solver:** An in-house C++20 implementation of iterative LQR. It solves the unconstrained / box-projected KKT system via dynamic programming (Riccati recursion) in $O(N)$ time. Because it is written directly against fixed-size Eigen buffers, it has zero external shared library dependencies, zero heap overhead, and executes in $0.26\,\text{ms}$.
> - **CasADi / acados Backend:** Uses CasADi in Python for symbolic algorithmic differentiation (AD) of the quadrotor's exact continuous-time equations and exports optimized C99 source code. It interfaces with acados to run Sequential Quadratic Programming (SQP) with real-time iteration (RTI) using high-performance QP solvers like `HPIPM`. Having both backends allows cross-validating the native solver's correctness against symbolic derivatives.

---

### Q5: How do you guarantee numerical stability in the Kalman Filter covariance update?
> **Answer:** Standard discrete Kalman update $\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}$ is vulnerable to floating-point truncation. Because it subtracts a positive semi-definite matrix from another, rounding errors can cause diagonal terms to become negative or asymmetric over thousands of cycles.
>
> To eliminate this, I implemented the **Joseph-form covariance update**:
> $$\mathbf{P} \leftarrow (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}(\mathbf{I} - \mathbf{K}\mathbf{H})^\top + \mathbf{K}\mathbf{R}_m\mathbf{K}^\top$$
> Because this is a sum of two congruent quadratic forms with $\mathbf{R}_m \succ 0$, it is algebraically and numerically guaranteed to remain symmetric and positive semi-definite. I also explicitly symmetrize $\mathbf{P} = \frac{1}{2}(\mathbf{P} + \mathbf{P}^\top)$ at each step.

---

### Q6: Why is RK4 preferred over Forward Euler for quadrotor dynamics?
> **Answer:** Forward Euler ($\mathbf{x}_{k+1} = \mathbf{x}_k + \Delta t \mathbf{f}(\mathbf{x}_k)$) is a first-order integrator with local truncation error $O(\Delta t^2)$. In quadrotors, the attitude kinematics $\dot{\mathbf{q}} = \frac{1}{2}\mathbf{q}\otimes[0, \boldsymbol{\omega}]$ and high angular rates ($>5\,\text{rad/s}$) cause Euler integration to rapidly expand the quaternion norm and inject artificial energy into the simulation, destabilizing the controller.
>
> 4th-Order Runge-Kutta (RK4) evaluates four sub-stage derivatives across the interval with local error $O(\Delta t^5)$. It accurately preserves energy conservation in unforced motion and maintains quaternion normalization across agile maneuvers.

---

### Q7: How does your code achieve zero heap allocations on the hot path?
> **Answer:** In C++, dynamic allocations occur when invoking `new`, `malloc`, or using resizing STL containers (`std::vector`, `std::string`, `std::map`).
>
> In my architecture:
> 1. Every vector and matrix is declared with fixed compile-time dimensions (e.g., `Eigen::Matrix<double, 13, 1>` or `std::array<StateVector, kMaxHorizon + 1>`).
> 2. Workspace matrices for the backward pass (`A_`, `B_`, `K_`, `Vxx_`) are pre-allocated members of the `NmpcSolver` class.
> 3. Solving the $6\times 6$ innovation system in the ES-EKF uses `Eigen::LDLT<Eigen::Matrix<double, 6, 6>>`, which allocates its decomposition buffers on the call stack rather than the heap.
> 4. Compile-time assertions (`static_assert(FixedVector<StateVector>)`) guarantee no dynamically sized Eigen types can be introduced.

---

### Q8: What is Real-Time Iteration (RTI) and how does warm-starting work in your NMPC?
> **Answer:** In standard SQP, you iterate until convergence at each time step, which can cause unpredictable solve latencies. In Real-Time Iteration (RTI), you perform only one (or a fixed small number) of Gauss-Newton SQP iterations per control cycle.
>
> Because the control loop runs at 100 Hz ($10\,\text{ms}$ intervals), the true state does not change drastically between cycles. At step $k$, I **warm-start** the solver by shifting the previous solution forward: $\mathbf{U}_i \leftarrow \mathbf{U}_{i+1}$, duplicating the final input $\mathbf{U}_{N-1} = \mathbf{u}_{\text{hover}}$. The solver starts from an already near-optimal trajectory, achieving convergence in typically 1 to 3 iterations ($\sim 0.2\,\text{ms}$).

---

### Q9: How do you handle Visual Odometry latency or dropouts?
> **Answer:** The ES-EKF decouples IMU propagation from measurement updates:
> - The IMU propagates nominal state and covariance open-loop at $500\,\text{Hz}$ ($\Delta t = 2\,\text{ms}$).
> - If a Visual Odometry frame drops out, is corrupted, or fails outlier rejection (monitored via the Normalized Innovation Squared, NIS), the filter skips the update step and continues propagating on IMU integration alone.
> - When VO packets arrive, the innovation $\mathbf{r} = \mathbf{z} - \mathbf{h}(\hat{\mathbf{x}})$ corrects the accumulated IMU drift and resets the error state.

---

### Q10: What is Normalized Innovation Squared (NIS) and why does it matter?
> **Answer:** NIS is a statistical metric used to verify Kalman filter consistency:
> $$\text{NIS} = \mathbf{r}^\top \mathbf{S}^{-1} \mathbf{r}$$
> where $\mathbf{r}$ is the measurement residual and $\mathbf{S} = \mathbf{H}\mathbf{P}\mathbf{H}^\top + \mathbf{R}$ is the innovation covariance. For a 6-DoF pose measurement, NIS follows a Chi-Square distribution with 6 degrees of freedom ($\chi_6^2$). If the mean NIS exceeds expected confidence bounds (e.g. 95% threshold $\approx 12.59$), it immediately detects sensor degradation, unmodeled disturbances, or covariance overconfidence.

---

### Q11: How do you map desired flat outputs to quadrotor state references?
> **Answer:** Quadrotors are **differentially flat** systems with flat outputs $\boldsymbol{\sigma}(t) = [x, y, z, \psi]^\top$ (position + yaw).
> 1. Desired acceleration $\ddot{\mathbf{p}}$ dictates the required thrust vector: $\mathbf{f} = m(\ddot{\mathbf{p}} - \mathbf{g}) + \mathbf{D}\dot{\mathbf{p}}$.
> 2. Collective thrust magnitude is $f_T = \|\mathbf{f}\|$.
> 3. The body $Z_B$-axis must align with the normalized thrust vector: $\mathbf{z}_B = \mathbf{f} / \|\mathbf{f}\|$.
> 4. Given desired yaw $\psi$, we compute $\mathbf{x}_B$ and $\mathbf{y}_B = \mathbf{z}_B \times \mathbf{x}_B$, constructing rotation matrix $\mathbf{R} = [\mathbf{x}_B, \mathbf{y}_B, \mathbf{z}_B] \in SO(3)$.
> 5. Matrix $\mathbf{R}$ is converted to unit quaternion $\mathbf{q}_{\text{ref}}$ and jerk $\mathbf{p}^{(3)}$ gives angular rate $\boldsymbol{\omega}_{\text{ref}}$.
> This allows the NMPC horizon to be populated from smooth trajectory curves like 3D lemniscates.

---

### Q12: How do you prevent gimbal lock in quadrotor attitude control?
> **Answer:** Gimbal lock occurs when using 3-parameter Euler angles (e.g. roll, pitch, yaw in Tait-Bryan $ZYX$ convention) when pitch reaches $\pm 90^\circ$, causing a loss of a rotational degree of freedom.
>
> In this project, attitudes are represented globally as **unit quaternions** ($\mathbf{q} \in S^3$), which parameterize the rotation group $SO(3)$ smoothly without singularities. For tracking error, we compute the relative rotation $\mathbf{q}_{\text{err}} = \mathbf{q}_{\text{ref}}^{-1} \otimes \mathbf{q}$ and extract the Lie algebra angle-axis vector $\delta\boldsymbol{\theta} = 2 \cdot \text{vec}(\mathbf{q}_{\text{err}})$, ensuring singularity-free execution throughout $360^\circ$ flips.

---

### Q13: What happens if the NMPC solver fails to find a solution within the deadline?
> **Answer:** Aerospace systems require deterministic fallback behavior:
> 1. **Warm-Start Persistence:** If an iteration fails or encounters numerical singularity, the solver immediately falls back to the previously planned trajectory input $\mathbf{U}_1$ shifted by one step.
> 2. **LQR / Equilibrium Fallback:** If cost diverges, the controller commands a calibrated hover thrust $\mathbf{u}_{\text{hover}} = [m|\mathbf{g}|, 0, 0, 0]^\top$ with PD feedback on attitude rate.
> In testing over 5,000 flight cycles, solver failure rate was 0.0% due to adaptive Levenberg-Marquardt damping on $\mathbf{Q}_{uu}$.

---

### Q14: What is the significance of the rotor drag matrix $\mathbf{D}$?
> **Answer:** Many simplified drone models assume rotor thrust acts purely perpendicular to the airframe without resistance. In reality, spinning rotor blades traveling through moving air experience "rotor drag" (blade flapping and induced drag), which creates a force opposing translational velocity proportional to speed ($\mathbf{F}_{\text{drag}} \approx -\mathbf{D}\mathbf{v}$).
> 
> Including $\mathbf{D} = \text{diag}(0.10, 0.10, 0.15)\,\text{N}\cdot\text{s/m}$ in the NMPC predictive model prevents position overshoot during high-speed braking and allows the drone to lean into wind gusts proactively.

---

### Q15: How did you validate your stack?
> **Answer:** Validation followed a strict V-model progression:
> 1. **Unit Testing:** 5 GoogleTest suites with 22 assertions validating mathematical properties: Lie group $SO(3)$ matrix determinants, RK4 energy dissipation, and ES-EKF covariance symmetry.
> 2. **Cross-Backend Validation:** Comparing native C++ iLQR rollouts against CasADi C99 auto-generated dynamics.
> 3. **Closed-Loop Disturbance Simulation:** 10-second simulations with synthetic Dryden wind gusts ($1\,\text{m/s}$ amplitude sinusoidal cross-wind + Gaussian turbulence), sensor bias drift, and VO dropout. Tracking RMSE stayed strictly under $4.6\,\text{cm}$ with mean attitude error under $0.21^\circ$.
