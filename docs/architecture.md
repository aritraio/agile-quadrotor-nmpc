# Quadrotor NMPC & ES-EKF Architecture Documentation

## 1. System Overview & High-Level Topology

This project implements a hard real-time autonomous aerial robotics software stack designed for high-agility quadrotor flight. It tightly couples a **15-State Error-State Extended Kalman Filter (ES-EKF)** running at high frequency with a **Non-Linear Model Predictive Controller (NMPC)** executing at $100\,\text{Hz}$.

```
 +-------------------------------------------------------------------------+
 |                               PLANT                                     |
 |  - 6-DoF Quadrotor on SO(3) with Rotor Drag Matrix D                    |
 |  - MuJoCo MJCF / Standalone RK4 (500 Hz Substeps)                       |
 |  - Dryden Wind Gusts + External Aerodynamic Disturbances                |
 +-------------------+---------------------------------+-------------------+
                     |                                 |
         IMU (500 Hz)| Specific Force                  | VO / Mocap (30 Hz)
         f_b = a - g | Angular Rate                    | Position + Quat
                     v                                 v
 +-------------------------------------------------------------------------+
 |                     STATE ESTIMATION: 15-STATE ES-EKF                   |
 |                                                                         |
 |  High-Rate Propagation (500 Hz):                                        |
 |    x_nom_{k+1} = f_kinematics(x_nom_k, u_imu)                           |
 |    P_{k+1} = F_d P_k F_d^T + Q_d                                        |
 |                                                                         |
 |  Low-Rate Update (30 Hz):                                               |
 |    Residual: r_p = z_p - p_nom,  r_theta = 2 vec(q_nom^{-1} * q_meas)   |
 |    Gain: K = P H^T (H P H^T + R)^{-1} via LDLT                          |
 |    Error Injection: x_nom = x_nom (+) dx,  Reset: dx <- 0               |
 |    Joseph Form: P = (I - KH) P (I - KH)^T + K R K^T                     |
 +-------------------------------------+-----------------------------------+
                                       |
                   Estimated Quad State| x_hat = [p, v, q, omega] in R^13
                   (Zero Heap Copy)    v
 +-------------------------------------------------------------------------+
 |                   TRAJECTORY GENERATION & REFERENCE                     |
 |                                                                         |
 |  Differential Flatness Mapping:                                         |
 |    Flat outputs: sigma(t) = [x, y, z, yaw]^T                            |
 |    Derivatives: dot(sigma), ddot(sigma) -> Orientation & Thrust         |
 |    Output: Horizon references X_ref[0..N], U_ref[0..N-1]                |
 +-------------------------------------+-----------------------------------+
                                       |
                   Horizon References  |
                   X_ref, U_ref        v
 +-------------------------------------------------------------------------+
 |                  NMPC CONTROLLER (100 Hz, <0.3 ms)                      |
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
 |    - RTI Warm-Starting across control cycles                            |
 |    - Dual-Backend: CasADi/acados C99 generated export                   |
 +-------------------------------------+-----------------------------------+
                                       | Optimal Actuation u_cmd = [f_T, tau]^T
                                       v
                                [ Back to Plant ]
```

---

## 2. Coordinate Frames & Conventions

The system defines three primary Cartesian coordinate frames complying with aerospace and robotics conventions:

1. **World Inertial Frame $\mathcal{W}$**:
   - Right-handed Cartesian frame $\{X_W, Y_W, Z_W\}$.
   - $Z_W$ points upward (opposite to gravity: $\mathbf{g} = [0, 0, -9.81]^\top\,\text{m/s}^2$).
   - Used for position $\mathbf{p} \in \mathbb{R}^3$ and inertial velocity $\mathbf{v} \in \mathbb{R}^3$.

2. **Body Frame $\mathcal{B}$**:
   - Rigidly affixed to the quadrotor's center of mass.
   - $X_B$ points along the forward structural arm / camera optical forward axis.
   - $Y_B$ points toward the port (left) arm.
   - $Z_B$ points upward along the collective thrust axis: $\mathbf{f}_{\text{thrust}} = [0, 0, f_T]^\top$.

3. **Attitude Parameterization**:
   - Unit quaternion $\mathbf{q} = [q_w, q_x, q_y, q_z]^\top = [q_w, \mathbf{q}_v^\top]^\top$ ($w$-first storage convention).
   - Represents the passive rotation mapping vectors from $\mathcal{B}$ to $\mathcal{W}$: $\mathbf{v}_W = \mathbf{R}(\mathbf{q}) \mathbf{v}_B$.
   - $\mathbf{R}(\mathbf{q}) \in SO(3)$ is computed via Rodrigues-equivalent quaternion transformation:
     $$\mathbf{R}(\mathbf{q}) = (q_w^2 - \|\mathbf{q}_v\|^2)\mathbf{I}_3 + 2 \mathbf{q}_v \mathbf{q}_v^\top + 2 q_w [\mathbf{q}_v]_\times$$

---

## 3. Mathematical State Representations

### Full Dynamics State ($\mathbb{R}^{13}$)
Used by the dynamic plant, trajectory generator, and NMPC solver:
$$\mathbf{x} = \begin{bmatrix} \mathbf{p} \\ \mathbf{v} \\ \mathbf{q} \\ \boldsymbol{\omega} \end{bmatrix} \in \mathbb{R}^{13}, \quad \mathbf{u} = \begin{bmatrix} f_T \\ \boldsymbol{\tau} \end{bmatrix} \in \mathbb{R}^4$$

- $\mathbf{p} \in \mathbb{R}^3$: Position in $\mathcal{W}$.
- $\mathbf{v} \in \mathbb{R}^3$: Linear velocity in $\mathcal{W}$.
- $\mathbf{q} \in S^3 \subset \mathbb{R}^4$: Unit quaternion attitude ($\|\mathbf{q}\| = 1$).
- $\boldsymbol{\omega} \in \mathbb{R}^3$: Body angular velocity in $\mathcal{B}$.
- $f_T \in \mathbb{R}$: Collective thrust $[0, u_{\max}]$.
- $\boldsymbol{\tau} \in \mathbb{R}^3$: Body torque vector generated by differential rotor speeds.

### Estimator Nominal State ($\mathbb{R}^{16}$)
Used by the ES-EKF for high-rate open-loop integration:
$$\mathbf{x}_{\text{nom}} = \begin{bmatrix} \mathbf{p} \\ \mathbf{v} \\ \mathbf{q} \\ \mathbf{b}_a \\ \mathbf{b}_g \end{bmatrix} \in \mathbb{R}^{16}$$
- $\mathbf{b}_a \in \mathbb{R}^3$: Accelerometer sensor bias in $\mathcal{B}$.
- $\mathbf{b}_g \in \mathbb{R}^3$: Gyroscope sensor bias in $\mathcal{B}$.

### Estimator Error State ($\mathbb{R}^{15}$)
Minimal Lie algebra parameterization avoiding quaternion covariance singularity:
$$\delta\mathbf{x} = \begin{bmatrix} \delta\mathbf{p} \\ \delta\mathbf{v} \\ \delta\boldsymbol{\theta} \\ \delta\mathbf{b}_a \\ \delta\mathbf{b}_g \end{bmatrix} \in \mathbb{R}^{15}$$
- $\delta\boldsymbol{\theta} \in \mathfrak{so}(3) \simeq \mathbb{R}^3$: Minimal 3-parameter attitude error representing rotation perturbation $\mathbf{R} \approx \mathbf{R}_{\text{nom}} (\mathbf{I}_3 + [\delta\boldsymbol{\theta}]_\times)$.

---

## 4. Component Deep Dive

### 4.1. Quadrotor Dynamics Model & RK4 Integrator
- **File:** `include/dynamics/quadrotor_model.hpp`, `rk4_integrator.hpp`
- **Translational Dynamics with Drag:**
  $$m \dot{\mathbf{v}} = m \mathbf{g} + \mathbf{R}(\mathbf{q}) \begin{bmatrix} 0 \\ 0 \\ f_T \end{bmatrix} - \mathbf{D}(\mathbf{v} - \mathbf{v}_{\text{wind}})$$
  where $\mathbf{D} = \text{diag}(d_x, d_y, d_z)$ accounts for rotor blade drag and aerodynamic resistance.
- **Rotational Dynamics:**
  $$\mathbf{J} \dot{\boldsymbol{\omega}} = \boldsymbol{\tau} - \boldsymbol{\omega} \times (\mathbf{J} \boldsymbol{\omega})$$
  with precomputed $\mathbf{J}^{-1}$ to eliminate runtime matrix inversions.
- **Runge-Kutta 4 (RK4) Discretization:**
  $$\mathbf{k}_1 = \mathbf{f}(\mathbf{x}_k, \mathbf{u}_k), \quad \mathbf{k}_2 = \mathbf{f}(\mathbf{x}_k + \tfrac{\Delta t}{2}\mathbf{k}_1, \mathbf{u}_k), \quad \mathbf{k}_3 = \mathbf{f}(\mathbf{x}_k + \tfrac{\Delta t}{2}\mathbf{k}_2, \mathbf{u}_k), \quad \mathbf{k}_4 = \mathbf{f}(\mathbf{x}_k + \Delta t \mathbf{k}_3, \mathbf{u}_k)$$
  $$\mathbf{x}_{k+1} = \mathbf{x}_k + \frac{\Delta t}{6} (\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4)$$
  Unit quaternion renormalization is enforced at each sub-stage to prevent numerical integration drift.

---

### 4.2. 15-State Error-State Kalman Filter (ES-EKF)
- **File:** `include/estimation/es_ekf.hpp`, `src/estimation/es_ekf.cpp`
- **High-Rate IMU Propagation (500 Hz):**
  - Specific force $\mathbf{a}_m$ and rate $\boldsymbol{\omega}_m$ are debiased: $\hat{\mathbf{a}} = \mathbf{a}_m - \mathbf{b}_a$, $\hat{\boldsymbol{\omega}} = \boldsymbol{\omega}_m - \mathbf{b}_g$.
  - Discrete transition Jacobian $\mathbf{F}_d = \mathbf{I}_{15} + \mathbf{F}_c \Delta t$:
    $$\mathbf{F}_d = \begin{bmatrix} 
    \mathbf{I}_3 & \mathbf{I}_3 \Delta t & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
    \mathbf{0} & \mathbf{I}_3 & -\mathbf{R}[\hat{\mathbf{a}}]_\times \Delta t & -\mathbf{R}\Delta t & \mathbf{0} \\
    \mathbf{0} & \mathbf{0} & \mathbf{I}_3 - [\hat{\boldsymbol{\omega}}]_\times \Delta t & \mathbf{0} & -\mathbf{I}_3 \Delta t \\
    \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}_3 & \mathbf{0} \\
    \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}_3
    \end{bmatrix}$$
  - Discrete process noise $\mathbf{Q}_d = \mathbf{G}_c \mathbf{Q}_c \mathbf{G}_c^\top \Delta t$.
- **Low-Rate Pose Measurement Update (30 Hz):**
  - Observation matrix $\mathbf{H} = \begin{bmatrix} \mathbf{I}_3 & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\ \mathbf{0} & \mathbf{0} & \mathbf{I}_3 & \mathbf{0} & \mathbf{0} \end{bmatrix} \in \mathbb{R}^{6 \times 15}$.
  - Orientation residual: $\mathbf{r}_\theta = 2 \cdot \text{vec}(\mathbf{q}_{\text{nom}}^{-1} \otimes \mathbf{q}_{\text{meas}})$.
  - Innovation covariance $\mathbf{S} = \mathbf{H}\mathbf{P}\mathbf{H}^\top + \mathbf{R}_m \in \mathbb{R}^{6 \times 6}$. Solved via `Eigen::LDLT` without dynamic heap allocations.
  - **Joseph-Form Covariance Update:**
    $$\mathbf{P} \leftarrow (\mathbf{I} - \mathbf{K}\mathbf{H}) \mathbf{P} (\mathbf{I} - \mathbf{K}\mathbf{H})^\top + \mathbf{K}\mathbf{R}_m\mathbf{K}^\top$$
    Guarantees strict symmetry and positive semi-definiteness (PSD) regardless of floating-point cancellation.
  - **Error Reset:** Error state $\delta\mathbf{x}$ is injected into nominal state, then reset $\delta\mathbf{x} \leftarrow \mathbf{0}$.

---

### 4.3. Real-Time NMPC Solver (iLQR / Gauss-Newton SQP)
- **File:** `include/controller/nmpc_solver.hpp`, `src/controller/nmpc_solver.cpp`
- **Horizon & Timing:** $N = 20$, $\Delta t = 0.05\,\text{s}$ (lookahead horizon $T = 1.0\,\text{s}$). Execution rate $100\,\text{Hz}$ (budget: $10\,\text{ms}$, actual: $\sim 0.26\,\text{ms}$).
- **Gauss-Newton Quadratization:**
  - Tracking error $\mathbf{e}_k \in \mathbb{R}^{12} = [\mathbf{p} - \mathbf{p}_{\text{ref}}, \mathbf{v} - \mathbf{v}_{\text{ref}}, 2\text{vec}(\mathbf{q}_{\text{ref}}^{-1} \otimes \mathbf{q}), \boldsymbol{\omega} - \boldsymbol{\omega}_{\text{ref}}]^\top$.
  - Exact Jacobian $\mathbf{J}_e = \frac{\partial \mathbf{e}}{\partial \mathbf{x}} \in \mathbb{R}^{12 \times 13}$.
  - Stage cost Hessian: $\mathbf{l}_{xx} \approx 2 \mathbf{J}_e^\top \mathbf{Q} \mathbf{J}_e$, gradient: $\mathbf{l}_x = 2 \mathbf{J}_e^\top \mathbf{Q} \mathbf{e}$.
- **Constraint Handling:**
  1. *Actuator bounds:* Projected clamping $u_{\min} \le u_i \le u_{\max}$.
  2. *Tilt constraint:* Smooth exterior penalty $w_{\text{tilt}} (q_x^2 + q_y^2 - \sin^2(\theta_{\max}/2))^2$.
  3. *Ellipsoidal obstacle avoidance:* Smooth penalty on intrusion:
     $$w_{\text{obs}} \max(0, 1 - (\mathbf{p} - \mathbf{p}_{\text{obs}})^\top \mathbf{A}_{\text{obs}} (\mathbf{p} - \mathbf{p}_{\text{obs}}))^2$$
- **Riccati Backward Pass:**
  Computes feedforward gain $\mathbf{k}_{\text{ff}}$ and feedback gain $\mathbf{K}$ recursively from $N-1$ down to $0$ with Levenberg-Marquardt damping $\mathbf{Q}_{uu} + \mu \mathbf{I}$.
- **Clamped Forward Pass with Armijo Line Search:**
  Rolls out candidate trajectory with feedback policy $\mathbf{u}_k = \mathbf{U}_k + \alpha \mathbf{k}_{\text{ff}, k} + \mathbf{K}_k (\mathbf{X}_{t, k} - \mathbf{X}_k)$, reducing step size $\alpha$ until cost decreases.

---

## 5. Memory Model & Real-Time Determinism

Deterministic real-time control requires eliminating unbounded OS system calls (e.g. `brk`, `mmap`, `malloc`) during the execution loop.

| Component | Buffer Location | Memory Type | Allocations per Control Loop |
| :--- | :--- | :--- | :---: |
| **Dynamics & RK4** | Stack | Fixed-size `Eigen::Matrix<double, 13, 1>` | **0** |
| **ES-EKF Matrices** | Member variables | Fixed-size `Eigen::Matrix<double, 15, 15>` | **0** |
| **ES-EKF Innovation Inversion** | Stack | `Eigen::LDLT<Eigen::Matrix<double, 6, 6>>` | **0** |
| **NMPC Prediction Arrays** | Preallocated arrays | `std::array<StateVector, kMaxHorizon + 1>` | **0** |
| **NMPC Gain Sequences** | Preallocated arrays | `std::array<KMat, kMaxHorizon>` | **0** |
| **Trajectory Horizon** | Stack/Member arrays | `std::array<StateVector, 33>` | **0** |

**Guarantees:**
- No standard heap containers (`std::vector`, `std::map`, `std::string`) are allocated inside `solve()`, `propagate()`, `update()`, or `step()`.
- Contiguous SIMD alignment ensures full hardware vectorization (`AVX2` on x86_64, `NEON` on ARM64 / Apple Silicon).
