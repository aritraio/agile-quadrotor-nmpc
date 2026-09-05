// SPDX-License-Identifier: MIT
// Receding-horizon NMPC: iterative LQR (Gauss-Newton SQP / RTI-style),
// Eigen-only, allocation-free steady-state loop.
//
// OCP:
//   min sum_{k=0}^{N-1} (||e_k||_Q^2 + ||u_k-uref_k||_R^2 + pen_k) + ||e_N||_P^2 + pen_N
//   s.t. X_{k+1} = f_RK4(X_k, U_k),  u_min <= U_k <= u_max,
//        tilt: qx^2+qy^2 <= sin^2(theta_max/2),
//        obstacle: (p-pobs)^T A (p-pobs) >= 1.
//
// Method: Gauss-Newton iLQR — linearize RK4 map (finite-diff Jacobians),
// quadratize tracking cost (J^T Q J), Riccati backward pass for feedforward +
// feedback gains, clamped forward pass with Armijo line search, warm-started
// across control cycles. All workspace preallocated (zero heap in solve()).
#pragma once

#include <array>
#include <cstddef>

#include "dynamics/quadrotor_model.hpp"
#include "dynamics/rk4_integrator.hpp"
#include "dynamics/types.hpp"

namespace quadrotor::controller {

inline constexpr int kMaxHorizon = 32;
inline constexpr int kMaxObstacles = 4;

struct EllipsoidObstacle {
  Vec3 center{Vec3::Zero()};
  Vec3 radii{Vec3(1.0, 1.0, 1.0)};  // semi-axes [m]
  Mat3 shape{Mat3::Identity()};     // A = diag(1/r^2), world-aligned

  void updateShape() noexcept {
    shape.setZero();
    for (Eigen::Index i = 0; i < 3; ++i) {
      const double r = radii(i) > 1e-6 ? radii(i) : 1e-6;
      shape(i, i) = 1.0 / (r * r);
    }
  }
};

struct NmpcConfig {
  int horizon{20};          // N
  double dt{0.05};          // [s]
  TrackWeightMatrix Q{TrackWeightMatrix::Identity()};  // 12x12 stage
  ControlMatrix R{ControlMatrix::Identity()};          // 4x4
  TrackWeightMatrix P{TrackWeightMatrix::Identity()};  // terminal
  ControlVector u_min{ControlVector::Zero()};
  ControlVector u_max{ControlVector::Zero()};
  ControlVector u_hover{ControlVector::Zero()};
  double tilt_max_rad{0.6};
  double tilt_weight{500.0};
  double obs_weight{2000.0};
  int max_iters{5};
  double grad_tol{1e-4};
  double init_step{1.0};
  double min_step{1e-3};
  double ilqr_reg{1e-4};  // Levenberg-Marquardt damping on Quu
  int num_obstacles{0};
  std::array<EllipsoidObstacle, kMaxObstacles> obstacles{};

  NmpcConfig();
};

struct NmpcInfo {
  double cost{0.0};
  double solve_time_ms{0.0};
  int iterations{0};
  double grad_norm{0.0};
  double max_tilt_violation{0.0};
  double min_obs_clearance{0.0};  // min over k of (d_k - 1); negative = violation
  bool constraints_ok{true};
  bool converged{false};
};

class NmpcSolver {
 public:
  explicit NmpcSolver(const NmpcConfig& cfg = NmpcConfig(),
                      const dynamics::QuadrotorParams& plant = dynamics::DefaultQuadParams());

  void setConfig(const NmpcConfig& cfg) noexcept;
  [[nodiscard]] const NmpcConfig& config() const noexcept { return cfg_; }
  void setModel(const dynamics::QuadrotorParams& p) noexcept;

  // Shift warm start (RTI-style): drop U_0, duplicate last.
  void shiftWarmStart() noexcept;

  // Solve OCP. All spans are caller-owned fixed buffers; no allocation.
  bool solve(const StateVector& x0, const StateVector* Xref, const ControlVector* Uref,
             ControlVector* Uopt_out, NmpcInfo& info) noexcept;

  [[nodiscard]] const StateVector& predictedState(std::size_t k) const noexcept {
    return X_[k];
  }

 private:
  using AMat = Eigen::Matrix<double, kStateDim, kStateDim>;
  using BMat = Eigen::Matrix<double, kStateDim, kInputDim>;
  using QxxMat = Eigen::Matrix<double, kStateDim, kStateDim>;
  using QuuMat = Eigen::Matrix<double, kInputDim, kInputDim>;
  using QxuMat = Eigen::Matrix<double, kStateDim, kInputDim>;
  using KMat = Eigen::Matrix<double, kInputDim, kStateDim>;

  void rollout(const StateVector& x0, const ControlVector* U, StateVector* X) const noexcept;
  void rolloutFeedback(const StateVector& x0, StateVector* Xt, ControlVector* Ut, double alpha) const noexcept;
  double stageCost(int k, const StateVector& x, const ControlVector& u, const StateVector& xr,
                   const ControlVector& ur) const noexcept;
  double terminalCost(const StateVector& xN, const StateVector& xrN) const noexcept;
  double totalCost(const StateVector& x0, const StateVector* Xref, const ControlVector* Uref,
                   const ControlVector* U) const noexcept;
  // Gauss-Newton quadratization: gradients + Hessians.
  void stageQuadratization(const StateVector& x, const ControlVector& u, const StateVector& xr,
                           const ControlVector& ur, StateVector& lx, ControlVector& lu,
                           QxxMat& lxx, QuuMat& luu) const noexcept;
  void terminalQuadratization(const StateVector& xN, const StateVector& xrN, StateVector& lx,
                              QxxMat& lxx) const noexcept;
  bool backwardPass(double reg, double& expected_reduction) noexcept;
  void linearize(const StateVector& x, const ControlVector& u, AMat& A, BMat& B) const noexcept;
  void trackingError(const StateVector& x, const StateVector& xr,
                     TrackErrorVector& e) const noexcept;
  void trackingJacobian(const StateVector& xr, Eigen::Matrix<double, 12, 13>& J) const noexcept;
  double tiltPenalty(const Vec4& q, Vec4& grad_q, Eigen::Matrix<double, 4, 4>* hess) const noexcept;
  double obstaclePenalty(const Vec3& p, Vec3& grad_p, Mat3* hess) const noexcept;
  void clampControl(ControlVector& u) const noexcept;
  void evaluateConstraints(const StateVector* X, NmpcInfo& info) const noexcept;

  NmpcConfig cfg_;
  dynamics::QuadrotorModel model_;
  dynamics::RK4Integrator integrator_;

  // Preallocated workspace (never resized).
  std::array<StateVector, kMaxHorizon + 1> X_{};
  std::array<StateVector, kMaxHorizon + 1> Xref_{};
  std::array<ControlVector, kMaxHorizon> U_{};
  std::array<ControlVector, kMaxHorizon> Uref_{};
  std::array<AMat, kMaxHorizon> A_{};
  std::array<BMat, kMaxHorizon> B_{};
  std::array<StateVector, kMaxHorizon> Lx_{};
  std::array<ControlVector, kMaxHorizon> Lu_{};
  std::array<QxxMat, kMaxHorizon> Lxx_{};
  std::array<QuuMat, kMaxHorizon> Luu_{};
  StateVector LxT_{};
  QxxMat LxxT_{};
  std::array<KMat, kMaxHorizon> K_{};
  std::array<ControlVector, kMaxHorizon> Kff_{};
  std::array<QxxMat, kMaxHorizon + 1> Vxx_{};
  std::array<StateVector, kMaxHorizon + 1> Vx_{};
  std::array<ControlVector, kMaxHorizon> Utrial_{};
  std::array<StateVector, kMaxHorizon + 1> Xtrial_{};
};

}  // namespace quadrotor::controller
