// SPDX-License-Identifier: MIT
// 6-DoF quadrotor dynamics on SO(3).
//
// State: x = [p(3), v(3), q(4 wxyz), omega(3)] in R^13
// Input: u = [f_T, tau(3)] in R^4
//
//   p_dot = v
//   m v_dot = m g + R(q) * [0,0,f_T]^T - D v
//   q_dot = 0.5 * q otimes [0, omega]
//   J w_dot = tau - w x (J w)
//
// All hot-loop methods are allocation-free and operate on caller-provided
// fixed-size Eigen buffers.
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "dynamics/types.hpp"

namespace quadrotor::dynamics {

// Physical parameters. Trivially copyable, constexpr-constructible.
struct QuadrotorParams {
  double mass{1.0};                      // [kg]
  Mat3 inertia{Mat3::Identity()};        // [kg m^2]
  Mat3 inertia_inv{Mat3::Identity()};    // precomputed inverse
  Vec3 drag{Vec3(0.1, 0.1, 0.15)};       // rotor drag diag(D) [N s / m]
  Vec3 gravity{Vec3(0.0, 0.0, -9.81)};   // [m/s^2]
  double max_thrust{25.0};               // per-collective [N]
  double max_torque{2.0};                // per-axis [N m]
  double max_tilt_rad{0.6};              // tilt limit [rad]

  constexpr QuadrotorParams() = default;

  void computeInertiaInverse() noexcept { inertia_inv = inertia.inverse(); }
};

// Default racing-quad parameters (constexpr-friendly factory declared here,
// defined in the .cpp to keep the header light).
QuadrotorParams DefaultQuadParams() noexcept;

class QuadrotorModel {
 public:
  explicit QuadrotorModel(const QuadrotorParams& params = DefaultQuadParams()) noexcept;

  // ----- parameter access -------------------------------------------------
  [[nodiscard]] const QuadrotorParams& params() const noexcept { return params_; }
  void setParams(const QuadrotorParams& p) noexcept { params_ = p; }

  // ----- core dynamics: xdot = f(x, u). No allocation. --------------------
  // x: 13-vector, u: 4-vector, xdot: 13-vector (output).
  void stateDerivative(const StateVector& x, const ControlVector& u,
                       StateVector& xdot) const noexcept;

  // ----- SO(3) helpers (static, allocation-free) ---------------------------
  // Quaternion (w,x,y,z) -> rotation matrix R(q) in SO(3), body->world.
  static void rotationMatrix(const Eigen::Ref<const Vec4>& q, Mat3& R) noexcept;
  static Mat3 rotationMatrix(const Eigen::Ref<const Vec4>& q) noexcept;

  // Quaternion derivative qdot = 0.5 * q otimes [0, omega].
  static void quaternionDerivative(const Eigen::Ref<const Vec4>& q,
                                   const Eigen::Ref<const Vec3>& omega,
                                   Vec4& qdot) noexcept;

  // Hamilton product q = q1 otimes q2 (both wxyz). Output wxyz.
  static void quaternionMultiply(const Eigen::Ref<const Vec4>& q1,
                                 const Eigen::Ref<const Vec4>& q2,
                                 Vec4& out) noexcept;

  // Conjugate (inverse for unit quaternions).
  static void quaternionConjugate(const Eigen::Ref<const Vec4>& q, Vec4& out) noexcept;

  // Normalize in place; returns pre-normalization norm (0 if degenerate).
  static double normalizeQuaternion(Eigen::Ref<Vec4> q) noexcept;

  // Left-multiplication matrix: q1 otimes q2 = L(q1) * q2.
  static void leftMultiplyMatrix(const Eigen::Ref<const Vec4>& q, Mat4& L) noexcept;

  // Tilt metric: sin^2 Tilt check uses qx^2 + qy^2 <= sin^2(theta_max/2).
  [[nodiscard]] bool satisfiesTilt(const Eigen::Ref<const Vec4>& q) const noexcept;
  static double tiltQxyNorm2(const Eigen::Ref<const Vec4>& q) noexcept;

  // Hover thrust for stationary flight (m * |g|).
  [[nodiscard]] double hoverThrust() const noexcept;

 private:
  QuadrotorParams params_;
};

}  // namespace quadrotor::dynamics
