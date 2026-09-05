// SPDX-License-Identifier: MIT
// 15-state Error-State EKF: nominal [p, v, q, ba, bg], error dx in R^15.
//
//   x_true = x_nominal (+) dx,  dx = [dp, dv, dtheta, dba, dbg]
//
// IMU propagation at 500 Hz (open-loop nominal + discrete covariance),
// VO/mocap pose updates at 30-60 Hz with Joseph-form correction,
// error injection and reset dx <- 0.
//
// Real-time discipline: all buffers are fixed-size Eigen members;
// propagate() and update() perform zero heap allocations.
#pragma once

#include "dynamics/types.hpp"
#include "estimation/sensor_types.hpp"

namespace quadrotor::estimation {

// Nominal state storage (16 scalars: q has 4).
struct NominalState {
  Vec3 p{Vec3::Zero()};
  Vec3 v{Vec3::Zero()};
  Vec4 q{Vec4(1.0, 0.0, 0.0, 0.0)};  // wxyz, body->world
  Vec3 ba{Vec3::Zero()};             // accel bias, body
  Vec3 bg{Vec3::Zero()};             // gyro bias, body
};

class ErrorStateEKF {
 public:
  explicit ErrorStateEKF(const EkfNoiseParams& noise = EkfNoiseParams{},
                         const Vec3& gravity = Vec3(0, 0, -9.81)) noexcept;

  void reset(const NominalState& state) noexcept;
  void reset() noexcept;

  // ----- high-rate IMU propagation ---------------------------------------
  // Returns false only for non-finite / non-positive dt.
  bool propagate(const ImuMeasurement& imu, double dt) noexcept;

  // ----- low-rate pose update --------------------------------------------
  // Fuses position + orientation. Returns false if meas invalid.
  bool update(const PoseMeasurement& z) noexcept;

  // ----- accessors (const, no copy on hot path where possible) -----------
  [[nodiscard]] const NominalState& nominal() const noexcept { return nom_; }
  [[nodiscard]] const ErrorMatrix& covariance() const noexcept { return P_; }
  [[nodiscard]] const EkfNoiseParams& noiseParams() const noexcept { return noise_; }

  // Export full 13-state quad state for the controller (omega from last IMU).
  void toQuadState(StateVector& x, const Vec3& omega_body_hint) const noexcept;

  // Innovation / NIS of the last update (for consistency monitoring).
  [[nodiscard]] double lastNis() const noexcept { return last_nis_; }
  [[nodiscard]] Vec3 lastPosInnovation() const noexcept { return last_rp_; }
  [[nodiscard]] Vec3 lastAttInnovation() const noexcept { return last_ra_; }

 private:
  // Small-angle quaternion helpers (local, allocation-free).
  static void quatMultiply(const Vec4& a, const Vec4& b, Vec4& out) noexcept;
  static void quatConjugate(const Vec4& a, Vec4& out) noexcept;
  static void quatNormalize(Vec4& q) noexcept;
  static void rotMatrix(const Vec4& q, Mat3& R) noexcept;
  static void skew(const Vec3& v, Mat3& S) noexcept;
  // q_out = q (+) dtheta  (inject attitude error).
  static void injectAttitude(Vec4& q, const Vec3& dtheta) noexcept;
  // dtheta = 2 * vec(q_ref^{-1} * q_meas), wrapped to [-pi, pi] via sign of w.
  static void orientationResidual(const Vec4& q_nom, const Vec4& q_meas,
                                  Vec3& dtheta) noexcept;

  NominalState nom_;
  ErrorMatrix P_{ErrorMatrix::Identity() * 1e-4};
  EkfNoiseParams noise_;
  Vec3 gravity_{Vec3(0, 0, -9.81)};

  double last_nis_{0.0};
  Vec3 last_rp_{Vec3::Zero()};
  Vec3 last_ra_{Vec3::Zero()};
};

}  // namespace quadrotor::estimation
