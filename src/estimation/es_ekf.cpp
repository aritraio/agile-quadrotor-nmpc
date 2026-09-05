// SPDX-License-Identifier: MIT
#include "estimation/es_ekf.hpp"

#include <cmath>

namespace quadrotor::estimation {

ErrorStateEKF::ErrorStateEKF(const EkfNoiseParams& noise, const Vec3& gravity) noexcept
    : noise_(noise), gravity_(gravity) {
  reset();
}

void ErrorStateEKF::reset(const NominalState& state) noexcept {
  nom_ = state;
  quatNormalize(nom_.q);
  P_.setZero();
  const double p = noise_.init_pos_std * noise_.init_pos_std;
  const double v = noise_.init_vel_std * noise_.init_vel_std;
  const double t = noise_.init_att_std * noise_.init_att_std;
  const double a = noise_.init_ba_std * noise_.init_ba_std;
  const double g = noise_.init_bg_std * noise_.init_bg_std;
  P_.block<3, 3>(0, 0).setIdentity();
  P_.block<3, 3>(0, 0) *= p;
  P_.block<3, 3>(3, 3).setIdentity();
  P_.block<3, 3>(3, 3) *= v;
  P_.block<3, 3>(6, 6).setIdentity();
  P_.block<3, 3>(6, 6) *= t;
  P_.block<3, 3>(9, 9).setIdentity();
  P_.block<3, 3>(9, 9) *= a;
  P_.block<3, 3>(12, 12).setIdentity();
  P_.block<3, 3>(12, 12) *= g;
  last_nis_ = 0.0;
  last_rp_.setZero();
  last_ra_.setZero();
}

void ErrorStateEKF::reset() noexcept {
  NominalState s;
  reset(s);
}

bool ErrorStateEKF::propagate(const ImuMeasurement& imu, double dt) noexcept {
  if (!(dt > 0.0) || !std::isfinite(dt) || dt > 0.1) return false;

  // Bias-corrected measurements.
  const Vec3 omega_hat = imu.gyro - nom_.bg;
  const Vec3 acc_hat = imu.accel - nom_.ba;

  Mat3 R;
  rotMatrix(nom_.q, R);

  // --- Nominal propagation (exact quaternion exp for attitude) -----------
  const double wnorm = omega_hat.norm();
  Vec4 dq;
  if (wnorm < 1e-12) {
    dq << 1.0, 0.5 * omega_hat(0) * dt, 0.5 * omega_hat(1) * dt, 0.5 * omega_hat(2) * dt;
  } else {
    const double half = 0.5 * wnorm * dt;
    const double s = std::sin(half) / wnorm;
    dq << std::cos(half), s * omega_hat(0), s * omega_hat(1), s * omega_hat(2);
  }
  Vec4 q_new;
  quatMultiply(nom_.q, dq, q_new);
  quatNormalize(q_new);
  nom_.q = q_new;

  const Vec3 acc_world = R * acc_hat + gravity_;
  nom_.p.noalias() += nom_.v * dt + 0.5 * acc_world * dt * dt;
  nom_.v.noalias() += acc_world * dt;
  // Biases: random-walk mean stays constant.

  // --- Discrete error covariance -----------------------------------------
  // Continuous-time Fc (15x15) and Gc (15x12):
  //   dp_dot = dv
  //   dv_dot = -R [acc_hat]x dtheta - R dba - R na
  //   dt_dot = -[omega_hat]x dtheta - dbg - ng
  //   dba_dot = nba, dbg_dot = nbg
  Mat3 Racc_skew, Wskew;
  {
    Mat3 S;
    skew(acc_hat, S);
    Racc_skew.noalias() = -R * S;
    skew(omega_hat, Wskew);
    Wskew = -Wskew;
  }

  // Fd = I + Fc*dt (+ 0.5 Fc^2 dt^2 omitted; first-order is standard at 500 Hz).
  // Build directly into Fd to avoid extra temporaries.
  Eigen::Matrix<double, 15, 15> Fd = Eigen::Matrix<double, 15, 15>::Identity();
  Fd.block<3, 3>(0, 3).setIdentity();
  Fd.block<3, 3>(0, 3) *= dt;                    // dp/dv
  Fd.block<3, 3>(3, 6) = Racc_skew * dt;         // dv/dtheta
  Fd.block<3, 3>(3, 9) = -R * dt;                // dv/dba
  Fd.block<3, 3>(6, 6) += Wskew * dt;            // dtheta/dtheta
  Fd.block<3, 3>(6, 12).setIdentity();
  Fd.block<3, 3>(6, 12) *= -dt;                  // dtheta/dbg

  // Qd = Gc Qc Gc^T dt. Qc = diag(sa^2, sg^2, sba^2, sbg^2).
  const double sa2 = noise_.accel_noise * noise_.accel_noise;
  const double sg2 = noise_.gyro_noise * noise_.gyro_noise;
  const double sba2 = noise_.accel_bias_rw * noise_.accel_bias_rw;
  const double sbg2 = noise_.gyro_bias_rw * noise_.gyro_bias_rw;

  Eigen::Matrix<double, 15, 15> Qd = Eigen::Matrix<double, 15, 15>::Zero();
  // dv block: R (sa^2 dt) R^T
  Qd.block<3, 3>(3, 3).noalias() = R * R.transpose() * (sa2 * dt);
  // dtheta block: sg^2 dt I
  Qd.block<3, 3>(6, 6).setIdentity();
  Qd.block<3, 3>(6, 6) *= (sg2 * dt);
  Qd.block<3, 3>(9, 9).setIdentity();
  Qd.block<3, 3>(9, 9) *= (sba2 * dt);
  Qd.block<3, 3>(12, 12).setIdentity();
  Qd.block<3, 3>(12, 12) *= (sbg2 * dt);

  P_ = Fd * P_ * Fd.transpose() + Qd;
  // Enforce symmetry (kills round-off drift, keeps PSD).
  P_ = 0.5 * (P_ + P_.transpose());
  return true;
}

bool ErrorStateEKF::update(const PoseMeasurement& z) noexcept {
  if (!z.valid) return false;

  // Residuals.
  last_rp_.noalias() = z.position - nom_.p;
  orientationResidual(nom_.q, z.orientation, last_ra_);

  Eigen::Matrix<double, 6, 1> r;
  r.segment<3>(0) = last_rp_;
  r.segment<3>(3) = last_ra_;

  // H (6x15): position observes dp, attitude observes dtheta.
  Eigen::Matrix<double, 6, 15> H = Eigen::Matrix<double, 6, 15>::Zero();
  H.block<3, 3>(0, 0).setIdentity();
  H.block<3, 3>(3, 6).setIdentity();

  Eigen::Matrix<double, 6, 6> Rm = Eigen::Matrix<double, 6, 6>::Zero();
  Rm.block<3, 3>(0, 0) = z.pos_cov;
  Rm.block<3, 3>(3, 3) = z.att_cov;

  // S = H P H^T + R; K = P H^T S^{-1}. 6x6 LDLT, no allocation beyond stack.
  Eigen::Matrix<double, 15, 6> PHt = P_ * H.transpose();
  Eigen::Matrix<double, 6, 6> S = H * PHt + Rm;
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(S);
  if (ldlt.info() != Eigen::Success) return false;
  Eigen::Matrix<double, 15, 6> K = ldlt.solve(PHt.transpose()).transpose();
  // Note: K = PHt * S^{-1}; solve via (S^{-1} PHt^T)^T.

  Eigen::Matrix<double, 15, 1> dx = K * r;

  // NIS for consistency monitoring: r^T S^{-1} r.
  Eigen::Matrix<double, 6, 1> Sinv_r = ldlt.solve(r);
  last_nis_ = r.dot(Sinv_r);

  // --- Inject error into nominal ------------------------------------------
  nom_.p.noalias() += dx.segment<3>(0);
  nom_.v.noalias() += dx.segment<3>(3);
  injectAttitude(nom_.q, dx.segment<3>(6));
  quatNormalize(nom_.q);
  nom_.ba.noalias() += dx.segment<3>(9);
  nom_.bg.noalias() += dx.segment<3>(12);

  // --- Joseph-form covariance update (guarantees symmetry/PSD) ------------
  Eigen::Matrix<double, 15, 15> IKH =
      Eigen::Matrix<double, 15, 15>::Identity() - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * Rm * K.transpose();
  P_ = 0.5 * (P_ + P_.transpose());
  return true;
}

void ErrorStateEKF::toQuadState(StateVector& x, const Vec3& omega_body_hint) const noexcept {
  x.segment<3>(StateIndex::kPx) = nom_.p;
  x.segment<3>(StateIndex::kVx) = nom_.v;
  x.segment<4>(StateIndex::kQw) = nom_.q;
  x.segment<3>(StateIndex::kWx) = omega_body_hint;
}

// ---------------------------------------------------------------------------
void ErrorStateEKF::quatMultiply(const Vec4& a, const Vec4& b, Vec4& out) noexcept {
  out(0) = a(0) * b(0) - a(1) * b(1) - a(2) * b(2) - a(3) * b(3);
  out(1) = a(0) * b(1) + a(1) * b(0) + a(2) * b(3) - a(3) * b(2);
  out(2) = a(0) * b(2) - a(1) * b(3) + a(2) * b(0) + a(3) * b(1);
  out(3) = a(0) * b(3) + a(1) * b(2) - a(2) * b(1) + a(3) * b(0);
}

void ErrorStateEKF::quatConjugate(const Vec4& a, Vec4& out) noexcept {
  out(0) = a(0);
  out(1) = -a(1);
  out(2) = -a(2);
  out(3) = -a(3);
}

void ErrorStateEKF::quatNormalize(Vec4& q) noexcept {
  const double n = q.norm();
  if (n > 1e-12) {
    q /= n;
  } else {
    q << 1.0, 0.0, 0.0, 0.0;
  }
}

void ErrorStateEKF::rotMatrix(const Vec4& q, Mat3& R) noexcept {
  double n = q.norm();
  double w = q(0), x = q(1), y = q(2), z = q(3);
  if (n > 1e-12) {
    w /= n;
    x /= n;
    y /= n;
    z /= n;
  } else {
    w = 1.0;
    x = y = z = 0.0;
  }
  const double ww = w * w, xx = x * x, yy = y * y, zz = z * z;
  R(0, 0) = ww + xx - yy - zz;
  R(0, 1) = 2.0 * (x * y - w * z);
  R(0, 2) = 2.0 * (x * z + w * y);
  R(1, 0) = 2.0 * (x * y + w * z);
  R(1, 1) = ww - xx + yy - zz;
  R(1, 2) = 2.0 * (y * z - w * x);
  R(2, 0) = 2.0 * (x * z - w * y);
  R(2, 1) = 2.0 * (y * z + w * x);
  R(2, 2) = ww - xx - yy + zz;
}

void ErrorStateEKF::skew(const Vec3& v, Mat3& S) noexcept {
  S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
}

void ErrorStateEKF::injectAttitude(Vec4& q, const Vec3& dtheta) noexcept {
  const double ang = dtheta.norm();
  Vec4 dq;
  if (ang < 1e-12) {
    dq << 1.0, 0.5 * dtheta(0), 0.5 * dtheta(1), 0.5 * dtheta(2);
  } else {
    const double s = std::sin(0.5 * ang) / ang;
    dq << std::cos(0.5 * ang), s * dtheta(0), s * dtheta(1), s * dtheta(2);
  }
  Vec4 out;
  quatMultiply(q, dq, out);
  q = out;
}

void ErrorStateEKF::orientationResidual(const Vec4& q_nom, const Vec4& q_meas,
                                        Vec3& dtheta) noexcept {
  Vec4 qn = q_nom, qm = q_meas;
  quatNormalize(qn);
  quatNormalize(qm);
  Vec4 qn_inv;
  quatConjugate(qn, qn_inv);
  Vec4 q_err;
  quatMultiply(qn_inv, qm, q_err);
  // Shortest-arc: enforce positive scalar part.
  if (q_err(0) < 0.0) q_err = -q_err;
  // Small-angle log map: dtheta ~= 2 * vec(q_err) (exact as angle->0).
  dtheta(0) = 2.0 * q_err(1);
  dtheta(1) = 2.0 * q_err(2);
  dtheta(2) = 2.0 * q_err(3);
}

}  // namespace quadrotor::estimation
