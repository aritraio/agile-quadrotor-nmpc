// SPDX-License-Identifier: MIT
#include "dynamics/quadrotor_model.hpp"

namespace quadrotor::dynamics {

QuadrotorParams DefaultQuadParams() noexcept {
  QuadrotorParams p;
  p.mass = 1.0;
  p.inertia.setZero();
  p.inertia(0, 0) = 0.0123;
  p.inertia(1, 1) = 0.0123;
  p.inertia(2, 2) = 0.022;
  p.computeInertiaInverse();
  p.drag = Vec3(0.1, 0.1, 0.15);
  p.gravity = Vec3(0.0, 0.0, -9.81);
  p.max_thrust = 25.0;
  p.max_torque = 2.0;
  p.max_tilt_rad = 0.6;
  return p;
}

QuadrotorModel::QuadrotorModel(const QuadrotorParams& params) noexcept : params_(params) {}

void QuadrotorModel::stateDerivative(const StateVector& x, const ControlVector& u,
                                     StateVector& xdot) const noexcept {
  const auto p = x.segment<3>(StateIndex::kPx);
  const auto v = x.segment<3>(StateIndex::kVx);
  const Vec4 q = x.segment<4>(StateIndex::kQw);
  const auto w = x.segment<3>(StateIndex::kWx);
  (void)p;

  const double fT = u(InputIndex::kThrust);
  const Vec3 tau = u.segment<3>(InputIndex::kTaux);

  Mat3 R;
  rotationMatrix(q, R);

  // Position kinematics.
  xdot.segment<3>(StateIndex::kPx) = v;

  // Translational dynamics: m dv = m g + R * [0,0,fT] - D v.
  const Vec3 thrust_world = R.col(2) * fT;
  const Vec3 drag_force(params_.drag(0) * v(0), params_.drag(1) * v(1),
                        params_.drag(2) * v(2));
  xdot.segment<3>(StateIndex::kVx) =
      params_.gravity + thrust_world / params_.mass - drag_force / params_.mass;

  // Attitude kinematics.
  Vec4 qdot;
  quaternionDerivative(q, w, qdot);
  xdot.segment<4>(StateIndex::kQw) = qdot;

  // Angular dynamics: J wdot = tau - w x (J w).
  const Vec3 Jw = params_.inertia * w;
  xdot.segment<3>(StateIndex::kWx) = params_.inertia_inv * (tau - w.cross(Jw));
}

void QuadrotorModel::rotationMatrix(const Eigen::Ref<const Vec4>& q, Mat3& R) noexcept {
  // Normalize defensively (cheap, keeps SO(3) exact under drift).
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
  const double wx = w * x, wy = w * y, wz = w * z;
  const double xy = x * y, xz = x * z, yz = y * z;
  R(0, 0) = ww + xx - yy - zz;
  R(0, 1) = 2.0 * (xy - wz);
  R(0, 2) = 2.0 * (xz + wy);
  R(1, 0) = 2.0 * (xy + wz);
  R(1, 1) = ww - xx + yy - zz;
  R(1, 2) = 2.0 * (yz - wx);
  R(2, 0) = 2.0 * (xz - wy);
  R(2, 1) = 2.0 * (yz + wx);
  R(2, 2) = ww - xx - yy + zz;
}

Mat3 QuadrotorModel::rotationMatrix(const Eigen::Ref<const Vec4>& q) noexcept {
  Mat3 R;
  rotationMatrix(q, R);
  return R;
}

void QuadrotorModel::quaternionDerivative(const Eigen::Ref<const Vec4>& q,
                                          const Eigen::Ref<const Vec3>& omega,
                                          Vec4& qdot) noexcept {
  // qdot = 0.5 * q otimes [0, omega]. With w-first convention:
  // qdot_w = -0.5 (x*wx + y*wy + z*wz), etc.
  const double qw = q(0), qx = q(1), qy = q(2), qz = q(3);
  const double wx = omega(0), wy = omega(1), wz = omega(2);
  qdot(0) = -0.5 * (qx * wx + qy * wy + qz * wz);
  qdot(1) = 0.5 * (qw * wx + qy * wz - qz * wy);
  qdot(2) = 0.5 * (qw * wy + qz * wx - qx * wz);
  qdot(3) = 0.5 * (qw * wz + qx * wy - qy * wx);
}

void QuadrotorModel::quaternionMultiply(const Eigen::Ref<const Vec4>& q1,
                                        const Eigen::Ref<const Vec4>& q2,
                                        Vec4& out) noexcept {
  Mat4 L;
  leftMultiplyMatrix(q1, L);
  out.noalias() = L * q2;
}

void QuadrotorModel::quaternionConjugate(const Eigen::Ref<const Vec4>& q, Vec4& out) noexcept {
  out(0) = q(0);
  out(1) = -q(1);
  out(2) = -q(2);
  out(3) = -q(3);
}

double QuadrotorModel::normalizeQuaternion(Eigen::Ref<Vec4> q) noexcept {
  const double n = q.norm();
  if (n > 1e-12) {
    q /= n;
  } else {
    q << 1.0, 0.0, 0.0, 0.0;
  }
  return n;
}

void QuadrotorModel::leftMultiplyMatrix(const Eigen::Ref<const Vec4>& q, Mat4& L) noexcept {
  const double w = q(0), x = q(1), y = q(2), z = q(3);
  L << w, -x, -y, -z, x, w, -z, y, y, z, w, -x, z, -y, x, w;
}

bool QuadrotorModel::satisfiesTilt(const Eigen::Ref<const Vec4>& q) const noexcept {
  const double s = std::sin(0.5 * params_.max_tilt_rad);
  return tiltQxyNorm2(q) <= s * s + 1e-12;
}

double QuadrotorModel::tiltQxyNorm2(const Eigen::Ref<const Vec4>& q) noexcept {
  double n = q.norm();
  double x = q(1), y = q(2);
  if (n > 1e-12) {
    x /= n;
    y /= n;
  }
  return x * x + y * y;
}

double QuadrotorModel::hoverThrust() const noexcept {
  return params_.mass * (-params_.gravity(2));
}

}  // namespace quadrotor::dynamics
