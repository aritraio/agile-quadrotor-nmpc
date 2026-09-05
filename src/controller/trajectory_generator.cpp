// SPDX-License-Identifier: MIT
#include "controller/trajectory_generator.hpp"

#include <cmath>

namespace quadrotor::controller {

TrajectoryGenerator::TrajectoryGenerator(const TrajectoryParams& p,
                                         const dynamics::QuadrotorParams& plant) noexcept
    : params_(p), model_(plant) {}

void TrajectoryGenerator::flatOutputs(double t, Vec3& pos, Vec3& vel, Vec3& acc) const noexcept {
  switch (params_.type) {
    case TrajectoryType::kLemniscate:
      lemniscate(t, pos, vel, acc);
      break;
    case TrajectoryType::kCircle:
      circle(t, pos, vel, acc);
      break;
    case TrajectoryType::kStep:
      step(t, pos, vel, acc);
      break;
    case TrajectoryType::kHover:
    default:
      hover(t, pos, vel, acc);
      break;
  }
}

void TrajectoryGenerator::reference(double t, StateVector& x_ref,
                                    ControlVector& u_ref) const noexcept {
  Vec3 p, v, a;
  flatOutputs(t, p, v, a);
  flatToState(p, v, a, x_ref, u_ref);
}

void TrajectoryGenerator::sampleHorizon(double t0, double dt, int N, StateVector* Xref,
                                        ControlVector* Uref) const noexcept {
  if (N > kMaxHorizon) N = kMaxHorizon;
  for (int k = 0; k <= N; ++k) {
    ControlVector u;
    reference(t0 + k * dt, Xref[k], u);
    if (k < N) Uref[k] = u;
  }
}

void TrajectoryGenerator::lemniscate(double t, Vec3& p, Vec3& v, Vec3& a) const noexcept {
  // Bernoulli-style figure-8: x = A sin(wt), y = A sin(wt)cos(wt), z = h + Az sin(2wt).
  const double A = params_.amplitude;
  const double w = params_.omega;
  const double h = params_.height;
  const double Az = params_.z_amplitude;
  const double s = std::sin(w * t), c = std::cos(w * t);
  const double s2 = std::sin(2 * w * t), c2 = std::cos(2 * w * t);
  p(0) = A * s;
  p(1) = A * s * c;
  p(2) = h + Az * s2;
  v(0) = A * w * c;
  v(1) = A * w * (c * c - s * s);
  v(2) = Az * 2 * w * c2;
  a(0) = -A * w * w * s;
  a(1) = A * w * w * (-4.0 * s * c);
  a(2) = -Az * 4 * w * w * s2;
}

void TrajectoryGenerator::circle(double t, Vec3& p, Vec3& v, Vec3& a) const noexcept {
  const double A = params_.amplitude;
  const double w = params_.omega;
  const double h = params_.height;
  const double ph = w * t;
  p(0) = A * std::cos(ph);
  p(1) = A * std::sin(ph);
  p(2) = h;
  v(0) = -A * w * std::sin(ph);
  v(1) = A * w * std::cos(ph);
  v(2) = 0.0;
  a(0) = -A * w * w * std::cos(ph);
  a(1) = -A * w * w * std::sin(ph);
  a(2) = 0.0;
}

void TrajectoryGenerator::step(double t, Vec3& p, Vec3& v, Vec3& a) const noexcept {
  // Minimum-jerk blend from step_from to step_to over 1.5 s after step_time.
  const double T = 1.5;
  double s = (t - params_.step_time) / T;
  if (s <= 0.0) {
    p = params_.step_from;
    v.setZero();
    a.setZero();
    return;
  }
  if (s >= 1.0) {
    p = params_.step_to;
    v.setZero();
    a.setZero();
    return;
  }
  const double s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
  const double poly = 10 * s3 - 15 * s4 + 6 * s5;
  const double dpoly = (30 * s2 - 60 * s3 + 30 * s4) / T;
  const double ddpoly = (60 * s - 180 * s2 + 120 * s3) / (T * T);
  const Vec3 d = params_.step_to - params_.step_from;
  p = params_.step_from + poly * d;
  v = dpoly * d;
  a = ddpoly * d;
}

void TrajectoryGenerator::hover(double t, Vec3& p, Vec3& v, Vec3& a) const noexcept {
  (void)t;
  p = params_.hover_point;
  v.setZero();
  a.setZero();
}

void TrajectoryGenerator::flatToState(const Vec3& p, const Vec3& v, const Vec3& a,
                                      StateVector& x, ControlVector& u) const noexcept {
  const Vec3& g = model_.params().gravity;
  const double m = model_.params().mass;

  // Desired thrust vector (world): m * (a - g).
  const Vec3 f_des = m * (a - g);
  const double fT = f_des.norm();
  Vec3 zb = Vec3(0, 0, 1);
  if (fT > 1e-9) zb = f_des / fT;

  // Yaw-projected frame.
  const double psi = params_.yaw;
  const Vec3 xc(std::cos(psi), std::sin(psi), 0.0);
  Vec3 yb = zb.cross(xc);
  if (yb.norm() < 1e-6) {
    yb = Vec3(-std::sin(psi), std::cos(psi), 0.0);
  } else {
    yb.normalize();
  }
  const Vec3 xb = yb.cross(zb);

  Mat3 R;
  R.col(0) = xb;
  R.col(1) = yb;
  R.col(2) = zb;
  Eigen::Quaterniond qe(R);
  qe.normalize();

  x.segment<3>(StateIndex::kPx) = p;
  x.segment<3>(StateIndex::kVx) = v;
  x(StateIndex::kQw) = qe.w();
  x(StateIndex::kQx) = qe.x();
  x(StateIndex::kQy) = qe.y();
  x(StateIndex::kQz) = qe.z();
  x.segment<3>(StateIndex::kWx).setZero();

  u(InputIndex::kThrust) = fT;
  u(InputIndex::kTaux) = 0.0;
  u(InputIndex::kTauy) = 0.0;
  u(InputIndex::kTauz) = 0.0;
}

}  // namespace quadrotor::controller
