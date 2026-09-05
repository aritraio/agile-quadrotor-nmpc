/// SPDX-License-Identifier: MIT
#include "quadrotor_env.hpp"

#include <cmath>

namespace quadrotor::simulation {

QuadrotorEnv::QuadrotorEnv(const dynamics::QuadrotorParams& params) : model_(params) {
  x_.setZero();
  x_(StateIndex::kQw) = 1.0;
  last_u_.setZero();
  last_u_(0) = params.mass * (-params.gravity(2));
}

void QuadrotorEnv::reset(const StateVector& x0) noexcept {
  x_ = x0;
  Vec4 q = x_.segment<4>(StateIndex::kQw);
  dynamics::QuadrotorModel::normalizeQuaternion(q);
  x_.segment<4>(StateIndex::kQw) = q;
  t_ = 0.0;
  step_count_ = 0;
  last_u_.setZero();
  last_u_(0) = model_.params().mass * (-model_.params().gravity(2));
}

void QuadrotorEnv::setSeed(std::uint32_t seed) noexcept {
  rng_.seed(seed);
  gauss_.reset();
}

void QuadrotorEnv::step(const ControlVector& u, double dt_ctrl, int n_sub) noexcept {
  if (n_sub < 1) n_sub = 1;
  const double dt = dt_ctrl / n_sub;
  // Clamp inputs to actuator limits (collective + per-axis torque).
  ControlVector uc = u;
  const auto& prm = model_.params();
  if (uc(0) < 0.0) uc(0) = 0.0;
  if (uc(0) > prm.max_thrust) uc(0) = prm.max_thrust;
  for (int i = 1; i < 4; ++i) {
    if (uc(i) < -prm.max_torque) uc(i) = -prm.max_torque;
    if (uc(i) > prm.max_torque) uc(i) = prm.max_torque;
  }
  StateVector nxt;
  last_u_ = uc;
  for (int i = 0; i < n_sub; ++i) {
    const Vec3 vw = windAt(t_);
    // RK4 with wind-aware drag: implement manual RK4 here since the model
    // functor signature lacks wind. Uses same Butcher tableau, no allocation.
    StateVector k1, k2, k3, k4, xt;
    dynamicsWithWind(x_, uc, vw, k1);
    xt = x_ + 0.5 * dt * k1;
    dynamicsWithWind(xt, uc, windAt(t_ + 0.5 * dt), k2);
    xt = x_ + 0.5 * dt * k2;
    dynamicsWithWind(xt, uc, windAt(t_ + 0.5 * dt), k3);
    xt = x_ + dt * k3;
    dynamicsWithWind(xt, uc, windAt(t_ + dt), k4);
    nxt = x_ + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    Vec4 q = nxt.segment<4>(StateIndex::kQw);
    dynamics::QuadrotorModel::normalizeQuaternion(q);
    nxt.segment<4>(StateIndex::kQw) = q;
    x_ = nxt;
    t_ += dt;
    ++step_count_;
  }
}

estimation::ImuMeasurement QuadrotorEnv::imu(double t) noexcept {
  estimation::ImuMeasurement m;
  // Exact specific force from stored collective thrust + relative airflow:
  //   vdot - g = R e3 fT/m - D(v - vw)/m  =>  f_body = R^T(vdot - g).
  const Vec3 v = x_.segment<3>(StateIndex::kVx);
  const Vec4 q = x_.segment<4>(StateIndex::kQw);
  Mat3 R;
  dynamics::QuadrotorModel::rotationMatrix(q, R);
  const Vec3 vw = wind_.base;  // mean component only (gust phase unknown to sensor)
  const Vec3 v_rel = v - vw;
  const auto& prm = model_.params();
  const Vec3 drag(prm.drag(0) * v_rel(0), prm.drag(1) * v_rel(1), prm.drag(2) * v_rel(2));
  const Vec3 e3f(0.0, 0.0, last_u_(0) / prm.mass);
  const Vec3 f_specific = e3f - R.transpose() * drag / prm.mass;
  m.accel = f_specific + sens_.accel_bias;
  const Vec3 w = x_.segment<3>(StateIndex::kWx);
  m.gyro = w + sens_.gyro_bias;
  // Additive white noise.
  for (int i = 0; i < 3; ++i) {
    m.accel(i) += gauss_(rng_) * sens_.accel_std;
    m.gyro(i) += gauss_(rng_) * sens_.gyro_std;
  }
  m.timestamp = t;
  return m;
}

estimation::PoseMeasurement QuadrotorEnv::vo(double t) noexcept {
  estimation::PoseMeasurement z;
  z.timestamp = t;
  const double period = 1.0 / sens_.vo_rate_hz;
  const long long tick = static_cast<long long>(std::floor(t / period + 1e-9));
  // Only emit on VO ticks (within half a physics substep of the tick time).
  const double tick_t = static_cast<double>(tick) * period;
  if (std::abs(t - tick_t) > 1e-9) {
    z.valid = false;
    return z;
  }
  if (sens_.vo_dropout_prob > 0.0) {
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    if (uni(rng_) < sens_.vo_dropout_prob) {
      z.valid = false;
      return z;
    }
  }
  z.valid = true;
  z.position = x_.segment<3>(StateIndex::kPx);
  Vec4 q = x_.segment<4>(StateIndex::kQw);
  dynamics::QuadrotorModel::normalizeQuaternion(q);
  // Position noise.
  for (int i = 0; i < 3; ++i) z.position(i) += gauss_(rng_) * sens_.pos_std;
  // Attitude noise: small random rotation.
  Vec3 eta(gauss_(rng_) * sens_.att_std, gauss_(rng_) * sens_.att_std,
           gauss_(rng_) * sens_.att_std);
  const double ang = eta.norm();
  Vec4 dq;
  if (ang < 1e-12) {
    dq << 1.0, 0.5 * eta(0), 0.5 * eta(1), 0.5 * eta(2);
  } else {
    dq << std::cos(0.5 * ang), std::sin(0.5 * ang) * eta(0) / ang,
        std::sin(0.5 * ang) * eta(1) / ang, std::sin(0.5 * ang) * eta(2) / ang;
  }
  Vec4 qn;
  dynamics::QuadrotorModel::quaternionMultiply(q, dq, qn);
  dynamics::QuadrotorModel::normalizeQuaternion(qn);
  z.orientation = qn;
  z.pos_cov.setZero();
  z.att_cov.setZero();
  z.pos_cov.diagonal().setConstant(sens_.pos_std * sens_.pos_std);
  z.att_cov.diagonal().setConstant(sens_.att_std * sens_.att_std);
  return z;
}

void QuadrotorEnv::rotationMatrix(const StateVector& x, Mat3& R) noexcept {
  dynamics::QuadrotorModel::rotationMatrix(x.segment<4>(StateIndex::kQw), R);
}

Vec3 QuadrotorEnv::windAt(double t) noexcept {
  Vec3 vw = wind_.base;
  if (wind_.gust_amp != 0.0) {
    vw(0) += wind_.gust_amp * std::sin(wind_.gust_freq * t);
    vw(1) += 0.5 * wind_.gust_amp * std::sin(1.7 * wind_.gust_freq * t + 1.3);
  }
  if (wind_.turbulence_std != 0.0) {
    vw(0) += gauss_(rng_) * wind_.turbulence_std;
    vw(1) += gauss_(rng_) * wind_.turbulence_std;
    vw(2) += 0.5 * gauss_(rng_) * wind_.turbulence_std;
  }
  return vw;
}

void QuadrotorEnv::dynamicsWithWind(const StateVector& x, const ControlVector& u,
                                    const Vec3& vw, StateVector& xdot) const noexcept {
  const auto v = x.segment<3>(StateIndex::kVx);
  const Vec4 q = x.segment<4>(StateIndex::kQw);
  const auto w = x.segment<3>(StateIndex::kWx);
  const auto& prm = model_.params();
  Mat3 R;
  dynamics::QuadrotorModel::rotationMatrix(q, R);
  xdot.segment<3>(StateIndex::kPx) = v;
  const Vec3 v_rel = v - vw;
  const Vec3 drag(prm.drag(0) * v_rel(0), prm.drag(1) * v_rel(1), prm.drag(2) * v_rel(2));
  const Vec3 thrust_world = R.col(2) * u(0);
  xdot.segment<3>(StateIndex::kVx) =
      prm.gravity + thrust_world / prm.mass - drag / prm.mass;
  Vec4 qdot;
  dynamics::QuadrotorModel::quaternionDerivative(q, w, qdot);
  xdot.segment<4>(StateIndex::kQw) = qdot;
  const Vec3 tau = u.segment<3>(1);
  const Vec3 Jw = prm.inertia * w;
  xdot.segment<3>(StateIndex::kWx) = prm.inertia_inv * (tau - w.cross(Jw));
}

}  // namespace quadrotor::simulation
