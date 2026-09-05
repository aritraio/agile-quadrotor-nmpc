// SPDX-License-Identifier: MIT
// Standalone quadrotor physics environment (MuJoCo-API-compatible harness).
// Wraps QuadrotorModel + RK4 at 500 Hz substeps, adds Dryden-like wind gusts
// and IMU/VO sensor noise. No heap allocation on the hot path.
#pragma once

#include <array>
#include <cstdint>
#include <random>

#include "dynamics/quadrotor_model.hpp"
#include "dynamics/types.hpp"
#include "estimation/sensor_types.hpp"

namespace quadrotor::simulation {

struct WindConfig {
  Vec3 base{Vec3::Zero()};      // mean wind [m/s], world
  double gust_amp{0.0};         // sinusoidal gust amplitude [m/s]
  double gust_freq{0.5};        // [rad/s]
  double turbulence_std{0.0};   // white turbulence [m/s]
};

struct SensorNoiseConfig {
  double accel_std{0.02};   // [m/s^2]
  double gyro_std{0.001};   // [rad/s]
  Vec3 accel_bias{Vec3::Zero()};
  Vec3 gyro_bias{Vec3::Zero()};
  double pos_std{0.01};         // VO [m]
  double att_std{0.005};        // VO attitude [rad]
  double vo_rate_hz{30.0};
  double vo_dropout_prob{0.0};
};

class QuadrotorEnv {
 public:
  explicit QuadrotorEnv(const dynamics::QuadrotorParams& params =
                            dynamics::DefaultQuadParams());

  void reset(const StateVector& x0) noexcept;
  [[nodiscard]] const StateVector& state() const noexcept { return x_; }
  [[nodiscard]] double time() const noexcept { return t_; }

  void setWind(const WindConfig& w) noexcept { wind_ = w; }
  void setSensorNoise(const SensorNoiseConfig& s) noexcept { sens_ = s; }
  void setSeed(std::uint32_t seed) noexcept;

  // Step physics by dt_ctrl with zero-order-hold input, using n_sub RK4
  // substeps. Applies wind as an external velocity disturbance via
  // relative-airflow drag: F_drag = -D (v - v_wind).
  void step(const ControlVector& u, double dt_ctrl, int n_sub = 5) noexcept;

  // Ideal IMU (specific force + rate) plus bias/noise.
  estimation::ImuMeasurement imu(double t) noexcept;
  // VO pose at ~vo_rate_hz; returns invalid measurement on dropout/off-tick.
  estimation::PoseMeasurement vo(double t) noexcept;

  // Ground-truth helpers for metrics.
  static void rotationMatrix(const StateVector& x, Mat3& R) noexcept;

 private:
  Vec3 windAt(double t) noexcept;
  void dynamicsWithWind(const StateVector& x, const ControlVector& u, const Vec3& vw,
                        StateVector& xdot) const noexcept;

  dynamics::QuadrotorModel model_;
  StateVector x_;
  ControlVector last_u_{ControlVector::Zero()};
  double t_{0.0};
  WindConfig wind_;
  SensorNoiseConfig sens_;
  std::mt19937 rng_{42};
  std::normal_distribution<double> gauss_{0.0, 1.0};
  long long step_count_{0};
};

}  // namespace quadrotor::simulation
