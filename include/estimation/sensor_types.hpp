// SPDX-License-Identifier: MIT
// Sensor message types (fixed-size, trivially copyable, no allocation).
#pragma once

#include "dynamics/types.hpp"

namespace quadrotor::estimation {

struct ImuMeasurement {
  Vec3 accel{Vec3::Zero()};  // specific force [m/s^2], body frame
  Vec3 gyro{Vec3::Zero()};   // angular rate [rad/s], body frame
  double timestamp{0.0};     // [s]
};

struct PoseMeasurement {
  Vec3 position{Vec3::Zero()};               // world frame [m]
  Vec4 orientation{Vec4(1, 0, 0, 0)};        // unit quaternion wxyz, world->body? body-in-world
  Mat3 pos_cov{Mat3::Identity() * 1e-4};     // measurement covariance
  Mat3 att_cov{Mat3::Identity() * 1e-4};     // attitude error covariance [rad^2]
  double timestamp{0.0};
  bool valid{true};
};

// Process / measurement noise parameters (continuous-time densities).
struct EkfNoiseParams {
  double accel_noise{1.0e-3};       // [m/s^2 / sqrt(Hz)] -> variance density
  double gyro_noise{1.0e-4};        // [rad/s / sqrt(Hz)]
  double accel_bias_rw{1.0e-5};     // bias random walk [m/s^2 / sqrt(Hz)]
  double gyro_bias_rw{1.0e-6};      // [rad/s / sqrt(Hz)]
  double init_pos_std{0.05};
  double init_vel_std{0.05};
  double init_att_std{0.02};        // [rad]
  double init_ba_std{0.05};
  double init_bg_std{0.01};
};

}  // namespace quadrotor::estimation
