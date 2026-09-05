// 15-state ES-EKF unit tests: consistency, convergence, bias estimation.
#include <cmath>
#include <gtest/gtest.h>
#include <random>

#include "dynamics/quadrotor_model.hpp"
#include "estimation/es_ekf.hpp"

using namespace quadrotor;
using namespace quadrotor::estimation;

namespace {

TEST(EsEkf, CovarianceSymmetricPSD) {
  ErrorStateEKF ekf;
  NominalState s;
  s.p << 1, 2, 3;
  ekf.reset(s);
  const ErrorMatrix& P = ekf.covariance();
  EXPECT_TRUE(P.isApprox(P.transpose(), 1e-12));
  Eigen::SelfAdjointEigenSolver<ErrorMatrix> es(P);
  EXPECT_GE(es.eigenvalues().minCoeff(), -1e-12);

  // Propagate + update keep symmetry/PSD.
  ImuMeasurement imu;
  imu.accel << 0, 0, 9.81;
  imu.gyro.setZero();
  for (int i = 0; i < 10; ++i) ASSERT_TRUE(ekf.propagate(imu, 0.002));
  PoseMeasurement z;
  z.position << 1, 2, 3;
  z.orientation << 1, 0, 0, 0;
  z.pos_cov.setIdentity();
  z.pos_cov *= 1e-4;
  z.att_cov.setIdentity();
  z.att_cov *= 1e-6;
  ASSERT_TRUE(ekf.update(z));
  const ErrorMatrix& P2 = ekf.covariance();
  EXPECT_TRUE(P2.isApprox(P2.transpose(), 1e-9));
  Eigen::SelfAdjointEigenSolver<ErrorMatrix> es2(P2);
  EXPECT_GE(es2.eigenvalues().minCoeff(), -1e-9);
}

TEST(EsEkf, StaticHoverConvergesWithNoisyImu) {
  EkfNoiseParams noise;
  noise.accel_noise = 0.02;
  noise.gyro_noise = 0.002;
  noise.accel_bias_rw = 1e-5;
  noise.gyro_bias_rw = 1e-6;
  ErrorStateEKF ekf(noise);

  NominalState init;
  init.p << 0.5, -0.3, 0.2;  // offset from truth
  init.v << 0.1, 0.1, -0.1;
  ekf.reset(init);

  std::mt19937 rng(0);
  std::normal_distribution<double> na(0.0, noise.accel_noise * std::sqrt(500.0));
  std::normal_distribution<double> ng(0.0, noise.gyro_noise * std::sqrt(500.0));
  std::normal_distribution<double> np(0.0, 0.02);
  std::normal_distribution<double> nt(0.0, 0.01);

  constexpr double dt = 0.002;
  double t = 0.0;
  for (int i = 0; i < 2500; ++i) {  // 5 s at 500 Hz
    ImuMeasurement imu;
    // Hover truth: specific force = [0,0,9.81], rate = 0.
    imu.accel << na(rng), na(rng), 9.81 + na(rng);
    imu.gyro << ng(rng), ng(rng), ng(rng);
    imu.timestamp = t;
    ASSERT_TRUE(ekf.propagate(imu, dt));
    t += dt;
    if (i % 17 == 0) {  // ~30 Hz VO
      PoseMeasurement z;
      z.position << np(rng), np(rng), np(rng);
      Vec3 eta(nt(rng), nt(rng), nt(rng));
      const double an = eta.norm();
      Vec4 dq;
      if (an < 1e-12)
        dq << 1, 0, 0, 0;
      else
        dq << std::cos(an / 2), std::sin(an / 2) * eta(0) / an,
            std::sin(an / 2) * eta(1) / an, std::sin(an / 2) * eta(2) / an;
      z.orientation = dq;  // truth attitude is identity
      z.pos_cov.setIdentity();
      z.pos_cov *= 0.02 * 0.02;
      z.att_cov.setIdentity();
      z.att_cov *= 0.01 * 0.01;
      z.timestamp = t;
      ASSERT_TRUE(ekf.update(z));
    }
  }
  // Converged near truth (origin, identity).
  EXPECT_LT(ekf.nominal().p.norm(), 0.15);
  EXPECT_LT(ekf.nominal().v.norm(), 0.15);
  Vec4 q = ekf.nominal().q;
  if (q(0) < 0) q = -q;
  const double ang = 2.0 * std::acos(std::min(1.0, std::abs(q(0))));
  EXPECT_LT(ang, 5.0 * M_PI / 180.0);  // < 5 deg
}

TEST(EsEkf, BiasEstimation) {
  EkfNoiseParams noise;
  noise.accel_noise = 0.01;
  noise.gyro_noise = 0.001;
  noise.accel_bias_rw = 1e-4;
  noise.gyro_bias_rw = 1e-5;
  noise.init_ba_std = 0.1;
  noise.init_bg_std = 0.02;
  ErrorStateEKF ekf(noise);
  ekf.reset(NominalState{});

  const Vec3 true_ba(0.1, -0.08, 0.05);
  const Vec3 true_bg(0.01, -0.015, 0.008);

  std::mt19937 rng(1);
  // NOTE: std-dev per sample = density * sqrt(rate).
  std::normal_distribution<double> na(0.0, noise.accel_noise * std::sqrt(500.0));
  std::normal_distribution<double> ng(0.0, noise.gyro_noise * std::sqrt(500.0));
  std::normal_distribution<double> np(0.0, 0.01);

  constexpr double dt = 0.002;
  double t = 0.0;
  for (int i = 0; i < 5000; ++i) {  // 10 s
    ImuMeasurement imu;
    imu.accel << true_ba(0) + na(rng), true_ba(1) + na(rng), 9.81 + true_ba(2) + na(rng);
    imu.gyro << true_bg(0) + ng(rng), true_bg(1) + ng(rng), true_bg(2) + ng(rng);
    imu.timestamp = t;
    ASSERT_TRUE(ekf.propagate(imu, dt));
    t += dt;
    if (i % 17 == 0) {
      PoseMeasurement z;
      z.position << np(rng), np(rng), np(rng);
      z.orientation << 1, 0, 0, 0;
      z.pos_cov.setIdentity();
      z.pos_cov *= 1e-4;
      z.att_cov.setIdentity();
      z.att_cov *= 2.5e-5;
      ASSERT_TRUE(ekf.update(z));
    }
  }
  // Biases partially observable through pose; expect error well below initial.
  EXPECT_LT((ekf.nominal().ba - true_ba).norm(), 0.08);
  EXPECT_LT((ekf.nominal().bg - true_bg).norm(), 0.02);
}

TEST(EsEkf, RejectsInvalidInputs) {
  ErrorStateEKF ekf;
  ImuMeasurement imu;
  imu.accel << 0, 0, 9.81;
  EXPECT_FALSE(ekf.propagate(imu, -0.001));
  EXPECT_FALSE(ekf.propagate(imu, 0.0));
  PoseMeasurement z;
  z.valid = false;
  EXPECT_FALSE(ekf.update(z));
  // Error reset semantics: after an update the filter remains consistent.
  z.valid = true;
  z.position.setZero();
  z.orientation << 1, 0, 0, 0;
  z.pos_cov.setIdentity();
  z.pos_cov *= 1e-4;
  z.att_cov.setIdentity();
  z.att_cov *= 1e-4;
  EXPECT_TRUE(ekf.update(z));
  Eigen::SelfAdjointEigenSolver<ErrorMatrix> es(ekf.covariance());
  EXPECT_GE(es.eigenvalues().minCoeff(), -1e-9);
}

}  // namespace
