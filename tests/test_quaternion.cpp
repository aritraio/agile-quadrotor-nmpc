// SO(3) quaternion kinematics unit tests.
#include <Eigen/Dense>
#include <cmath>
#include <gtest/gtest.h>

#include "dynamics/quadrotor_model.hpp"

using namespace quadrotor;
using namespace quadrotor::dynamics;

namespace {

constexpr double kTol = 1e-9;

TEST(Quaternion, RotationMatrixIsSO3) {
  // Identity.
  Vec4 q0(1, 0, 0, 0);
  Mat3 R = QuadrotorModel::rotationMatrix(q0);
  EXPECT_TRUE(R.isApprox(Mat3::Identity(), kTol));
  EXPECT_NEAR(R.determinant(), 1.0, 1e-12);

  // 90 deg about z: (w=cos45, z=sin45).
  const double c = std::cos(M_PI / 4.0), s = std::sin(M_PI / 4.0);
  Vec4 qz(c, 0, 0, s);
  R = QuadrotorModel::rotationMatrix(qz);
  Mat3 Rz;
  Rz << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  EXPECT_TRUE(R.isApprox(Rz, 1e-12));
  EXPECT_TRUE((R.transpose() * R).isApprox(Mat3::Identity(), 1e-12));
  EXPECT_NEAR(R.determinant(), 1.0, 1e-12);

  // Random unit quats stay on SO(3).
  Vec4 qr(0.5, 0.5, 0.5, 0.5);
  QuadrotorModel::normalizeQuaternion(qr);
  R = QuadrotorModel::rotationMatrix(qr);
  EXPECT_TRUE((R.transpose() * R).isApprox(Mat3::Identity(), 1e-12));
  EXPECT_NEAR(R.determinant(), 1.0, 1e-12);
}

TEST(Quaternion, FrameTransformation) {
  // Body x-axis maps to world y under +90 deg yaw.
  const double c = std::cos(M_PI / 4.0), s = std::sin(M_PI / 4.0);
  Vec4 qz(c, 0, 0, s);
  const Mat3 R = QuadrotorModel::rotationMatrix(qz);
  const Vec3 bx(1, 0, 0);
  const Vec3 wy = R * bx;
  EXPECT_TRUE(wy.isApprox(Vec3(0, 1, 0), 1e-12));
  // Thrust axis invariant under yaw.
  EXPECT_TRUE((R * Vec3(0, 0, 1)).isApprox(Vec3(0, 0, 1), 1e-12));
}

TEST(Quaternion, DerivativeMatchesFiniteDifference) {
  Vec4 q(1, 0, 0, 0);
  Vec3 w(0.3, -0.5, 1.2);
  Vec4 qdot;
  QuadrotorModel::quaternionDerivative(q, w, qdot);
  // qdot for identity should be 0.5*[0, w].
  EXPECT_NEAR(qdot(0), 0.0, 1e-12);
  EXPECT_NEAR(qdot(1), 0.5 * w(0), 1e-12);
  EXPECT_NEAR(qdot(2), 0.5 * w(1), 1e-12);
  EXPECT_NEAR(qdot(3), 0.5 * w(2), 1e-12);

  // Finite-difference check at a non-trivial attitude.
  Vec4 q1(0.7071, 0.0, 0.7071, 0.0);
  QuadrotorModel::normalizeQuaternion(q1);
  QuadrotorModel::quaternionDerivative(q1, w, qdot);
  constexpr double eps = 1e-8;
  // Integrate q + eps*qdot vs quaternion exp.
  Vec4 qpert = q1 + eps * qdot;
  QuadrotorModel::normalizeQuaternion(qpert);
  // Exact increment: q (+) exp(w*eps/2).
  const double n = w.norm();
  Vec4 dq;
  dq(0) = std::cos(0.5 * n * eps);
  dq.tail<3>() = std::sin(0.5 * n * eps) * w / n;
  Vec4 qexact;
  QuadrotorModel::quaternionMultiply(q1, dq, qexact);
  QuadrotorModel::normalizeQuaternion(qexact);
  EXPECT_LT((qpert - qexact).norm(), 1e-6);
}

TEST(Quaternion, MultiplyAndConjugateAreInverse) {
  Vec4 a(0.8, 0.2, 0.4, 0.4);
  Vec4 b(0.3, -0.5, 0.6, 0.55);
  QuadrotorModel::normalizeQuaternion(a);
  QuadrotorModel::normalizeQuaternion(b);
  Vec4 ab, ainv, back;
  QuadrotorModel::quaternionMultiply(a, b, ab);
  QuadrotorModel::quaternionConjugate(a, ainv);
  QuadrotorModel::quaternionMultiply(ainv, ab, back);
  // Shortest-arc sign.
  if (back(0) < 0) back = -back;
  Vec4 bn = b;
  if (bn(0) < 0) bn = -bn;
  EXPECT_LT((back - bn).norm(), 1e-12);
}

TEST(Quaternion, NormalizationPreservation) {
  // Constant-rate spin for 10 s must keep |q| = 1 to machine precision
  // when renormalized per RK4 substep (mirrors integrator behavior).
  Vec4 q(1, 0, 0, 0);
  const Vec3 w(0.0, 0.0, 1.0);
  constexpr double dt = 0.002;
  for (int i = 0; i < 5000; ++i) {
    Vec4 qdot;
    QuadrotorModel::quaternionDerivative(q, w, qdot);
    q += dt * qdot;
    QuadrotorModel::normalizeQuaternion(q);
  }
  EXPECT_NEAR(q.norm(), 1.0, 1e-12);
  // After 10 s at 1 rad/s about z, yaw = 10 rad mod 2pi.
  const double yaw = 10.0;
  Vec4 qexp(std::cos(yaw / 2), 0, 0, std::sin(yaw / 2));
  QuadrotorModel::normalizeQuaternion(qexp);
  Vec4 qcmp = q, qe = qexp;
  if (qcmp(0) * qe(0) < 0) qcmp = -qcmp;
  EXPECT_LT((qcmp - qe).norm(), 1e-3);
}

}  // namespace
