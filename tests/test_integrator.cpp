// RK4 integrator + 6-DoF dynamics unit tests.
#include <cmath>
#include <gtest/gtest.h>

#include "dynamics/quadrotor_model.hpp"
#include "dynamics/rk4_integrator.hpp"

using namespace quadrotor;
using namespace quadrotor::dynamics;

namespace {

StateVector HoverState(const QuadrotorParams& /*p*/, const Vec3& pos = Vec3::Zero()) {
  StateVector x = StateVector::Zero();
  x.segment<3>(StateIndex::kPx) = pos;
  x(StateIndex::kQw) = 1.0;
  return x;
}

ControlVector HoverInput(const QuadrotorParams& p) {
  ControlVector u = ControlVector::Zero();
  u(0) = p.mass * 9.81;
  return u;
}

TEST(Integrator, HoverIsEquilibrium) {
  const QuadrotorParams prm = DefaultQuadParams();
  QuadrotorModel model(prm);
  RK4Integrator rk4;
  const StateVector x0 = HoverState(prm);
  const ControlVector u = HoverInput(prm);
  StateVector x1;
  rk4.step(model, x0, u, 0.01, x1);
  EXPECT_TRUE(x1.segment<3>(StateIndex::kPx).isApprox(x0.segment<3>(StateIndex::kPx), 1e-9));
  EXPECT_TRUE(x1.segment<3>(StateIndex::kVx).isApprox(Vec3::Zero(), 1e-9));
  EXPECT_NEAR(x1.segment<4>(StateIndex::kQw).norm(), 1.0, 1e-12);
  EXPECT_TRUE(x1.segment<3>(StateIndex::kWx).isApprox(Vec3::Zero(), 1e-9));
}

TEST(Integrator, FreeFallMatchesAnalytic) {
  // No drag, no thrust: v(t) = g t, p(t) = 0.5 g t^2 (RK4 is exact here).
  QuadrotorParams prm = DefaultQuadParams();
  prm.drag.setZero();
  QuadrotorModel model(prm);
  RK4Integrator rk4;
  StateVector x = HoverState(prm);
  ControlVector u = ControlVector::Zero();
  constexpr double dt = 0.01;
  constexpr int n = 100;  // 1 s
  StateVector out;
  rk4.integrate(model, x, u, dt, n, out);
  EXPECT_NEAR(out(StateIndex::kVz), -9.81, 1e-6);
  EXPECT_NEAR(out(StateIndex::kPz), -0.5 * 9.81, 1e-6);
}

TEST(Integrator, DragDeceleratesAsExpected) {
  // Zero gravity + zero thrust: m dv = -D v -> v(t) = v0 exp(-D/m t).
  QuadrotorParams prm = DefaultQuadParams();
  prm.gravity.setZero();
  QuadrotorModel model(prm);
  RK4Integrator rk4;
  StateVector x = StateVector::Zero();
  x(StateIndex::kQw) = 1.0;
  x(StateIndex::kVx) = 5.0;
  ControlVector u = ControlVector::Zero();
  constexpr double dt = 0.005;
  StateVector out;
  rk4.integrate(model, x, u, dt, 200, out);  // 1 s
  const double expected = 5.0 * std::exp(-prm.drag(0) / prm.mass * 1.0);
  EXPECT_NEAR(out(StateIndex::kVx), expected, 1e-4);
  EXPECT_LT(std::abs(out(StateIndex::kVx)), 5.0);  // energy dissipated
  // Transverse axes remain zero.
  EXPECT_NEAR(out(StateIndex::kVy), 0.0, 1e-9);
  EXPECT_NEAR(out(StateIndex::kVz), 0.0, 1e-9);
}

TEST(Integrator, ConstantVelocityWithoutForces) {
  // No gravity, no drag: position integrates exactly.
  QuadrotorParams prm = DefaultQuadParams();
  prm.gravity.setZero();
  prm.drag.setZero();
  QuadrotorModel model(prm);
  RK4Integrator rk4;
  StateVector x = StateVector::Zero();
  x(StateIndex::kQw) = 1.0;
  x.segment<3>(StateIndex::kVx) << 1.0, 2.0, 3.0;
  ControlVector u = ControlVector::Zero();
  StateVector out;
  rk4.integrate(model, x, u, 0.01, 100, out);
  EXPECT_TRUE(out.segment<3>(StateIndex::kPx).isApprox(Vec3(1, 2, 3), 1e-9));
}

TEST(Integrator, QuaternionNormPreservedOverAgileMotion) {
  const QuadrotorParams prm = DefaultQuadParams();
  QuadrotorModel model(prm);
  RK4Integrator rk4;
  StateVector x = StateVector::Zero();
  x(StateIndex::kQw) = 1.0;
  x.segment<3>(StateIndex::kWx) << 2.0, 1.0, 3.0;
  ControlVector u;
  u << prm.mass * 9.81, 0.5, -0.3, 0.2;
  StateVector cur = x, nxt;
  for (int i = 0; i < 1000; ++i) {
    rk4.step(model, cur, u, 0.002, nxt);
    cur = nxt;
    ASSERT_NEAR(cur.segment<4>(StateIndex::kQw).norm(), 1.0, 1e-9) << "step " << i;
  }
}

TEST(Integrator, AngularDynamicsConservesMomentumWithoutTorque) {
  // Spherical inertia + zero torque: |w| constant (Euler equations reduce to wdot=0
  // for isotropic J; use near-isotropic check on magnitude for our J).
  QuadrotorParams prm = DefaultQuadParams();
  prm.gravity.setZero();
  prm.drag.setZero();
  QuadrotorModel model(prm);
  RK4Integrator rk4;
  StateVector x = StateVector::Zero();
  x(StateIndex::kQw) = 1.0;
  x.segment<3>(StateIndex::kWx) << 0.0, 0.0, 5.0;  // pure yaw spin, Jz axis
  ControlVector u = ControlVector::Zero();
  StateVector out;
  rk4.integrate(model, x, u, 0.002, 500, out);
  EXPECT_NEAR(out.segment<3>(StateIndex::kWx).norm(), 5.0, 1e-6);
}

}  // namespace
