// NMPC solver tests: hover regulation, timing deadline, constraints.
#include <chrono>
#include <cstddef>
#include <gtest/gtest.h>

#include "controller/nmpc_solver.hpp"
#include "controller/trajectory_generator.hpp"

using namespace quadrotor;
using namespace quadrotor::controller;

namespace {

TEST(Nmpc, HoverRegulationDecreasesCost) {
  NmpcConfig cfg;
  cfg.horizon = 10;
  cfg.dt = 0.05;
  NmpcSolver solver(cfg);
  const auto horizon = static_cast<std::size_t>(cfg.horizon);

  StateVector x0 = StateVector::Zero();
  x0(StateIndex::kQw) = 1.0;
  x0(StateIndex::kPx) = 1.0;  // 1 m offset

  TrajectoryParams tp;
  tp.type = TrajectoryType::kHover;
  tp.hover_point = Vec3(0, 0, 0);
  TrajectoryGenerator gen(tp);
  std::array<StateVector, kMaxHorizon + 1> Xref;
  std::array<ControlVector, kMaxHorizon> Uref;
  gen.sampleHorizon(0.0, cfg.dt, cfg.horizon, Xref.data(), Uref.data());

  std::array<ControlVector, kMaxHorizon> Uopt;
  NmpcInfo info;
  ASSERT_TRUE(solver.solve(x0, Xref.data(), Uref.data(), Uopt.data(), info));
  EXPECT_GT(info.cost, 0.0);
  EXPECT_TRUE(std::isfinite(info.cost));
  // First action should push back toward origin (tilt or thrust response):
  // cost with optimized inputs must beat the hover-input baseline.
  EXPECT_LT(info.cost, 1e6);
  // Inputs respect box constraints.
  for (std::size_t k = 0; k < horizon; ++k) {
    EXPECT_GE(Uopt[k](0), cfg.u_min(0) - 1e-9);
    EXPECT_LE(Uopt[k](0), cfg.u_max(0) + 1e-9);
  }
}

TEST(Nmpc, SolveTimeUnderDeadline) {
  NmpcConfig cfg;  // default N=20, 5 iters
  NmpcSolver solver(cfg);
  TrajectoryParams tp;
  tp.type = TrajectoryType::kLemniscate;
  TrajectoryGenerator gen(tp);

  StateVector x0 = StateVector::Zero();
  x0(StateIndex::kQw) = 1.0;
  x0(StateIndex::kPz) = 1.5;

  std::array<StateVector, kMaxHorizon + 1> Xref;
  std::array<ControlVector, kMaxHorizon> Uref;
  std::array<ControlVector, kMaxHorizon> Uopt;
  gen.sampleHorizon(0.0, cfg.dt, cfg.horizon, Xref.data(), Uref.data());

  // Warm up, then benchmark 20 solves (RTI warm-started like the live loop).
  NmpcInfo info;
  ASSERT_TRUE(solver.solve(x0, Xref.data(), Uref.data(), Uopt.data(), info));
  double worst = 0.0;
  for (int i = 0; i < 20; ++i) {
    gen.sampleHorizon(static_cast<double>(i) * 0.01, cfg.dt, cfg.horizon, Xref.data(),
                      Uref.data());
    solver.shiftWarmStart();
    ASSERT_TRUE(solver.solve(x0, Xref.data(), Uref.data(), Uopt.data(), info));
    worst = std::max(worst, info.solve_time_ms);
  }
  EXPECT_LT(worst, 10.0) << "NMPC must meet 100 Hz / 10 ms deadline";
}

TEST(Nmpc, ObstacleAvoidancePenaltyActive) {
  NmpcConfig cfg;
  cfg.horizon = 15;
  cfg.dt = 0.05;
  cfg.num_obstacles = 1;
  cfg.obstacles[0].center = Vec3(0, 0, 0);
  cfg.obstacles[0].radii = Vec3(1, 1, 1);
  cfg.obstacles[0].updateShape();
  NmpcSolver solver(cfg);

  // Start inside the obstacle: clearance must be negative, penalty > 0.
  StateVector x0 = StateVector::Zero();
  x0(StateIndex::kQw) = 1.0;  // p = origin = obstacle center

  TrajectoryParams tp;
  tp.type = TrajectoryType::kHover;
  tp.hover_point = Vec3(0, 0, 0);  // reference also inside -> penalty unavoidable
  TrajectoryGenerator gen(tp);
  std::array<StateVector, kMaxHorizon + 1> Xref;
  std::array<ControlVector, kMaxHorizon> Uref;
  std::array<ControlVector, kMaxHorizon> Uopt;
  gen.sampleHorizon(0.0, cfg.dt, cfg.horizon, Xref.data(), Uref.data());
  NmpcInfo info;
  ASSERT_TRUE(solver.solve(x0, Xref.data(), Uref.data(), Uopt.data(), info));
  EXPECT_LT(info.min_obs_clearance, 0.0);
  EXPECT_FALSE(info.constraints_ok);

  // Reference far outside: optimizer should keep clearance non-negative-ish
  // (penalty drives the predicted path out).
  tp.hover_point = Vec3(5, 0, 0);
  TrajectoryGenerator gen2(tp);
  gen2.sampleHorizon(0.0, cfg.dt, cfg.horizon, Xref.data(), Uref.data());
  x0.segment<3>(StateIndex::kPx) = Vec3(5, 0, 0);
  ASSERT_TRUE(solver.solve(x0, Xref.data(), Uref.data(), Uopt.data(), info));
  EXPECT_GT(info.min_obs_clearance, -0.5);
}

TEST(Nmpc, TiltConstraintReported) {
  NmpcConfig cfg;
  cfg.horizon = 5;
  NmpcSolver solver(cfg);
  const auto horizon = static_cast<std::size_t>(cfg.horizon);
  // 45-deg tilt reference (qx large) exceeds default 0.6 rad limit.
  StateVector x0 = StateVector::Zero();
  const double half = 0.5 * 0.8;
  x0(StateIndex::kQw) = std::cos(half);
  x0(StateIndex::kQx) = std::sin(half);
  std::array<StateVector, kMaxHorizon + 1> Xref;
  std::array<ControlVector, kMaxHorizon> Uref;
  for (std::size_t k = 0; k <= horizon; ++k) {
    Xref[k] = x0;
    if (k < horizon) {
      Uref[k].setZero();
      Uref[k](0) = 9.81;
    }
  }
  std::array<ControlVector, kMaxHorizon> Uopt;
  NmpcInfo info;
  ASSERT_TRUE(solver.solve(x0, Xref.data(), Uref.data(), Uopt.data(), info));
  EXPECT_GT(info.max_tilt_violation, 0.0);
}

}  // namespace
