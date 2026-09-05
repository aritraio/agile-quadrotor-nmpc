// Dual-backend parity: CasADi-exported C (via wrapper) vs native C++ model.
// Verifies identical RK4 maps and OCP costs; benchmarks horizon rollout.
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <random>
#include <gtest/gtest.h>

#include "controller/casadi_nmpc_wrapper.hpp"
#include "controller/nmpc_solver.hpp"
#include "controller/trajectory_generator.hpp"
#include "dynamics/quadrotor_model.hpp"
#include "dynamics/rk4_integrator.hpp"

using namespace quadrotor;
using namespace quadrotor::controller;
using namespace quadrotor::controller::generated;

namespace {

ParamArray NativeParamsToArray(const dynamics::QuadrotorParams& p) {
  ParamArray a{};
  a[0] = p.mass;
  a[1] = p.inertia(0, 0);
  a[2] = p.inertia(1, 1);
  a[3] = p.inertia(2, 2);
  a[4] = p.drag(0);
  a[5] = p.drag(1);
  a[6] = p.drag(2);
  a[7] = p.gravity(0);
  a[8] = p.gravity(1);
  a[9] = p.gravity(2);
  a[10] = 0.6;
  a[11] = 500.0;
  a[12] = 2000.0;
  return a;
}

TEST(CasadiBackend, Rk4MatchesNative) {
  const dynamics::QuadrotorParams prm = dynamics::DefaultQuadParams();
  const dynamics::QuadrotorModel model(prm);
  const dynamics::RK4Integrator rk4;
  const ParamArray p = NativeParamsToArray(prm);

  std::mt19937 rng(42);
  std::uniform_real_distribution<double> uni_p(-2.0, 2.0);
  std::uniform_real_distribution<double> uni_v(-3.0, 3.0);
  std::uniform_real_distribution<double> uni_w(-2.0, 2.0);
  std::uniform_real_distribution<double> uni_u(-1.0, 1.0);

  for (int trial = 0; trial < 20; ++trial) {
    StateVector x;
    x << uni_p(rng), uni_p(rng), uni_p(rng), uni_v(rng), uni_v(rng), uni_v(rng),
        uni_p(rng), uni_p(rng), uni_p(rng), uni_p(rng), uni_w(rng), uni_w(rng),
        uni_w(rng);
    Vec4 q = x.segment<4>(StateIndex::kQw);
    dynamics::QuadrotorModel::normalizeQuaternion(q);
    x.segment<4>(StateIndex::kQw) = q;
    ControlVector u;
    u << 9.81 + uni_u(rng) * 5.0, uni_u(rng), uni_u(rng), uni_u(rng);

    StateVector xn_native;
    rk4.step(model, x, u, 0.05, xn_native);
    StateVector xn_gen;
    CasadiGeneratedModel::rk4StepEigen(x, u, p, 0.05, xn_gen);
    // Sign-agnostic quaternion compare (q ~ -q).
    double pos_err = (xn_native.segment<3>(0) - xn_gen.segment<3>(0)).norm();
    double vel_err = (xn_native.segment<3>(3) - xn_gen.segment<3>(3)).norm();
    double ome_err = (xn_native.segment<3>(10) - xn_gen.segment<3>(10)).norm();
    Vec4 qn = xn_native.segment<4>(6);
    Vec4 qg = xn_gen.segment<4>(6);
    const double qerr = std::min((qn - qg).norm(), (qn + qg).norm());
    EXPECT_LT(pos_err, 1e-9) << "trial " << trial;
    EXPECT_LT(vel_err, 1e-9) << "trial " << trial;
    EXPECT_LT(ome_err, 1e-9) << "trial " << trial;
    EXPECT_LT(qerr, 1e-9) << "trial " << trial;
  }
}

TEST(CasadiBackend, CostMatchesNativeFormulation) {
  // Reference diagonal weights matching NmpcConfig defaults.
  std::array<double, 12> qw{};
  for (std::size_t i = 0; i < 3U; ++i) qw[i] = 20.0;
  for (std::size_t i = 3U; i < 6U; ++i) qw[i] = 5.0;
  for (std::size_t i = 6U; i < 9U; ++i) qw[i] = 8.0;
  for (std::size_t i = 9U; i < 12U; ++i) qw[i] = 0.5;
  std::array<double, 12> qw_term{};
  for (std::size_t i = 0; i < 12U; ++i) qw_term[i] = qw[i] * 5.0;
  const std::array<double, 4> rw{0.05, 0.05, 0.05, 0.05};

  StateVector x = StateVector::Zero();
  x(StateIndex::kQw) = 1.0;
  x(StateIndex::kPx) = 0.5;
  StateVector xr = StateVector::Zero();
  xr(StateIndex::kQw) = 1.0;
  ControlVector u;
  u << 10.0, 0.1, -0.05, 0.02;
  ControlVector ur;
  ur << 9.81, 0, 0, 0;

  // Manual Eigen reference (same math as NmpcSolver::stageCost without penalties).
  TrackErrorVector e;
  e.segment<3>(0) = x.segment<3>(0) - xr.segment<3>(0);
  e.segment<3>(3) = x.segment<3>(3) - xr.segment<3>(3);
  e.segment<3>(6).setZero();  // identity vs identity
  e.segment<3>(9) = x.segment<3>(10) - xr.segment<3>(10);
  double expected = 0.0;
  for (std::size_t i = 0; i < 12U; ++i) expected += qw[i] * e[static_cast<Eigen::Index>(i)] * e[static_cast<Eigen::Index>(i)];
  for (std::size_t i = 0; i < 4U; ++i) {
    const double du = u(static_cast<Eigen::Index>(i)) - ur(static_cast<Eigen::Index>(i));
    expected += rw[i] * du * du;
  }
  const double got = CasadiGeneratedModel::stageCost(
      x.data(), u.data(), xr.data(), ur.data(), qw.data(), rw.data(), 0.6, 500.0,
      nullptr, nullptr, 0, 2000.0);
  EXPECT_NEAR(got, expected, 1e-9);

  // Tilt penalty active case: 0.8 rad pitch exceeds 0.6 limit.
  StateVector xt = StateVector::Zero();
  const double half = 0.5 * 0.8;
  xt(StateIndex::kQw) = std::cos(half);
  xt(StateIndex::kQx) = std::sin(half);
  const double tilt_cost = CasadiGeneratedModel::stageCost(
      xt.data(), ur.data(), xt.data(), ur.data(), qw.data(), rw.data(), 0.6, 500.0,
      nullptr, nullptr, 0, 2000.0);
  EXPECT_GT(tilt_cost, 0.0);

  // Obstacle penalty: inside unit sphere.
  const std::array<double, 3> obsc{0.0, 0.0, 0.0};
  const std::array<double, 3> obsa{1.0, 1.0, 1.0};
  StateVector xo = StateVector::Zero();
  xo(StateIndex::kQw) = 1.0;
  const double obs_cost = CasadiGeneratedModel::stageCost(
      xo.data(), ur.data(), xo.data(), ur.data(), qw.data(), rw.data(), 0.6, 500.0,
      obsc.data(), obsa.data(), 1, 2000.0);
  EXPECT_NEAR(obs_cost, 2000.0, 1e-6);
  (void)qw_term;
}

TEST(CasadiBackend, HorizonRolloutBenchmark) {
  const dynamics::QuadrotorParams prm = dynamics::DefaultQuadParams();
  const ParamArray p = NativeParamsToArray(prm);
  TrajectoryParams tp;
  tp.type = TrajectoryType::kLemniscate;
  TrajectoryGenerator gen(tp, prm);
  constexpr std::size_t kN = 20;
  constexpr double kDt = 0.05;
  std::array<StateVector, kMaxHorizon + 1> Xref;
  std::array<ControlVector, kMaxHorizon> Uref;
  gen.sampleHorizon(0.0, kDt, static_cast<int>(kN), Xref.data(), Uref.data());

  StateVector x0 = Xref[0];
  // Time N-step rollout through the generated backend (the cost-dominant kernel
  // shared with any CasADi/acados SQP iteration).
  auto t0 = std::chrono::steady_clock::now();
  double worst_ms = 0.0;
  for (int rep = 0; rep < 100; ++rep) {
    auto ti = std::chrono::steady_clock::now();
    std::array<double, 13> xc{};
    std::array<double, 13> xn{};
    for (std::size_t i = 0; i < 13U; ++i) xc[i] = x0(static_cast<Eigen::Index>(i));
    double acc = 0.0;
    std::array<double, 12> qw{};
    for (std::size_t i = 0; i < 12U; ++i) qw[i] = 1.0;
    const std::array<double, 4> rw{0.05, 0.05, 0.05, 0.05};
    for (std::size_t k = 0; k < kN; ++k) {
      CasadiGeneratedModel::rk4Step(xc.data(), Uref[k].data(), p.data(), kDt, xn.data());
      acc += CasadiGeneratedModel::stageCost(xn.data(), Uref[k].data(), Xref[k + 1U].data(),
                                             Uref[k].data(), qw.data(), rw.data(), 0.6,
                                             500.0, nullptr, nullptr, 0, 2000.0);
      xc = xn;
    }
    auto tj = std::chrono::steady_clock::now();
    worst_ms = std::max(worst_ms, std::chrono::duration<double, std::milli>(tj - ti).count());
    EXPECT_TRUE(std::isfinite(acc));
  }
  auto t1 = std::chrono::steady_clock::now();
  const double mean_ms =
      std::chrono::duration<double, std::milli>(t1 - t0).count() / 100.0;
  EXPECT_LT(worst_ms, 10.0) << "generated rollout must meet 100 Hz deadline";
  EXPECT_LT(mean_ms, 10.0);
}

}  // namespace
