// Closed-loop node: ES-EKF (500 Hz IMU / 30 Hz VO) -> TrajectoryGenerator ->
// NMPC (100 Hz) -> QuadrotorEnv plant. Logs CSV metrics per control cycle.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

#include "controller/nmpc_solver.hpp"
#include "controller/trajectory_generator.hpp"
#include "dynamics/quadrotor_model.hpp"
#include "dynamics/types.hpp"
#include "estimation/es_ekf.hpp"
#include "quadrotor_env.hpp"

using namespace quadrotor;

namespace {

double AttitudeErrorDeg(const Vec4& q_true, const Vec4& q_est) {
  Vec4 a = q_true, b = q_est;
  a.normalize();
  b.normalize();
  const double d = std::abs(a.dot(b));
  const double ang = 2.0 * std::acos(std::min(1.0, d));
  return ang * 180.0 / M_PI;
}

}  // namespace

int main(int argc, char** argv) {
  std::string traj = "lemniscate";
  double sim_time = 10.0;
  std::string log_path = "logs/flight_log.csv";
  if (argc > 1) traj = argv[1];
  if (argc > 2) sim_time = std::stod(argv[2]);
  if (argc > 3) log_path = argv[3];

  // --- Plant & estimator setup -------------------------------------------
  dynamics::QuadrotorParams plant = dynamics::DefaultQuadParams();
  simulation::QuadrotorEnv env(plant);

  StateVector x0 = StateVector::Zero();
  x0(StateIndex::kQw) = 1.0;
  x0(StateIndex::kPz) = 1.5;
  env.reset(x0);

  simulation::WindConfig wind;
  wind.base = Vec3(0.3, 0.0, 0.0);
  wind.gust_amp = 1.0;
  wind.gust_freq = 0.5;
  wind.turbulence_std = 0.05;
  env.setWind(wind);
  simulation::SensorNoiseConfig sens;
  sens.accel_std = 0.02;
  sens.gyro_std = 0.002;
  sens.accel_bias = Vec3(0.02, -0.015, 0.01);
  sens.gyro_bias = Vec3(0.002, -0.001, 0.0015);
  sens.pos_std = 0.01;
  sens.att_std = 0.005;
  sens.vo_rate_hz = 30.0;
  env.setSensorNoise(sens);
  env.setSeed(7);

  estimation::EkfNoiseParams ekf_noise;
  ekf_noise.accel_noise = 0.02;
  ekf_noise.gyro_noise = 0.002;
  estimation::ErrorStateEKF ekf(ekf_noise);

  // --- Trajectory + NMPC ---------------------------------------------------
  controller::TrajectoryParams tp;
  if (traj == "hover")
    tp.type = controller::TrajectoryType::kHover;
  else if (traj == "step")
    tp.type = controller::TrajectoryType::kStep;
  else if (traj == "circle")
    tp.type = controller::TrajectoryType::kCircle;
  else
    tp.type = controller::TrajectoryType::kLemniscate;
  tp.amplitude = 2.0;
  tp.omega = 0.8;
  tp.height = 1.5;
  tp.hover_point = Vec3(0, 0, 1.5);
  controller::TrajectoryGenerator traj_gen(tp, plant);

  // Start on-trajectory for fair tracking metrics (avoids acquisition transient).
  {
    StateVector xr0;
    ControlVector ur0;
    traj_gen.reference(0.0, xr0, ur0);
    x0 = xr0;
    env.reset(x0);
  }

  estimation::NominalState init_nom2;
  init_nom2.p = x0.segment<3>(StateIndex::kPx);
  init_nom2.v = x0.segment<3>(StateIndex::kVx);
  init_nom2.q = x0.segment<4>(StateIndex::kQw);
  ekf.reset(init_nom2);

  controller::NmpcConfig nmpc_cfg;
  nmpc_cfg.horizon = 20;
  nmpc_cfg.dt = 0.05;
  nmpc_cfg.num_obstacles = 1;
  nmpc_cfg.obstacles[0].center = Vec3(3.0, 0.0, 1.5);
  nmpc_cfg.obstacles[0].radii = Vec3(0.8, 0.8, 1.2);
  nmpc_cfg.obstacles[0].updateShape();
  controller::NmpcSolver nmpc(nmpc_cfg, plant);

  // --- Timing ---------------------------------------------------------------
  constexpr double kDtCtrl = 0.01;  // 100 Hz
  constexpr double kDtImu = 0.002;  // 500 Hz
  const int steps = static_cast<int>(sim_time / kDtCtrl);

  std::ofstream log(log_path);
  log << "t,refx,refy,refz,px,py,pz,estx,esty,estz,att_err_deg,solve_ms,cost\n";

  double sum_sq = 0.0, sum_att = 0.0, worst_ms = 0.0, sum_ms = 0.0;
  long long n = 0;
  Vec3 last_gyro = Vec3::Zero();

  std::array<StateVector, controller::kMaxHorizon + 1> Xref;
  std::array<ControlVector, controller::kMaxHorizon> Uref;
  std::array<ControlVector, controller::kMaxHorizon> Uopt;
  for (auto& u : Uopt) u = nmpc_cfg.u_hover;

  double t = 0.0;
  for (int k = 0; k < steps; ++k) {
    // 1) Estimate -> build quad state for control.
    StateVector x_hat;
    ekf.toQuadState(x_hat, last_gyro);

    // 2) Reference horizon + solve.
    traj_gen.sampleHorizon(t, nmpc_cfg.dt, nmpc_cfg.horizon, Xref.data(), Uref.data());
    controller::NmpcInfo info;
    // Warm-start from previous solution each cycle (RTI-style).
    if (k > 0) nmpc.shiftWarmStart();
    const bool ok = nmpc.solve(x_hat, Xref.data(), Uref.data(), Uopt.data(), info);
    if (!ok) {
      std::fprintf(stderr, "[main_node] NMPC solve failed at t=%.2f\n", t);
      for (auto& u : Uopt) u = nmpc_cfg.u_hover;
    }
    const ControlVector u_cmd = Uopt[0];

    // 3) Step plant at 100 Hz (5x 500 Hz substeps inside env).
    env.step(u_cmd, kDtCtrl, 5);
    t += kDtCtrl;

    // 4) High-rate IMU propagation (5x per control cycle).
    for (int s = 0; s < 5; ++s) {
      const double ti = t - kDtCtrl + s * kDtImu;
      auto imu = env.imu(ti + kDtImu);
      last_gyro = imu.gyro;
      ekf.propagate(imu, kDtImu);
    }
    // 5) Low-rate VO update.
    auto vo = env.vo(t);
    if (vo.valid) ekf.update(vo);

    // 6) Metrics.
    const StateVector& xt = env.state();
    StateVector xr;
    ControlVector ur;
    traj_gen.reference(t, xr, ur);
    const double e = (xt.segment<3>(0) - xr.segment<3>(0)).norm();
    const double ae =
        AttitudeErrorDeg(xt.segment<4>(6), ekf.nominal().q);
    sum_sq += e * e;
    sum_att += ae;
    sum_ms += info.solve_time_ms;
    worst_ms = std::max(worst_ms, info.solve_time_ms);
    ++n;
    log << t << "," << xr(0) << "," << xr(1) << "," << xr(2) << "," << xt(0) << ","
        << xt(1) << "," << xt(2) << "," << ekf.nominal().p(0) << ","
        << ekf.nominal().p(1) << "," << ekf.nominal().p(2) << "," << ae << ","
        << info.solve_time_ms << "," << info.cost << "\n";

    if (k % 100 == 0) {
      std::printf("[t=%5.2fs] |e|=%.3f m att=%.2f deg solve=%.2f ms cost=%.1f %s\n", t,
                  e, ae, info.solve_time_ms, info.cost,
                  info.constraints_ok ? "" : "[CONSTRAINT]");
    }
  }

  const double denom = static_cast<double>(std::max(1LL, n));
  const double rmse = std::sqrt(sum_sq / denom);
  std::printf("\n==== Flight summary (%s, %.1fs) ====\n", traj.c_str(), sim_time);
  std::printf("Tracking RMSE : %.4f m\n", rmse);
  std::printf("Mean att err  : %.3f deg\n", sum_att / denom);
  std::printf("Solve mean/max: %.3f / %.3f ms (deadline 10 ms)\n", sum_ms / denom,
              worst_ms);
  std::printf("Log written to %s\n", log_path.c_str());
  return 0;
}
