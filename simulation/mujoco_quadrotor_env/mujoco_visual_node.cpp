// Visual closed-loop node: prefers the native MuJoCo MJCF bridge when
// available, otherwise falls back to the standalone QuadrotorEnv harness.
// Usage: mujoco_visual_node [traj] [sim_time] [log_path] [mjcf_path]
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

#include "controller/nmpc_solver.hpp"
#include "controller/trajectory_generator.hpp"
#include "dynamics/quadrotor_model.hpp"
#include "dynamics/types.hpp"
#include "estimation/es_ekf.hpp"
#include "mujoco_native_bridge.hpp"
#include "quadrotor_env.hpp"

using namespace quadrotor;

namespace {

double AttitudeErrorDeg(const Vec4& q_true, const Vec4& q_est) {
  Vec4 a = q_true;
  Vec4 b = q_est;
  a.normalize();
  b.normalize();
  const double d = std::abs(a.dot(b));
  const double ang = 2.0 * std::acos(std::min(1.0, d));
  return ang * 180.0 / M_PI;
}

}  // namespace

int main(int argc, char** argv) {
  std::string traj = "lemniscate";
  double sim_time = 8.0;
  std::string log_path = "logs/visual_log.csv";
  std::string mjcf = "simulation/mujoco_quadrotor_env/quadrotor.xml";
  if (argc > 1) traj = argv[1];
  if (argc > 2) sim_time = std::stod(argv[2]);
  if (argc > 3) log_path = argv[3];
  if (argc > 4) mjcf = argv[4];

  const dynamics::QuadrotorParams plant = dynamics::DefaultQuadParams();

  controller::TrajectoryParams tp;
  if (traj == "hover") {
    tp.type = controller::TrajectoryType::kHover;
  } else if (traj == "step") {
    tp.type = controller::TrajectoryType::kStep;
  } else if (traj == "circle") {
    tp.type = controller::TrajectoryType::kCircle;
  } else {
    tp.type = controller::TrajectoryType::kLemniscate;
  }
  tp.amplitude = 2.0;
  tp.omega = 0.8;
  tp.height = 1.5;
  tp.hover_point = Vec3(0, 0, 1.5);
  controller::TrajectoryGenerator traj_gen(tp, plant);

  StateVector xr0;
  ControlVector ur0;
  traj_gen.reference(0.0, xr0, ur0);

  // Prefer native MuJoCo; fall back to the standalone harness.
  simulation::MujocoNativeBridge mj_bridge;
  std::string mj_err;
  const bool use_mujoco = mj_bridge.load(mjcf, mj_err);
  simulation::QuadrotorEnv fallback(plant);
  if (use_mujoco) {
    mj_bridge.reset(xr0);
    std::printf("[visual] MuJoCo backend active (%s)\n", mjcf.c_str());
  } else {
    fallback.reset(xr0);
    simulation::WindConfig wind;
    wind.base = Vec3(0.3, 0.0, 0.0);
    wind.gust_amp = 1.0;
    wind.gust_freq = 0.5;
    fallback.setWind(wind);
    std::printf("[visual] fallback backend (%s)\n", mj_err.c_str());
  }

  estimation::EkfNoiseParams ekf_noise;
  estimation::ErrorStateEKF ekf(ekf_noise);
  estimation::NominalState init_nom;
  init_nom.p = xr0.segment<3>(StateIndex::kPx);
  init_nom.v = xr0.segment<3>(StateIndex::kVx);
  init_nom.q = xr0.segment<4>(StateIndex::kQw);
  ekf.reset(init_nom);

  controller::NmpcConfig nmpc_cfg;
  controller::NmpcSolver nmpc(nmpc_cfg, plant);

  constexpr double kDtCtrl = 0.01;
  constexpr double kDtImu = 0.002;
  const auto steps = static_cast<int>(sim_time / kDtCtrl);
  std::ofstream log(log_path);
  log << "t,refx,refy,refz,px,py,pz,backend\n";

  double sum_sq = 0.0;
  double worst_ms = 0.0;
  long long n = 0;
  Vec3 last_gyro = Vec3::Zero();
  std::array<StateVector, controller::kMaxHorizon + 1> Xref;
  std::array<ControlVector, controller::kMaxHorizon> Uref;
  std::array<ControlVector, controller::kMaxHorizon> Uopt;
  for (auto& u : Uopt) u = nmpc_cfg.u_hover;

  double t = 0.0;
  for (int k = 0; k < steps; ++k) {
    StateVector x_hat;
    ekf.toQuadState(x_hat, last_gyro);
    traj_gen.sampleHorizon(t, nmpc_cfg.dt, nmpc_cfg.horizon, Xref.data(), Uref.data());
    controller::NmpcInfo info;
    if (k > 0) nmpc.shiftWarmStart();
    (void)nmpc.solve(x_hat, Xref.data(), Uref.data(), Uopt.data(), info);
    const ControlVector u_cmd = Uopt[0];

    StateVector xt;
    if (use_mujoco) {
      mj_bridge.step(u_cmd, kDtCtrl);
      mj_bridge.state(xt);
      t += kDtCtrl;
      // Propagate estimator with ideal kinematics-derived IMU (visual mode).
      estimation::ImuMeasurement imu;
      imu.accel = Vec3(0.0, 0.0, 9.81);
      imu.gyro = xt.segment<3>(StateIndex::kWx);
      last_gyro = imu.gyro;
      for (int s = 0; s < 5; ++s) ekf.propagate(imu, kDtImu);
    } else {
      fallback.step(u_cmd, kDtCtrl, 5);
      t += kDtCtrl;
      for (int s = 0; s < 5; ++s) {
        const double ti = t - kDtCtrl + static_cast<double>(s) * kDtImu;
        auto imu = fallback.imu(ti + kDtImu);
        last_gyro = imu.gyro;
        ekf.propagate(imu, kDtImu);
      }
      auto vo = fallback.vo(t);
      if (vo.valid) ekf.update(vo);
      xt = fallback.state();
    }

    StateVector xr;
    ControlVector ur;
    traj_gen.reference(t, xr, ur);
    const double e = (xt.segment<3>(0) - xr.segment<3>(0)).norm();
    sum_sq += e * e;
    worst_ms = std::max(worst_ms, info.solve_time_ms);
    ++n;
    log << t << "," << xr(0) << "," << xr(1) << "," << xr(2) << "," << xt(0) << ","
        << xt(1) << "," << xt(2) << "," << (use_mujoco ? "mujoco" : "fallback") << "\n";
  }
  const double denom = static_cast<double>(n > 0 ? n : 1);
  std::printf("[visual] RMSE %.4f m worst %.3f ms backend %s\n", std::sqrt(sum_sq / denom),
              worst_ms, use_mujoco ? "mujoco" : "fallback");
  (void)AttitudeErrorDeg;
  return 0;
}
