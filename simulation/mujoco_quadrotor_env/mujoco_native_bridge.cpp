// SPDX-License-Identifier: MIT
#include "mujoco_native_bridge.hpp"

#include <cmath>
#include <cstddef>
#include <cstring>

namespace quadrotor::simulation {

MujocoNativeBridge::~MujocoNativeBridge() {
#ifdef QUADROTOR_HAS_MUJOCO
  if (data_ != nullptr) {
    mj_deleteData(data_);
    data_ = nullptr;
  }
  if (model_ != nullptr) {
    mj_deleteModel(model_);
    model_ = nullptr;
  }
#endif
}

bool MujocoNativeBridge::load(const std::string& mjcf_path, std::string& err) noexcept {
#ifdef QUADROTOR_HAS_MUJOCO
  char error[1024] = {0};
  mjModel* m = mj_loadXML(mjcf_path.c_str(), nullptr, error, sizeof(error));
  if (m == nullptr) {
    err.assign(error);
    return false;
  }
  mjData* d = mj_makeData(m);
  if (d == nullptr) {
    mj_deleteModel(m);
    err = "mj_makeData failed";
    return false;
  }
  if (model_ != nullptr) mj_deleteModel(model_);
  if (data_ != nullptr) mj_deleteData(data_);
  model_ = m;
  data_ = d;
  loaded_ = true;
  time_ = 0.0;
  err.clear();
  return true;
#else
  (void)mjcf_path;
  err = "MuJoCo not available: fallback to standalone QuadrotorEnv";
  loaded_ = false;
  return false;
#endif
}

void MujocoNativeBridge::reset(const StateVector& x0) noexcept {
  cached_ = x0;
  time_ = 0.0;
  last_rotor_ = {{0.0, 0.0, 0.0, 0.0}};
#ifdef QUADROTOR_HAS_MUJOCO
  if (!loaded_) return;
  // qpos: [px(3), quat(wxyz 4)] + rotor hinges(0); qvel: [v(3), w(3)] + hinges.
  data_->qpos[0] = x0(StateIndex::kPx);
  data_->qpos[1] = x0(StateIndex::kPy);
  data_->qpos[2] = x0(StateIndex::kPz);
  data_->qpos[3] = x0(StateIndex::kQw);
  data_->qpos[4] = x0(StateIndex::kQx);
  data_->qpos[5] = x0(StateIndex::kQy);
  data_->qpos[6] = x0(StateIndex::kQz);
  data_->qvel[0] = x0(StateIndex::kVx);
  data_->qvel[1] = x0(StateIndex::kVy);
  data_->qvel[2] = x0(StateIndex::kVz);
  data_->qvel[3] = x0(StateIndex::kWx);
  data_->qvel[4] = x0(StateIndex::kWy);
  data_->qvel[5] = x0(StateIndex::kWz);
  for (int i = 6; i < model_->nv && i < 10; ++i) data_->qvel[i] = 0.0;
  for (int i = 7; i < model_->nq && i < 11; ++i) data_->qpos[i] = 0.0;
  mj_forward(model_, data_);
  syncStateFromMj();
#endif
}

void MujocoNativeBridge::applyControl(const ControlVector& u) noexcept {
  // X4 allocation inverse: fT/4 +/- roll/pitch/yaw mixes (arm L = 0.19 m).
  constexpr double kArm = 0.19;
  const double ft = u(0);
  const double tx = u(1);
  const double ty = u(2);
  const double tz = u(3);
  double f0 = ft * 0.25 + tx * 0.25 / kArm + ty * 0.25 / kArm + tz * 0.25;
  double f1 = ft * 0.25 + tx * 0.25 / kArm - ty * 0.25 / kArm - tz * 0.25;
  double f2 = ft * 0.25 - tx * 0.25 / kArm + ty * 0.25 / kArm - tz * 0.25;
  double f3 = ft * 0.25 - tx * 0.25 / kArm - ty * 0.25 / kArm + tz * 0.25;
  if (f0 < 0.0) f0 = 0.0;
  if (f1 < 0.0) f1 = 0.0;
  if (f2 < 0.0) f2 = 0.0;
  if (f3 < 0.0) f3 = 0.0;
  last_rotor_ = {{f0, f1, f2, f3}};
#ifdef QUADROTOR_HAS_MUJOCO
  if (!loaded_) return;
  const int n = model_->nu < 4 ? model_->nu : 4;
  const double f[4] = {f0, f1, f2, f3};
  for (int i = 0; i < n; ++i) data_->ctrl[i] = f[i];
#endif
}

void MujocoNativeBridge::step(const ControlVector& u, double dt_ctrl) noexcept {
#ifdef QUADROTOR_HAS_MUJOCO
  if (!loaded_) return;
  applyControl(u);
  const double h = (model_->opt.timestep > 0.0) ? model_->opt.timestep : 0.002;
  double remaining = dt_ctrl;
  while (remaining > 1e-12) {
    const double hstep = (remaining < h) ? remaining : h;
    (void)hstep;
    mj_step(model_, data_);
    remaining -= h;
  }
  time_ = data_->time;
  syncStateFromMj();
#else
  (void)u;
  (void)dt_ctrl;
#endif
}

void MujocoNativeBridge::state(StateVector& x) const noexcept { x = cached_; }

void MujocoNativeBridge::syncStateFromMj() noexcept {
#ifdef QUADROTOR_HAS_MUJOCO
  if (!loaded_) return;
  cached_(StateIndex::kPx) = data_->qpos[0];
  cached_(StateIndex::kPy) = data_->qpos[1];
  cached_(StateIndex::kPz) = data_->qpos[2];
  cached_(StateIndex::kQw) = data_->qpos[3];
  cached_(StateIndex::kQx) = data_->qpos[4];
  cached_(StateIndex::kQy) = data_->qpos[5];
  cached_(StateIndex::kQz) = data_->qpos[6];
  cached_(StateIndex::kVx) = data_->qvel[0];
  cached_(StateIndex::kVy) = data_->qvel[1];
  cached_(StateIndex::kVz) = data_->qvel[2];
  cached_(StateIndex::kWx) = data_->qvel[3];
  cached_(StateIndex::kWy) = data_->qvel[4];
  cached_(StateIndex::kWz) = data_->qvel[5];
  Vec4 q = cached_.segment<4>(StateIndex::kQw);
  const double n = q.norm();
  if (n > 1e-12) {
    cached_.segment<4>(StateIndex::kQw) /= n;
  } else {
    cached_(StateIndex::kQw) = 1.0;
    cached_(StateIndex::kQx) = 0.0;
    cached_(StateIndex::kQy) = 0.0;
    cached_(StateIndex::kQz) = 0.0;
  }
  time_ = data_->time;
#endif
}

}  // namespace quadrotor::simulation
