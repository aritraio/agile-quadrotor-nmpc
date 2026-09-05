// SPDX-License-Identifier: MIT
// Differential-flatness trajectory generator: position + yaw -> full
// 13-state + feedforward collective thrust. Allocation-free sampling.
#pragma once

#include <array>

#include "controller/nmpc_solver.hpp"  // for kMaxHorizon
#include "dynamics/quadrotor_model.hpp"
#include "dynamics/types.hpp"

namespace quadrotor::controller {

enum class TrajectoryType : int { kHover = 0, kLemniscate = 1, kStep = 2, kCircle = 3 };

struct TrajectoryParams {
  TrajectoryType type{TrajectoryType::kLemniscate};
  double amplitude{2.0};     // lemniscate/circle radius [m]
  double omega{0.8};         // base frequency [rad/s]
  double height{1.5};        // nominal height [m]
  double z_amplitude{0.3};   // vertical excitation [m]
  double yaw{0.0};           // fixed yaw [rad]
  Vec3 hover_point{Vec3(0, 0, 1.5)};
  Vec3 step_from{Vec3(0, 0, 1.5)};
  Vec3 step_to{Vec3(2, 0, 1.5)};
  double step_time{2.0};  // switch time [s]
};

class TrajectoryGenerator {
 public:
  explicit TrajectoryGenerator(const TrajectoryParams& p = TrajectoryParams{},
                               const dynamics::QuadrotorParams& plant =
                                   dynamics::DefaultQuadParams()) noexcept;

  void setParams(const TrajectoryParams& p) noexcept { params_ = p; }
  void setModel(const dynamics::QuadrotorParams& m) noexcept { model_.setParams(m); }

  // Sample flat outputs (p, v, a) at time t.
  void flatOutputs(double t, Vec3& pos, Vec3& vel, Vec3& acc) const noexcept;

  // Full reference state + feedforward input at time t.
  void reference(double t, StateVector& x_ref, ControlVector& u_ref) const noexcept;

  // Fill horizon refs Xref[0..N], Uref[0..N-1] starting at t0 with step dt.
  void sampleHorizon(double t0, double dt, int N, StateVector* Xref,
                     ControlVector* Uref) const noexcept;

 private:
  void lemniscate(double t, Vec3& p, Vec3& v, Vec3& a) const noexcept;
  void circle(double t, Vec3& p, Vec3& v, Vec3& a) const noexcept;
  void step(double t, Vec3& p, Vec3& v, Vec3& a) const noexcept;
  void hover(double t, Vec3& p, Vec3& v, Vec3& a) const noexcept;
  void flatToState(const Vec3& p, const Vec3& v, const Vec3& a, StateVector& x,
                   ControlVector& u) const noexcept;

  TrajectoryParams params_;
  dynamics::QuadrotorModel model_;
};

}  // namespace quadrotor::controller
