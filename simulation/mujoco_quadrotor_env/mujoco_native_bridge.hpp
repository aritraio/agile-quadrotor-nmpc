// SPDX-License-Identifier: MIT
// Optional native MuJoCo bridge for quadrotor.xml.
//
// When MuJoCo is available (CMake finds <mujoco/mujoco.h> and defines
// QUADROTOR_HAS_MUJOCO), this bridge steps the full MJCF model with visual
// parity (airframe, 4 thruster sites, ground, ellipsoidal obstacle).
// Otherwise it compiles to a stub reporting available() == false so callers
// gracefully fall back to simulation::QuadrotorEnv (standalone harness).
//
// Wrapper hot path performs no heap allocation; MuJoCo owns its mjModel/mjData.
#pragma once

#include <array>
#include <cstddef>
#include <string>

#include "dynamics/types.hpp"

#ifdef QUADROTOR_HAS_MUJOCO
#include <mujoco/mujoco.h>
#endif

namespace quadrotor::simulation {

class MujocoNativeBridge {
 public:
  MujocoNativeBridge() = default;
  ~MujocoNativeBridge();

  MujocoNativeBridge(const MujocoNativeBridge&) = delete;
  MujocoNativeBridge& operator=(const MujocoNativeBridge&) = delete;

  /// Load an MJCF model. Returns false with err filled when MuJoCo is absent
  /// or the file cannot be parsed.
  bool load(const std::string& mjcf_path, std::string& err) noexcept;
  [[nodiscard]] bool available() const noexcept { return loaded_; }
  [[nodiscard]] double time() const noexcept { return time_; }

  void reset(const StateVector& x0) noexcept;

  /// Step by dt_ctrl (subdivided into MJCF 2 ms steps). Maps collective +
  /// torques to 4 rotor thrusts via the X4 allocation inverse.
  /// No heap allocation on call.
  void step(const ControlVector& u, double dt_ctrl) noexcept;

  void state(StateVector& x) const noexcept;
  void rotorThrusts(double (&f)[4]) const noexcept {
    f[0] = last_rotor_[0];
    f[1] = last_rotor_[1];
    f[2] = last_rotor_[2];
    f[3] = last_rotor_[3];
  }

 private:
  void syncStateFromMj() noexcept;
  void applyControl(const ControlVector& u) noexcept;

  bool loaded_{false};
  double time_{0.0};
  StateVector cached_{StateVector::Zero()};
  std::array<double, 4> last_rotor_{{0.0, 0.0, 0.0, 0.0}};
#ifdef QUADROTOR_HAS_MUJOCO
  mjModel* model_{nullptr};
  mjData* data_{nullptr};
#endif
};

}  // namespace quadrotor::simulation
