// SPDX-License-Identifier: MIT
// Explicit 4th-order Runge-Kutta integrator, allocation-free.
//
// x_{k+1} = f_RK4(x_k, u_k, dt). The Model concept requires:
//   void stateDerivative(const StateVector&, const ControlVector&, StateVector&) const noexcept;
#pragma once

#include <concepts>

#include "dynamics/types.hpp"

namespace quadrotor::dynamics {

template <typename Model>
concept DynamicsModel = requires(const Model& m, const StateVector& x, const ControlVector& u,
                                 StateVector& xd) {
  { m.stateDerivative(x, u, xd) } noexcept;
};

class RK4Integrator {
 public:
  RK4Integrator() = default;

  // Single step with zero-order hold on u. All temporaries are stack locals.
  template <DynamicsModel Model>
  void step(const Model& model, const StateVector& x, const ControlVector& u, double dt,
            StateVector& x_next) const noexcept {
    StateVector k1, k2, k3, k4, xtmp;
    model.stateDerivative(x, u, k1);
    xtmp.noalias() = x + (0.5 * dt) * k1;
    normalizeQuatInState(xtmp);
    model.stateDerivative(xtmp, u, k2);
    xtmp.noalias() = x + (0.5 * dt) * k2;
    normalizeQuatInState(xtmp);
    model.stateDerivative(xtmp, u, k3);
    xtmp.noalias() = x + dt * k3;
    normalizeQuatInState(xtmp);
    model.stateDerivative(xtmp, u, k4);
    x_next.noalias() = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    normalizeQuatInState(x_next);
  }

  // Convenience: integrate over n steps of dt (e.g., 500 Hz substeps).
  template <DynamicsModel Model>
  void integrate(const Model& model, const StateVector& x0, const ControlVector& u, double dt,
                 int n_steps, StateVector& x_out) const noexcept {
    StateVector cur = x0;
    StateVector nxt;
    for (int i = 0; i < n_steps; ++i) {
      step(model, cur, u, dt, nxt);
      cur = nxt;
    }
    x_out = cur;
  }

 private:
  static void normalizeQuatInState(StateVector& x) noexcept {
    const double n = x.segment<4>(StateIndex::kQw).norm();
    if (n > 1e-12) {
      x.segment<4>(StateIndex::kQw) /= n;
    } else {
      x(StateIndex::kQw) = 1.0;
      x(StateIndex::kQx) = 0.0;
      x(StateIndex::kQy) = 0.0;
      x(StateIndex::kQz) = 0.0;
    }
  }
};

}  // namespace quadrotor::dynamics
