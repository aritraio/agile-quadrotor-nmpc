// SPDX-License-Identifier: MIT
// Dual-solver backend: C++ wrapper over the CasADi/acados-exported pure C OCP.
//
// scripts/export_ocp_casadi.py emits generated/quadrotor_dynamics_rk4.{h,c} and
// generated/quadrotor_cost.{h,c} (allocation-free C99, -Wconversion clean).
// With -DUSE_ACADOS=ON (CMake option QUADROTOR_USE_ACADOS) those objects are
// compiled and linked; this wrapper calls them directly via extern "C".
// Otherwise it uses the inline reference implementation below, which is
// formula-identical to the generated C (verified by tests/test_casadi_backend).
//
// Hot path: zero heap allocation; raw-pointer + Eigen overloads; noexcept.
#pragma once

#include <array>
#include <cmath>
#include <cstddef>

#include "dynamics/types.hpp"

#ifdef QUADROTOR_USE_CASADI_BACKEND
#include "quadrotor_cost.h"
#include "quadrotor_dynamics_rk4.h"
#endif

namespace quadrotor::controller::generated {

// Parameter pack layout (mirrors generated C QUAD_NP = 13):
//   p = [m, Jx, Jy, Jz, Dx, Dy, Dz, gx, gy, gz, tilt_max, tilt_w, obs_w]
inline constexpr std::size_t kNumParams = 13;
using ParamArray = std::array<double, kNumParams>;
// World-aligned ellipsoid lists (match generated C obs_c/obs_a layout).
inline constexpr std::size_t kMaxObs = 4;
using ObsCenterArray = std::array<double, kMaxObs * 3U>;
using ObsShapeArray = std::array<double, kMaxObs * 3U>;  // diag(1/r^2) per obs

[[nodiscard]] inline ParamArray defaultParams() noexcept {
  return ParamArray{1.0, 0.0123, 0.0123, 0.022, 0.10, 0.10, 0.15,
                    0.0, 0.0, -9.81, 0.6, 500.0, 2000.0};
}

// --- Inline reference implementation (formula-identical to generated C) ----
inline void stateDerivativeRef(const double* x, const double* u, const double* p,
                               double* xdot) noexcept {
  const double m = p[0];
  const double jx = p[1];
  const double jy = p[2];
  const double jz = p[3];
  const double dx = p[4];
  const double dy = p[5];
  const double dz = p[6];
  const double gx = p[7];
  const double gy = p[8];
  const double gz = p[9];
  double n =
      std::sqrt(x[6] * x[6] + x[7] * x[7] + x[8] * x[8] + x[9] * x[9]);
  double qw = 1.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  if (n > 1e-12) {
    qw = x[6] / n;
    qx = x[7] / n;
    qy = x[8] / n;
    qz = x[9] / n;
  }
  // R(q) column 2 (thrust axis).
  const double r02 = 2.0 * (qx * qz + qw * qy);
  const double r12 = 2.0 * (qy * qz - qw * qx);
  const double r22 = qw * qw - qx * qx - qy * qy + qz * qz;
  xdot[0] = x[3];
  xdot[1] = x[4];
  xdot[2] = x[5];
  const double ft = u[0];
  xdot[3] = gx + r02 * ft / m - dx * x[3] / m;
  xdot[4] = gy + r12 * ft / m - dy * x[4] / m;
  xdot[5] = gz + r22 * ft / m - dz * x[5] / m;
  // qdot = 0.5 q otimes [0, omega].
  const double wx = x[10];
  const double wy = x[11];
  const double wz = x[12];
  // NOTE: use normalized quaternion for the kinematics (matches C).
  xdot[6] = -0.5 * (qx * wx + qy * wy + qz * wz);
  xdot[7] = 0.5 * (qw * wx + qy * wz - qz * wy);
  xdot[8] = 0.5 * (qw * wy + qz * wx - qx * wz);
  xdot[9] = 0.5 * (qw * wz + qx * wy - qy * wx);
  const double jwx = jx * wx;
  const double jwy = jy * wy;
  const double jwz = jz * wz;
  xdot[10] = (u[1] - (wy * jwz - wz * jwy)) / jx;
  xdot[11] = (u[2] - (wz * jwx - wx * jwz)) / jy;
  xdot[12] = (u[3] - (wx * jwy - wy * jwx)) / jz;
}

inline void normalizeQuatRef(double* q) noexcept {
  const double n =
      std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n > 1e-12) {
    q[0] /= n;
    q[1] /= n;
    q[2] /= n;
    q[3] /= n;
  } else {
    q[0] = 1.0;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 0.0;
  }
}

inline void rk4StepRef(const double* x, const double* u, const double* p, double dt,
                       double* x_next) noexcept {
  double k1[13];
  double k2[13];
  double k3[13];
  double k4[13];
  double xt[13];
  stateDerivativeRef(x, u, p, k1);
  for (std::size_t i = 0; i < 13U; ++i) xt[i] = x[i] + 0.5 * dt * k1[i];
  normalizeQuatRef(&xt[6]);
  stateDerivativeRef(xt, u, p, k2);
  for (std::size_t i = 0; i < 13U; ++i) xt[i] = x[i] + 0.5 * dt * k2[i];
  normalizeQuatRef(&xt[6]);
  stateDerivativeRef(xt, u, p, k3);
  for (std::size_t i = 0; i < 13U; ++i) xt[i] = x[i] + dt * k3[i];
  normalizeQuatRef(&xt[6]);
  stateDerivativeRef(xt, u, p, k4);
  for (std::size_t i = 0; i < 13U; ++i) {
    x_next[i] = x[i] + (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
  }
  normalizeQuatRef(&x_next[6]);
}

inline void trackingErrorRef(const double* x, const double* xr, double* e) noexcept {
  for (std::size_t i = 0; i < 3U; ++i) {
    e[i] = x[i] - xr[i];
    e[3U + i] = x[3U + i] - xr[3U + i];
    e[9U + i] = x[10U + i] - xr[10U + i];
  }
  // dtheta = 2*vec(qr_conj * q), shortest arc.
  double nr = std::sqrt(xr[6] * xr[6] + xr[7] * xr[7] + xr[8] * xr[8] + xr[9] * xr[9]);
  double nq = std::sqrt(x[6] * x[6] + x[7] * x[7] + x[8] * x[8] + x[9] * x[9]);
  double ar[4] = {xr[6], xr[7], xr[8], xr[9]};
  double aq[4] = {x[6], x[7], x[8], x[9]};
  if (nr > 1e-12) {
    for (std::size_t i = 0; i < 4U; ++i) ar[i] /= nr;
  } else {
    ar[0] = 1.0;
    ar[1] = 0.0;
    ar[2] = 0.0;
    ar[3] = 0.0;
  }
  if (nq > 1e-12) {
    for (std::size_t i = 0; i < 4U; ++i) aq[i] /= nq;
  } else {
    aq[0] = 1.0;
    aq[1] = 0.0;
    aq[2] = 0.0;
    aq[3] = 0.0;
  }
  const double qe0 = ar[0] * aq[0] + ar[1] * aq[1] + ar[2] * aq[2] + ar[3] * aq[3];
  const double qe1 = ar[0] * aq[1] - ar[1] * aq[0] - ar[2] * aq[3] + ar[3] * aq[2];
  const double qe2 = ar[0] * aq[2] + ar[1] * aq[3] - ar[2] * aq[0] - ar[3] * aq[1];
  const double qe3 = ar[0] * aq[3] - ar[1] * aq[2] + ar[2] * aq[1] - ar[3] * aq[0];
  const double sgn = (qe0 < 0.0) ? -1.0 : 1.0;
  e[6] = 2.0 * sgn * qe1;
  e[7] = 2.0 * sgn * qe2;
  e[8] = 2.0 * sgn * qe3;
}

// --- Public facade: dispatches to generated C when linked, else reference --
class CasadiGeneratedModel {
 public:
  static void stateDerivative(const double* x, const double* u, const double* p,
                              double* xdot) noexcept {
#ifdef QUADROTOR_USE_CASADI_BACKEND
    ::quadrotor_state_derivative(x, u, p, xdot);
#else
    stateDerivativeRef(x, u, p, xdot);
#endif
  }

  static void rk4Step(const double* x, const double* u, const double* p, double dt,
                      double* x_next) noexcept {
#ifdef QUADROTOR_USE_CASADI_BACKEND
    ::quadrotor_rk4_step(x, u, p, dt, x_next);
#else
    rk4StepRef(x, u, p, dt, x_next);
#endif
  }

  static void trackingError(const double* x, const double* xr, double* e) noexcept {
#ifdef QUADROTOR_USE_CASADI_BACKEND
    ::quadrotor_tracking_error(x, xr, e);
#else
    trackingErrorRef(x, xr, e);
#endif
  }

  // Diagonal-weight stage/terminal costs with tilt + obstacle penalties.
  // qw_stage/qw_term: 12 diag entries; rw: 4 diag entries.
  static double stageCost(const double* x, const double* u, const double* xr,
                          const double* ur, const double* qw_stage, const double* rw,
                          double tilt_max, double tilt_w, const double* obs_c,
                          const double* obs_a, int nobs, double obs_w) noexcept {
#ifdef QUADROTOR_USE_CASADI_BACKEND
    return ::quadrotor_stage_cost(x, u, xr, ur, qw_stage, rw, tilt_max, tilt_w,
                                  obs_c, obs_a, nobs, obs_w);
#else
    double e[12];
    trackingErrorRef(x, xr, e);
    double c = 0.0;
    for (std::size_t i = 0; i < 12U; ++i) c += qw_stage[i] * e[i] * e[i];
    for (std::size_t i = 0; i < 4U; ++i) {
      const double du = u[i] - ur[i];
      c += rw[i] * du * du;
    }
    // Tilt penalty.
    {
      const double s = std::sin(0.5 * tilt_max);
      double n = std::sqrt(x[6] * x[6] + x[7] * x[7] + x[8] * x[8] + x[9] * x[9]);
      double qx = x[7];
      double qy = x[8];
      if (n > 1e-12) {
        qx /= n;
        qy /= n;
      }
      const double v = qx * qx + qy * qy - s * s;
      if (v > 0.0) c += tilt_w * v * v;
    }
    // Obstacle penalties.
    for (int o = 0; o < nobs; ++o) {
      const auto b = static_cast<std::size_t>(o) * 3U;
      const double d0 = x[0] - obs_c[b];
      const double d1 = x[1] - obs_c[b + 1U];
      const double d2 = x[2] - obs_c[b + 2U];
      const double m =
          d0 * d0 * obs_a[b] + d1 * d1 * obs_a[b + 1U] + d2 * d2 * obs_a[b + 2U];
      if (m < 1.0) {
        const double v = 1.0 - m;
        c += obs_w * v * v;
      }
    }
    return c;
#endif
  }

  static double terminalCost(const double* xn, const double* xrn,
                             const double* qw_term, double tilt_max, double tilt_w,
                             const double* obs_c, const double* obs_a, int nobs,
                             double obs_w) noexcept {
#ifdef QUADROTOR_USE_CASADI_BACKEND
    return ::quadrotor_terminal_cost(xn, xrn, qw_term, tilt_max, tilt_w, obs_c,
                                     obs_a, nobs, obs_w);
#else
    double e[12];
    trackingErrorRef(xn, xrn, e);
    double c = 0.0;
    for (std::size_t i = 0; i < 12U; ++i) c += qw_term[i] * e[i] * e[i];
    {
      const double s = std::sin(0.5 * tilt_max);
      double n = std::sqrt(xn[6] * xn[6] + xn[7] * xn[7] + xn[8] * xn[8] + xn[9] * xn[9]);
      double qx = xn[7];
      double qy = xn[8];
      if (n > 1e-12) {
        qx /= n;
        qy /= n;
      }
      const double v = qx * qx + qy * qy - s * s;
      if (v > 0.0) c += tilt_w * v * v;
    }
    for (int o = 0; o < nobs; ++o) {
      const auto b = static_cast<std::size_t>(o) * 3U;
      const double d0 = xn[0] - obs_c[b];
      const double d1 = xn[1] - obs_c[b + 1U];
      const double d2 = xn[2] - obs_c[b + 2U];
      const double m =
          d0 * d0 * obs_a[b] + d1 * d1 * obs_a[b + 1U] + d2 * d2 * obs_a[b + 2U];
      if (m < 1.0) {
        const double v = 1.0 - m;
        c += obs_w * v * v;
      }
    }
    return c;
#endif
  }

  // Eigen conveniences (still allocation-free: caller provides storage).
  static void rk4StepEigen(const StateVector& x, const ControlVector& u,
                           const ParamArray& p, double dt, StateVector& out) noexcept {
    rk4Step(x.data(), u.data(), p.data(), dt, out.data());
  }
};

}  // namespace quadrotor::controller::generated
