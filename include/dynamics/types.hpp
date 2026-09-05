// SPDX-License-Identifier: MIT
// Fixed-size real-time types for the quadrotor stack.
// Zero heap allocation: only Eigen fixed-size matrices and std::array.
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <concepts>
#include <cstddef>

namespace quadrotor {

// ---------------------------------------------------------------------------
// Compile-time dimensions (single source of truth)
// ---------------------------------------------------------------------------
inline constexpr int kPosDim = 3;
inline constexpr int kVelDim = 3;
inline constexpr int kQuatDim = 4;   // stored as (w, x, y, z)
inline constexpr int kOmegaDim = 3;
inline constexpr int kStateDim = 13;  // [p, v, q, omega]
inline constexpr int kInputDim = 4;   // [f_T, tau_x, tau_y, tau_z]
inline constexpr int kErrorDim = 15;  // [dp, dv, dtheta, dba, dbg]
inline constexpr int kMeasDim = 6;    // [p(3), dtheta(3)] for VO/mocap pose
inline constexpr int kNoiseDim = 12;  // [na, ng, nba, nbg]

// State index map into the 13-vector. Quaternion stored w-first.
struct StateIndex {
  static constexpr int kPx = 0, kPy = 1, kPz = 2;
  static constexpr int kVx = 3, kVy = 4, kVz = 5;
  static constexpr int kQw = 6, kQx = 7, kQy = 8, kQz = 9;
  static constexpr int kWx = 10, kWy = 11, kWz = 12;
};
struct InputIndex {
  static constexpr int kThrust = 0;
  static constexpr int kTaux = 1, kTauy = 2, kTauz = 3;
};

// ---------------------------------------------------------------------------
// Fixed-size Eigen aliases (stack-allocated, SIMD-vectorizable)
// ---------------------------------------------------------------------------
using StateVector = Eigen::Matrix<double, kStateDim, 1>;
using StateMatrix = Eigen::Matrix<double, kStateDim, kStateDim>;
using ControlVector = Eigen::Matrix<double, kInputDim, 1>;
using ControlMatrix = Eigen::Matrix<double, kInputDim, kInputDim>;
using ErrorVector = Eigen::Matrix<double, kErrorDim, 1>;
using ErrorMatrix = Eigen::Matrix<double, kErrorDim, kErrorDim>;
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec4 = Eigen::Matrix<double, 4, 1>;
using Mat3 = Eigen::Matrix<double, 3, 3>;
using Mat4 = Eigen::Matrix<double, 4, 4>;

// Error-state (12-dim tracking error: dp, dv, dtheta, domega) used by NMPC cost.
inline constexpr int kTrackErrDim = 12;
using TrackErrorVector = Eigen::Matrix<double, kTrackErrDim, 1>;
using TrackWeightMatrix = Eigen::Matrix<double, kTrackErrDim, kTrackErrDim>;

// Concept: fixed-size Eigen column vector (no dynamic allocation).
template <typename T>
concept FixedVector = requires {
  typename T::Scalar;
  { T::RowsAtCompileTime } -> std::convertible_to<int>;
  { T::ColsAtCompileTime } -> std::convertible_to<int>;
} && (T::RowsAtCompileTime != Eigen::Dynamic) && (T::ColsAtCompileTime == 1);

static_assert(FixedVector<StateVector>);
static_assert(FixedVector<ControlVector>);
static_assert(FixedVector<ErrorVector>);

}  // namespace quadrotor
