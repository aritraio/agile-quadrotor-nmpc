// SPDX-License-Identifier: MIT
#include "controller/nmpc_solver.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>

namespace quadrotor::controller {

NmpcConfig::NmpcConfig() {
  horizon = 20;
  dt = 0.05;
  Q.setZero();
  Q.block<3, 3>(0, 0).setIdentity();
  Q.block<3, 3>(0, 0) *= 20.0;
  Q.block<3, 3>(3, 3).setIdentity();
  Q.block<3, 3>(3, 3) *= 5.0;
  Q.block<3, 3>(6, 6).setIdentity();
  Q.block<3, 3>(6, 6) *= 8.0;
  Q.block<3, 3>(9, 9).setIdentity();
  Q.block<3, 3>(9, 9) *= 0.5;
  R.setZero();
  R(0, 0) = 0.05;
  R(1, 1) = 0.05;
  R(2, 2) = 0.05;
  R(3, 3) = 0.05;
  P = Q;
  P *= 5.0;
  const double hover = 1.0 * 9.81;
  u_hover << hover, 0, 0, 0;
  u_min << 0.0, -2.0, -2.0, -2.0;
  u_max << 25.0, 2.0, 2.0, 2.0;
  tilt_max_rad = 0.6;
  tilt_weight = 500.0;
  obs_weight = 2000.0;
  max_iters = 5;
  grad_tol = 1e-4;
  init_step = 1.0;
  min_step = 1e-3;
  ilqr_reg = 1e-4;
  num_obstacles = 0;
}

NmpcSolver::NmpcSolver(const NmpcConfig& cfg, const dynamics::QuadrotorParams& plant)
    : cfg_(cfg), model_(plant) {
  if (cfg_.horizon > kMaxHorizon) cfg_.horizon = kMaxHorizon;
  if (cfg_.horizon < 1) cfg_.horizon = 1;
  for (auto& u : U_) u = cfg_.u_hover;
  for (auto& u : Utrial_) u = cfg_.u_hover;
}

void NmpcSolver::setConfig(const NmpcConfig& cfg) noexcept {
  cfg_ = cfg;
  if (cfg_.horizon > kMaxHorizon) cfg_.horizon = kMaxHorizon;
  if (cfg_.horizon < 1) cfg_.horizon = 1;
}

void NmpcSolver::setModel(const dynamics::QuadrotorParams& p) noexcept { model_.setParams(p); }

void NmpcSolver::shiftWarmStart() noexcept {
  const auto n = static_cast<std::size_t>(cfg_.horizon);
  for (std::size_t k = 0; k + 1U < n; ++k) U_[k] = U_[k + 1U];
  U_[n - 1U] = cfg_.u_hover;
}

bool NmpcSolver::solve(const StateVector& x0, const StateVector* Xref, const ControlVector* Uref,
                       ControlVector* Uopt_out, NmpcInfo& info) noexcept {
  const auto t_start = std::chrono::steady_clock::now();
  const auto n = static_cast<std::size_t>(cfg_.horizon);

  for (std::size_t k = 0; k <= n; ++k) Xref_[k] = Xref[k];
  for (std::size_t k = 0; k < n; ++k) Uref_[k] = Uref[k];

  rollout(x0, U_.data(), X_.data());
  double J = totalCost(x0, Xref_.data(), Uref_.data(), U_.data());

  int iters = 0;
  double ff_norm = 0.0;
  bool converged = false;
  double reg = cfg_.ilqr_reg;

  for (int it = 0; it < cfg_.max_iters; ++it) {
    for (std::size_t k = 0; k < n; ++k) linearize(X_[k], U_[k], A_[k], B_[k]);
    for (std::size_t k = 0; k < n; ++k) {
      stageQuadratization(X_[k], U_[k], Xref_[k], Uref_[k], Lx_[k], Lu_[k], Lxx_[k],
                          Luu_[k]);
    }
    terminalQuadratization(X_[n], Xref_[n], LxT_, LxxT_);

    double expected = 0.0;
    // Adaptive damping: bump reg until backward pass succeeds.
    bool ok = false;
    for (int r = 0; r < 5; ++r) {
      if (backwardPass(reg, expected)) {
        ok = true;
        break;
      }
      reg *= 10.0;
    }
    if (!ok) break;

    ff_norm = 0.0;
    for (std::size_t k = 0; k < n; ++k) ff_norm += Kff_[k].squaredNorm();
    ff_norm = std::sqrt(ff_norm);
    if (!std::isfinite(ff_norm)) break;

    // Forward line search over step sizes with feedback policy.
    double alpha = cfg_.init_step;
    bool improved = false;
    double J_trial = J;
    while (alpha >= cfg_.min_step - 1e-12) {
      rolloutFeedback(x0, Xtrial_.data(), Utrial_.data(), alpha);
      J_trial = totalCost(x0, Xref_.data(), Uref_.data(), Utrial_.data());
      if (J_trial < J) {
        improved = true;
        break;
      }
      alpha *= 0.5;
    }
    if (!improved) {
      reg *= 10.0;
      if (reg > 1e4) break;
      continue;  // retry with higher damping (counts as same iter budget)
    }
    // Accept.
    for (std::size_t k = 0; k < n; ++k) U_[k] = Utrial_[k];
    for (std::size_t k = 0; k <= n; ++k) X_[k] = Xtrial_[k];
    J = J_trial;
    ++iters;
    reg = std::max(cfg_.ilqr_reg, reg * 0.2);
    if (ff_norm < cfg_.grad_tol || std::abs(expected) < 1e-9) {
      converged = true;
      break;
    }
  }

  for (std::size_t k = 0; k < n; ++k) Uopt_out[k] = U_[k];

  const auto t_end = std::chrono::steady_clock::now();
  info.cost = J;
  info.solve_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
  info.iterations = iters;
  info.grad_norm = ff_norm;
  info.converged = converged;
  evaluateConstraints(X_.data(), info);
  return std::isfinite(J);
}

void NmpcSolver::rollout(const StateVector& x0, const ControlVector* U,
                         StateVector* X) const noexcept {
  const auto n = static_cast<std::size_t>(cfg_.horizon);
  X[0] = x0;
  Vec4 q0 = X[0].segment<4>(StateIndex::kQw);
  dynamics::QuadrotorModel::normalizeQuaternion(q0);
  X[0].segment<4>(StateIndex::kQw) = q0;
  for (std::size_t k = 0; k < n; ++k) integrator_.step(model_, X[k], U[k], cfg_.dt, X[k + 1U]);
}

void NmpcSolver::rolloutFeedback(const StateVector& x0, StateVector* Xt, ControlVector* Ut,
                                 double alpha) const noexcept {
  const auto n = static_cast<std::size_t>(cfg_.horizon);
  Xt[0] = x0;
  for (std::size_t k = 0; k < n; ++k) {
    ControlVector u = U_[k];
    u.noalias() += alpha * Kff_[k] + K_[k] * (Xt[k] - X_[k]);
    clampControl(u);
    Ut[k] = u;
    integrator_.step(model_, Xt[k], Ut[k], cfg_.dt, Xt[k + 1U]);
  }
}

void NmpcSolver::trackingError(const StateVector& x, const StateVector& xr,
                               TrackErrorVector& e) const noexcept {
  e.segment<3>(0).noalias() = x.segment<3>(StateIndex::kPx) - xr.segment<3>(StateIndex::kPx);
  e.segment<3>(3).noalias() = x.segment<3>(StateIndex::kVx) - xr.segment<3>(StateIndex::kVx);
  Vec4 qr = xr.segment<4>(StateIndex::kQw);
  Vec4 q = x.segment<4>(StateIndex::kQw);
  {
    double n = qr.norm();
    if (n > 1e-12)
      qr /= n;
    else
      qr << 1, 0, 0, 0;
    n = q.norm();
    if (n > 1e-12)
      q /= n;
    else
      q << 1, 0, 0, 0;
  }
  Vec4 qr_inv;
  qr_inv << qr(0), -qr(1), -qr(2), -qr(3);
  Mat4 L;
  dynamics::QuadrotorModel::leftMultiplyMatrix(qr_inv, L);
  Vec4 qe = L * q;
  if (qe(0) < 0.0) qe = -qe;
  e(6) = 2.0 * qe(1);
  e(7) = 2.0 * qe(2);
  e(8) = 2.0 * qe(3);
  e.segment<3>(9).noalias() = x.segment<3>(StateIndex::kWx) - xr.segment<3>(StateIndex::kWx);
}

void NmpcSolver::trackingJacobian(const StateVector& xr,
                                  Eigen::Matrix<double, 12, 13>& J) const noexcept {
  J.setZero();
  J.block<3, 3>(0, StateIndex::kPx).setIdentity();
  J.block<3, 3>(3, StateIndex::kVx).setIdentity();
  Vec4 qr = xr.segment<4>(StateIndex::kQw);
  double n = qr.norm();
  if (n > 1e-12)
    qr /= n;
  else
    qr << 1, 0, 0, 0;
  Vec4 qr_inv;
  qr_inv << qr(0), -qr(1), -qr(2), -qr(3);
  Mat4 L;
  dynamics::QuadrotorModel::leftMultiplyMatrix(qr_inv, L);
  J.block<3, 4>(6, StateIndex::kQw) = 2.0 * L.block<3, 4>(1, 0);
  J.block<3, 3>(9, StateIndex::kWx).setIdentity();
}

double NmpcSolver::tiltPenalty(const Vec4& q, Vec4& grad_q,
                               Eigen::Matrix<double, 4, 4>* hess) const noexcept {
  grad_q.setZero();
  if (hess != nullptr) hess->setZero();
  const double s = std::sin(0.5 * cfg_.tilt_max_rad);
  const double smax2 = s * s;
  double n = q.norm();
  Vec4 qn = q;
  if (n > 1e-12)
    qn /= n;
  else
    qn << 1, 0, 0, 0;
  const double v = qn(1) * qn(1) + qn(2) * qn(2) - smax2;
  if (v <= 0.0) return 0.0;
  const double pen = cfg_.tilt_weight * v * v;
  grad_q(1) = cfg_.tilt_weight * 4.0 * v * qn(1);
  grad_q(2) = cfg_.tilt_weight * 4.0 * v * qn(2);
  if (hess != nullptr) {
    const double w = cfg_.tilt_weight;
    (*hess)(1, 1) = 4.0 * w * (2.0 * qn(1) * qn(1) + v);
    (*hess)(2, 2) = 4.0 * w * (2.0 * qn(2) * qn(2) + v);
    (*hess)(1, 2) = (*hess)(2, 1) = 8.0 * w * qn(1) * qn(2);
  }
  return pen;
}

double NmpcSolver::obstaclePenalty(const Vec3& p, Vec3& grad_p, Mat3* hess) const noexcept {
  grad_p.setZero();
  if (hess != nullptr) hess->setZero();
  double total = 0.0;
  const auto nobs = static_cast<std::size_t>(cfg_.num_obstacles);
  for (std::size_t i = 0; i < nobs; ++i) {
    const auto& ob = cfg_.obstacles[i];
    const Vec3 d = p - ob.center;
    const Vec3 ad = ob.shape * d;
    const double m = d.dot(ad);
    if (m < 1.0) {
      const double v = 1.0 - m;
      total += cfg_.obs_weight * v * v;
      grad_p.noalias() += (-4.0 * cfg_.obs_weight * v) * ad;
      if (hess != nullptr) {
        hess->noalias() += (8.0 * cfg_.obs_weight) * (ad * ad.transpose());
        hess->noalias() += (4.0 * cfg_.obs_weight * v) * ob.shape;
      }
    }
  }
  return total;
}

double NmpcSolver::stageCost(int k, const StateVector& x, const ControlVector& u,
                             const StateVector& xr, const ControlVector& ur) const noexcept {
  (void)k;
  TrackErrorVector e;
  trackingError(x, xr, e);
  const double track = e.dot(cfg_.Q * e);
  const ControlVector du = u - ur;
  const double ctrl = du.dot(cfg_.R * du);
  Vec4 gq;
  gq.setZero();
  const double tilt = tiltPenalty(x.segment<4>(StateIndex::kQw), gq, nullptr);
  Vec3 gp;
  gp.setZero();
  const double obs = obstaclePenalty(x.segment<3>(StateIndex::kPx), gp, nullptr);
  return track + ctrl + tilt + obs;
}

double NmpcSolver::terminalCost(const StateVector& xN,
                                const StateVector& xrN) const noexcept {
  TrackErrorVector e;
  trackingError(xN, xrN, e);
  double c = e.dot(cfg_.P * e);
  Vec4 gq;
  gq.setZero();
  c += tiltPenalty(xN.segment<4>(StateIndex::kQw), gq, nullptr);
  Vec3 gp;
  gp.setZero();
  c += obstaclePenalty(xN.segment<3>(StateIndex::kPx), gp, nullptr);
  return c;
}

double NmpcSolver::totalCost(const StateVector& x0, const StateVector* Xref,
                             const ControlVector* Uref, const ControlVector* U) const noexcept {
  const auto n = static_cast<std::size_t>(cfg_.horizon);
  std::array<StateVector, static_cast<std::size_t>(kMaxHorizon) + 1U> xloc{};
  xloc[0] = x0;
  double j = 0.0;
  for (std::size_t k = 0; k < n; ++k) {
    j += stageCost(0, xloc[k], U[k], Xref[k], Uref[k]);
    integrator_.step(model_, xloc[k], U[k], cfg_.dt, xloc[k + 1U]);
  }
  j += terminalCost(xloc[n], Xref[n]);
  return j;
}

void NmpcSolver::stageQuadratization(const StateVector& x, const ControlVector& u,
                                     const StateVector& xr, const ControlVector& ur,
                                     StateVector& lx, ControlVector& lu, QxxMat& lxx,
                                     QuuMat& luu) const noexcept {
  TrackErrorVector e;
  trackingError(x, xr, e);
  Eigen::Matrix<double, 12, 13> J;
  trackingJacobian(xr, J);
  const TrackErrorVector qe = cfg_.Q * e;
  lx.noalias() = J.transpose() * (2.0 * qe);
  lu.noalias() = 2.0 * cfg_.R * (u - ur);
  lxx.noalias() = J.transpose() * (2.0 * cfg_.Q) * J;
  luu = 2.0 * cfg_.R;
  // Penalty gradients + Gauss-Newton Hessians.
  Vec4 gq;
  Eigen::Matrix<double, 4, 4> hq;
  tiltPenalty(x.segment<4>(StateIndex::kQw), gq, &hq);
  lx.segment<4>(StateIndex::kQw) += gq;
  lxx.block<4, 4>(StateIndex::kQw, StateIndex::kQw) += hq;
  Vec3 gp;
  Mat3 hp;
  obstaclePenalty(x.segment<3>(StateIndex::kPx), gp, &hp);
  lx.segment<3>(StateIndex::kPx) += gp;
  lxx.block<3, 3>(StateIndex::kPx, StateIndex::kPx) += hp;
  // Regularize quaternion null direction (unit-norm gauge).
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(kStateDim); ++i) lxx(i, i) += 1e-9;
}

void NmpcSolver::terminalQuadratization(const StateVector& xN, const StateVector& xrN,
                                        StateVector& lx, QxxMat& lxx) const noexcept {
  TrackErrorVector e;
  trackingError(xN, xrN, e);
  Eigen::Matrix<double, 12, 13> J;
  trackingJacobian(xrN, J);
  const TrackErrorVector pe = cfg_.P * e;
  lx.noalias() = J.transpose() * (2.0 * pe);
  lxx.noalias() = J.transpose() * (2.0 * cfg_.P) * J;
  Vec4 gq;
  Eigen::Matrix<double, 4, 4> hq;
  tiltPenalty(xN.segment<4>(StateIndex::kQw), gq, &hq);
  lx.segment<4>(StateIndex::kQw) += gq;
  lxx.block<4, 4>(StateIndex::kQw, StateIndex::kQw) += hq;
  Vec3 gp;
  Mat3 hp;
  obstaclePenalty(xN.segment<3>(StateIndex::kPx), gp, &hp);
  lx.segment<3>(StateIndex::kPx) += gp;
  lxx.block<3, 3>(StateIndex::kPx, StateIndex::kPx) += hp;
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(kStateDim); ++i) lxx(i, i) += 1e-9;
}

bool NmpcSolver::backwardPass(double reg, double& expected_reduction) noexcept {
  const auto n = static_cast<std::size_t>(cfg_.horizon);
  Vxx_[n] = LxxT_;
  Vx_[n] = LxT_;
  // Symmetrize terminal.
  Vxx_[n] = 0.5 * (Vxx_[n] + Vxx_[n].transpose());
  expected_reduction = 0.0;

  for (std::size_t kk = n; kk-- > 0U;) {
    const AMat& a = A_[kk];
    const BMat& b = B_[kk];
    const std::size_t kn = kk + 1U;
    // Q-terms.
    StateVector qx = Lx_[kk] + a.transpose() * Vx_[kn];
    ControlVector qu = Lu_[kk] + b.transpose() * Vx_[kn];
    QxxMat qxx = Lxx_[kk] + a.transpose() * Vxx_[kn] * a;
    QuuMat quu = Luu_[kk] + b.transpose() * Vxx_[kn] * b;
    QxuMat qxu = a.transpose() * Vxx_[kn] * b;  // 13x4
    // Symmetrize + damp.
    qxx = 0.5 * (qxx + qxx.transpose());
    quu = 0.5 * (quu + quu.transpose());
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(kInputDim); ++i) quu(i, i) += reg;

    Eigen::LDLT<QuuMat> ldlt(quu);
    if (ldlt.info() != Eigen::Success) return false;
    // Check positive-definiteness via reconstructed diagonal sign.
    if ((ldlt.vectorD().array() <= 1e-12).any()) return false;

    ControlVector kff = ldlt.solve(-qu);
    KMat kmat = ldlt.solve(-qxu.transpose());

    // Clamp feedforward to sane range to avoid wild line-search starts.
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(kInputDim); ++i) {
      if (!std::isfinite(kff(i))) return false;
      const double lim = (i == 0) ? 10.0 : 2.0;
      if (kff(i) > lim) kff(i) = lim;
      if (kff(i) < -lim) kff(i) = -lim;
    }
    Kff_[kk] = kff;
    K_[kk] = kmat;

    expected_reduction += -qu.dot(kff) - 0.5 * kff.dot(quu * kff);

    // Value update (Joseph-like, symmetrized).
    StateVector vx = qx + kmat.transpose() * qu + kmat.transpose() * quu * kff + qxu * kff;
    QxxMat vxx = qxx + kmat.transpose() * quu * kmat + kmat.transpose() * qxu.transpose() +
                 qxu * kmat;
    vxx = 0.5 * (vxx + vxx.transpose());
    Vx_[kk] = vx;
    Vxx_[kk] = vxx;
  }
  return std::isfinite(expected_reduction);
}

void NmpcSolver::linearize(const StateVector& x, const ControlVector& u, AMat& A,
                           BMat& B) const noexcept {
  constexpr double kEps = 1e-7;
  StateVector x0n = x;
  {
    Vec4 q = x0n.segment<4>(StateIndex::kQw);
    dynamics::QuadrotorModel::normalizeQuaternion(q);
    x0n.segment<4>(StateIndex::kQw) = q;
  }
  StateVector base_next;
  integrator_.step(model_, x0n, u, cfg_.dt, base_next);
  StateVector xp = x0n;
  for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(kStateDim); ++j) {
    xp = x0n;
    xp(j) += kEps;
    if (j >= static_cast<Eigen::Index>(StateIndex::kQw) &&
        j <= static_cast<Eigen::Index>(StateIndex::kQz)) {
      Vec4 q = xp.segment<4>(StateIndex::kQw);
      dynamics::QuadrotorModel::normalizeQuaternion(q);
      xp.segment<4>(StateIndex::kQw) = q;
    }
    StateVector xn;
    integrator_.step(model_, xp, u, cfg_.dt, xn);
    A.col(j).noalias() = (xn - base_next) / kEps;
  }
  for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(kInputDim); ++j) {
    ControlVector up = u;
    up(j) += kEps;
    StateVector xn;
    integrator_.step(model_, x0n, up, cfg_.dt, xn);
    B.col(j).noalias() = (xn - base_next) / kEps;
  }
}

void NmpcSolver::clampControl(ControlVector& u) const noexcept {
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(kInputDim); ++i) {
    if (u(i) < cfg_.u_min(i)) u(i) = cfg_.u_min(i);
    if (u(i) > cfg_.u_max(i)) u(i) = cfg_.u_max(i);
  }
}

void NmpcSolver::evaluateConstraints(const StateVector* X, NmpcInfo& info) const noexcept {
  const auto n = static_cast<std::size_t>(cfg_.horizon);
  const double s = std::sin(0.5 * cfg_.tilt_max_rad);
  const double smax2 = s * s;
  double max_tilt_viol = 0.0;
  double min_clear = 1e9;
  const auto nobs = static_cast<std::size_t>(cfg_.num_obstacles);
  for (std::size_t k = 0; k <= n; ++k) {
    const Vec4 q = X[k].segment<4>(StateIndex::kQw);
    double qn = q.norm();
    double qx = q(1);
    double qy = q(2);
    if (qn > 1e-12) {
      qx /= qn;
      qy /= qn;
    }
    const double v = qx * qx + qy * qy - smax2;
    if (v > max_tilt_viol) max_tilt_viol = v;
    const Vec3 p = X[k].segment<3>(StateIndex::kPx);
    for (std::size_t i = 0; i < nobs; ++i) {
      const auto& ob = cfg_.obstacles[i];
      const Vec3 d = p - ob.center;
      const double m = d.dot(ob.shape * d);
      const double clear = m - 1.0;
      if (clear < min_clear) min_clear = clear;
    }
  }
  if (cfg_.num_obstacles == 0) min_clear = 1e9;
  info.max_tilt_violation = max_tilt_viol > 0.0 ? max_tilt_viol : 0.0;
  info.min_obs_clearance = min_clear;
  info.constraints_ok = (info.max_tilt_violation <= 1e-6) &&
                        (cfg_.num_obstacles == 0 || info.min_obs_clearance >= -1e-6);
}

}  // namespace quadrotor::controller
