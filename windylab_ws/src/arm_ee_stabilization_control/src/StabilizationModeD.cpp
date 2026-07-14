#include "StabilizationModeD.hpp"

#include <algorithm>
#include <cmath>

#include <pinocchio/spatial/explog.hpp>

#include "PinocchioIkSolver.hpp"

namespace {
double Clamp(double value, double lo, double hi) {
  return std::max(lo, std::min(hi, value));
}
}  // namespace

StabilizationModeD::StabilizationModeD(const Params& params) : params_(params) {}

void StabilizationModeD::Reset(
    const std::array<double, PinocchioDynamicsModel::kDof>& q) {
  q_ref_ = q;
  q_ref_init_ = true;
  ndo_momentum_hat_.setZero();
  ndo_disturbance_.setZero();
  ndo_init_ = false;
}

Eigen::Matrix<double, 6, 1> StabilizationModeD::SatTaskError(
    const Eigen::Matrix<double, 6, 1>& task_error,
    const double sat_a,
    const double sat_epsilon) {
  Eigen::Matrix<double, 6, 1> sat;
  for (int i = 0; i < 6; ++i) {
    const double x = sat_a * task_error[i];
    if (x >= sat_epsilon) {
      sat[i] = 1.0;
    } else if (x <= -sat_epsilon) {
      sat[i] = -1.0;
    } else {
      sat[i] = std::sin(x);
    }
  }
  return sat;
}

Eigen::Matrix<double, 6, 1> StabilizationModeD::PotentialGradient(
    const Eigen::Matrix<double, 6, 1>& task_error,
    const Eigen::Matrix<double, 6, 1>& potential_kp,
    const Eigen::Matrix<double, 6, 1>& sigma_sq,
    const int potential_N) {
  Eigen::Matrix<double, 6, 1> grad = Eigen::Matrix<double, 6, 1>::Zero();
  if (potential_N < 1) {
    return grad;
  }
  for (int i = 0; i < 6; ++i) {
    const double err_sq = task_error[i] * task_error[i];
    if (err_sq <= sigma_sq[i]) {
      continue;
    }
    const double excess = err_sq - sigma_sq[i];
    const double power = std::pow(excess, potential_N - 1);
    grad[i] = potential_kp[i] * static_cast<double>(potential_N) * power * 2.0 *
              task_error[i];
  }
  return grad;
}

Eigen::Matrix<double, 6, 6> StabilizationModeD::DampedJacobianInverse(
    const Eigen::Matrix<double, 6, 6>& jacobian,
    const double damping) {
  const Eigen::Matrix<double, 6, 6> lhs =
      jacobian * jacobian.transpose() +
      damping * damping * Eigen::Matrix<double, 6, 6>::Identity();
  return jacobian.transpose() * lhs.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());
}

std::array<double, PinocchioDynamicsModel::kDof>
StabilizationModeD::ComputeSatJointVelocity(
    const Eigen::Matrix<double, 6, 1>& task_error,
    const Eigen::Matrix<double, 6, 1>& v_des,
    const Eigen::Matrix<double, 6, 6>& jacobian) const {
  const Eigen::Matrix<double, 6, 1> grad = PotentialGradient(
      task_error, params_.potential_kp, params_.sigma_sq, params_.potential_N);
  const Eigen::Matrix<double, 6, 1> sat = SatTaskError(
      task_error, params_.sat_a, params_.sat_epsilon);
  const Eigen::Matrix<double, 6, 6> jsharp =
      DampedJacobianInverse(jacobian, params_.clik_damping);

  // Wang Eq.(21): q̇_re = T2^† ẋ_d − α T2^T ε1 − kd T2^† sat(aΔx)
  Eigen::Matrix<double, 6, 1> dq =
      jsharp * v_des - params_.alpha * jacobian.transpose() * grad -
      params_.kd_sat * jsharp * sat;

  std::array<double, PinocchioDynamicsModel::kDof> dq_out{};
  for (size_t i = 0; i < PinocchioDynamicsModel::kDof; ++i) {
    dq_out[i] = Clamp(
        dq(static_cast<Eigen::Index>(i)),
        -params_.max_joint_velocity,
        params_.max_joint_velocity);
  }
  return dq_out;
}

Eigen::Matrix<double, 6, 1> StabilizationModeD::ComputeSatTaskTorque(
    const Eigen::Matrix<double, 6, 6>& jacobian,
    const Eigen::Matrix<double, 6, 1>& sat) const {
  const Eigen::Matrix<double, 6, 1> task_wrench =
      params_.sat_task_ff.cwiseProduct(sat);
  return jacobian.transpose() * task_wrench;
}

Eigen::Matrix<double, 6, 1> StabilizationModeD::UpdateNdo(
    PinocchioDynamicsModel& dynamics,
    const Eigen::VectorXd& q_full,
    const std::array<double, PinocchioDynamicsModel::kDof>& v,
    const Eigen::VectorXd& tau_before_ndo,
    const double dt) {
  if (params_.ndo_gain <= 1e-6) {
    ndo_disturbance_.setZero();
    return ndo_disturbance_;
  }

  const Eigen::MatrixXd M_full = dynamics.ComputeMassMatrix(q_full);
  const Eigen::Matrix<double, 6, 6> M_arm = dynamics.ExtractArmMassMatrix(M_full);
  Eigen::Matrix<double, 6, 1> v_arm = Eigen::Matrix<double, 6, 1>::Zero();
  for (size_t i = 0; i < PinocchioDynamicsModel::kDof; ++i) {
    v_arm[static_cast<Eigen::Index>(i)] = v[i];
  }
  const Eigen::Matrix<double, 6, 1> tau_arm =
      dynamics.ExtractArmSubvector(tau_before_ndo);
  const Eigen::VectorXd nle_full =
      dynamics.ComputeNonLinearEffects(q_full, dynamics.PackV(v));
  const Eigen::Matrix<double, 6, 1> nle_arm = dynamics.ExtractArmSubvector(nle_full);
  const Eigen::Matrix<double, 6, 1> p = M_arm * v_arm;

  if (!ndo_init_) {
    ndo_momentum_hat_ = p;
    ndo_init_ = true;
  }

  ndo_momentum_hat_ += (tau_arm + ndo_disturbance_ - nle_arm) * dt;
  ndo_disturbance_ = params_.ndo_gain * (p - ndo_momentum_hat_);

  Eigen::Matrix<double, 6, 1> tau_ndo = ndo_disturbance_;
  for (int i = 0; i < 6; ++i) {
    tau_ndo[i] = Clamp(
        tau_ndo[i], -params_.ndo_torque_limit, params_.ndo_torque_limit);
  }
  return tau_ndo;
}

StabilizationModeD::Output StabilizationModeD::Step(
    PinocchioDynamicsModel& dynamics,
    const std::array<double, PinocchioDynamicsModel::kDof>& q,
    const std::array<double, PinocchioDynamicsModel::kDof>& v,
    const pinocchio::SE3& T_des_base,
    const Eigen::Matrix<double, 6, 1>& v_des,
    const double dt,
    const PinocchioDynamicsModel::OperationalGains& osc_gains,
    PinocchioIkSolver* ik_solver,
    const int ik_cycle_iters) {
  Output out;
  out.tau = Eigen::VectorXd::Zero(dynamics.model().nv);

  const Eigen::VectorXd q_full = dynamics.PackQ(q);
  const Eigen::VectorXd v_full = dynamics.PackV(v);
  const pinocchio::SE3 T_ee = dynamics.ComputeEePoseInBase(q_full);

  Eigen::Matrix<double, 6, 1> task_error;
  task_error.head<3>() = T_des_base.translation() - T_ee.translation();
  task_error.tail<3>() = pinocchio::log3(
      T_ee.rotation().transpose() * T_des_base.rotation());

  const Eigen::Matrix<double, 6, 6> J = dynamics.ComputeArmJacobian(q_full);
  out.dq_ref = ComputeSatJointVelocity(task_error, v_des, J);

  if (!q_ref_init_) {
    q_ref_ = q;
    q_ref_init_ = true;
  }
  for (size_t i = 0; i < PinocchioDynamicsModel::kDof; ++i) {
    q_ref_[i] = PinocchioIkSolver::UnwrapNear(
        q_ref_[i] + out.dq_ref[i] * dt, q[i]);
  }
  out.q_ref = q_ref_;
  out.q_plan = out.q_ref;

  const Eigen::Matrix<double, 6, 1> grad = PotentialGradient(
      task_error, params_.potential_kp, params_.sigma_sq, params_.potential_N);
  const Eigen::Matrix<double, 6, 1> sat = SatTaskError(
      task_error, params_.sat_a, params_.sat_epsilon);
  out.twist_cmd = v_des - params_.alpha * grad - params_.kd_sat * sat;

  Eigen::Matrix<double, 6, 1> v_des_aug = v_des;
  if (params_.sat_vdes_gain.norm() > 1e-9) {
    v_des_aug += params_.sat_vdes_gain.cwiseProduct(sat);
  }

  out.tau = dynamics.ComputeOperationalTorque(
      q_full, v_full, T_des_base, v_des_aug, osc_gains, &out.metrics);

  if (params_.sat_task_ff.norm() > 1e-9) {
    const Eigen::Matrix<double, 6, 1> tau_sat = ComputeSatTaskTorque(J, sat);
    std::array<double, PinocchioDynamicsModel::kDof> tau_sat_arr{};
    for (size_t i = 0; i < PinocchioDynamicsModel::kDof; ++i) {
      tau_sat_arr[i] = tau_sat[static_cast<Eigen::Index>(i)];
    }
    dynamics.AddArmJointTorques(out.tau, tau_sat_arr);
  }

  out.tau_ndo = UpdateNdo(dynamics, q_full, v, out.tau, dt);
  std::array<double, PinocchioDynamicsModel::kDof> tau_ndo_arr{};
  for (size_t i = 0; i < PinocchioDynamicsModel::kDof; ++i) {
    tau_ndo_arr[i] = out.tau_ndo[static_cast<Eigen::Index>(i)];
  }
  dynamics.AddArmJointTorques(out.tau, tau_ndo_arr);

  if (ik_solver != nullptr) {
    ik_solver->set_current_q(q);
    auto ik_result = ik_solver->SolveSe3(T_des_base, ik_cycle_iters);
    if (!ik_result.acceptable) {
      ik_result = ik_solver->RefineSe3(T_des_base, ik_cycle_iters * 2);
    }
    out.q_plan = ik_solver->current_q();
  }

  (void)ik_cycle_iters;

  return out;
}
