#include "VelocityWbcController.hpp"

#include <algorithm>
#include <cmath>

VelocityWbcController::VelocityWbcController(BoxQpSolver qp_solver)
    : qp_solver_(std::move(qp_solver)) {}

double VelocityWbcController::UnwrapNear(double angle, double reference) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  while (angle - reference > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle - reference < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

Eigen::Matrix<double, 6, 1> VelocityWbcController::ComputeTaskCommand(
    const Eigen::Matrix<double, 6, 1>& task_error,
    const Eigen::Matrix<double, 6, 1>& feedforward,
    const Params& params,
    State* state,
    const double dt,
    const bool accumulate_integral) const {
  if (state != nullptr && accumulate_integral) {
    state->integral_error += task_error * dt;
    for (int i = 0; i < 6; ++i) {
      state->integral_error[i] = std::clamp(
          state->integral_error[i],
          -params.integral_limit[i],
          params.integral_limit[i]);
    }
  }
  return params.kp.cwiseProduct(task_error)
      + params.ki.cwiseProduct(state != nullptr ? state->integral_error
                                                : Eigen::Matrix<double, 6, 1>::Zero())
      + feedforward;
}

std::array<double, PinocchioArmModel::kDof> VelocityWbcController::SolveJointVelocity(
    const Eigen::Matrix<double, 6, 6>& jacobian,
    const Eigen::Matrix<double, 6, 1>& task_command,
    const std::array<double, PinocchioArmModel::kDof>& q,
    const std::array<double, PinocchioArmModel::kDof>& q_reference,
    const Params& params,
    State* state) const {
  const Eigen::Matrix<double, 6, 6> lhs =
      jacobian * jacobian.transpose()
      + params.clik_damping * params.clik_damping
            * Eigen::Matrix<double, 6, 6>::Identity();
  const Eigen::Matrix<double, 6, 6> jsharp =
      jacobian.transpose() * lhs.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());

  Eigen::Matrix<double, 6, 1> dq = jsharp * task_command;

  if (params.nullspace_weight > 1e-9) {
    const Eigen::Matrix<double, 6, 6> nullspace_projector =
        Eigen::Matrix<double, 6, 6>::Identity() - jsharp * jacobian;
    Eigen::Matrix<double, 6, 1> q_err = Eigen::Matrix<double, 6, 1>::Zero();
    for (size_t i = 0; i < PinocchioArmModel::kDof; ++i) {
      q_err[static_cast<Eigen::Index>(i)] =
          UnwrapNear(q_reference[i], q[i]) - q[i];
    }
    dq += params.nullspace_weight * nullspace_projector * q_err;
  }

  std::array<double, PinocchioArmModel::kDof> out{};
  for (size_t i = 0; i < PinocchioArmModel::kDof; ++i) {
    out[i] = std::clamp(
        dq[static_cast<Eigen::Index>(i)],
        -params.max_joint_velocity,
        params.max_joint_velocity);
  }
  if (state != nullptr) {
    state->prev_q_dot = out;
  }
  (void)qp_solver_;
  return out;
}
