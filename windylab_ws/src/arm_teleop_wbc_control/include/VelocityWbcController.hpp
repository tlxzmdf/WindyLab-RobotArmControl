#pragma once

#include <array>

#include <Eigen/Dense>

#include "BoxQpSolver.hpp"
#include "PinocchioArmModel.hpp"

/// Velocity-level QP-WBC with integral task action.
class VelocityWbcController {
 public:
  struct Params {
    Eigen::Matrix<double, 6, 1> task_weight{Eigen::Matrix<double, 6, 1>::Ones()};
    Eigen::Matrix<double, 6, 1> kp{Eigen::Matrix<double, 6, 1>::Zero()};
    Eigen::Matrix<double, 6, 1> ki{Eigen::Matrix<double, 6, 1>::Zero()};
    Eigen::Matrix<double, 6, 1> integral_limit{Eigen::Matrix<double, 6, 1>::Constant(0.5)};
    double nullspace_weight{0.15};
    double nullspace_rate{2.5};
    double clik_damping{0.035};
    double regularization{0.02};
    double max_joint_velocity{4.0};
  };

  struct State {
    Eigen::Matrix<double, 6, 1> integral_error{Eigen::Matrix<double, 6, 1>::Zero()};
    std::array<double, PinocchioArmModel::kDof> prev_q_dot{};
  };

  explicit VelocityWbcController(BoxQpSolver qp_solver = BoxQpSolver{});

  Eigen::Matrix<double, 6, 1> ComputeTaskCommand(
      const Eigen::Matrix<double, 6, 1>& task_error,
      const Eigen::Matrix<double, 6, 1>& feedforward,
      const Params& params,
      State* state,
      double dt,
      bool accumulate_integral = true) const;

  std::array<double, PinocchioArmModel::kDof> SolveJointVelocity(
      const Eigen::Matrix<double, 6, 6>& jacobian,
      const Eigen::Matrix<double, 6, 1>& task_command,
      const std::array<double, PinocchioArmModel::kDof>& q,
      const std::array<double, PinocchioArmModel::kDof>& q_reference,
      const Params& params,
      State* state) const;

  static double UnwrapNear(double angle, double reference);

 private:
  BoxQpSolver qp_solver_;
};
