#pragma once

#include <array>

#include <Eigen/Dense>
#include <pinocchio/spatial/se3.hpp>

#include "PinocchioDynamicsModel.hpp"

class PinocchioIkSolver;

/// Mode D: Wang-2024-style sat velocity planner + OSC + lightweight NDO.
/// Implemented in a separate module; modes A/B/C code paths are unchanged.
class StabilizationModeD {
 public:
  struct Params {
    Eigen::Matrix<double, 6, 1> potential_kp{
        Eigen::Matrix<double, 6, 1>::Zero()};
    Eigen::Matrix<double, 6, 1> sigma_sq{
        Eigen::Matrix<double, 6, 1>::Zero()};
    double alpha{1.0};
    double kd_sat{2.5};
    double sat_a{10.0};
    double sat_epsilon{0.08};
    int potential_N{2};
    double clik_damping{0.05};
    double max_joint_velocity{3.0};
    /// Augment OSC desired twist: v_des += sat_vdes_gain ⊙ sat(e).
    Eigen::Matrix<double, 6, 1> sat_vdes_gain{
        Eigen::Matrix<double, 6, 1>::Zero()};
    /// Optional task-space sat torque feedforward: tau += J^T (K_sat ⊙ sat(e)).
    Eigen::Matrix<double, 6, 1> sat_task_ff{
        Eigen::Matrix<double, 6, 1>::Zero()};
    /// Momentum-observer gain for lightweight NDO disturbance estimate.
    double ndo_gain{5.0};
    double ndo_torque_limit{10.0};
  };

  struct Output {
    Eigen::VectorXd tau;
    std::array<double, PinocchioDynamicsModel::kDof> dq_ref{};
    std::array<double, PinocchioDynamicsModel::kDof> q_ref{};
    std::array<double, PinocchioDynamicsModel::kDof> q_plan{};
    PinocchioDynamicsModel::OperationalResult metrics{};
    Eigen::Matrix<double, 6, 1> twist_cmd{
        Eigen::Matrix<double, 6, 1>::Zero()};
    Eigen::Matrix<double, 6, 1> tau_ndo{
        Eigen::Matrix<double, 6, 1>::Zero()};
  };

  explicit StabilizationModeD(const Params& params);

  void Reset(const std::array<double, PinocchioDynamicsModel::kDof>& q);

  Output Step(
      PinocchioDynamicsModel& dynamics,
      const std::array<double, PinocchioDynamicsModel::kDof>& q,
      const std::array<double, PinocchioDynamicsModel::kDof>& v,
      const pinocchio::SE3& T_des_base,
      const Eigen::Matrix<double, 6, 1>& v_des,
      double dt,
      const PinocchioDynamicsModel::OperationalGains& osc_gains,
      PinocchioIkSolver* ik_solver,
      int ik_cycle_iters);

 private:
  static Eigen::Matrix<double, 6, 1> SatTaskError(
      const Eigen::Matrix<double, 6, 1>& task_error,
      double sat_a,
      double sat_epsilon);

  static Eigen::Matrix<double, 6, 1> PotentialGradient(
      const Eigen::Matrix<double, 6, 1>& task_error,
      const Eigen::Matrix<double, 6, 1>& potential_kp,
      const Eigen::Matrix<double, 6, 1>& sigma_sq,
      int potential_N);

  static Eigen::Matrix<double, 6, 6> DampedJacobianInverse(
      const Eigen::Matrix<double, 6, 6>& jacobian,
      double damping);

  std::array<double, PinocchioDynamicsModel::kDof> ComputeSatJointVelocity(
      const Eigen::Matrix<double, 6, 1>& task_error,
      const Eigen::Matrix<double, 6, 1>& v_des,
      const Eigen::Matrix<double, 6, 6>& jacobian) const;

  Eigen::Matrix<double, 6, 1> ComputeSatTaskTorque(
      const Eigen::Matrix<double, 6, 6>& jacobian,
      const Eigen::Matrix<double, 6, 1>& sat) const;

  Eigen::Matrix<double, 6, 1> UpdateNdo(
      PinocchioDynamicsModel& dynamics,
      const Eigen::VectorXd& q_full,
      const std::array<double, PinocchioDynamicsModel::kDof>& v,
      const Eigen::VectorXd& tau_before_ndo,
      double dt);

  Params params_;
  Eigen::Matrix<double, 6, 1> ndo_momentum_hat_{
      Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> ndo_disturbance_{
      Eigen::Matrix<double, 6, 1>::Zero()};
  bool ndo_init_{false};
  std::array<double, PinocchioDynamicsModel::kDof> q_ref_{};
  bool q_ref_init_{false};
};
