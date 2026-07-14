#pragma once

#include <array>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>

/// Pinocchio Lagrangian dynamics for A-L1-GAMMA 6-DOF arm.
class PinocchioDynamicsModel {
 public:
  static constexpr size_t kDof = 6;

  struct OperationalGains {
    Eigen::Matrix<double, 6, 1> kp{Eigen::Matrix<double, 6, 1>::Zero()};
    Eigen::Matrix<double, 6, 1> kd{Eigen::Matrix<double, 6, 1>::Zero()};
    double lambda{0.05};
  };

  struct OperationalResult {
    Eigen::Matrix<double, 6, 1> task_error{Eigen::Matrix<double, 6, 1>::Zero()};
    double position_error{0.0};
    double orientation_error{0.0};
  };

  PinocchioDynamicsModel(
      const std::string& urdf_path,
      const std::vector<std::string>& joint_names,
      const std::string& base_frame,
      const std::string& ee_frame);

  const pinocchio::Model& model() const { return model_; }
  pinocchio::Data& data() { return data_; }

  Eigen::VectorXd PackQ(const std::array<double, kDof>& q) const;
  Eigen::VectorXd PackV(const std::array<double, kDof>& v) const;
  void UnpackQ(const Eigen::VectorXd& q, std::array<double, kDof>& out) const;
  void UnpackV(const Eigen::VectorXd& v, std::array<double, kDof>& out) const;

  Eigen::MatrixXd ComputeMassMatrix(const Eigen::VectorXd& q);
  Eigen::VectorXd ComputeNonLinearEffects(
      const Eigen::VectorXd& q, const Eigen::VectorXd& v);

  /// Computed-torque tracking: tau = M(q)(a_d + Kp*e + Kd*e_dot) + nle(q,v)
  Eigen::VectorXd ComputeTrackingTorque(
      const Eigen::VectorXd& q,
      const Eigen::VectorXd& v,
      const Eigen::VectorXd& q_d,
      const Eigen::VectorXd& v_d,
      const Eigen::VectorXd& a_d,
      const Eigen::VectorXd& kp,
      const Eigen::VectorXd& kd);

  Eigen::VectorXd ForwardDynamics(
      const Eigen::VectorXd& q,
      const Eigen::VectorXd& v,
      const Eigen::VectorXd& tau);

  pinocchio::SE3 ComputeEePoseInBase(const Eigen::VectorXd& q) const;

  /// 6x6 arm Jacobian (LWA linear + LOCAL angular), same convention as OSC.
  Eigen::Matrix<double, 6, 6> ComputeArmJacobian(
      const Eigen::VectorXd& q) const;

  /// CLIK joint velocity: dq = J^T (J J^T + λ² I)^{-1} (K ⊙ e + v_task).
  /// Avoids numerical differentiation of joint targets.
  std::array<double, kDof> ComputeClikJointVelocity(
      const Eigen::VectorXd& q,
      const pinocchio::SE3& T_des_base,
      const Eigen::Matrix<double, 6, 1>& v_task_des,
      const Eigen::Matrix<double, 6, 1>& clik_kp,
      double damping,
      double max_joint_velocity = 3.0) const;

  /// CLIK with Liegeois null-space bias toward a reference configuration.
  /// dq = J^# (K·e + v) + (I - J^# J) k (q_ref - q), continuous near wrist singularities.
  std::array<double, kDof> ComputeClikJointVelocityWithNullSpace(
      const Eigen::VectorXd& q,
      const pinocchio::SE3& T_des_base,
      const Eigen::Matrix<double, 6, 1>& v_task_des,
      const std::array<double, kDof>& q_ref,
      const Eigen::Matrix<double, 6, 1>& clik_kp,
      double damping,
      double nullspace_gain,
      double max_joint_velocity = 3.0) const;

  /// Dynamically consistent operational-space control in base frame.
  /// T_des_base moves with the mount so the arm compensates base disturbance.
  /// v_des is the 6D spatial velocity of the moving task target (world-aligned).
  Eigen::VectorXd ComputeOperationalTorque(
      const Eigen::VectorXd& q,
      const Eigen::VectorXd& v,
      const pinocchio::SE3& T_des_base,
      const Eigen::Matrix<double, 6, 1>& v_des,
      const OperationalGains& gains,
      OperationalResult* metrics = nullptr);

  Eigen::Vector3d ComputeEePositionInWorld(const Eigen::VectorXd& q) const;

  /// Add per-arm-joint torques into the full nv torque vector.
  void AddArmJointTorques(
      Eigen::VectorXd& tau,
      const std::array<double, kDof>& arm_tau) const;

  Eigen::Matrix<double, 6, 6> ExtractArmMassMatrix(const Eigen::MatrixXd& M_full) const;
  Eigen::Matrix<double, 6, 1> ExtractArmSubvector(const Eigen::VectorXd& full) const;

 private:
  pinocchio::Model model_;
  mutable pinocchio::Data data_{pinocchio::Model()};
  pinocchio::FrameIndex base_frame_id_{0};
  pinocchio::FrameIndex ee_frame_id_{0};
  std::vector<int> q_indices_;
  std::vector<int> v_indices_;
};
