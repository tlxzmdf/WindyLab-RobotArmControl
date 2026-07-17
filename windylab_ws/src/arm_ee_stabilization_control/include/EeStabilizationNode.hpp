#pragma once

#include <array>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "BaseDisturbanceGenerator.hpp"
#include "PinocchioDynamicsModel.hpp"
#include "PinocchioIkSolver.hpp"
#include "StabilizationModeD.hpp"

class EeStabilizationNode : public rclcpp::Node {
 public:
  EeStabilizationNode();

 private:
  void ControlLoop();
  void PublishJointStates(
      const BaseDisturbanceGenerator::MountPose& mount, const rclcpp::Time& stamp);
  void PublishHardwareCommand(
      const std::array<double, PinocchioDynamicsModel::kDof>& q_cmd,
      const std::array<double, PinocchioDynamicsModel::kDof>& dq_cmd,
      const Eigen::VectorXd& tau,
      const rclcpp::Time& stamp);
  void PublishMarkers(
      const BaseDisturbanceGenerator::MountPose& mount, const rclcpp::Time& stamp);
  void PublishMetrics(
      const PinocchioDynamicsModel::OperationalResult& result,
      const BaseDisturbanceGenerator::MountPose& mount,
      const rclcpp::Time& stamp);
  void PublishReferenceJoints(
      const std::array<double, PinocchioDynamicsModel::kDof>& q_plan,
      const std::array<double, PinocchioDynamicsModel::kDof>& q_cmd,
      const rclcpp::Time& stamp);
  void JointFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void MasterJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void MountDisturbanceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  bool UpdateTeleopTarget(double dt);
  pinocchio::SE3 ComputeMasterEeInWorld() const;
  BaseDisturbanceGenerator::MountPose GetMountPose(double dt);
  BaseDisturbanceGenerator::MountPose GetMountFromTf(double dt);
  bool LookupBaseTransform(Eigen::Vector3d* position, Eigen::Quaterniond* orientation);

  pinocchio::SE3 ComputeTargetInBase(
      const BaseDisturbanceGenerator::MountPose& mount) const;
  Eigen::Matrix<double, 6, 1> ComputeDesiredTaskVelocityAnalytic(
      const BaseDisturbanceGenerator::MountPose& mount_world, double dt) const;
  std::array<double, PinocchioDynamicsModel::kDof> ComputeHardwareJointVelocity(
      const pinocchio::SE3& T_des_base,
      const Eigen::Matrix<double, 6, 1>& v_task_des) const;
  pinocchio::SE3 ComputeEeInWorld(
      const BaseDisturbanceGenerator::MountPose& mount) const;
  pinocchio::SE3 MountToWorldBase(
      const BaseDisturbanceGenerator::MountPose& mount) const;
  pinocchio::SE3 MountToWorldDrone(
      const BaseDisturbanceGenerator::MountPose& mount) const;
  BaseDisturbanceGenerator::MountPose ApplyMountAnchor(
      const BaseDisturbanceGenerator::MountPose& mount) const;
  std::array<double, 6> MountToJointValues(
      const BaseDisturbanceGenerator::MountPose& mount) const;
  bool IsMountOffsetReachable(
      const BaseDisturbanceGenerator::MountPose& offset_pose);
  void SetupDisturbanceValidator();

  std::unique_ptr<PinocchioDynamicsModel> dynamics_;
  std::unique_ptr<PinocchioIkSolver> ik_solver_;
  std::unique_ptr<BaseDisturbanceGenerator> disturbance_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr student_cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_feedback_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr master_joint_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr mount_disturbance_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr error_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr reference_pub_;

  std::vector<std::string> mount_joint_names_;
  std::vector<std::string> arm_joint_names_;
  std::vector<std::string> hardware_joint_names_;
  double control_rate_{500.0};
  double torque_limit_{45.0};
  double hw_torque_limit_{9.0};
  // Mode C anti-jitter: zero MIT velocity channel; LPF on torque FF (0 = off).
  bool hw_zero_dq_{false};
  double hw_torque_lpf_alpha_{0.0};
  double q_des_filter_alpha_{0.93};
  bool use_ik_joint_control_{true};
  bool kinematic_stabilization_{true};
  bool use_mode_d_{false};
  bool use_mode_e_{false};
  bool hardware_mode_{false};
  bool feedback_ready_{false};
  bool use_torque_feedforward_{false};
  std::string base_source_{"simulated"};
  std::string world_frame_{"world"};
  std::string base_frame_name_{"base_link"};
  size_t hardware_dof_{7};
  double joint7_value_{0.0};
  int ik_cycle_iters_{14};
  int ik_refine_iters_{22};
  int ik_recovery_iters_{36};
  int ik_validate_iters_{12};
  double ik_reach_pos_tol_{0.015};
  double ik_reach_orient_tol_{0.10};
  Eigen::VectorXd kp_joint_{Eigen::VectorXd::Zero(6)};
  Eigen::VectorXd kd_joint_{Eigen::VectorXd::Zero(6)};
  Eigen::Matrix<double, 6, 1> clik_kp_{Eigen::Matrix<double, 6, 1>::Zero()};
  double clik_damping_{0.05};
  double max_joint_velocity_{3.0};
  std::array<double, PinocchioDynamicsModel::kDof> max_joint_velocity_vec_{};
  // Mode B: extra LPF on joint1 q* (0 = off). Lateral shake couples strongly into j1.
  double hw_j1_q_filter_{0.0};
  double q1_extra_filt_{0.0};
  bool q1_extra_filt_init_{false};
  // Phase 1 anti-chatter: soft tanh rate limit (vs hard clamp) + wrist hold near target.
  bool hw_soft_rate_limit_{false};
  double hw_wrist_hold_pos_m_{0.0};
  double hw_wrist_hold_orient_rad_{0.0};
  double hw_wrist_hold_max_plane_vel_{0.0};
  // Outer-loop isolation: task FF scale + adaptive CLIK + optional q* CLIK correct.
  double hw_task_ff_scale_{1.0};
  double hw_task_ff_orient_scale_{1.0};
  double clik_kp_err_boost_{1.0};
  double clik_kp_err_ref_m_{0.04};
  bool hw_clik_q_correct_{false};
  double hw_clik_q_correct_gain_{0.0};
  double last_plane_speed_mps_{0.0};
  // Adaptive filters: none | fixed | error_adaptive | one_euro
  std::string q_des_filter_mode_{"fixed"};
  double q_des_filter_alpha_lo_{0.55};
  double q_des_filter_alpha_hi_{0.93};
  double q_des_filter_err_ref_m_{0.04};
  double q_des_one_euro_mincutoff_{1.0};
  double q_des_one_euro_beta_{0.35};
  double q_des_one_euro_dcutoff_{1.0};
  std::array<double, PinocchioDynamicsModel::kDof> q_des_euro_hat_{};
  std::array<double, PinocchioDynamicsModel::kDof> q_des_euro_dhat_{};
  bool q_des_euro_init_{false};
  std::string hw_j1_filter_mode_{"fixed"};
  double hw_j1_filter_alpha_lo_{0.55};
  double last_ee_pos_err_m_{0.0};
  std::string tf_velocity_filter_mode_{"fixed"};
  double tf_one_euro_mincutoff_{1.2};
  double tf_one_euro_beta_{0.25};
  double tf_one_euro_dcutoff_{1.0};
  Eigen::Vector3d tf_lin_euro_dhat_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d tf_ang_euro_dhat_{Eigen::Vector3d::Zero()};
  double tf_velocity_filter_alpha_{0.85};
  double ctc_vd_scale_{0.95};

  // Mode E: CLIK-integrated q* + nullspace continuity (low-lag anti-jitter).
  bool mode_e_clik_integrate_{true};
  int mode_e_clik_substeps_{4};
  double mode_e_nullspace_gain_{0.45};
  double mode_e_ik_correct_pos_m_{0.010};
  double mode_e_ik_correct_orient_rad_{0.10};
  double mode_e_task_deadband_pos_m_{0.004};
  double mode_e_task_deadband_orient_rad_{0.035};
  double mode_e_jump_reject_rad_{0.22};

  std::array<double, PinocchioDynamicsModel::kDof> q_{};
  std::array<double, PinocchioDynamicsModel::kDof> q_des_filtered_{};
  std::array<double, PinocchioDynamicsModel::kDof> v_{};
  std::array<double, PinocchioDynamicsModel::kDof> tau_hw_filt_{};
  bool ref_init_{false};
  bool teleop_mode_{false};
  bool master_ready_{false};
  bool teleop_vel_init_{false};
  double teleop_target_filter_{0.35};
  double teleop_ik_nullspace_gain_{0.35};
  double teleop_wrist_singularity_damping_scale_{0.08};
  int teleop_clik_substeps_{8};
  Eigen::Matrix<double, 6, 1> teleop_clik_kp_{Eigen::Matrix<double, 6, 1>::Zero()};
  std::string teleop_control_mode_{"ee_stabilization"};
  std::string master_joint_topic_{"/master/joint_states"};
  std::string mount_disturbance_topic_{"/mount_disturbance/pose"};
  BaseDisturbanceGenerator::MountPose external_mount_{};
  BaseDisturbanceGenerator::MountPose prev_external_mount_{};
  bool external_mount_ready_{false};
  std::array<double, PinocchioDynamicsModel::kDof> master_q_{};
  std::unique_ptr<PinocchioDynamicsModel> master_fk_;
  pinocchio::SE3 prev_T_des_base_{pinocchio::SE3::Identity()};
  uint64_t control_step_{0};
  double last_solve_time_us_{0.0};

  PinocchioDynamicsModel::OperationalGains osc_gains_;
  std::unique_ptr<StabilizationModeD> mode_d_;
  std::array<double, PinocchioDynamicsModel::kDof> mode_d_dq_ref_{};
  rclcpp::Time last_joint_stamp_{0, 0, RCL_ROS_TIME};

  pinocchio::SE3 ee_target_world_{pinocchio::SE3::Identity()};
  double mount_base_offset_z_{0.02};
  std::string arm_root_name_{"机载端"};
  std::string arm_ee_name_{"末端"};
  std::deque<Eigen::Vector3d> base_trail_;
  static constexpr size_t kTrailLength = 400;

  Eigen::Vector3d mount_anchor_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d mount_anchor_rpy_{Eigen::Vector3d::Zero()};
  bool mount_anchor_set_{false};

  std::optional<Eigen::Vector3d> last_tf_position_;
  std::optional<Eigen::Quaterniond> last_tf_orientation_;
  Eigen::Vector3d tf_lin_vel_filt_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d tf_ang_vel_filt_{Eigen::Vector3d::Zero()};
  bool tf_vel_filt_init_{false};
  rclcpp::Time last_tf_stamp_{0, 0, RCL_ROS_TIME};
};
